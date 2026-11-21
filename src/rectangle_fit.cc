#include "src/rectangle_fit.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <numbers>
#include <random>
#include <unordered_map>
#include <utility>

#include "src/portable_random.h"

namespace cell {
namespace {

using Eigen::Vector2d;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// `v` turned a quarter turn anticlockwise.
Vector2d Perp(const Vector2d& v) { return {-v.y(), v.x()}; }

Vector2d Rotate(const Vector2d& v, double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {c * v.x() - s * v.y(), s * v.x() + c * v.y()};
}

// Twice the signed area of the triangle o, a, b: positive when the three
// turn anticlockwise, zero when they are in a line.
double Cross(const Vector2d& o, const Vector2d& a, const Vector2d& b) {
  return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

bool Finite(const Vector2d& p) {
  return std::isfinite(p.x()) && std::isfinite(p.y());
}

bool AllFinite(const Points2d& points) {
  for (const Vector2d& p : points) {
    if (!Finite(p)) return false;
  }
  return true;
}

// ----------------------------------------------------------------- the grid

// A square cell of the plane, by its column and row.
struct Cell {
  std::int64_t x = 0;
  std::int64_t y = 0;
  bool operator==(const Cell&) const = default;
};

struct CellHash {
  std::size_t operator()(const Cell& c) const {
    return std::hash<std::uint64_t>()(static_cast<std::uint64_t>(c.x) *
                                          0x9E3779B97F4A7C15ull ^
                                      static_cast<std::uint64_t>(c.y));
  }
};

// The cell a finite point is in. Absurdly distant points share the outermost
// cells rather than overflow the index.
Cell CellOf(const Vector2d& p, double cell_size) {
  const double limit = 1e15;
  return {static_cast<std::int64_t>(
              std::floor(std::clamp(p.x() / cell_size, -limit, limit))),
          static_cast<std::int64_t>(
              std::floor(std::clamp(p.y() / cell_size, -limit, limit)))};
}

// Dots filed by cell, so that a dot's neighbours are found by looking in its
// own cell and the eight around it instead of everywhere.
using Grid = std::unordered_map<Cell, std::vector<int>, CellHash>;

// ------------------------------------------------------------- the sides

// Where side k of a rectangle is: its outward normal, its direction going
// anticlockwise round the rectangle, the distance from the centre to it, and
// half its length.
struct SideFrame {
  Vector2d outward;
  Vector2d along;
  double half_depth = 0.0;
  double half_length = 0.0;
};

SideFrame FrameOf(const OrientedRectangle& r, int k) {
  SideFrame f;
  const bool long_side = k % 2 == 0;
  switch (k) {
    case 0: f.outward = -r.Across(); break;
    case 1: f.outward = r.axis; break;
    case 2: f.outward = r.Across(); break;
    default: f.outward = -r.axis; break;
  }
  f.along = Perp(f.outward);
  f.half_depth = 0.5 * (long_side ? r.width : r.length);
  f.half_length = 0.5 * (long_side ? r.length : r.width);
  return f;
}

// The dots' coordinates in a rectangle's own frame, from its centre: `u`
// along its axis and `v` across. Kept as plain numbers, because everything
// after this runs over every dot several times per fit.
struct Projection {
  std::vector<double> u;
  std::vector<double> v;
};

void Project(const std::vector<double>& xs, const std::vector<double>& ys,
             const OrientedRectangle& r, Projection* out) {
  const double cx = r.centre.x();
  const double cy = r.centre.y();
  const double ax = r.axis.x();
  const double ay = r.axis.y();
  const int n = static_cast<int>(xs.size());
  out->u.resize(n);
  out->v.resize(n);
  for (int i = 0; i < n; ++i) {
    const double dx = xs[i] - cx;
    const double dy = ys[i] - cy;
    out->u[i] = dx * ax + dy * ay;
    out->v[i] = dy * ax - dx * ay;
  }
}

// A dot's depth from the centre toward side k (its distance along the
// side's outward normal) and its place along the side, from the rectangle
// frame's u and v. As in FrameOf: side 0 faces -v, 1 faces +u, 2 faces +v,
// 3 faces -u.
struct SidePlace {
  double depth;
  double s;
};

SidePlace PlaceOn(int k, double u, double v) {
  switch (k) {
    case 0: return {-v, u};
    case 1: return {u, v};
    case 2: return {v, -u};
    default: return {-u, -v};
  }
}

// Equal strips across a side, from `start` (measured along the side from
// the middle) onward.
struct Strips {
  double start = 0.0;
  double width = 1.0;
  int count = 0;

  // The strip a position along the side falls in, or -1 for none.
  int Of(double s) const {
    const double k = std::floor((s - start) / width);
    return k >= 0.0 && k < count ? static_cast<int>(k) : -1;
  }
};

Strips StripsOf(const SideFrame& f, double end_margin, double strip_width) {
  const double half = f.half_length * (1.0 - 2.0 * end_margin);
  Strips strips;
  strips.count = std::max(1, static_cast<int>(std::floor(2.0 * half /
                                                         strip_width)));
  strips.width = 2.0 * half / strips.count;
  strips.start = -half;
  return strips;
}

// A strip's outermost dot. `s` is along the side from its middle, `t`
// outward from the side's line: negative inside the rectangle.
struct EdgeDot {
  int strip = -1;
  int point = -1;
  double s = 0.0;
  double t = -kInfinity;
};

// Of the dots in the rectangle's half nearer side k, the outermost in each
// strip. Strips with no such dot are left out.
std::vector<EdgeDot> FindEdgeDots(const Projection& frame, int k,
                                  double half_depth, const Strips& strips) {
  std::vector<EdgeDot> best(strips.count);
  for (int i = 0; i < static_cast<int>(frame.u.size()); ++i) {
    const SidePlace place = PlaceOn(k, frame.u[i], frame.v[i]);
    if (place.depth < 0.0) continue;
    const int strip = strips.Of(place.s);
    if (strip < 0) continue;
    const double t = place.depth - half_depth;
    if (t > best[strip].t) best[strip] = {strip, i, place.s, t};
  }
  std::vector<EdgeDot> found;
  for (const EdgeDot& dot : best) {
    if (dot.point >= 0) found.push_back(dot);
  }
  return found;
}

// A line across a side's frame: t = offset + slope s.
struct Line {
  double offset = 0.0;
  double slope = 0.0;
  double At(double s) const { return offset + slope * s; }
};

// The least-squares line through the dots at `use`. Nothing if they do not
// spread along the side.
std::optional<Line> LeastSquaresLine(const std::vector<EdgeDot>& dots,
                                     const std::vector<int>& use) {
  if (use.size() < 2) return std::nullopt;
  double mean_s = 0.0;
  double mean_t = 0.0;
  for (int j : use) {
    mean_s += dots[j].s;
    mean_t += dots[j].t;
  }
  mean_s /= static_cast<double>(use.size());
  mean_t /= static_cast<double>(use.size());
  double sxx = 0.0;
  double sxy = 0.0;
  for (int j : use) {
    sxx += (dots[j].s - mean_s) * (dots[j].s - mean_s);
    sxy += (dots[j].s - mean_s) * (dots[j].t - mean_t);
  }
  if (!(sxx > 0.0)) return std::nullopt;
  Line line;
  line.slope = sxy / sxx;
  line.offset = mean_t - line.slope * mean_s;
  return line;
}

std::vector<int> Within(const std::vector<EdgeDot>& dots, const Line& line,
                        double tolerance) {
  std::vector<int> near;
  for (int j = 0; j < static_cast<int>(dots.size()); ++j) {
    if (std::abs(dots[j].t - line.At(dots[j].s)) <= tolerance) {
      near.push_back(j);
    }
  }
  return near;
}

// Of lines through two of the dots at `candidates`, the one with the most
// of them within `tolerance`, as in FitPlaneRansac.
std::optional<Line> RansacLine(const std::vector<EdgeDot>& dots,
                               const std::vector<int>& candidates,
                               double tolerance, std::mt19937& rng) {
  const int m = static_cast<int>(candidates.size());
  if (m < 2) return std::nullopt;
  // UniformIndex, not std::uniform_int_distribution, so that a seed picks
  // the same pairs with every standard library.
  auto pick = [m](std::mt19937& r) { return UniformIndex(r, m); };
  Line best;
  int best_count = 0;
  int iterations = 300;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    const EdgeDot& a = dots[candidates[pick(rng)]];
    const EdgeDot& b = dots[candidates[pick(rng)]];
    const double ds = b.s - a.s;
    if (!(std::abs(ds) > 0.0)) continue;
    Line line;
    line.slope = (b.t - a.t) / ds;
    line.offset = a.t - line.slope * a.s;
    int count = 0;
    for (int j : candidates) {
      if (std::abs(dots[j].t - line.At(dots[j].s)) <= tolerance) ++count;
    }
    if (count > best_count) {
      best_count = count;
      best = line;
      // Two dots both on the side with probability fraction^2; stop once
      // missing it every time is less likely than 1 in 1000.
      const double fraction = static_cast<double>(count) / m;
      const double miss = 1.0 - fraction * fraction;
      if (miss <= 0.0) break;
      const double enough = std::ceil(std::log(1e-3) / std::log(miss));
      if (enough < iterations) iterations = static_cast<int>(enough);
    }
  }
  if (best_count < 2) return std::nullopt;
  return best;
}

// Root mean square distance of the dots at `use` from `line`.
double RmsAbout(const std::vector<EdgeDot>& dots, const std::vector<int>& use,
                const Line& line) {
  if (use.empty()) return 0.0;
  double squares = 0.0;
  for (int j : use) {
    const double r = dots[j].t - line.At(dots[j].s);
    squares += r * r;
  }
  return std::sqrt(squares / static_cast<double>(use.size()));
}

// A side's line and the indices of the edge dots on it.
struct SideLine {
  Line line;
  std::vector<int> kept;
};

// Least squares on the line's supporters, repeated until they settle.
SideLine Polish(const std::vector<EdgeDot>& dots, Line line,
                double tolerance) {
  std::vector<int> kept = Within(dots, line, tolerance);
  for (int round = 0; round < 5; ++round) {
    const std::optional<Line> refit = LeastSquaresLine(dots, kept);
    if (!refit) break;
    std::vector<int> next = Within(dots, *refit, tolerance);
    if (next.size() < 2) break;
    const bool settled = next == kept;
    line = *refit;
    kept = std::move(next);
    if (settled) break;
  }
  return {line, kept};
}

// The side's line through its edge dots, robustly.
//
// The line with most support is not always the edge. Where something in
// front hides most of a side, the boundary of what hides it can be the
// longest straight run of edge dots, with the true edge's dots beyond it.
// Dots cannot lie beyond the true edge, apart from a few strays. So while
// enough edge dots (`min_count`) lie beyond the line found, a line is
// sought among those alone, and if they make one, it is taken instead.
std::optional<SideLine> FitSideLine(const std::vector<EdgeDot>& dots,
                                    double tolerance, int min_count,
                                    std::mt19937& rng) {
  std::vector<int> candidates(dots.size());
  for (int j = 0; j < static_cast<int>(dots.size()); ++j) candidates[j] = j;
  std::optional<SideLine> result;
  for (int round = 0; round < 4; ++round) {
    const std::optional<Line> sample =
        RansacLine(dots, candidates, tolerance, rng);
    if (!sample) break;
    SideLine fit = Polish(dots, *sample, tolerance);
    if (result && static_cast<int>(fit.kept.size()) < min_count) break;
    std::vector<int> beyond;
    for (int j = 0; j < static_cast<int>(dots.size()); ++j) {
      if (dots[j].t - fit.line.At(dots[j].s) > tolerance) beyond.push_back(j);
    }
    result = std::move(fit);
    if (static_cast<int>(beyond.size()) < min_count) break;
    candidates = std::move(beyond);
  }
  return result;
}

// Where the side's edge is, by counting dots: see
// RefineOptions::correct_edge_bias. `outermost` is the line through the
// edge dots, as t in the side's frame. Only dots in the strips marked in
// `use` count, so that hidden stretches of the side do not.
//
// Dots between depths `low` and `high` inside, weighted by a smooth bump
// that adds up to 1 over that band, give how many dots there are per unit
// of depth. Dots from the same band outward, weighted by a ramp that rises
// smoothly from 0 to 1 across the band and stays at 1 beyond, give a count
// that, divided by the first, is the depth from the band's middle to the
// edge. Dots more than 4 spacings beyond the outermost line are strays and
// not counted.
//
// The weights are smooth so that the sums do not depend on where the rows
// of dots happen to fall: a row moving across a sharp cut would add or drop
// a whole row's worth at once. Moving a dot within the counted region,
// as a flying pixel or a ragged edge does, changes nothing.
std::optional<double> CountedEdge(const Projection& frame, int k,
                                  double half_depth, const Strips& strips,
                                  const std::vector<bool>& use,
                                  double outermost, double spacing) {
  // 4 to 12 spacings inside: deeper than the ragged edge, and wide enough
  // to span several rows of dots. Shallower on a rectangle too small for
  // it, which no module is.
  const double scale = std::min(1.0, 1.2 * half_depth / (12.0 * spacing));
  const double low = outermost - 12.0 * spacing * scale;
  const double high = outermost - 4.0 * spacing * scale;
  const double beyond = outermost + 4.0 * spacing;
  const double band = high - low;
  double count = 0.0;
  double per_depth = 0.0;
  for (int i = 0; i < static_cast<int>(frame.u.size()); ++i) {
    const SidePlace place = PlaceOn(k, frame.u[i], frame.v[i]);
    const double t = place.depth - half_depth;
    if (t < low || t > beyond) continue;
    const int strip = strips.Of(place.s);
    if (strip < 0 || !use[strip]) continue;
    const double x = (t - low) / band;
    if (x >= 1.0) {
      count += 1.0;
    } else {
      const double turn = 2.0 * std::numbers::pi * x;
      count += x - std::sin(turn) / (2.0 * std::numbers::pi);
      per_depth += (1.0 - std::cos(turn)) / band;
    }
  }
  if (!(per_depth > 0.0)) return std::nullopt;
  return 0.5 * (low + high) + count / per_depth;
}

// Where two lines, each a point and a direction, cross.
Vector2d Intersect(const Vector2d& p1, const Vector2d& d1, const Vector2d& p2,
                   const Vector2d& d2) {
  const double denominator = d1.x() * d2.y() - d1.y() * d2.x();
  const Vector2d w = p2 - p1;
  const double u = (w.x() * d2.y() - w.y() * d2.x()) / denominator;
  return p1 + u * d1;
}

const char* kSideNames[4] = {"long side 0", "short side 1", "long side 2",
                             "short side 3"};

std::optional<RectangleFit> Refuse(RectangleRefusal* refusal,
                                   RectangleFitFailure reason,
                                   std::string detail) {
  if (refusal != nullptr) {
    refusal->reason = reason;
    refusal->detail = std::move(detail);
  }
  return std::nullopt;
}

template <typename... Args>
std::string Format(const char* format, Args... args) {
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer), format, args...);
  return buffer;
}

constexpr double kDegree = std::numbers::pi / 180.0;

// Strips along a side are this many dot spacings wide: wide enough that
// every strip holds dots close to the edge whatever the grid's slant, narrow
// enough to give many edge dots, some 110 along a long side and 50 along a
// short one on the cell's grid. Wider strips were tried and did no better.
constexpr double kStripSpacings = 3.0;

}  // namespace

// ------------------------------------------------------------- rectangles

double OrientedRectangle::Angle() const {
  return std::atan2(axis.y(), axis.x());
}

std::array<Vector2d, 4> OrientedRectangle::Corners() const {
  const Vector2d a = 0.5 * length * axis;
  const Vector2d b = 0.5 * width * Across();
  return {centre - a - b, centre + a - b, centre + a + b, centre - a + b};
}

OrientedRectangle Canonical(const OrientedRectangle& rectangle) {
  OrientedRectangle r = rectangle;
  if (r.axis.norm() > 0.0) r.axis.normalize();
  if (r.width > r.length) {
    std::swap(r.length, r.width);
    r.axis = r.Across();
  }
  if (r.axis.x() < 0.0 || (r.axis.x() == 0.0 && r.axis.y() < 0.0)) {
    r.axis = -r.axis;
  }
  return r;
}

// ------------------------------------------------------------ dot spacing

std::optional<double> DotSpacing(const Points2d& points) {
  std::vector<double> xs;
  std::vector<double> ys;
  for (const Vector2d& p : points) {
    if (!Finite(p)) continue;
    xs.push_back(p.x());
    ys.push_back(p.y());
  }
  const int n = static_cast<int>(xs.size());
  if (n < 2) return std::nullopt;

  // A cell size for the grid that is sure to be larger than the nearest
  // neighbour distance: twice the spacing the dots would have if they filled
  // the box around them evenly. The box leaves out the farthest 1% on each
  // side, so that a few strays far away do not blow it up.
  auto extent = [n](std::vector<double>& v, int skip) {
    std::nth_element(v.begin(), v.begin() + skip, v.end());
    const double low = v[skip];
    std::nth_element(v.begin(), v.begin() + (n - 1 - skip), v.end());
    return v[n - 1 - skip] - low;
  };
  double cell_size = 0.0;
  double box_side = 0.0;  // of a square with the box's area
  for (int skip : {n / 100, 0}) {
    const double wx = extent(xs, skip);
    const double wy = extent(ys, skip);
    box_side = std::sqrt(wx * wy);
    cell_size = std::max(2.0 * box_side / std::sqrt(n),
                         2.0 * std::max(wx, wy) / n);
    if (cell_size > 0.0) break;
  }
  if (!(cell_size > 0.0) || !std::isfinite(cell_size)) return std::nullopt;

  // Nearest neighbours are looked for only in a patch round the middle of
  // the cloud, holding a few thousand dots: the middle of a face is typical
  // of it, and filing every dot in a grid would cost more than everything
  // else here. The patch is centred on the median x and y, and sized from
  // the density in the box. A neighbour farther than a cell may be missed;
  // such a dot is a stray and has no say in the median anyway.
  double half = kInfinity;
  double middle_x = 0.0;
  double middle_y = 0.0;
  if (n > 4000) {
    std::nth_element(xs.begin(), xs.begin() + n / 2, xs.end());
    std::nth_element(ys.begin(), ys.begin() + n / 2, ys.end());
    middle_x = xs[n / 2];
    middle_y = ys[n / 2];
    half = 0.5 * std::sqrt(2000.0 / n) * box_side;
  }
  std::vector<double> nearest;
  for (const double patch : {half, kInfinity}) {
    auto in_patch = [&](const Vector2d& p, double margin) {
      return std::abs(p.x() - middle_x) <= patch + margin &&
             std::abs(p.y() - middle_y) <= patch + margin;
    };
    // The patch's dots, and those within a cell of it, by cell; their
    // coordinates are copied out as plain numbers for the search.
    Grid grid;
    std::vector<double> px;
    std::vector<double> py;
    std::vector<int> samples;
    for (const Vector2d& p : points) {
      if (!Finite(p) || !in_patch(p, cell_size)) continue;
      const int j = static_cast<int>(px.size());
      grid[CellOf(p, cell_size)].push_back(j);
      px.push_back(p.x());
      py.push_back(p.y());
      if (in_patch(p, 0.0)) samples.push_back(j);
    }
    // A median of 500 is as good as one of all of them.
    const int stride = std::max(1, static_cast<int>(samples.size()) / 500);
    for (size_t k = 0; k < samples.size(); k += stride) {
      const int i = samples[k];
      const Cell home = CellOf(Vector2d(px[i], py[i]), cell_size);
      double best = kInfinity;
      for (std::int64_t cx = -1; cx <= 1; ++cx) {
        for (std::int64_t cy = -1; cy <= 1; ++cy) {
          const auto found = grid.find({home.x + cx, home.y + cy});
          if (found == grid.end()) continue;
          for (int j : found->second) {
            const double dx = px[j] - px[i];
            const double dy = py[j] - py[i];
            const double d = dx * dx + dy * dy;
            if (d > 0.0 && d < best) best = d;
          }
        }
      }
      if (best < kInfinity) nearest.push_back(std::sqrt(best));
    }
    // A hollow cloud may have nothing in the middle: then try them all.
    if (nearest.size() >= 50 || patch == kInfinity) break;
    nearest.clear();
  }
  if (nearest.empty()) return std::nullopt;
  const auto middle = nearest.begin() + nearest.size() / 2;
  std::nth_element(nearest.begin(), middle, nearest.end());
  return *middle;
}

// --------------------------------------------------------------- clusters

std::vector<int> LargestCluster(const Points2d& points, double cell_size) {
  if (!(std::isfinite(cell_size) && cell_size > 0.0)) return {};

  // Number the cells that hold a dot.
  std::unordered_map<Cell, int, CellHash> number;
  std::vector<Cell> cells;
  std::vector<int> cell_of(points.size(), -1);
  // Neighbouring dots in the list, as a camera's rows give them, are often
  // in the same cell, so the last cell is remembered.
  Cell last{std::numeric_limits<std::int64_t>::min(), 0};
  int last_number = -1;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    if (!Finite(points[i])) continue;
    const Cell c = CellOf(points[i], cell_size);
    if (!(c == last)) {
      const auto [entry, added] =
          number.try_emplace(c, static_cast<int>(cells.size()));
      if (added) cells.push_back(c);
      last = c;
      last_number = entry->second;
    }
    cell_of[i] = last_number;
  }

  // Join touching cells into clumps. Union-find: every cell points toward
  // another in its clump, and following the pointers ends at the one cell
  // that names the clump. Joining two clumps points one's name at the
  // other's. Each pair of touching cells is met once, from the cell to its
  // left, below-left, below or below-right.
  std::vector<int> parent(cells.size());
  for (int a = 0; a < static_cast<int>(cells.size()); ++a) parent[a] = a;
  auto root = [&parent](int a) {
    while (parent[a] != a) {
      parent[a] = parent[parent[a]];  // Shortens the path for next time.
      a = parent[a];
    }
    return a;
  };
  const std::int64_t offsets[4][2] = {{1, 0}, {-1, 1}, {0, 1}, {1, 1}};
  for (int a = 0; a < static_cast<int>(cells.size()); ++a) {
    for (const auto& offset : offsets) {
      const auto other =
          number.find({cells[a].x + offset[0], cells[a].y + offset[1]});
      if (other == number.end()) continue;
      const int ra = root(a);
      const int rb = root(other->second);
      if (ra != rb) parent[ra] = rb;
    }
  }

  std::vector<int> size(cells.size(), 0);
  for (int c : cell_of) {
    if (c >= 0) ++size[root(c)];
  }
  int best = -1;
  for (int c : cell_of) {
    if (c < 0) continue;
    const int r = root(c);
    if (best < 0 || size[r] > size[best]) best = r;
  }

  std::vector<int> kept;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    if (cell_of[i] >= 0 && root(cell_of[i]) == best) kept.push_back(i);
  }
  return kept;
}

// ------------------------------------------------------------ convex hull

Points2d ConvexHull(const Points2d& points) {
  // Andrew's monotone chain. Sorted left to right, the points are walked
  // twice, once for the lower half of the hull and once, right to left, for
  // the upper. A point that does not make a left turn from the last two kept
  // is inside (or on an edge), so the last kept point is dropped until it
  // does.
  //
  // Sorting is the slow part, so most points are thrown out first. The
  // points farthest out in eight directions (along x, y and the diagonals)
  // are hull corners, in anticlockwise order. A point strictly inside the
  // polygon they make cannot be a hull corner. For a cloud filling a
  // rectangle, that is nearly all of them.
  std::vector<int> finite;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    if (Finite(points[i])) finite.push_back(i);
  }
  Points2d sorted;
  if (finite.size() > 64) {
    const double directions[8][2] = {{1, 0},  {1, 1},   {0, 1},  {-1, 1},
                                     {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
    std::array<int, 8> extreme{};
    std::array<double, 8> farthest;
    farthest.fill(-kInfinity);
    for (int i : finite) {
      const double x = points[i].x();
      const double y = points[i].y();
      for (int d = 0; d < 8; ++d) {
        const double reach = directions[d][0] * x + directions[d][1] * y;
        if (reach > farthest[d]) {
          farthest[d] = reach;
          extreme[d] = i;
        }
      }
    }
    std::vector<int> fence;
    for (int d = 0; d < 8; ++d) {
      const Vector2d& p = points[extreme[d]];
      if (fence.empty() ||
          (p != points[fence.back()] && p != points[fence.front()])) {
        fence.push_back(extreme[d]);
      }
    }
    // Each fence edge as a x + b y > c for points strictly inside.
    const int m = static_cast<int>(fence.size());
    std::vector<std::array<double, 3>> sides;
    for (int e = 0; e < m; ++e) {
      const Vector2d& p = points[fence[e]];
      const Vector2d& q = points[fence[(e + 1) % m]];
      sides.push_back({p.y() - q.y(), q.x() - p.x(),
                       (p.y() - q.y()) * p.x() + (q.x() - p.x()) * p.y()});
    }
    for (int i : finite) {
      const double x = points[i].x();
      const double y = points[i].y();
      bool inside = m >= 3;
      for (int e = 0; inside && e < m; ++e) {
        inside = sides[e][0] * x + sides[e][1] * y > sides[e][2];
      }
      if (!inside) sorted.push_back(points[i]);
    }
  } else {
    for (int i : finite) sorted.push_back(points[i]);
  }
  auto before = [](const Vector2d& a, const Vector2d& b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  };
  std::sort(sorted.begin(), sorted.end(), before);
  sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
  const int n = static_cast<int>(sorted.size());
  if (n < 3) return sorted;

  Points2d hull(2 * n);
  int k = 0;
  for (int i = 0; i < n; ++i) {  // Lower half.
    while (k >= 2 && Cross(hull[k - 2], hull[k - 1], sorted[i]) <= 0.0) --k;
    hull[k++] = sorted[i];
  }
  for (int i = n - 2, lower = k + 1; i >= 0; --i) {  // Upper half.
    while (k >= lower && Cross(hull[k - 2], hull[k - 1], sorted[i]) <= 0.0) {
      --k;
    }
    hull[k++] = sorted[i];
  }
  // The last point is the first again.
  hull.resize(k - 1);
  return hull;
}

// -------------------------------------------------- minimum-area rectangle

std::optional<OrientedRectangle> MinAreaRectangle(const Points2d& points) {
  const Points2d hull = ConvexHull(points);
  const int n = static_cast<int>(hull.size());
  if (n < 3) return std::nullopt;
  auto next = [n](int k) { return (k + 1) % n; };
  // How many steps anticlockwise k is past i.
  auto ahead = [n](int k, int i) { return (k - i + n) % n; };

  // For edge i, with `edge` along it and `up` at right angles into the hull:
  // `right` is the corner farthest forward along the edge, `top` the one
  // farthest from it, `left` the one farthest back. Going anticlockwise from
  // the edge they come in that order. As the edge moves on, each moves on
  // too, so it is stepped forward from where it was, and only while that
  // still gains. A marker is first brought up to the one before it in that
  // order, from which the stepping only ever gains until it should stop.
  // The edge vectors are not normalised for the stepping: only the signs
  // of the dot products matter there.
  double best_area = kInfinity;
  OrientedRectangle best;
  int right = 1;
  int top = 1;
  int left = 1;
  for (int i = 0; i < n; ++i) {
    const Vector2d edge = hull[next(i)] - hull[i];
    const Vector2d up = Perp(edge);
    if (ahead(right, i) < 1) right = next(i);
    for (int step = 0;
         step < n && edge.dot(hull[next(right)] - hull[right]) > 0.0; ++step) {
      right = next(right);
    }
    if (ahead(top, i) < ahead(right, i)) top = right;
    for (int step = 0; step < n && up.dot(hull[next(top)] - hull[top]) > 0.0;
         ++step) {
      top = next(top);
    }
    if (ahead(left, i) < ahead(top, i)) left = top;
    for (int step = 0;
         step < n && edge.dot(hull[next(left)] - hull[left]) < 0.0; ++step) {
      left = next(left);
    }

    const double norm = edge.norm();
    const Vector2d u = edge / norm;
    const double forward = u.dot(hull[right] - hull[i]);
    const double back = u.dot(hull[left] - hull[i]);
    const double height = Perp(u).dot(hull[top] - hull[i]);
    const double area = (forward - back) * height;
    if (area < best_area) {
      best_area = area;
      best.axis = u;
      best.length = forward - back;
      best.width = height;
      best.centre =
          hull[i] + 0.5 * (forward + back) * u + 0.5 * height * Perp(u);
    }
  }
  if (!(best_area > 0.0) || !std::isfinite(best_area)) return std::nullopt;
  return Canonical(best);
}

// ----------------------------------------------------------------- failure

const char* FailureName(RectangleFitFailure failure) {
  switch (failure) {
    case RectangleFitFailure::kNone: return "none";
    case RectangleFitFailure::kBadOptions: return "bad options";
    case RectangleFitFailure::kNonFinite: return "non-finite input";
    case RectangleFitFailure::kTooFewPoints: return "too few points";
    case RectangleFitFailure::kDegenerate: return "degenerate";
    case RectangleFitFailure::kSideMissing: return "side missing";
    case RectangleFitFailure::kNotSquare: return "not square";
    case RectangleFitFailure::kSizeMismatch: return "size mismatch";
  }
  return "unknown";
}

// ------------------------------------------------------------- refinement

std::optional<RectangleFit> RefineRectangle(
    const Points2d& points, const OrientedRectangle& first_guess,
    const RefineOptions& options, RectangleRefusal* refusal) {
  using Failure = RectangleFitFailure;
  const RefineOptions& o = options;
  if (o.iterations < 1 || !(o.end_margin >= 0.0 && o.end_margin < 0.45) ||
      o.min_side_dots < 2 ||
      !(o.min_side_support >= 0.0 && o.min_side_support <= 1.0) ||
      !(o.max_angle_error > 0.0 && o.max_angle_error < std::numbers::pi / 4) ||
      !(std::isfinite(o.spacing) && o.spacing >= 0.0) ||
      !(std::isfinite(o.known_length) && o.known_length >= 0.0) ||
      !(std::isfinite(o.known_width) && o.known_width >= 0.0)) {
    return Refuse(refusal, Failure::kBadOptions, "refine option out of range");
  }
  if (!first_guess.centre.allFinite() || !first_guess.axis.allFinite() ||
      !(first_guess.axis.norm() > 0.0) ||
      !(std::isfinite(first_guess.length) && first_guess.length > 0.0) ||
      !(std::isfinite(first_guess.width) && first_guess.width > 0.0)) {
    return Refuse(refusal, Failure::kBadOptions, "first guess has no area");
  }
  if (!AllFinite(points)) {
    return Refuse(refusal, Failure::kNonFinite,
                  "a point has a coordinate that is not finite");
  }
  if (points.size() < 3) {
    return Refuse(refusal, Failure::kTooFewPoints,
                  Format("%zu points", points.size()));
  }
  double spacing = o.spacing;
  if (spacing == 0.0) {
    const std::optional<double> measured = DotSpacing(points);
    if (!measured) {
      return Refuse(refusal, Failure::kDegenerate,
                    "no dot spacing: the points are all the same");
    }
    spacing = *measured;
  }
  const double strip_width = kStripSpacings * spacing;
  const double tolerance = 2.5 * spacing;
  // A side's edge dots scatter about its line by at least this much, for
  // weighing the sides against each other: a straighter run of dots than
  // this is luck, not a better side.
  const double least_scatter = 0.1 * spacing;

  std::vector<double> xs(points.size());
  std::vector<double> ys(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    xs[i] = points[i].x();
    ys[i] = points[i].y();
  }
  Projection frame;

  std::mt19937 rng(o.seed);
  OrientedRectangle rect = Canonical(first_guess);

  // What each side's last round found.
  struct SideRound {
    Strips strips;
    std::vector<EdgeDot> dots;
    std::optional<SideLine> fit;   // through the edge dots, robustly
    double mean_s = 0.0;   // of the edge dots on the line
    double spread = 0.0;   // their sum of squared s about mean_s
    double rms = 0.0;      // their distance from the line
    bool usable = false;
    bool unsquare = false;
    double angle_error = 0.0;
    double edge_correction = 0.0;
    bool from_known_size = false;
    Vector2d pivot = Vector2d::Zero();  // a point on its own line
  };
  std::array<SideRound, 4> sides;
  double rms_all = 0.0;

  for (int round = 0; round < o.iterations; ++round) {
    // Edge dots and a line through them, for each side.
    Project(xs, ys, rect, &frame);
    for (int k = 0; k < 4; ++k) {
      SideRound& side = sides[k];
      side = SideRound();
      const SideFrame f = FrameOf(rect, k);
      side.strips = StripsOf(f, o.end_margin, strip_width);
      side.dots = FindEdgeDots(frame, k, f.half_depth, side.strips);
      const int needed = std::max(
          o.min_side_dots,
          static_cast<int>(std::ceil(o.min_side_support * side.strips.count)));
      side.fit = FitSideLine(side.dots, tolerance, needed, rng);
      if (!side.fit || static_cast<int>(side.fit->kept.size()) < needed) {
        continue;
      }
      side.rms = RmsAbout(side.dots, side.fit->kept, side.fit->line);
      double mean_s = 0.0;
      for (int j : side.fit->kept) mean_s += side.dots[j].s;
      mean_s /= static_cast<double>(side.fit->kept.size());
      double spread = 0.0;
      for (int j : side.fit->kept) {
        spread += (side.dots[j].s - mean_s) * (side.dots[j].s - mean_s);
      }
      side.mean_s = mean_s;
      side.spread = spread;
      side.usable = spread > 0.0;
    }

    // One direction for all four. In every side's own frame, turning the
    // rectangle anticlockwise by a small angle a tilts the side's line by
    // slope -a, so a single slope fitted to all four sides' edge dots at
    // once (each side keeping its own offset) is the rectangle's direction.
    // Each side's dots are weighted by how tightly they follow its line, so
    // a side whose edge dots scatter does not drag three straight ones.
    // That matters most where the grid's rows run along the sides: there a
    // side's edge dots lie on one row for some of its length and on the
    // next row in for the rest, a step that tilts its line, while the
    // others lie on single rows and are true. A side that disagrees by more
    // than its own scatter allows is set aside, worst first, and the rest
    // refitted.
    double shared = 0.0;
    for (;;) {
      double sxy = 0.0;
      double sxx = 0.0;
      for (const SideRound& side : sides) {
        if (!side.usable) continue;
        const double scatter = std::max(side.rms, least_scatter);
        const double weight = 1.0 / (scatter * scatter);
        double mean_t = 0.0;
        for (int j : side.fit->kept) mean_t += side.dots[j].t;
        mean_t /= static_cast<double>(side.fit->kept.size());
        for (int j : side.fit->kept) {
          sxy += weight * (side.dots[j].s - side.mean_s) *
                 (side.dots[j].t - mean_t);
        }
        sxx += weight * side.spread;
      }
      if (!(sxx > 0.0)) {
        return Refuse(refusal, Failure::kSideMissing,
                      "no side has edge dots on a straight line");
      }
      shared = sxy / sxx;
      int worst = -1;
      double worst_excess = 0.0;
      for (int k = 0; k < 4; ++k) {
        SideRound& side = sides[k];
        if (!side.usable) continue;
        side.angle_error = std::atan(shared) - std::atan(side.fit->line.slope);
        const double standard_error =
            std::max(side.rms, least_scatter) / std::sqrt(side.spread);
        const double excess = std::abs(side.angle_error) -
                              (o.max_angle_error + 4.0 * standard_error);
        if (excess > worst_excess) {
          worst_excess = excess;
          worst = k;
        }
      }
      if (worst < 0) break;
      sides[worst].usable = false;
      sides[worst].unsquare = true;
    }

    // At most one side may be missing, and only if the known size can
    // stand in for it.
    int missing = -1;
    for (int k = 0; k < 4; ++k) {
      if (sides[k].usable) continue;
      const double known = k % 2 == 0 ? o.known_width : o.known_length;
      const SideRound& side = sides[k];
      std::string why;
      if (side.unsquare) {
        why = Format("%s is %.2f degrees out of square", kSideNames[k],
                     std::abs(side.angle_error) / kDegree);
      } else {
        why = Format("%s has %d of %d strips on a straight line",
                     kSideNames[k],
                     side.fit ? static_cast<int>(side.fit->kept.size()) : 0,
                     side.strips.count);
      }
      if (missing >= 0 || !(known > 0.0)) {
        return Refuse(refusal,
                      side.unsquare ? Failure::kNotSquare
                                    : Failure::kSideMissing,
                      why);
      }
      missing = k;
    }

    // Turn the rectangle to the shared direction.
    rect.axis = Rotate(rect.axis, -std::atan(shared)).normalized();
    Project(xs, ys, rect, &frame);

    // Move each side out to its edge.
    std::array<double, 4> offset{};
    for (int k = 0; k < 4; ++k) {
      SideRound& side = sides[k];
      if (!side.usable) continue;
      const SideFrame f = FrameOf(rect, k);
      // The line through the edge dots, now along the shared direction.
      double outermost = 0.0;
      for (int j : side.fit->kept) {
        const int i = side.dots[j].point;
        outermost += PlaceOn(k, frame.u[i], frame.v[i]).depth - f.half_depth;
      }
      outermost /= static_cast<double>(side.fit->kept.size());
      offset[k] = outermost;
      if (o.correct_edge_bias) {
        // Strips whose edge dot is on the line, and whose neighbours' are
        // too, so that strips next to a hidden stretch are left out.
        std::vector<bool> on_line(side.strips.count, false);
        for (int j : side.fit->kept) on_line[side.dots[j].strip] = true;
        std::vector<bool> use(side.strips.count, false);
        bool any = false;
        for (int s = 0; s < side.strips.count; ++s) {
          use[s] = on_line[s] && (s == 0 || on_line[s - 1]) &&
                   (s + 1 == side.strips.count || on_line[s + 1]);
          any = any || use[s];
        }
        if (!any) use = on_line;
        const std::optional<double> counted = CountedEdge(
            frame, k, f.half_depth, side.strips, use, outermost, spacing);
        if (counted) offset[k] = *counted;
      }
      side.edge_correction = offset[k] - outermost;
    }
    if (missing >= 0) {
      const int opposite = (missing + 2) % 4;
      const bool long_side = missing % 2 == 0;
      const double known = long_side ? o.known_width : o.known_length;
      const double current = long_side ? rect.width : rect.length;
      offset[missing] = known - current - offset[opposite];
      sides[missing].from_known_size = true;
    }

    // A point on each side's own line, for corners where the lines meet.
    for (int k = 0; k < 4; ++k) {
      const SideFrame f = FrameOf(rect, k);
      SideRound& side = sides[k];
      side.pivot = rect.centre + side.mean_s * f.along +
                   (f.half_depth + offset[k]) * f.outward;
    }

    const Vector2d a = rect.axis;
    const Vector2d b = rect.Across();
    rect.centre += 0.5 * (offset[1] - offset[3]) * a +
                   0.5 * (offset[2] - offset[0]) * b;
    rect.length += offset[1] + offset[3];
    rect.width += offset[0] + offset[2];
    if (!(rect.length > 0.0 && rect.width > 0.0) || !rect.centre.allFinite() ||
        !std::isfinite(rect.length) || !std::isfinite(rect.width)) {
      return Refuse(refusal, Failure::kDegenerate,
                    "the sides crossed: the dots do not fill a rectangle");
    }

    double squares = 0.0;
    int used = 0;
    for (const SideRound& side : sides) {
      if (!side.usable) continue;
      squares += side.rms * side.rms * side.fit->kept.size();
      used += static_cast<int>(side.fit->kept.size());
    }
    rms_all = used > 0 ? std::sqrt(squares / used) : 0.0;
  }

  // The corners, in the frame of the last round.
  std::array<Vector2d, 4> corners = rect.Corners();
  if (!o.square_corners) {
    std::array<Vector2d, 4> direction;
    for (int k = 0; k < 4; ++k) {
      // A side placed from the known size has no line of its own.
      const double turn = sides[k].usable ? sides[k].angle_error : 0.0;
      direction[k] = Rotate(FrameOf(rect, k).along, turn);
    }
    for (int k = 0; k < 4; ++k) {
      const int before = (k + 3) % 4;
      corners[k] = Intersect(sides[before].pivot, direction[before],
                             sides[k].pivot, direction[k]);
    }
    for (const Vector2d& corner : corners) {
      if (!corner.allFinite()) {
        return Refuse(refusal, Failure::kDegenerate,
                      "two adjacent sides are parallel");
      }
    }
    rect.centre = 0.25 * (corners[0] + corners[1] + corners[2] + corners[3]);
    const Vector2d along = (corners[1] - corners[0]) + (corners[2] - corners[3]);
    rect.axis = along.normalized();
    rect.length = 0.5 * ((corners[1] - corners[0]).norm() +
                         (corners[2] - corners[3]).norm());
    rect.width = 0.5 * ((corners[2] - corners[1]).norm() +
                        (corners[3] - corners[0]).norm());
  }

  // Report in the canonical direction. Turning the rectangle by q quarter
  // turns renumbers its sides and corners by q.
  const OrientedRectangle canonical = Canonical(rect);
  int q = 0;
  if (canonical.axis.dot(rect.axis) < -0.5) {
    q = 2;
  } else if (canonical.axis.dot(rect.Across()) > 0.5) {
    q = 1;
  } else if (canonical.axis.dot(rect.Across()) < -0.5) {
    q = 3;
  }

  RectangleFit fit;
  fit.rectangle = canonical;
  fit.first_guess = Canonical(first_guess);
  fit.spacing = spacing;
  fit.dots_given = static_cast<int>(points.size());
  fit.dots_kept = static_cast<int>(points.size());
  fit.rms_residual = rms_all;
  for (int k = 0; k < 4; ++k) {
    fit.corners[k] = corners[(k + q) % 4];
  }
  for (int k = 0; k < 4; ++k) {
    const SideRound& side = sides[(k + q) % 4];
    SideFit& out = fit.sides[k];
    out.start = fit.corners[k];
    out.end = fit.corners[(k + 1) % 4];
    out.outward = -Perp(out.end - out.start).normalized();
    out.strips = side.strips.count;
    out.edge_dots = static_cast<int>(side.dots.size());
    out.dots_used =
        side.usable ? static_cast<int>(side.fit->kept.size()) : 0;
    out.support = out.strips > 0
                      ? static_cast<double>(out.dots_used) / out.strips
                      : 0.0;
    out.rms_residual = side.usable ? side.rms : 0.0;
    out.angle_error = side.angle_error;
    out.edge_correction = side.usable ? side.edge_correction : 0.0;
    out.from_known_size = side.from_known_size;
    fit.used_known_size = fit.used_known_size || side.from_known_size;
  }
  return fit;
}

// ------------------------------------------------------------ the whole fit

std::optional<RectangleFit> FitRectangle(const Points2d& points,
                                         double expected_length,
                                         double expected_width,
                                         const RectangleFitOptions& options,
                                         RectangleRefusal* refusal) {
  using Failure = RectangleFitFailure;
  if (!(std::isfinite(expected_length) && expected_length > 0.0 &&
        std::isfinite(expected_width) && expected_width > 0.0)) {
    return Refuse(refusal, Failure::kBadOptions, "expected size not positive");
  }
  if (expected_width > expected_length) {
    std::swap(expected_length, expected_width);
  }
  if (!(std::isfinite(options.size_tolerance) &&
        options.size_tolerance > 0.0) ||
      !(std::isfinite(options.refine.spacing) &&
        options.refine.spacing >= 0.0) ||
      options.min_points < 3 ||
      !(std::isfinite(options.cluster_cell_size) &&
        options.cluster_cell_size >= 0.0)) {
    return Refuse(refusal, Failure::kBadOptions, "fit option out of range");
  }
  if (!AllFinite(points)) {
    return Refuse(refusal, Failure::kNonFinite,
                  "a point has a coordinate that is not finite");
  }
  const int given = static_cast<int>(points.size());
  if (given < options.min_points) {
    return Refuse(refusal, Failure::kTooFewPoints,
                  Format("%d points, need %d", given, options.min_points));
  }

  // Drop strays.
  // The spacing, from all the dots: a few strays do not move a median.
  RefineOptions refine = options.refine;
  if (refine.spacing == 0.0) {
    const std::optional<double> spacing = DotSpacing(points);
    if (!spacing) {
      return Refuse(refusal, Failure::kDegenerate,
                    "no dot spacing: the points are all the same");
    }
    refine.spacing = *spacing;
  }
  const double cell_size = options.cluster_cell_size > 0.0
                               ? options.cluster_cell_size
                               : 3.0 * refine.spacing;
  Points2d clump;
  for (int i : LargestCluster(points, cell_size)) clump.push_back(points[i]);
  const int kept = static_cast<int>(clump.size());
  if (kept < options.min_points) {
    return Refuse(refusal, Failure::kTooFewPoints,
                  Format("%d points in the largest clump, need %d", kept,
                         options.min_points));
  }

  // First guess.
  const std::optional<OrientedRectangle> guess = MinAreaRectangle(clump);
  if (!guess || guess->width < 2.0 * refine.spacing) {
    return Refuse(refusal, Failure::kDegenerate,
                  "the dots have no area: they lie along a line");
  }

  if (options.allow_missing_side) {
    refine.known_length = expected_length;
    refine.known_width = expected_width;
  } else {
    refine.known_length = 0.0;
    refine.known_width = 0.0;
  }
  std::optional<RectangleFit> fit =
      RefineRectangle(clump, *guess, refine, refusal);
  if (!fit) return std::nullopt;
  fit->first_guess = *guess;
  fit->dots_given = given;
  fit->dots_kept = kept;

  // The size check. A dimension set from the known size cannot be checked
  // by its value; instead, no more than a few dots may lie beyond the side
  // that was placed.
  const OrientedRectangle& r = fit->rectangle;
  const double tolerance = options.size_tolerance;
  bool length_known = false;
  bool width_known = false;
  for (int k = 0; k < 4; ++k) {
    const SideFit& side = fit->sides[k];
    if (!side.from_known_size) continue;
    (k % 2 == 0 ? width_known : length_known) = true;
    int beyond = 0;
    for (const Vector2d& p : clump) {
      if ((p - side.start).dot(side.outward) > tolerance) ++beyond;
    }
    if (beyond > std::max(20, kept / 1000)) {
      return Refuse(refusal, Failure::kSizeMismatch,
                    Format("%d dots lie beyond %s, placed from the known size",
                           beyond, kSideNames[k]));
    }
  }
  if (!length_known && std::abs(r.length - expected_length) > tolerance) {
    return Refuse(refusal, Failure::kSizeMismatch,
                  Format("length %.4f m, expected %.4f +- %.4f m", r.length,
                         expected_length, tolerance));
  }
  if (!width_known && std::abs(r.width - expected_width) > tolerance) {
    return Refuse(refusal, Failure::kSizeMismatch,
                  Format("width %.4f m, expected %.4f +- %.4f m", r.width,
                         expected_width, tolerance));
  }
  if (refusal != nullptr) *refusal = RectangleRefusal();
  return fit;
}

}  // namespace cell
