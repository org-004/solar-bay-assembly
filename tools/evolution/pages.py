#!/usr/bin/env python3
"""Pages, captions and registry entries from versions/versions.json and scores/*.json.

Usage:
  pages.py write [--root DIR]     write docs/directions/<id>.md, the cycle-time
                                  charts docs/directions/<id>-cycle.svg and the
                                  README's results block
  pages.py check [--root DIR]     (also --check) compare with the files; print the
                                  ones that differ, exit 1; else "pages: up to date"
  pages.py caption ID [--root DIR]
                                  the lines burned into the recording of version ID
  pages.py add --layout NAME --title T --changed C --why W --status chosen|candidate
        [--commit HASH|this] [--competed-with vNN] [--direction ID]
        --recording PATH [--recording-source PATH --recorded-at HASH --shows TEXT]
                                  append a version for layouts/NAME.json, its plan
                                  and its scorecard to versions/versions.json

The README's results go between the lines
  <!-- BEGIN results: written by tools/evolution/pages.py; do not edit -->
  <!-- END results -->
Output is deterministic: no times, no machine names, "\\n" line ends, UTF-8.
Python 3 standard library only. Exit 2 on a usage or data error.
"""
import argparse
import datetime
import hashlib
import json
import os
import subprocess
import sys
from xml.sax.saxutils import escape as xml_escape

BEGIN = "<!-- BEGIN results: written by tools/evolution/pages.py; do not edit -->"
END = "<!-- END results -->"
REPO_URL = "https://github.com/org-004/solar-bay-assembly/commit/"
COUNTS = {"arms", "part_count"}
THREE_SIG = {"failures_per_bay", "physics_share"}


class Error(Exception):
    pass


# ---------------------------------------------------------------- loading

def repo_root():
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def read_text(path):
    with open(path, encoding="utf-8", newline="") as f:
        return f.read()


def load_json(path):
    try:
        return json.loads(read_text(path))
    except (OSError, ValueError) as e:
        raise Error("cannot read %s: %s" % (path, e))


def sha256_of(path):
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError as e:
        raise Error("cannot read %s: %s" % (path, e))


class Registry:
    def __init__(self, root):
        self.root = root
        self.data = load_json(os.path.join(root, "versions", "versions.json"))
        self.directions = self.data.get("directions", [])
        self.versions = sorted(self.data.get("versions", []), key=lambda v: v["id"])
        self._cards = {}
        self._hashes = {}
        self._git_done = False
        for v in self.versions:
            if v.get("scores"):
                self._cards[v["id"]] = load_json(os.path.join(root, v["scores"]))

    def card(self, v):
        return self._cards.get(v["id"])

    def of_direction(self, d):
        return [v for v in self.versions if v["direction"] == d["id"]]

    def commit(self, v):
        """The commit's short hash, or the resolution text for 'this'."""
        if v["commit"] != "this":
            return v["commit"]
        if v["id"] not in self._hashes:
            self._hashes[v["id"]] = self._resolve(v["id"])
        return self._hashes[v["id"]]

    def _resolve(self, vid):
        # A page cannot hold the hash of the commit that adds it, and a page
        # that looked the hash up would differ before and after landing and
        # between a full and a shallow clone. So the page names the commit by
        # the git log command that finds it, the same text everywhere.
        return ("the commit that added %s: `git log --format=%%h -S '\"id\": \"%s\"' "
                "-- versions/versions.json | tail -1`" % (vid, vid))

    def commit_link(self, v):
        c = self.commit(v)
        return c if v["commit"] == "this" else "[%s](%s%s)" % (c, REPO_URL, c)


# ---------------------------------------------------------------- figures

def norm_unit(name, unit):
    if name == "cycle_s" and unit == "s":
        return "s per bay"
    if name == "arms":
        return "arms"
    if unit == "1":
        return ""
    return unit


def items(reg, v):
    """name -> {value, unit, known, rank, better, basis, source} for a version."""
    out = {}
    for f in v.get("figures", []):
        out[f["name"]] = {"value": f.get("value"), "unit": norm_unit(f["name"], f.get("unit", "")),
                          "known": f.get("known", "none"), "source": f.get("source", "")}
    card = reg.card(v)
    if card:
        for m in card.get("measures", []):
            out[m["name"]] = {"value": m.get("value"), "unit": norm_unit(m["name"], m.get("unit", "")),
                              "known": m.get("known", "none"), "rank": m.get("rank", ""),
                              "better": m.get("better", ""), "basis": m.get("basis", "")}
    return out


def fmt(name, value):
    if value is None:
        return "none"
    if isinstance(value, str):
        return value
    if name.endswith("_s"):
        return "%.2f" % value
    if name.endswith("_kg"):
        return "%.1f" % value
    if name in COUNTS:
        return str(int(round(value)))
    if name in THREE_SIG:
        return "%.3g" % value
    if float(value).is_integer():
        return str(int(value))
    return "%.6g" % value


def with_unit(name, it):
    if it is None or it["value"] is None:
        return "none"
    s = fmt(name, it["value"])
    unit = it["unit"]
    if unit == "arms" and it["value"] == 1:
        unit = "arm"
    return s + (" " + unit if unit else "")


def cell(reg, v, name):
    return with_unit(name, items(reg, v).get(name))


def plain(reg, v, name):
    it = items(reg, v).get(name)
    return "none" if it is None else fmt(name, it["value"])


def known_cell(reg, v, name):
    """The value with its unit and, when it is known, how it is known."""
    it = items(reg, v).get(name)
    text = cell(reg, v, name)
    if it is None or it.get("value") is None:
        return text
    return "%s (%s)" % (text, it["known"])


def cycle_known(reg, v):
    it = items(reg, v).get("cycle_s")
    return "none" if it is None or it["value"] is None else it["known"]


def esc(s):
    return str(s).replace("|", "\\|").replace("\n", " ")


def table(header, rows):
    lines = ["| " + " | ".join(header) + " |", "|" + "|".join(["---"] * len(header)) + "|"]
    for r in rows:
        lines.append("| " + " | ".join(esc(c) for c in r) + " |")
    return lines


def gate_status(card):
    gates = card.get("gates", [])
    bad = [g["name"] for g in gates if not g.get("pass")]
    if not gates:
        return "none"
    return "all %d pass" % len(gates) if not bad else "FAIL: " + ", ".join(bad)


# ---------------------------------------------------------------- chosen/best

def best_version(reg, d):
    vs = reg.of_direction(d)
    chosen = [v for v in vs if v["status"] == "chosen"]
    if chosen:
        return chosen[-1], True
    rest = [v for v in vs if v["status"] != "candidate"]
    return (rest[-1], False) if rest else (None, False)


def recording_name(v):
    return os.path.basename(v["recording"])


def caption_lines(reg, v):
    lines = ["%s  %s  %s" % (v["id"], v["date"], v["title"])]
    its = items(reg, v)
    parts = []
    cyc = its.get("cycle_s")
    if cyc is None or cyc["value"] is None:
        parts.append("cycle time: none at this version")
    else:
        parts.append("cycle %s (%s)" % (with_unit("cycle_s", cyc), cyc["known"]))
    arms = its.get("arms")
    if arms is not None and arms["value"] is not None:
        n = int(round(arms["value"]))
        parts.append("%d %s" % (n, "arm" if n == 1 else "arms"))
    lines.append(", ".join(parts))
    src = v.get("recording_source")
    if src and src.get("added_at") and src["added_at"] not in (v["commit"], reg.commit(v) if v["commit"] == "this" else v["commit"]):
        lines.append("recorded at %s" % src["added_at"])
    return lines


def alt_text(reg, v):
    t = "; ".join(caption_lines(reg, v)[:2])
    return t.replace("[", "(").replace("]", ")")


# ---------------------------------------------------------------- tables

def measure_table(card):
    rows = []
    for m in card.get("measures", []):
        rows.append([m.get("rank", ""), m["name"], fmt(m["name"], m.get("value")),
                     norm_unit(m["name"], m.get("unit", "")) or "-", m.get("better", ""),
                     m.get("known", ""), m.get("basis", "")])
    return table(["Rank", "Measure", "Value", "Unit", "Better", "How known", "Basis"], rows)


def gate_table(card):
    rows = []
    for g in card.get("gates", []):
        rows.append([g["name"], "%s %s" % (fmt(g["name"], g.get("figure")), g.get("unit", "")),
                     "%s %s %s" % (g.get("holds", ""), fmt(g["name"], g.get("limit")), g.get("unit", "")),
                     "pass" if g.get("pass") else "FAIL"])
    return table(["Gate", "Figure", "Limit", "Result"], rows)


def figure_table(v):
    rows = []
    for f in v.get("figures", []):
        rows.append([f["name"], fmt(f["name"], f.get("value")), f.get("unit", ""),
                     f.get("known", ""), f.get("source", "")])
    return table(["Figure", "Value", "Unit", "How known", "Source"], rows)


def state_blocks(reg, v, heading_note=""):
    out = []
    card = reg.card(v)
    if card:
        out += measure_table(card) + [""] + gate_table(card) + [""]
        out += ["The measures and gates are defined in [the scorecard](../scorecard.md).", ""]
    if v.get("figures"):
        out += figure_table(v) + [""]
    if not card and not v.get("figures"):
        out += ["This version records no figures.", ""]
    return out


def compare_table(reg, prev, v):
    a, b = items(reg, prev), items(reg, v)
    rows = []
    for name in sorted(set(a) & set(b)):
        x, y = a[name], b[name]
        if x["value"] is None or y["value"] is None:
            diff = "not comparable: not known"
        elif x["unit"] != y["unit"]:
            diff = "not comparable: %s vs %s" % (x["unit"] or "no unit", y["unit"] or "no unit")
        else:
            d = round(y["value"] - x["value"], 9)
            diff = ("+" if d > 0 else "") + fmt(name, d) + (" " + y["unit"] if y["unit"] else "")
        if x["known"] != y["known"]:
            diff += " (%s \u2192 %s)" % (x["known"], y["known"])
        rows.append([name, with_unit(name, x), with_unit(name, y), diff])
    if not rows:
        return ["No measure or figure is shared by the two versions.", ""]
    return table(["Measure", "Before (%s)" % prev["id"], "Now (%s)" % v["id"], "Difference"], rows) + [""]


def link_files(v):
    parts = []
    if v.get("layout"):
        parts.append("[layout](../../%s)" % v["layout"]["path"])
    if v.get("plan"):
        parts.append("[plan](../../%s)" % v["plan"]["path"])
    if v.get("scores"):
        parts.append("[scorecard](../../%s)" % v["scores"])
    return "**Files.** " + ", ".join(parts) if parts else ""


def competition_table(reg, v, cands):
    rows = []
    for w in [v] + cands:
        rows.append([w["id"] + (" (this version)" if w is v else ""),
                     cell(reg, w, "cycle_s"), plain(reg, w, "arms"), plain(reg, w, "part_count"),
                     cell(reg, w, "mass_in_transit_kg"), plain(reg, w, "failures_per_bay"),
                     cycle_known(reg, w), "[%s](../evolution/%s)" % (recording_name(w), recording_name(w)),
                     "[%s](../../%s)" % (os.path.basename(w["layout"]["path"]), w["layout"]["path"])
                     if w.get("layout") else "none",
                     reg.commit_link(w), w["status"]])
    return table(["Version", "Cycle time", "Arms", "Part count", "Mass in transit",
                  "Failures per bay", "Cycle time is", "Recording", "Layout", "Commit", "Status"], rows)


# ---------------------------------------------------------------- direction page

def chart_unit(reg, vs):
    for v in reversed(vs):
        it = items(reg, v).get("cycle_s")
        if it and it["value"] is not None:
            return it["unit"]
    return None


def direction_page(reg, d):
    vs = reg.of_direction(d)
    L = ["# " + d["title"], "", d["description"], "", "**Limits.** " + d["limits"], ""]
    if not vs:
        L += ["No version yet: nothing of this direction has been built or planned.", "",
              "[Back to the README](../../README.md)", ""]
        return "\n".join(L)
    best, chosen = best_version(reg, d)
    L += ["## Where it stands", ""]
    if best is None:
        L += ["Only candidates exist: no version of this direction is chosen or settled.", ""]
    else:
        if chosen:
            L += ["The chosen version is %s, \"%s\"." % (best["id"], best["title"]), ""]
        else:
            L += ["No version is chosen; the newest version that is not a candidate is %s, \"%s\"."
                  % (best["id"], best["title"]), ""]
        L += state_blocks(reg, best)
    L += ["## Progress", ""]
    rows = []
    for v in vs:
        rows.append([v["id"], v["date"], v["title"] + (" (candidate)" if v["status"] == "candidate" else ""),
                     cell(reg, v, "cycle_s"), plain(reg, v, "arms"), plain(reg, v, "part_count"),
                     cell(reg, v, "mass_in_transit_kg"), cycle_known(reg, v)])
    L += table(["Version", "Date", "Title", "Cycle time", "Arms", "Part count", "Mass in transit",
                "Cycle time is"], rows) + [""]
    unit = chart_unit(reg, vs)
    if unit is None:
        L += ["No version of this direction has a cycle time yet, so there is no chart.", ""]
    else:
        L += ["![Cycle time of each version](%s-cycle.svg)" % d["id"], "",
              "A filled circle is a cycle time that is planned, a square one that is measured, a hollow "
              "circle one that is modelled; a lighter marker is a candidate that was not chosen."]
        left = [v["id"] for v in vs if (items(reg, v).get("cycle_s") or {}).get("value") is not None
                and items(reg, v)["cycle_s"]["unit"] != unit]
        if left:
            L[-1] += (" The chart shows %s only; %s %s left out because the cycle time there is in a "
                      "different unit." % (unit, ", ".join(left), "is" if len(left) == 1 else "are"))
        L += [""]
    settled = [v for v in vs if v["status"] != "candidate"]
    for i in range(len(settled) - 1, -1, -1):
        v = settled[i]
        L += ["## %s %s" % (v["id"], v["title"]), "",
              "![%s](../evolution/%s)" % (alt_text(reg, v), recording_name(v)), "",
              "%s, %s, commit %s" % (v["date"], v["status"], reg.commit_link(v)), "",
              "**What changed.** " + v["changed"], "", "**Why.** " + v["why"], ""]
        L += state_blocks(reg, v)
        if i > 0:
            L += ["**Against %s.**" % settled[i - 1]["id"], "",
                  "Each figure as the two versions' own sources report it; the conditions of a run "
                  "may differ between them (see each figure's source).", ""] + compare_table(reg, settled[i - 1], v)
        else:
            L += ["First version of this direction.", ""]
        cands = [w for w in vs if w.get("competed_with") == v["id"]]
        if cands:
            L += ["**Candidates it was chosen over.**", ""] + competition_table(reg, v, cands) + [""]
        files = link_files(v)
        if files:
            L += [files, ""]
    L += ["[Back to the README](../../README.md)", ""]
    return "\n".join(L)


# ---------------------------------------------------------------- chart

def nice_max(x):
    import math
    if x <= 0:
        return 1.0
    mag = 10 ** math.floor(math.log10(x))
    for m in (1, 2, 2.5, 5, 10):
        if x <= m * mag:
            return m * mag
    return 10 * mag


def chart_svg(reg, d):
    vs = reg.of_direction(d)
    unit = chart_unit(reg, vs)
    if unit is None:
        return None
    W, H, ml, mr, mt, mb = 640, 300, 72, 24, 44, 56
    pts = []
    for i, v in enumerate(vs):
        it = items(reg, v).get("cycle_s")
        if it and it["value"] is not None and it["unit"] == unit:
            pts.append((i, v, it))
    top = nice_max(max(p[2]["value"] for p in pts) * 1.05)
    pw, ph = W - ml - mr, H - mt - mb
    n = len(vs)

    def X(i):
        return ml + pw * (i + 0.5) / n

    def Y(val):
        return mt + ph * (1 - val / top)

    S = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
         'font-family="DejaVu Sans, Arial, sans-serif" font-size="11">' % (W, H, W, H),
         '<title>Cycle time of each version, %s</title>' % xml_escape(unit),
         '<rect width="%d" height="%d" fill="#ffffff"/>' % (W, H),
         '<text x="%d" y="22" font-size="14" font-weight="bold" fill="#222222">Cycle time of each version (%s)</text>'
         % (ml, xml_escape(unit))]
    for k in range(6):
        val = top * k / 5
        y = Y(val)
        S.append('<line x1="%d" y1="%.2f" x2="%d" y2="%.2f" stroke="#e1e5ea" stroke-width="1"/>' % (ml, y, W - mr, y))
        S.append('<text x="%d" y="%.2f" text-anchor="end" fill="#444444">%s</text>' % (ml - 6, y + 4, "%g" % round(val, 4)))
    S.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#444444"/>' % (ml, mt + ph, W - mr, mt + ph))
    S.append('<text transform="translate(16 %.2f) rotate(-90)" text-anchor="middle" fill="#444444">cycle time (%s)</text>'
             % (mt + ph / 2, xml_escape(unit)))
    for i, v in enumerate(vs):
        S.append('<text x="%.2f" y="%d" text-anchor="middle" fill="#444444">%s</text>' % (X(i), mt + ph + 16, xml_escape(v["id"])))
    line = [p for p in pts if p[1]["status"] != "candidate"]
    if len(line) > 1:
        S.append('<polyline fill="none" stroke="#1f5fa8" stroke-width="1.5" points="%s"/>'
                 % " ".join("%.2f,%.2f" % (X(p[0]), Y(p[2]["value"])) for p in line))
    for i, v, it in pts:
        colour = "#9aa5b1" if v["status"] == "candidate" else "#1f5fa8"
        fill = "#ffffff" if it["known"] == "modelled" else colour
        if it["known"] == "measured":
            S.append('<rect x="%.2f" y="%.2f" width="10" height="10" fill="%s" stroke="%s" stroke-width="2"/>'
                     % (X(i) - 5, Y(it["value"]) - 5, fill, colour))
        else:
            S.append('<circle cx="%.2f" cy="%.2f" r="5" fill="%s" stroke="%s" stroke-width="2"/>'
                     % (X(i), Y(it["value"]), fill, colour))
        S.append('<text x="%.2f" y="%.2f" text-anchor="middle" fill="#222222">%s</text>'
                 % (X(i), Y(it["value"]) - 10, fmt("cycle_s", it["value"])))
    ly = H - 12
    S.append('<circle cx="%d" cy="%d" r="4" fill="#1f5fa8" stroke="#1f5fa8" stroke-width="2"/>' % (ml, ly - 4))
    S.append('<text x="%d" y="%d" fill="#444444">planned</text>' % (ml + 10, ly))
    S.append('<rect x="%d" y="%d" width="8" height="8" fill="#1f5fa8" stroke="#1f5fa8" stroke-width="2"/>' % (ml + 86, ly - 8))
    S.append('<text x="%d" y="%d" fill="#444444">measured</text>' % (ml + 100, ly))
    S.append('<circle cx="%d" cy="%d" r="4" fill="#ffffff" stroke="#1f5fa8" stroke-width="2"/>' % (ml + 180, ly - 4))
    S.append('<text x="%d" y="%d" fill="#444444">modelled</text>' % (ml + 190, ly))
    S.append('<circle cx="%d" cy="%d" r="4" fill="#9aa5b1" stroke="#9aa5b1" stroke-width="2"/>' % (ml + 270, ly - 4))
    S.append('<text x="%d" y="%d" fill="#444444">candidate</text>' % (ml + 280, ly))
    S.append("</svg>")
    return "\n".join(S) + "\n"


# ---------------------------------------------------------------- README

def results_block(reg):
    L = []
    for d in reg.directions:
        L += ["### " + d["title"], ""]
        link = "[%s: progress and versions](docs/directions/%s.md)" % (d["title"], d["id"])
        best, chosen = best_version(reg, d)
        if best is None:
            L += ["No version yet.", "", link, ""]
            continue
        card = reg.card(best)
        L += ["![%s](%s)" % (alt_text(reg, best), best["recording"]), ""]
        row = [best["id"], cell(reg, best, "cycle_s") + ("" if cycle_known(reg, best) == "none"
               else " (%s)" % cycle_known(reg, best)),
               plain(reg, best, "failures_per_bay"), plain(reg, best, "arms"), plain(reg, best, "part_count"),
               known_cell(reg, best, "placement_mm"), cell(reg, best, "mass_in_transit_kg"),
               gate_status(card) if card else "none"]
        L += table(["Version", "Cycle time", "Failures per bay", "Arms", "Part count", "Placement error",
                    "Mass in transit", "Gates"], [row]) + ["", link, ""]
    return "\n".join(L)


def readme_text(reg):
    path = os.path.join(reg.root, "README.md")
    text = read_text(path)
    lines = text.split("\n")
    try:
        b = lines.index(BEGIN)
        e = lines.index(END, b + 1)
    except ValueError:
        raise Error("README.md lacks the lines\n%s\n%s" % (BEGIN, END))
    new = lines[:b + 1] + [""] + results_block(reg).split("\n") + lines[e:]
    return "\n".join(new)


def everything(reg):
    files = {}
    for d in reg.directions:
        files["docs/directions/%s.md" % d["id"]] = direction_page(reg, d)
        svg = chart_svg(reg, d)
        if svg is not None:
            files["docs/directions/%s-cycle.svg" % d["id"]] = svg
    files["README.md"] = readme_text(reg)
    return files


# ---------------------------------------------------------------- add

def git_date(root, commit):
    if commit == "this":
        return datetime.date.today().isoformat()
    try:
        return subprocess.run(["git", "-C", root, "log", "-1", "--format=%ad", "--date=short", commit],
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        raise Error("cannot read the date of commit %s" % commit)


def do_add(args, root):
    path = os.path.join(root, "versions", "versions.json")
    data = load_json(path)
    versions = data["versions"]
    lpath, ppath, spath = ("layouts/%s.json" % args.layout, "plans/%s.json" % args.layout,
                           "scores/%s.json" % args.layout)
    for rel in (lpath, ppath, spath):
        if not os.path.exists(os.path.join(root, rel)):
            raise Error("%s does not exist (a scorecard needs the layout, its plan and its scores)" % rel)
    lsha, psha = sha256_of(os.path.join(root, lpath)), sha256_of(os.path.join(root, ppath))
    for v in versions:
        for key, sha in (("layout", lsha), ("plan", psha)):
            if v.get(key) and v[key].get("sha256") == sha:
                raise Error("the %s file is already registered with the same sha256 in %s" % (key, v["id"]))
    direction = args.direction
    if not direction:
        layout = load_json(os.path.join(root, lpath))
        if layout.get("base") != "flat_rack":
            raise Error("cannot tell the direction of %s (base %r): pass --direction" % (lpath, layout.get("base")))
        direction = "one-40ft-flat-rack"
    if direction not in [d["id"] for d in data["directions"]]:
        raise Error("unknown direction %s" % direction)
    if args.competed_with and args.competed_with not in [v["id"] for v in versions]:
        raise Error("unknown version %s" % args.competed_with)
    nid = "v%02d" % (max([int(v["id"][1:]) for v in versions] + [0]) + 1)
    entry = {"id": nid, "date": git_date(root, args.commit), "commit": args.commit, "title": args.title,
             "direction": direction, "status": args.status, "changed": args.changed, "why": args.why,
             "recording": args.recording}
    if args.recording_source:
        entry["recording_source"] = {"path": args.recording_source, "added_at": args.recorded_at,
                                     "shows": args.shows}
    if args.competed_with:
        entry["competed_with"] = args.competed_with
    entry["layout"] = {"path": lpath, "sha256": lsha}
    entry["plan"] = {"path": ppath, "sha256": psha}
    entry["scores"] = spath
    if args.status == "chosen":
        for v in versions:
            if v["direction"] == direction and v["status"] == "chosen":
                v["status"] = "superseded"
    versions.append(entry)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(data, indent=2, ensure_ascii=False) + "\n")
    print("added %s (%s) to versions/versions.json" % (nid, direction))


# ---------------------------------------------------------------- main

def main(argv):
    p = argparse.ArgumentParser(description="pages from the version registry", add_help=True)
    p.add_argument("command", nargs="?", help="write, check, caption or add")
    p.add_argument("id", nargs="?", help="version id (caption)")
    p.add_argument("--check", action="store_true")
    p.add_argument("--root", default=repo_root())
    p.add_argument("--layout")
    p.add_argument("--title")
    p.add_argument("--changed")
    p.add_argument("--why")
    p.add_argument("--status", choices=["chosen", "candidate"])
    p.add_argument("--commit", default="this")
    p.add_argument("--competed-with", dest="competed_with")
    p.add_argument("--direction")
    p.add_argument("--recording")
    p.add_argument("--recording-source", dest="recording_source")
    p.add_argument("--recorded-at", dest="recorded_at")
    p.add_argument("--shows")
    args = p.parse_args(argv)
    cmd = "check" if args.check and not args.command else args.command
    root = os.path.abspath(args.root)
    try:
        if cmd == "add":
            for k in ("layout", "title", "changed", "why", "status", "recording"):
                if getattr(args, k) is None:
                    raise Error("add needs --%s" % k)
            if args.recording_source and not (args.recorded_at and args.shows):
                raise Error("--recording-source needs --recorded-at and --shows")
            do_add(args, root)
            return 0
        reg = Registry(root)
        if cmd == "caption":
            v = next((v for v in reg.versions if v["id"] == args.id), None)
            if v is None:
                raise Error("no version %s" % args.id)
            print("\n".join(caption_lines(reg, v)))
            return 0
        if cmd == "write":
            for rel, text in sorted(everything(reg).items()):
                full = os.path.join(root, rel)
                os.makedirs(os.path.dirname(full), exist_ok=True)
                with open(full, "w", encoding="utf-8", newline="\n") as f:
                    f.write(text)
            print("pages: written")
            return 0
        if cmd == "check":
            bad = []
            for rel, text in sorted(everything(reg).items()):
                full = os.path.join(root, rel)
                if not os.path.exists(full) or read_text(full) != text:
                    bad.append(rel)
            if bad:
                for rel in bad:
                    print("pages: differs or missing: " + rel)
                return 1
            print("pages: up to date")
            return 0
        p.print_usage(sys.stderr)
        return 2
    except Error as e:
        print("pages: " + str(e), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
