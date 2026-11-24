# Stop 6: Seeing

How one picture from the camera above the cell becomes "the front module is here, facing
this way", accurately enough to put eight suction cups on it. This is the look step of
the cycle. It is also where you first meet code that says "I do not know" on purpose.
About a day.

## Read

1. `src/camera_model.h`, the comment at the top: the pinhole model in ten lines.
2. `src/depth_camera.h`, the comment at the top (depth against range, and why the image
   is made by casting rays), then `DepthImage` and `DepthNoise`.
3. `src/plane_fit.h`, all of it, and `FitPlaneRansac` in `src/plane_fit.cc`.
4. `src/module_finder.h`, the comment at the top: the seven steps and "Creases". Then
   skim the declarations; they follow the steps in order.
5. `src/rectangle_fit.h`, the comment at the top and the long comment on
   `RefineOptions::correct_edge_bias`.
6. `src/portable_random.h`, the comment at the top.
7. `docs/img/pallet_depth.png`: the picture on the left, the flattened glass on the
   right.

`src/held_module_finder.h` is the same steps run from below, on the module the arm
holds over the check camera. Read its top comment after the lab.

## The idea

**A depth image and a pinhole.** A depth image is a picture whose pixels hold distances:
here 640 by 480 of them, each a number of metres or "no reading". Imagine every ray of
light passing through one point, the camera's centre, on its way to the image. A point
twice as far away lands half as far from the image's middle. One number sets the scale,
the focal length, here about 659 pixels (`print_cell` prints it). So at 4 m one pixel
covers 4 / 659 m, about 6 mm. That is the grain of this stop's raw material, and the
answer will come out far finer than it.

**From a pixel to a point.** A pixel's value is the *depth*, the distance along the
camera's viewing axis, not the *range* along the pixel's own slanting ray. The header
explains why: the pixel's column and row give a direction, and times the depth that is
a point in the camera's frame. The point is then moved into the world through the
camera's pose, as in stop 2: `camera.pose * point`.

**Calibration.** The camera's pose is its calibration: where the code believes the
camera is. If the belief is off, every point moves with it, rigidly, and nothing in the
picture can show it. Angle times distance again: a camera thought to point one
milliradian off puts a glass 4 m away 4 mm from where it is. `Cell::CalibratedCamera`
is the belief and `Cell::DepthCamera` the truth; perception is handed the first.

**Crop.** Most of the picture is floor. Only the points where the design says the stack
can be are kept: a box round every place the front glass could stand, given how far off
a forklift may set a pallet down (`PalletPrior`, `CropFromPrior`). The box comes from
the drawing and the tolerance, never from where the modules truly are.

**The plane.** The glass is flat, so its points lie on a plane. *Least squares* takes
all the points given and finds the plane with the smallest total squared distance to
them; one stray surface drags it. *RANSAC* picks three points at random (three points
fix a plane), counts how many other points lie close to that plane, and repeats. The
plane with the most supporters wins, and only then is least squares run, on its
supporters alone. If half the points are on the glass, one sample in eight lands on it,
and the header works out that 52 samples make a miss a one-in-a-thousand event. Here
RANSAC is also told which way the glass faces, so the stack's top and the pallet's
deck, which are as large, are never counted.

**Noise, and why many points beat it.** A real depth camera's readings scatter. The
simulated one is given a model of that: scatter that grows with the square of the depth
(the header's figure is 3.3 mm at 3.8 m), pixels at depth edges that drop out or float
between two surfaces, and readings in whole millimetres. The numbers are an assumption,
not a datasheet, and the header says so. The plane does not inherit one point's
scatter: averaging N independent errors shrinks them by the square root of N, and the
glass has tens of thousands of points.

**The outline.** The glass's points are dropped onto the plane and become dots in 2D
(`PlaneFrame::Flatten`). `FitRectangle` finds the 2.3 by 1.1 m rectangle they fill: the
largest clump, a first guess from the smallest enclosing rectangle, then a straight
line through the outermost dots of each side. Dots are pixel centres, so the outermost
sit a little inside the true edge; the fit corrects for that by counting dots. With
modules behind it, the glass's top edge runs into the stack's top surface with no step
in depth, only a bend. The header's "Creases" section says what that does and how the
long edges are measured instead. Read it slowly: it is the part of the finder that came
from looking at wrong answers.

**Refusing is an answer.** `FindFrontModule` returns `std::optional<ModuleEstimate>`:
a pose, or nothing with a reason (`FinderFailure`). A rectangle of the wrong size, a
pixel inside the outline that does not read the glass (the arm in the way), an empty
pallet: all end in "nothing", and the cycle stops with the reason in words. A finder
that always answers will one day send the cups to the wrong place.

**How sure.** The estimate carries `PoseUncertainty`, one standard deviation for each
part of the pose, from the scatter of the points used. It covers what varies from
picture to picture. It cannot cover the calibration, which is the same in every one.

## C++ you meet here

**An image is a vector with arithmetic.** `DepthImage` holds `std::vector<float> depth`
and two sizes. Pixel (u, v) is `depth[v * width + u]`; the methods `index`, `at` and
`has_reading` say that once. There are two `at`s: one for a `const` image returning a
`float`, one returning `float&`, a reference you can assign through. A range-based
`for` walks a container without an index, like Python's `for z in depth`:
`for (float& z : image->depth)` in `AddDepthNoise` visits every pixel. The `&` makes
`z` the pixel itself; without it `z` would be a copy and the image would not change.

**`float`, `double` and `static_cast`.** A `float` is 32 bits, about seven digits; a
`double` is 64, about sixteen. The image is `float`, the geometry `double`. C++ converts
between number types silently, sometimes losing something, so this code writes each
conversion out: `static_cast<float>(depth)` in `RenderDepth`, and
`static_cast<int>(points.size())` wherever a container's unsigned size meets an `int`.

**NaN means "no reading".** A floating-point number can hold "not a number"
(`kNoReading` in `src/depth_camera.cc`); `std::isfinite` tests for it. Every comparison
with NaN is false, even `x == x`, and the code leans on that: `if (!(depth <=
options.max_depth)) continue;` in `RenderDepth` skips a NaN, where `if (depth >
options.max_depth)` would let it through.

**Lambdas and `<algorithm>`.** A lambda is a function written where it is used. The
square brackets say what it may see of the code around it:

- `[](float z) { return std::isfinite(z); }` in `DepthImage::CountReadings` sees
  nothing outside itself. `std::count_if` calls it on every element.
- `[m]` in `RansacLine` (`src/rectangle_fit.cc`) copies `m` in: capture by value.
- `[&parent]` in `LargestCluster` refers to the caller's `parent`: capture by
  reference. The lambda can change it and must not outlive it.
- `[&]` in `FindFrontModule` (`refuse`) refers to whatever it mentions: the
  repository's idiom for "fill in the report and return nothing".

The standard algorithms take a pair of positions, `v.begin()` and `v.end()`:
`std::sort` with a lambda saying which of two comes first (`before` in `ConvexHull`),
`std::nth_element` for a median without a full sort, `std::count_if`, `std::clamp`.

**Random numbers that repeat.** `std::mt19937` is a generator: from one seed it gives
the same sequence of 32-bit numbers on every machine. RANSAC, the noise and each
trial's scene draw from one, which is why a seed reproduces a run. Two traps, both in
the comment of `src/portable_random.h`. The library's *distributions*
(`std::normal_distribution` and its relatives) differ from one standard library to the
next, so code whose results must match everywhere draws by hand (`UniformIndex` and
`NormalDraw` there, `Draws` in `src/depth_camera.cc`). And in `f(draw(rng), draw(rng))`
C++ does not fix which argument is worked out first; the two kinds of machine this
project uses disagree. Take each draw in a statement of its own.

**One template.** `Format` in `src/rectangle_fit.cc` begins `template <typename...
Args>`: one function for any number of arguments of any types, here wrapping
`snprintf`. The standard library is built from templates, but `src/` writes only two.

## Lab

Write your prediction down before each run.

1. **What the camera sees.** `bazel run --config=opt //:print_depth`, then with
   `--noise`, then `--noise --last`. How many pixels read the front module? Why do
   some pixels read nothing? Compare the front module's depth range with and without
   noise: what kind of pixel explains the change?
2. **Many points against noise.** `bazel run --config=opt //:print_plane_fit --
   --pretty` fits made-up clouds whose points are scattered about 2 mm off the plane.
   Before running: roughly how far off will the fitted plane be? Look at the last two
   lines.
3. **Noise on the real finder.** `bazel run --config=opt //:print_perception --
   --pretty --trials 5`, then add `--noise`. Which error column is not zero even
   without noise, and what limits it? Compare the worst errors with the 6 mm pixel.
4. **A calibration that is off.** Add `--calibration-error 0,0.0573` (one milliradian)
   to the noisy run. Predict the size of the error first, from the camera's height and
   where the stack stands. How does it change down the `k` column, and why? Then find
   "error over the finder's own sigma" and compare it with run 3's.
5. **Refusing.** `--noise --trials 2 --arm-home` takes the pictures with the arm in
   front of the stack. What does the finder say at each stack depth, and which of the
   seven steps said it?
6. **What the creases are worth.** `--noise --trials 5 --no-creases`. Which column
   changed, by how much, and which way? Match that to the header's explanation.

## Pitfalls

- **Depth mistaken for range.** They differ by 17% at this image's corners. A point
  cloud built with the wrong one is bowed.
- **Pixel centres.** Pixel (c, r) has its centre at (c + 0.5, r + 0.5). Leave the half
  out and everything shifts by half a pixel, 3 mm here.
- **Trusting the estimate's sigma to cover calibration.** It cannot. Run 4 shows it.
- **Giving RANSAC a cloud where the wanted plane is not the biggest.** The most
  supported plane wins, whatever it is. Crop first, or say which way it faces.
- **Using the truth to tune perception.** A crop taken from where the modules are would
  pass every test and work on no real cell.
- **Two draws in one expression.** Two platforms that disagree, and a test that fails
  on only one of them.

## In the MoveIt cells

Finding the part is not MoveIt's job. MoveIt plans motion; it can take a point cloud to
mark space as occupied so that plans avoid it, but something else says where the module
is.

- A camera driver publishes the images, the camera's *intrinsics* (focal lengths and
  principal point) and often a ready-made point cloud. Depth images come as 32-bit
  floats in metres with NaN for no reading, or as 16-bit whole millimetres with 0 for
  none: the same two forms as here.
- A perception node turns that into a pose in the camera's frame, and tf2 (stop 2)
  carries it into the robot's frame through a camera-to-robot transform.
- Measuring that transform is *hand-eye calibration*: the arm shows a target to the
  camera from many poses, and the one transform that explains every view is solved
  for. It is a procedure with a residual, not a number in a file, and it changes when
  a camera is knocked. A real lens also distorts, so the intrinsics are calibrated too.
- The building blocks are libraries: PCL and Open3D for RANSAC planes in point clouds,
  OpenCV for the smallest enclosing rectangle and for calibration.
- Glass is hard. It reflects and transmits, and depth cameras read it badly. The
  README counts this camera's easy view of glass among the simulation's kindnesses.

Ask your team which cameras the cells use, how and how often they are calibrated, and
what the residual is. Then work out what that residual is in millimetres at the part.

## Reading

- Richard Szeliski, *Computer Vision: Algorithms and Applications* (2nd edition), free
  at <https://szeliski.org/Book/>: section 2.1, for the pinhole camera done properly.
- Martin Fischler and Robert Bolles, "Random Sample Consensus: A Paradigm for Model
  Fitting with Applications to Image Analysis and Automated Cartography",
  *Communications of the ACM*, 1981. The RANSAC paper; its first pages are the method.
- OpenCV, "Camera Calibration and 3D Reconstruction",
  <https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html>: the introduction, for
  intrinsics and distortion on a real camera, and `calibrateHandEye` for hand-eye.
- Shreeyak Sajjan and others, "ClearGrasp: 3D Shape Estimation of Transparent Objects
  for Manipulation", ICRA 2020. Current work on the weakness this simulation hides.

## Office hours

Be ready to explain, with the chapter closed:

- how a pixel and a depth become a point in the world, and where calibration enters;
- why RANSAC and not plain least squares, and why three points;
- why the fitted pose is far finer than one pixel or one reading's noise;
- three ways the finder refuses, and why each is better than an answer;
- what the estimate's uncertainty covers and what it cannot.

Questions worth asking your teacher:

- Why is the image made by casting rays and not by rendering with the graphics card?
- A calibration error cannot be seen in one picture. How does a cell ever find it?
