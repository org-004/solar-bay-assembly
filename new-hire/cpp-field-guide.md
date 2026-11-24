# C++ field guide

Every C++ feature this path teaches, at the stop where it first appears, with a place
in this repository to see it used. Read a stop's entries after its chapter, as a
summary, or come here when a later file uses something you have forgotten.

## The C++ this repository uses, and does not

It is C++20 written in a deliberately small subset.

- **No exceptions.** `src/` contains no `throw`, `try` or `catch`. A function that can
  fail returns a `std::optional` or a `bool`, and says why through a report struct or a
  `std::string*`.
- **No inheritance and no virtual functions** in `src/`. Where a value can be one of
  several kinds it is a `std::variant` or an `enum class`. The only base classes are
  GoogleTest's fixtures in `tests/`.
- **No shared ownership.** There is no `std::shared_ptr` anywhere. Every object has one
  owner and everything else borrows.
- **Almost no templates of its own:** two small ones in `src/`. It uses the standard
  library's and Eigen's all the time.
- **Google's naming:** `TypeName`, `FunctionName()`, `variable_name`, `member_`,
  `kConstant`, and accessors named like variables (`duration()`).
- **Inputs are `const T&`, outputs are `T*`,** and the comment above a declaration is
  the contract.

ROS 2 code uses all of the first three freely. That is a different dialect of the same
language, not a mistake; know which one you are reading.

## Stop 0: Watch it work

- **Compiled language.** Source is translated to machine code by a build before it runs;
  there is no interpreter.
  Here: the first `bazel build` of stop 0.
- **Bazel target.** A named thing the build can make: `cc_library` (code for other
  targets), `cc_binary` (a program), `cc_test` (a program whose exit status is the
  verdict). Like a crate or a package with a build script.
  Here: `BUILD.bazel`.
- **`bazel run //:x -- args`.** Build if needed, then run; everything after the lone `--`
  goes to the program.
  Here: every lab.
- **`--config=opt`.** Build with the compiler's optimiser on; 5 to 70 times faster here.
  Here: `.bazelrc`.
- **`MODULE.bazel`.** The list of outside libraries with exact versions, as `go.mod` or
  `package.json` is.
  Here: `MODULE.bazel`.

## Stop 1: The smallest program

- **Header and source file.** A `.cc` file is compiled by itself; a `.h` file holds what
  several share, and `#include` pastes its text in. There is no import system.
  Here: `src/mj_handles.h`, `src/step_arm.cc`.
- **Include guard.** The `#ifndef X_H_ ... #endif` wrapper that stops a header being
  pasted twice.
  Here: top and bottom of every header.
- **Namespace.** A named scope for names, like a package: `cell::ModelPtr`.
  Here: `namespace cell` everywhere.
- **Value, pointer, reference.** `T` is the object itself and is copied on assignment;
  `T*` is an address that may be null; `T&` is another name for an existing object. Python
  and Java variables are all the third kind.
  Here: `LoadModel`, `MakeData` in `src/mj_handles.h`.
- **`const`.** "not changed through this name". Here `const T&` marks an input and `T*` an
  output.
  Here: `MakeData(const mjModel&)`.
- **Ownership and RAII.** No garbage collector: a resource is released by the destructor
  of the object that owns it, when that object goes out of scope. Closest to Rust's
  ownership, or a `with` block that covers all memory.
  Here: `ModelPtr`, `DataPtr`.
- **`std::unique_ptr`.** A pointer with exactly one owner that deletes what it points at;
  it can be moved, not copied. `.get()` lends the raw pointer.
  Here: `src/mj_handles.h`.
- **Custom deleter.** A small type telling `unique_ptr` which function frees the resource.
  Here: `ModelDeleter`, `DataDeleter`.
- **`using Name = Type`.** A short name for a long type.
  Here: `ModelPtr`.
- **`inline` function in a header.** Lets every `.cc` that includes the header carry the
  definition without a clash.
  Here: `LoadModel`.
- **`nullptr` and optional outputs.** `std::string* error = nullptr`: pass an address to
  receive a message, or nothing.
  Here: `LoadModel`.

## Stop 2: Where things are

- **Fixed-size value types.** `Eigen::Vector3d`, `Matrix3d`, `Isometry3d` hold their
  numbers in place and copy them on assignment.
  Here: `TipPose` in `src/arm.cc`.
- **Dynamic-size vector.** `Eigen::VectorXd` keeps its numbers on the heap, with a size
  chosen at run time; used for a set of joint angles.
  Here: `JointAngles` in `src/arm.cc`.
- **Operator overloading.** A library defines `*`, `+` and the rest for its own types, and
  the types decide the meaning at compile time.
  Here: `tip * module_in_tip` in `src/held_module_finder.cc`.
- **Passing by const reference.** `const Eigen::Isometry3d& pose` reads the caller's
  object without copying it.
  Here: every function in `src/held_module_finder.h`.
- **A function call on the left of `=`.** An accessor that returns a reference lets you
  assign through it.
  Here: `pose.translation() = ...` in `TipPose`.
- **`Eigen::Map`.** A view of memory owned by somebody else as an Eigen object, with no
  copy; `RowMajor` says how the numbers are laid out.
  Here: `TipPose`.
- **`auto` and Eigen.** `auto x = a * b` keeps a recipe, not a result, and can dangle;
  write Eigen types out.
  Here: house habit throughout `src/`.
- **Dereferencing an optional.** `*opt` takes the contents without checking that there are
  any.
  Here: `*GripPose(...)` in `PickPlace::PlanPlace`.

## Stop 3: Moving one axis

- **Class.** Data bundled with the functions that work on it, with a public face and
  private workings; like a class in Java, but Python has no enforced `private`.
  Here: `Profile` in `src/profile.h`.
- **`const` member function.** A member function that promises not to change the object,
  enforced by the compiler.
  Here: `Profile::duration`, `Profile::At` in `src/profile.h`.
- **Declaration and definition.** The header says a function exists (`Sample At(double t)
  const;`), the `.cc` gives its body under the class's name (`Sample Profile::At(double t)
  const { ... }`).
  Here: `src/profile.h` and `src/profile.cc`.
- **Private constructor with static factories.** Outside code cannot build a blank object;
  it must call a `static` function on the type, which returns a valid object or nothing;
  like a `@classmethod` constructor or Go's `NewThing() (Thing, error)`.
  Here: `Profile::Trapezoid`, `Profile::SCurve`, `JointMove::Plan`.
- **`std::optional<T>`.** A box holding a `T` or nothing (`std::nullopt`); test it with
  `if (!x)`, look inside with `*x` or `x->`; looking inside an empty one is undefined
  behaviour, not an exception; compare Java's `Optional`, Python's `T | None`.
  Here: returned by `Profile::SCurve`, used in `src/print_profile.cc`.
- **Plain struct with default member initializers.** All members public, each with a
  default written beside it, so `Sample{}` is all zeros.
  Here: `Limits`, `Sample` in `src/profile.h`; `JointLimits`, `JointSample` in
  `src/joint_move.h`.
- **Nested private struct.** A helper type declared inside a class's private part, which
  nobody outside can name.
  Here: `Profile::Segment`.
- **`enum class`.** A closed set of named values that must be written in full
  (`ProfileShape::kSCurve`) and does not turn into an integer by itself.
  Here: `ProfileShape` in `src/joint_move.h`.
- **`std::vector<T>`.** A growable array (`push_back`, `back()`, `empty()`), visited with
  `for (const T& x : v)`.
  Here: `Profile::segments_`, the loop in `Profile::At`.

## Stop 4: Motors

- **`std::array<T, N>`.** Exactly N values stored in place, the size part of the type; a
  `std::vector` can grow and lives on the heap.
  Here: `Arm::qpos`, `Arm::dof`, `Arm::motor` in `src/arm.h`.
- **`static constexpr` member.** A compile-time constant that belongs to the type, written
  `Arm::kJoints` from outside.
  Here: `Arm::kJoints`, `MoveOptions::kDefaultSpeed`, `ArmFollower::kArrivedAngle`.
- **Index arithmetic into C arrays.** MuJoCo stores every joint's numbers in flat arrays;
  `model.jnt_range[2 * joint + 1]` is a joint's upper limit; nothing checks the index.
  Here: `FindArm` in `src/arm.cc`, `ArmFollower::Update`.
- **Member initializer list and `explicit`.** `: arm_(arm), settings_(settings) {}` builds
  members straight from the arguments; `explicit` forbids silent conversion to the class.
  Here: `ArmFollower`'s constructor.
- **Options struct with a default argument.** `const MoveOptions& options = {}` means "all
  defaults unless given"; one field is changed by name with a designated initializer,
  `{.speed = 1.0}`; the nearest C++ has to keyword arguments.
  Here: `ArmFollower::MoveTo`, `ArmFollower::MoveAlong` in `src/arm_follower.cc`, the tests.
- **`std::variant` and `std::visit`.** A value that is exactly one of several types and
  knows which; `std::visit` calls a function with whichever it holds; a tagged union, with
  no base class.
  Here: `ArmFollower::Leg` in `src/arm_follower.h`.
- **Lambda.** A function written in place: `[t](const auto& m) { return m.At(t); }`; the
  brackets list what it captures, and an `auto` parameter makes it work for any type that
  fits.
  Here: `ArmFollower::Leg::At`, the `refuse` helpers in `src/arm_follower.cc`.
- **Scratch members to avoid allocation.** Vectors kept as members so a function called
  every tick does not ask the heap for memory.
  Here: `ArmFollower::acceleration_`, `torque_`.

## Stop 5: Pose to joint angles

- **Unnamed namespace.** Everything inside `namespace { ... }` in a `.cc` can be used only
  in that file; C++'s "not exported", like a lowercase name in Go.
  Here: `Evaluate`, `PoseError`, `Converged`, `Clamp` in `src/ik.cc`.
- **`constexpr` tuning constant.** A value fixed at compile time, here always with a
  comment saying what was tried and why it stands.
  Here: `kDampingUp`, `kWindow` in `src/ik.cc`.
- **Type alias for a fixed-size matrix.** `using Vector6d = Eigen::Matrix<double, 6, 1>;`
  names a type whose size the compiler knows, so it needs no allocation.
  Here: `Vector6d`, `Matrix6d` in `src/ik.cc`.
- **`head<3>()` and `tail<3>()`.** Views of the first or last three entries of a vector;
  the number in angle brackets is a compile-time size.
  Here: `PoseError`, `Converged` in `src/ik.cc`.
- **Report struct with an optional out-parameter.** The answer comes back as the return
  value and the reasons go into a struct the caller may pass or not (`IkReport* report =
  nullptr`); inside, `IkReport& r = report != nullptr ? *report : local;` lets the code
  write to one name either way.
  Here: `SolveIk`, `SolveIkPath`.
- **Status `enum class` with a `switch`.** Every way an operation can end, as named
  values; a `switch` with a `case` for each and no `default` makes the compiler warn when
  one is added and forgotten.
  Here: `IkStatus` in `src/ik.h`, `IkStatusName` in `src/pick_place.cc`.
- **Loop with exits.** `while (true)` with a `break` for each way out, the status set just
  before.
  Here: the loop in `SolveIk`.
- **Solving a linear system.** `a.ldlt().solve(b)` factorises the matrix and solves with
  the factors; code does not compute an inverse.
  Here: the step in `SolveIk`.

## Stop 6: Seeing

- **An image as a vector with index arithmetic.** A 2D picture kept as one flat
  `std::vector<float>` with pixel (u, v) at `depth[v * width + u]`, wrapped in small
  accessor methods so the sum is written once; like a NumPy array's flat buffer, with the
  indexing done by hand.
  Here: `DepthImage` (`index`, `at`, `has_reading`) in `src/depth_camera.h`.
- **Const and non-const overloads of one accessor.** Two methods of the same name, one for
  a `const` object returning a value and one returning a reference you can assign through.
  Here: `DepthImage::at` in `src/depth_camera.h`; used as `image->at(u, v) = ...` in
  `AddEdgeEffects` in `src/depth_camera.cc`.
- **Range-based `for`.** Walking a container without an index, like Python's `for z in
  depth`; with `&` the loop variable is the element itself and can change it, without `&`
  it is a copy.
  Here: `for (float& z : image->depth)` in `AddDepthNoise` (`src/depth_camera.cc`); `for
  (int i : indices)` in `FitSubset` (`src/plane_fit.cc`).
- **`float`, `double` and `static_cast`.** 32-bit and 64-bit floating point (about seven
  and sixteen digits); C++ converts between number types silently, so this code writes
  each conversion out with `static_cast<T>(x)`.
  Here: `static_cast<float>(depth)` in `RenderDepth`, `static_cast<int>(points.size())` in
  `FitPlaneRansac`.
- **NaN as "no reading".** A floating-point value meaning "not a number", for which every
  comparison is false (even `x == x`); tested with `std::isfinite`, and conditions are
  written so that a NaN falls on the safe side.
  Here: `kNoReading` and `if (!(depth <= options.max_depth))` in `src/depth_camera.cc`;
  `InliersOf` in `src/plane_fit.cc`.
- **Lambdas and their captures.** A function written where it is used; the square brackets
  say what it may see: `[]` nothing, `[m]` a copy of `m`, `[&parent]` the caller's
  `parent` itself, `[&]` whatever it mentions by reference; like a Python `lambda` or a
  JavaScript arrow function, except that you choose copy or reference and a reference must
  not outlive its target.
  Here: `DepthImage::CountReadings`, `RansacLine` (`pick`), `LargestCluster` (`root`),
  `FindFrontModule` (`refuse`).
- **`<algorithm>` functions on a pair of positions.** Standard routines that take
  `v.begin()` and `v.end()` and often a lambda: `std::sort`, `std::nth_element` (a median
  without a full sort), `std::count_if`, `std::unique`, `std::clamp`.
  Here: `ConvexHull` and `DotSpacing` in `src/rectangle_fit.cc`;
  `DepthImage::CountReadings`.
- **Seeded random numbers, and what is not portable.** `std::mt19937` gives the same
  32-bit sequence from the same seed on every machine, but the library's distributions
  (`std::normal_distribution` and relatives) differ between standard libraries, and the
  order in which a call's arguments are evaluated is not fixed, so two draws in one
  expression can swap between platforms.
  Here: `src/portable_random.h` (`UniformUnit`, `UniformReal`, `UniformIndex`,
  `NormalDraw`); `Draws` in `src/depth_camera.cc`; `Uniform` in `src/cell.cc`.
- **A variadic function template.** One function written for any number of arguments of
  any types (`template <typename... Args>`), the compiler making a version for each use;
  one of only two templates written in `src/`.
  Here: `Format` in `src/rectangle_fit.cc` (the other is in `src/print_rrt.cc`).

## Stop 7: The cell and the truth

- **A class that owns its parts.** Members live exactly as long as the object that holds
  them and are destroyed with it, last declared first; there is no garbage collector
  deciding later.
  Here: `Cell` in `src/cell.h` holds the model, the state, the conveyor and the suction
  (`model_`, `data_`, `conveyor_`, `suction_`).
- **`std::optional` as a member built late.** A member that starts empty because it cannot
  be made until something else exists, is filled later and can be replaced; reached with
  `*member` and `member->`.
  Here: `std::optional<Conveyor> conveyor_` and `std::optional<Suction> suction_` in `Cell`,
  filled in `Cell::Load` and replaced in `Cell::Reset`.
- **Move-only types and `std::move`.** A class holding a `unique_ptr` cannot be copied,
  only moved (its contents handed over, the source left empty); returning a local moves
  it, and `std::move(x)` asks for a move where a copy would otherwise happen; nearest to
  Rust's ownership.
  Here: `Cell::Load` ending `return cell;` into `std::optional<Cell>`; `inliers =
  std::move(next)` in `FitPlaneRansac`; `modules_(std::move(modules))` in `Suction`'s
  constructor.
- **Borrowed pointers and references.** A plain pointer or reference kept as a member owns
  nothing and is only good while its target lives and stays put; the comment is the only
  guard.
  Here: `Conveyor`'s `const mjModel* model_` (`Conveyor::Find`), `PickPlace`'s `Cell&
  cell_`.
- **Const and non-const accessor pairs.** Two accessors of one name; a `const` object gets
  the read-only one, so a function taking `const Cell&` can read but not write.
  Here: `Cell::model()`, `Cell::data()`, `Cell::conveyor()`; `Cell::suction()` exists only
  in the `const` form.
- **Function overloading.** Several functions sharing a name and differing in parameters,
  the compiler choosing by the arguments.
  Here: `MeasurePlacement(cell, i, slot)` and `MeasurePlacement(cell, i)` in `src/score.h`.
- **`inline constexpr` constants in headers.** A compile-time constant defined in a header
  for every file that includes it, with one definition program-wide.
  Here: `kModuleLength` and its neighbours in `src/pallet.h`; `kRestSpeed` in `src/score.h`.
- **`.at(i)` against `[i]`.** `.at(i)` checks the index and stops the program when it is
  out of range; `[i]` does not check, and out of range it reads or writes stray memory.
  Here: `Cell::module_state` and `Cell::TruePlacedSlot` use `.at`; `Cell::Grip` uses `[]` on
  an index it has just looked up.

## Stop 8: Will it hit anything

- **A private copy of a C struct.** A class that copies a library's data so that it may
  rewrite the copy freely, leaving the original untouched; like deep-copying a config
  object before mutating it.
  Here: `CollisionChecker::Create` in `src/collision.cc` (`mj_copyModel`, and the checker's
  own `mjData`).
- **Bit masks and bitwise operators.** An integer used as a row of yes-or-no flags; `|`
  joins flags, `&` keeps the shared ones, `~` flips them, `&= ~x` switches `x` off (the
  same operators as in Python and Java, used far more when talking to C libraries).
  Here: `kLinkBit` and its neighbours, `Collides` and `CollisionChecker::ApplyFilters` in
  `src/collision.cc`.
- **An enum class with a chosen underlying type.** `enum class Role : unsigned char`
  stores each value in one byte.
  Here: `CollisionChecker::Role` in `src/collision.h`.
- **`std::pair` and structured bindings.** Two values held together, and `const auto& [a,
  b]` to unpack them into names, as Python's `for a, b in pairs`.
  Here: `CollisionChecker::SetAllowedContacts` and the loop over `distance_pairs_` in
  `CollisionChecker::Query`, `src/collision.cc`.
- **Sorting and searching with a comparator lambda.** `std::sort` and `std::find_if` take
  a small function saying which comes first or what to look for; ties are broken on
  purpose so the order repeats.
  Here: `CollisionChecker::Query` in `src/collision.cc`.
- **A query method that is not `const`.** A method that reads like a question but writes
  scratch state, so one object must not be shared between threads.
  Here: `CollisionChecker::IsFree` and `Check` in `src/collision.h` ("One checker per
  thread").
- **Counting instead of timing.** Report work in a count that is the same on every
  machine, and time only as information.
  Here: `CollisionChecker::queries()` in `src/collision.h`.

## Stop 9: Finding a path

- **`std::function`.** A value that holds anything callable with a given signature, like a
  function type in Go or a `Callable` in Python; the seam that lets the planner know
  nothing about the world it plans in.
  Here: `IsFree` in `src/rrt.h`, supplied by `PlanMove` in `src/motion_planner.cc` and by
  the toy worlds in `src/rrt_worlds.h`.
- **A lambda that captures by reference.** `[&checker](...) { ... }` holds a reference to
  `checker`, so the lambda must not outlive it.
  Here: the `is_free` lambda in `PlanMove`, `src/motion_planner.cc`.
- **`std::function` against a template parameter.** A template would be a little faster
  and would move the code into the header; `std::function` keeps it in an ordinary source
  file.
  Here: `PlanRrtConnect` in `src/rrt.h`.
- **Type aliases.** `using Name = Type;` gives a long type a short name without making a
  new type.
  Here: `JointPath` in `src/rrt.h`, `Clock` in `src/rrt.cc`.
- **Fixed-width integers.** `std::int64_t` and `std::uint32_t` say exactly how many bits;
  used where a count could pass two billion, or where a seed must be 32 bits.
  Here: `RrtReport::collision_queries`, `RrtOptions::seed`, `PlanSeed` in
  `src/motion_planner.cc`.
- **`std::chrono::steady_clock`.** A clock that never jumps, for measuring how long
  something took and for time budgets.
  Here: `SecondsSince` in `src/rrt.cc` and `src/motion_planner.cc`.
- **A helper lambda with a trailing return type.** `[&](...) -> ReturnType { ... }` states
  what the lambda returns; used to make every early exit one line.
  Here: `fail` and `finish` in `PlanRrtConnect`, `src/rrt.cc`.
- **A guard object.** A small class whose constructor saves a setting and whose destructor
  puts it back, on every way out of the function; stop 1's ownership idea used for
  something that is not memory (Python's `with`, Go's `defer`); `= delete` forbids copying
  it.
  Here: `MarginGuard` in `src/motion_planner.cc`.

## Stop 10: Timing a path

- **Layout of a long numerical file.** Constants with their reasoning first, then structs
  and small functions private to the file inside an unnamed namespace, then the few public
  functions last; read it from the public function down.
  Here: `src/path_timing.cc` (`Stretch`, `Timing`, `Shape`, `Polyline`, then
  `Trajectory::Plan` and `Trajectory::At`).
- **A nested type.** A struct declared inside a class; nesting only scopes the name
  (`Trajectory::Piece`) and, unlike a Java inner class, links it to no instance.
  Here: `Trajectory::Piece` (public) and `Trajectory::Segment` (private) in
  `src/path_timing.h`.
- **Named tolerances for floating-point comparisons.** Computed doubles are compared
  against a small named constant, never with `==`.
  Here: `kSamePoint`, `kStraightOn`, `kSameTime` in `src/path_timing.cc`.
- **`!(x > 0.0)` against `x <= 0.0`.** A NaN fails every comparison, so the negated form
  also rejects NaN.
  Here: the check in the `append` lambda and the option checks at the top of
  `Trajectory::Plan`, `src/path_timing.cc`.
- **Try candidates, keep the best.** A lambda that evaluates one candidate and overwrites
  the best so far, called from loops over small constant arrays; the best starts as an
  empty `std::optional`.
  Here: `consider`, `kShares` and `kSimplify` in `Trajectory::Plan`.
- **Bisection, and assigning through a conditional.** Halving an interval a fixed number
  of times to find where a yes-or-no test flips; `(test ? low : high) = mid;` assigns to
  whichever variable the test picks.
  Here: `Reach`, `Peak` and `StretchLimits` in `src/path_timing.cc`.
- **Reference invalidation.** A reference or iterator into a `std::vector` is dead once
  the vector grows or erases, because the elements live in the vector's own storage
  (unlike a Python or Java list of references).
  Here: the `append` lambda in `Trajectory::Plan` uses `segments.back()` before `push_back`;
  `Merge` edits a copy.

## Stop 11: The cycle

- **A large class.** A small public part, many private helper functions, and the state
  kept in member variables; the header lists private members too, because the compiler
  needs them to know the object's size, where Java or Python would hide them.
  Here: `PickPlace` in `src/pick_place.h`, bodies written `PickPlace::Name` in
  `src/pick_place.cc`.
- **A state machine as an enum and a switch.** The steps are an `enum class` and one
  `switch` per question ("what does this step do when it begins", "is it over"); several
  `case` labels can share a body, a case that declares a variable needs braces, and
  leaving out `default:` makes the compiler warn about a step nobody handled.
  Here: `PickPlaceStep`, `PickPlace::BeginStep`, `PickPlace::Advance`, `PickPlace::Next`.
- **Optional members for "not known yet".** A `std::optional` member is empty until
  something has produced its value and is emptied again with `.reset()`; like a nullable
  field, but said in the type.
  Here: `estimate_`, `release_tip_`, `held_in_tip_` in `PickPlace`, reset in
  `PickPlace::Start`.
- **A reference as a member.** A member declared `T&` is another name for an object owned
  elsewhere; it does not keep that object alive, so the class carries a written contract
  about lifetimes that a garbage collector would otherwise enforce.
  Here: `Cell& cell_` and the constructor's comment "must outlive this object" in
  `src/pick_place.h`.
- **A stored callback.** A `std::function` kept as a member, handed in by value and moved
  into place with `std::move`, and called if set; the lambda's body belongs to the caller,
  which is why a scorer's hook may read the truth when the script may not.
  Here: `PlacedHook`, `CheckHook`, `set_placed_hook` in `src/pick_place.h`; the lambdas in
  `RunTrial`, `src/trial_runner.cc`.
- **`mutable`.** Marks a member that functions declared `const` may still write, for a
  workspace that is not part of what the object is.
  Here: `scratch_` in `PickPlace`.
- **A lambda called on the spot.** `[] { ... }()` runs several statements where only an
  expression is allowed, such as a member's default value or a `const` variable set from
  code with early exits.
  Here: `check_pose` in `PickPlaceOptions`; `found` in `PickPlace::PlanCarry`.
- **`std::snprintf` into a buffer.** C's `printf` writing into a fixed `char` array, never
  past the size given; the nearest thing to an f-string this code uses.
  Here: `PickPlace::Shortfall`, `PickPlace::RunCheck`.

## Stop 12: Proving it

- **Threads.** `std::thread` starts a function running at the same time as the caller, on
  another core if there is one, and `join` waits for it; unlike Python's threads these
  truly run in parallel.
  Here: the pool in `RunTrials`, `src/trial_runner.cc`.
- **Data race.** Two threads touching the same variable at once, at least one writing,
  with nothing to order them; in C++ the result is undefined behaviour, not merely a wrong
  number.
  Here: avoided in `RunTrials` by giving each thread its own `Cell`.
- **`std::atomic`.** A variable whose single operations (such as `next++`) are
  indivisible, so threads can share it without a lock.
  Here: `next` and `load_failed` in `RunTrials`.
- **`std::mutex` and `std::lock_guard`.** A lock, and an object that takes it and releases
  it when its scope ends, on every path (ownership by scope again, as `unique_ptr` in stop
  1).
  Here: `load_mutex` and `done_mutex` in `RunTrials`.
- **A `double` as exact text.** `std::to_chars` writes the shortest text that
  `std::from_chars` reads back to the very same `double`, where `printf("%f")` rounds.
  Here: `Number` and `ParseNumber` in `src/trial_runner.cc`; used by `FormatFailureRecord`
  and `CompareFailure`.
- **GoogleTest.** `TEST` defines a test and `TEST_F` one that uses a fixture (a class
  whose `SetUp` runs first; the only inheritance and `override` in the repository);
  `EXPECT_...` records a failure and goes on, `ASSERT_...` stops the test; `EXPECT_NEAR`
  compares floating point within a tolerance; `SCOPED_TRACE` labels failures with, say,
  the trial's number.
  Here: `tests/profile_test.cc`, `tests/step_test.cc`, `tests/gate_test.cc`.
- **`std::getenv`.** Reads an environment variable as C text, or returns a null pointer if
  it is not set, which must be checked.
  Here: `EnvInt` in `tests/gate_test.cc`.
- **Counts, not seconds.** A test bounds a machine-independent count of work (collision
  queries, iterations) and only prints the wall time.
  Here: the statistics tests in `tests/motion_planner_test.cc`.

## Stop 13: Towards a real cell

- **Bringing in a library.** No one package manager: a library is headers plus compiled
  code that must agree with yours on compiler, flags, language version and shared
  dependencies.
  Here: `MODULE.bazel`, `docs/ompl-integration.md`.
- **One definition rule.** A program may hold only one definition of each type and
  function; two versions of one library break it without a diagnosis.
  Here: "One Eigen in the binary" in `docs/ompl-integration.md`.
- **Pinned dependencies.** Exact versions and hashes, so the same source builds the same
  program later.
  Here: `MODULE.bazel`, `bazel/kuka.bzl`.

