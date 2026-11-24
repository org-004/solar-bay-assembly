# New hire path

Two weeks to go from "I write software" to "I can find my way round a robot cell's
code, change it, and say what the change did". It is written for an engineer who is
fluent in some language, new to C++, new to robotics, and whose maths is rusty.

You were hired to do two things: fix problems in the factory cells that exist, which
run on ROS 2 and MoveIt, and research faster and better ones. This repository is where
the second happens. It is the in-house planner, still being built: a whole cell in
simulation, with perception, planning and control written by hand so that every part
can be read. The path teaches the ideas here, where nothing is hidden, and each stop
says what the same thing looks like in the MoveIt cells.

## How it works

There are fifteen stops. Each has a chapter in this directory, and each chapter has the
same parts:

| Part | What it is for |
|---|---|
| Read | the files and symbols to read, in order |
| The idea | the robotics, from nothing, with the maths refreshed where it is used |
| C++ you meet here | the language features in that code, against languages you know |
| Lab | experiments: predict, change something, run, explain the difference |
| Pitfalls | the mistakes people make |
| In the MoveIt cells | the same piece in the production stack |
| Reading | a foundation, a practice and something current |
| Office hours | what to be ready to explain, and questions to bring |

You do the reading and the lab on your own. Then you hold **office hours** with an AI
agent acting as teacher. It asks you questions, you ask it questions, and neither of
you is marked. Its questions and the results of the labs are in `teacher/`; read those
after a session if you like, not before.

To start a session, today and every day, from the top of the repository:

```
claude "Read new-hire/AGENTS.md and start office hours"
```

The agent keeps a notebook for you (`new-hire/notebook.md`, ignored by git) so that
each session picks up where the last one stopped. Any coding agent that can read files
and run commands will do; give it the same sentence.

## Before the first session

- Install Bazelisk as `bazel` (<https://github.com/bazelbuild/bazelisk>). Stop 0 walks
  through the first build, which takes several minutes.
- A display is needed only for the viewer programs. The recordings in `docs/` stand in.
- Python with matplotlib and numpy is needed only to redraw the README's plots
  (`tools/plots/README.md`). No lab depends on it.

## The path

| Stop | Chapter | What you can do afterwards |
|---|---|---|
| 0 | [Watch it work](00-watch-it-work.md) | run the cell and read its table |
| 1 | [The smallest program](01-smallest-program.md) | read a C++ file; say who owns what |
| 2 | [Where things are](02-where-things-are.md) | chain and invert poses without guessing |
| 3 | [Moving one axis](03-moving-one-axis.md) | explain speed, acceleration and jerk limits |
| 4 | [Motors](04-motors.md) | say why an arm lags its command, and what is done about it |
| 5 | [Pose to joint angles](05-pose-to-joint-angles.md) | explain inverse kinematics and where it breaks |
| 6 | [Seeing](06-seeing.md) | follow a depth image to a module's pose |
| 7 | [The cell and the truth](07-the-cell-and-the-truth.md) | tell what a simulator fakes from what it models |
| 8 | [Will it hit anything](08-will-it-hit-anything.md) | explain a collision check and its margins |
| 9 | [Finding a path](09-finding-a-path.md) | explain a sampling planner and when it fails |
| 10 | [Timing a path](10-timing-a-path.md) | tell a path from a trajectory |
| 11 | [The cycle](11-the-cycle.md) | follow a failure to the step and the reason |
| 12 | [Proving it](12-proving-it.md) | reproduce a failure and judge a change |
| 13 | [Towards a real cell](13-towards-a-real-cell.md) | say what this planner still lacks |
| 14 | [A question of your own](14-a-question-of-your-own.md) | run a small experiment and report it |

Stops 0 to 6 are the first week and 7 to 14 the second. A stop is done when you can
talk through its "be ready to explain" list with the chapter closed.

Three threads run through every stop: how the parts work together, the C++, and the
robotics with its production counterpart. If you want one of them on its own:

- [C++ field guide](cpp-field-guide.md): every language feature, at the stop where it
  first appears.
- [Glossary](glossary.md): the robotics and C++ terms, a line each.
- [Reading list](reading.md): everything the chapters recommend, in one place.

## Rules of the road

They are the repository's (`AGENTS.md`), and they apply to people as much as to agents.

- **Sensor data only.** Control code reads what a real cell could sense. Anything named
  `True...` is for setting up scenes and for scoring.
- **A number comes from a run.** Do not quote a figure you have not reproduced.
- **What "correct" means is the owner's.** Do not loosen a tolerance to make something
  pass.
- **Experiments happen in a throwaway worktree.** `new-hire/AGENTS.md` has the four
  commands. Lab edits are never committed.

## Keeping this material true

For whoever maintains it.

- Chapters point at code by file and symbol name, never by line number.
- The chapters lean on the long comments at the top of the headers. When a header's
  comment changes, the chapter that points at it may need a line changed too.
- Every lab result in `teacher/` carries the date and commit it was measured at. After a
  change that moves the README's numbers, rerun the labs of the stops it touches.
- The "In the MoveIt cells" sections describe ROS 2 and MoveIt in general. What is
  particular to the company's cells belongs in them once somebody who knows writes it.
