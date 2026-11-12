Submission date: 1.12.26 (23:59)
# Prerequisites
make sure argos3 is installed:
```sh
argos3 -q all
```
the expected output is a printout of all available plugins (there are a lot).
- if argos isn't installed - refer to https://github.com/OmriPer/pipuck_sim/blob/main/docs/installation.md
# Exercise Instructions
Write a controller that visits three lights (red, green and blue) **in a given order**, and then comes back home, without running out of battery.

The robot starts on its charger. The lights are somewhere in an arena full of obstacles, and some of them are hidden behind obstacles. The battery does not last for the whole mission, so the robot will usually have to go back to the charger in the middle, and then **carry on from where it stopped**.

The executive of your robot must be a **behavior tree** that you write yourself, as in Tutorial 5.

## The mission
The referee (the loop functions in the workspace) checks:
- **A visit**: the robot stands still (slower than 1 cm/s) with its center within **15 cm** of the light, for **2 consecutive seconds**.
- **The order**: set in the `.argos` file (below). Only a visit to the next light in the order counts. Visiting a light out of order counts for nothing.
- **Coming home**: after all three visits, the robot stands still within 15 cm of the charger for 2 seconds.

The run ends after **10 minutes**, when the battery is empty, or when the robot is home with all three lights visited.

**The score of a single run = 25 for each light visited in order + 25 for coming home** (at most 100).

## The world
- The arena is always 4 m x 4 m, centered at (0, 0), with walls around it. Inside there are obstacles, 30 cm tall: they block the robot, and **hide the lights from its camera**.
- The robot starts at a random free configuration. **That position is the charger.**
- The three lights are at random free positions, at least 1 m from each other and from the charger, and at most 3.5 m from the charger.
- The obstacles, the start and the lights are different in the test setup. **Don't hard-code** any of them.
- **The order of the lights** is a parameter in the controller's `<params>` tag in the `.argos` file, for example `<params order="blue,red,green" />`. The referee reads the same parameter. The test runs use other orders, so read it; don't hard-code it. The starting code already reads it into `m_vecOrder` (see `sensors/light_order.hpp`): `m_vecOrder[0]` is the first light, with its name and the color the camera reports for it.
- The robot has 10 minutes of simulated time.

## The battery
The simulated Pi-puck has no battery, so the referee runs one for it, by the rule in `sensors/battery.hpp`. **Don't modify that file.**
- It starts full: **100%**.
- It loses **0.8% every second**, moving or not, so a full battery lasts 125 seconds.
- While the center of the robot is within **15 cm of the charger** (the same zone where coming home counts), it also gains **5% every second**: a full charge from empty takes about 24 seconds.
- At **0%** the robot is dead and the run ends.

Before every `ControlStep()`, the referee writes the battery into your controller, which gets it by inheriting from `CBatteryEquipped` (the starting code already does; keep it). Read the level, in percent, with `Battery().Level()`, and where the charger is with `Battery().Home()`.

## Sensors
- **Rangefinders**: 8 of them around the robot, each giving a distance in meters, up to 0.2 (20 cm). Their labels, seen from above, with the robot's front up:
  ```
              7   0
           6         1
          5           2
             4     3
  ```
  Read them with `m_pcRangefinders->Visit([&](const CCI_PiPuckRangefindersSensor::SInterface& s) { ... s.Label ... s.Proximity ... });`
- **Camera**: an omnidirectional camera that sees the lights as colored blobs, each with an angle relative to the robot's front (left is positive) and a distance **in centimeters**. It sees about 2.5 m, and not through obstacles. Each blob in `m_pcCamera->GetReadings().BlobList` has a `Color` (compare it with `m_vecOrder[i].Color`), an `Angle` and a `Distance`.
- **Positioning**: the robot's exact position and heading, as in Tutorial 4.
- **The battery**: above.
- **The clock**: `m_pcSystem->GetTime()`, in seconds.

Don't add sensors to the `.argos` file, and don't change it except for `random_seed` and `order`.

Don't access the simulator (`CSimulator`, the space, or the loop functions) from your controller: your robot knows only what its sensors tell it.

The robot does **not** get a map, and you don't need one: the exercise is about the executive, not about planning.

## Controller Design
1. **Your own behavior tree.** Write the engine yourself: a node interface with `Tick()` returning `SUCCESS`, `FAILURE` or `RUNNING` (and a way to halt a running node), and at least `Sequence`, `Fallback` and `ReactiveFallback`. You may add more nodes (see below). No behavior-tree libraries. You may read the Tutorial 5 code, but write your own, and say in your README what you took from where.
2. **The executive ticks the tree once per `ControlStep()`.** All the mission logic is in the tree, not in `if`s around it.
3. **Controllers follow the contract** from Tutorial 5: each leaf does one job, one step per tick, and reports `SUCCESS` ("I'm done") or `FAILURE` ("I can't") when it is.
4. **Each leaf lights the LEDs in its own color.** List the leaves and their colors in your README.

Some more nodes you may find useful (optional; write them yourself too):
- **`ReactiveSequence`**: like `Sequence`, but starts from the first child on every tick, and halts a running later child when an earlier one no longer succeeds. Good for a guard that must keep holding, for example `ReactiveSequence(BatteryOK?, Mission)`.
- **`Inverter`**: swaps its child's `SUCCESS` and `FAILURE` (`RUNNING` stays `RUNNING`).
- **`ForceFailure`** / **`ForceSuccess`**: returns `FAILURE` / `SUCCESS` whenever its child finishes, whatever the child returned.
- **`RetryUntilSuccessful(n)`** / **`Repeat(n)`**: runs its child again when it fails / succeeds, up to `n` times.

# Work Space Structure
The `ex2_ws` Repository - https://github.com/OmriPer/ex2_ws
contains 4 directories:
- **`controllers/`**: Contains the source code for robot controllers.
- **`experiments/`**: Contains the `.argos` XML configuration files for simulations.
- **`loop_functions/`**: Contains the referee, and the code that draws the lights' rings, the charger and the battery level.
- **`sensors/`**: Contains the battery, and the code that reads the order of the lights.

You only need to write your code under the `controllers/` directory. Add your source files to `controllers/CMakeLists.txt`. \
**Don't modify** any other directory - so your setup is identical to the test setup.
# Rules
1. Make sure your code is tidy and well-documented.
2. Make sure your ID is written at the top of any file you modify.
3. Name any source you used in your README file.
# Files to submit
1. You should submit your project in the same structure as `ex2_ws` (*zipped*).
2. Add a README file with your ID, your leaves and their LED colors, and a list of any external source you used.
3. **Do not submit any executable files, or any files that can be regenerated** (for example the `build/` directory).
# Self Test
For self-examination you can:
1. compile your workspace
`./compile_ws.sh`
2. run the two example arenas
`argos3 -c experiments/config2a.argos`
`argos3 -c experiments/config2b.argos`
- if this process fails, it will probably fail in the test environment as well.

The referee prints the result in the ARGoS log window when the run ends.

The window draws a ring around each light where a visit counts (filled once it is visited), a yellow ring around the charger (15 cm), where the robot charges and where coming home counts, and the battery level above the robot.

To get another start and other places for the lights, change `random_seed` at the top of the `.argos` file; for another order, change `order`. Your controller will be tested with several seeds and orders, in arenas you have not seen.
# Grading
| Part | Weight |
|---|---|
| Average score over the test runs | 60% |
| Your own behavior tree: the engine, the tree, and the other requirements of Controller Design | 30% |
| Tidy and documented code, and a README | 10% |

The referee counts the visits, coming home and the battery. The behavior tree and the other design requirements are checked manually, from your code and from the robot in the simulator.
