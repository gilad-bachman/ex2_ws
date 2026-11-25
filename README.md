# Exercise 2 — Three Lights on a Battery, with a Behavior Tree

**ID: 211397054**

A Pi-puck controller that visits the red, green and blue lights in the order given in the `.argos` file, recharges whenever the battery gets low (and then carries on where it stopped), and finally comes home. The executive is a behavior tree, written from scratch (no BT libraries).

## Build and run

```sh
./compile_ws.sh
argos3 -c experiments/config2a.argos
argos3 -c experiments/config2b.argos
```

All my code is in `controllers/`. No other directory was changed.

## Files

| File | What it holds |
|---|---|
| `controllers/behavior_tree.hpp` | **The engine.** `EStatus`, the node interface `CNode` (`Tick()`, `Halt()`), and the control nodes: `Sequence`, `Fallback`, `ReactiveFallback`, `ForceFailure`, `RetryUntilSuccessful`. Generic: knows nothing about the robot |
| `controllers/blackboard.hpp` | **The blackboard.** This step's sensor readings, and the mission's memory (where each light is, which were visited) |
| `controllers/actuators.hpp/.cpp` | The wheels and LEDs, plus the movement helpers the leaves use: `Stop`, `DriveTo` (go to a point around obstacles) and `FollowWall` |
| `controllers/leaves.hpp/.cpp` | **The leaves**: the controllers and the conditions |
| `controllers/mission.hpp/.cpp` | **The plan**: builds the subtrees (`VisitLight`, `GetThere`, `Recharge`, `ComeHome`) |
| `controllers/controller2.hpp/.cpp` | The ARGoS controller: fills the blackboard, builds the tree in `Init()`, ticks it once per `ControlStep()` |

## The executive

`ControlStep()` does two things: it writes this step's sensor readings into the blackboard, and it ticks the root of the tree **once**. All the mission's decisions are in the tree.

Writing the blackboard also records **light sightings**: every blob the camera reports in one of the three colors is turned into a world position (`position + distance · (cos, sin)(heading + angle)`, the camera's distance converted from cm to m), whichever leaf is running. This is sensing, not a decision: no leaf is chosen there.

### The tree

```
ReactiveFallback                              checks the battery every tick
├── Recharge (Sequence)                       the interrupt
│   ├── IsBatteryLow?
│   ├── GetThere(home)
│   └── Charge                                until full
└── Mission (Sequence)
    ├── VisitLight(0)                         the first light in the order
    ├── VisitLight(1)
    ├── VisitLight(2)
    └── ComeHome (Sequence)
        ├── GetThere(home)
        └── StayHome                          stand still 2.5 s on the charger

VisitLight(n)  (the same subtree for every light, given its place in the order)
Fallback
├── IsLightVisited?(n)                        done before (e.g. before a recharge): skip
└── Sequence
    ├── Fallback                              find it
    │   ├── IsLightKnown?(n)
    │   └── SearchLight(n)
    ├── GetThere(light n)
    └── Visit(n)                              stand still 2.5 s, mark it visited

GetThere(target)  (Bug2, Lecture 7)
RetryUntilSuccessful                          never gives up
└── Fallback
    ├── GoTo(target)                          along the m-line
    └── ForceFailure                          stuck: get unstuck, then GoTo again
        └── Fallback
            ├── FollowWall(target)            Bug2 boundary following
            └── WalkAround                    Bug2 found no leave point
```

**Carrying on after a recharge.** When `IsBatteryLow?` says yes, the `ReactiveFallback` halts the mission. Once the battery is full, the mission starts again from its first child; the lights already visited answer `IsLightVisited?` with SUCCESS at once, so the robot goes on with the next light.

**Why the Recharge Sequence does not ask again.** Once `IsBatteryLow?` has said yes, the (non-reactive) Sequence goes on with `GetThere(home)` and `Charge` without asking it again. On the charger the threshold is only the 5% reserve, so asking every tick would send the robot away before it has charged.

### The battery threshold

From the battery rule in the README: the battery loses 0.8% every second (`CBattery::DRAIN_PER_SECOND`), and the robot drives at 0.154 m/s, so the straight trip home costs about 5.2% per meter. The real way home is longer (obstacles, going around them), so:

```
low  ⇔  level < 2 × (0.8 / 0.154) × distance_to_home  +  5%
```

## The leaves and their LED colors

The robot's LEDs are in the same medium the camera sees the lights in, so no leaf uses red, green or blue.

| Leaf | LED | One job | SUCCESS | FAILURE |
|---|---|---|---|---|
| `GoTo(target)` | yellow | Drive to a point, around small obstacles (P control on the heading + a push from close obstacles) | within 5 cm of the point | no progress (5 cm closer) for 5 s: the hit point |
| `FollowWall(target)` | magenta | Bug2: follow the obstacle's boundary from the hit point | on the m-line again, closer to the target than the hit point | back at the hit point with no leave point, or 20 s |
| `WalkAround` | cyan | Drive in a random direction for a while, around obstacles | after 6 s | never |
| `SearchLight(n)` | orange | A random walk, a new direction every 5 s | light *n* is known (seen now or earlier) | never |
| `Visit(n)` | white | Stand still at light *n* for 2.5 s (the referee needs 2 s), then mark it visited | 2.5 s still | drifted more than 12 cm from the light |
| `Charge` | purple | Stand still on the charger | battery ≥ 99% | not on the charger |
| `StayHome` | brown | Stand still on the charger for 2.5 s, to come home | 2.5 s still | not on the charger |

**Conditions** answer at once and never drive the robot, so they do not set the LEDs; in the same tick an action leaf always sets them after a condition:

| Condition | SUCCESS when |
|---|---|
| `IsLightVisited?(n)` | light *n* was already visited |
| `IsLightKnown?(n)` | the robot knows where light *n* is |
| `IsBatteryLow?` | below the threshold above |

## Getting around obstacles: Bug2

`GetThere` is Bug2 from Lecture 7 (slide 6). `GoTo` drives along the m-line, the straight line from where it started to the target, and records it. When `GoTo` gets stuck, that is the hit point, and `FollowWall` follows the boundary on one side, leaving only where it meets the m-line again strictly closer to the target than the hit point. If it gets back to the hit point without leaving, it fails (Bug2's failure detection).

Differences from the slide, and why:
- **The hit point** is "GoTo made no progress for 5 s", not contact: `GoTo` already slides around small obstacles by fusion, so wall following is only needed when that fails.
- **The side** is the side the obstacle is seen on (then kept for the whole boundary), not one fixed side: following a wall that is on the other side would circle in place.
- **A 20 s timeout and a `WalkAround` fallback**: Bug2's guarantees assume ideal sensing and a point robot (Lecture 7, slide 9), and the battery limits how long the robot can follow a boundary.

## Testing

Headless runs (the visualization removed from copies of the configs), with other `random_seed`s and all six orders:

| Runs | Score |
|---|---|
| `config2a` and `config2b`, seeds 1–10, their own orders (20 runs) | 100 in all |
| `config2a` and `config2b`, seeds 11–15, all six orders (60 runs) | 100 in all |

Average time: 91 s (`config2a`), 107 s (`config2b`); the slowest run, 402 s, was a long search.

## Sources

- **Tutorial 5 (the executive):**
  - The behavior tree's nodes, as presented there: the `Tick()` of `Sequence` and `Fallback` follows slide 30 (`m_unCurrent` as the memory), and `ReactiveFallback` follows slide 33 (start from the first child every tick; halt the running child when an earlier one takes over).
  - The controller contract (one job per step, RUNNING/SUCCESS/FAILURE, an LED color per leaf), the blackboard, and the battery interrupt as a `ReactiveFallback` at the root (slides 12–13, 32–34).
  - Fusion (slides 23–25): `DriveTo` adds a pull towards the target to a push away from every rangefinder that sees something close.
  - `ForceFailure` and `RetryUntilSuccessful` are from the list of optional nodes in the assignment.
- **Tutorial 4 (control):** going to a point by P control on the heading (heading error wrapped with `SignedNormalize()`, turning in place when it is large: slides 14–16), and the PD wall follower (slides 7–10) used by `FollowWall`.
- **Lectures 6 and 7:** the Bug algorithms; `GetThere` is Bug2 (Lecture 7, slide 6), with the differences listed above.
- **ARGoS source code:** the Pi-puck's top wheel speed (`CCI_PiPuckDifferentialDriveActuator::MAX_WHEEL_VELOCITY`, 0.154 m/s) and the direction each rangefinder faces (from the sensors' positions in `MAP_SENSOR_CONFIG`, `ci_pipuck_rangefinders_sensor.cpp`).
- **AI assistance:** this work was developed with the help of an AI coding assistant (Claude, by Anthropic), which wrote code for the engine, the leaves, the subtrees and the movement helpers following my design decisions, and helped test it. <!-- TODO: check the course's policy on AI-generated code, and describe here exactly which parts are yours and which the assistant's. -->
