# Exercise 2

**ID: 211397054**

## The leaves and their LED colors

| Leaf | LED | What it does |
|---|---|---|
| `GoTo` | yellow | Drives to a point, around small obstacles |
| `FollowWall` | magenta | Bug2: follows an obstacle's boundary until it meets the line to the target again, closer than where it started |
| `WalkAround` | cyan | Drives in a random direction for a while, to get unstuck |
| `SearchLight` | orange | Random walk until the next light is seen |
| `Visit` | white | Stands still at a light for 2.5 s and marks it visited |
| `Charge` | purple | Stands still on the charger until the battery is full |
| `StayHome` | brown | Stands still on the charger for 2.5 s at the end |
| `IsLightVisited?` | gray | Condition: was this light already visited? |
| `IsLightKnown?` | pink | Condition: does the robot know where this light is? |
| `IsBatteryLow?` | light gray | Condition: is the battery too low for the way home? |

Conditions answer at once, so in a tick that goes on to an action leaf, that leaf's color is the one shown. No leaf uses red, green or blue, so the camera cannot mistake the robot's LEDs for a light.

## Sources

- **Tutorial 5:** the behavior tree nodes (`Sequence` and `Fallback` follow slide 30, `ReactiveFallback` slide 33), the controller contract, the blackboard, the battery interrupt at the root (slides 32–34), and fusion for obstacle avoidance (slides 23–25).
- **Tutorial 4:** going to a point by P control on the heading (slides 14–16), and the PD wall follower (slides 7–10).
- **Lectures 6 and 7:** the Bug algorithms; `GoTo` + `FollowWall` are Bug2 (Lecture 7, slide 6).
- **ARGoS source code:** the Pi-puck's top wheel speed (`MAX_WHEEL_VELOCITY`) and the rangefinder directions (`ci_pipuck_rangefinders_sensor.cpp`).
- **AI assistance:** developed with the help of an AI coding assistant (Claude, by Anthropic). <!-- TODO: describe which parts, per the course's policy. -->
