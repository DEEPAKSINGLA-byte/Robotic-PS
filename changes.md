# Scenario 0 changes: what changed and why

Updated: 2026-09-19, Asia/Kolkata  
Project: `tasks/task1/ackermann_kinodynamic_sim`

This file describes the functional changes currently kept in
`apps/candidate_template.cpp`. Temporary debug logging, captured images, test
scripts, and simulator edits were removed after validation.

## The original failure

The old client sent `Q\n`, called `read()` once, and expected that single read
to contain the complete `CONFIG` and `GRID` responses. TCP is a continuous
ordered byte stream, so a read can contain only part of a message, an early
`TELEMETRY` message, or several messages together. Scenario 0 has a 500 by 500
grid, much larger than the old 16 KB receive buffer.

The direct symptom was `Map: 0x0 resolution: 0m`. The client then planned with
missing map and pose data, so its result was not valid.

## Reliable TCP input and output

`readLine()` keeps an internal `pending` buffer. It appends every socket read
until it finds a newline, returns one complete protocol line, and preserves
extra bytes for the next line. It retries interrupted reads (`EINTR`). This is
used both while waiting for CONFIG and GRID and later for TELEMETRY.

`sendMessage()` loops until all bytes of a `Q`, `TRAJ`, or `CTRL` message have
been sent. It retries interrupted sends and uses `MSG_NOSIGNAL` so a closed
server connection does not terminate the client unexpectedly.

## CONFIG and GRID checks

The client now waits until it has received both CONFIG and GRID. CONFIG must
parse successfully with positive map dimensions and resolution. GRID must have
exactly the number of cells it declares, and that count must equal `rows * cols`.
If any check fails, the client exits instead of starting Hybrid A* with invalid
data. The map variables are initialized, preventing accidental use of
uninitialized values.

## Planned trajectory upload

After initial planning and after every replan, the client sends a `TRAJ`
message containing `x y yaw v` for each path sample. The supplied simulator
already supports this message and draws the path in its visualizer. Vehicle
motion still comes only from `CTRL` commands.

## Hybrid A* changes

The simulator limits steering changes to 1 radian per second. The planner now
uses the same limit while propagating every motion primitive. Previously it
assumed steering could change instantly, which can create paths the simulated
vehicle cannot follow. The state stores the actual rate-limited steering angle;
the requested primitive steering remains in `delta_used` for search cost and
path reconstruction.

The active A* node is copied before adding successors. A vector can reallocate
when a successor is appended, making a reference to its current element invalid.
Copying prevents this undefined behavior.

Each primitive contains five 0.1-second integration steps. Collision checking
checks every step, but the old returned path kept only each primitive endpoint.
The path now contains every checked integration sample, giving the tracker and
visualizer the same dense trajectory that was collision-checked.

## Tracker changes

The controller uses the nearest dense reference sample to calculate cross-track
and heading errors. The lateral steering sign was corrected: if the rear axle
is left of the path, the correction must steer right toward it. It also adds the
next planned steering value as feed-forward, then uses feedback to remove the
remaining error:

```text
steering = planned steering - 0.5 * cross-track error + heading error
```

For reverse segments, heading error is evaluated in the reverse driving
direction. Reverse support exists in the planner and controller, but the
successful Scenario 0 route was forward-only.

## Stopping and replanning

The simulator reports success only when position error is below 0.40 m, yaw
error is below 0.30 rad, and speed is below 0.2 m/s. When the position and yaw
conditions are met, the client commands `CTRL 0 0` so the car actually stops.

When tracking error exceeds the existing replan thresholds, the client stops,
plans from the current telemetry pose, uploads the new trajectory, and restarts
tracking. It also sends a stop before closing the socket and returns success
only after confirmed goal telemetry.

## Scenario 0 result

The cleaned implementation was built with CMake and run against the supplied
Scenario 0 simulator. Both client and simulator confirmed the goal. The run had
no collision, finished with 0.374171 m position error, 0.0763 rad yaw error,
and 0 m/s final speed. It reached the goal 8.45 simulation seconds after the
first client telemetry. No replan was needed.

## What remains to test

Scenario 0 verifies connection, complete map transfer, planning, trajectory
upload, tracking, stopping, collision checking, and the goal check for this
map. It does not prove reverse tracking, direction changes, recovery replanning,
or Scenarios 1 through 3. Those require separate simulator runs.
