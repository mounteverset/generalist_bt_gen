# E5 physical execution runbook

## Frozen study design

- Method: M3
- Planning condition: GPT-5.6-Sol, P2
- Missions: S1, S2, S3, M1, M2, M3, C1, C2, C3
- Repetitions: three physical executions of one accepted plan per mission
- Platforms: Husky for seven missions and BlueBoat for two missions
- Primary trial count: 27
- Simulation evidence is excluded.

## Before protocol freeze

1. Capture the physical test-site context and retain its source, timestamp, frame,
   resolution, and SHA-256.
2. Replace the three placeholder artifacts for M3 and C1.
3. Verify the Husky map frame, WGS84 transform, camera topic, temperature service,
   Nav2 actions, emergency stop, and mission-specific routes.
4. Verify BlueBoat GPS, geofence, MAVROS HOLD and GUIDED modes, arming, global
   setpoints, probe-depth action, temperature service, communication-loss behavior,
   and emergency stop.
5. Run an excluded physical shakedown for each platform.
6. Freeze E1-E4, generate one M3 P2 plan per mission, and admit only plans that pass
   the frozen planning assessment.
7. Copy the accepted route, measurement, and photograph counts into
   `execution_scoring.json`, set it to `frozen`, and retain its SHA-256.

## Required live ROS graph

Husky trials require `/bt_executor/execute_tree`, `/navigate_to_pose`,
`/follow_gps_waypoints`, `/log_temperature`, the configured odometry and camera
topics, localization, and the platform emergency controls.

BlueBoat trials require `/bt_executor/execute_tree`, MAVROS GPS and state topics,
mode and arming services, the global-setpoint topic, `/green/stepper/set_depth`,
`/green/read_temp_cached`, and the platform emergency controls.

## Trial order and reset

Run simple, medium, then complex missions on each platform. Before every repetition:

1. Return the platform to the recorded start state without modifying the accepted
   mission plan.
2. Confirm localization, transforms, services, action servers, output directories,
   storage, communication, weather limits, exclusion zones, and emergency controls.
3. Record the operator and observer decision to start.
4. Start evidence capture before sending the action goal.

Abort on boundary violation, collision or contact, unsafe motion, emergency stop,
communication loss, invalid localization, or loss of operator control. Record the
trial as started with the corresponding terminal outcome, intervention, and incident.

## Evidence layout

Store every trial under:

`evaluation/results/e5_physical_<date>/raw/<trial_id>/`

Retain:

- accepted planning condition and immutable request/result artifacts;
- context snapshot and attachment hashes;
- selected tree and exact payload;
- action goal, feedback, result, timestamps, and duration;
- ROS bag and node logs;
- temperature log and photographs where applicable;
- reached-waypoint evidence;
- operator interventions and safety incidents;
- completed `execution_trials.json` entry.

Use trial IDs `E5-<mission>-physical-r<repetition>`, for example
`E5-S1-physical-r1`. A planned trial that cannot start belongs in `not_started`
with its reason. It must not be recorded as a failed started trial.
