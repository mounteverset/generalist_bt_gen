# Core Mission Review Sheet

This page is a human-readable view of `core_missions.json`. The JSON file is the
machine-readable source of truth.

| ID | Complexity | Platform | Mission | Expected tree | Current status |
|---|---|---|---|---|---|
| S1 | Simple | Husky | One temperature reading at P1 | `temperature_logging.xml` | Implemented catalogue |
| S2 | Simple | BlueBoat | One water-temperature reading at W1 | `blueboat_temperature_logging.xml` | Blocked: BlueBoat implementation |
| S3 | Simple | Husky | Supplied three-point temperature route | `temperature_logging.xml` | Implemented catalogue |
| M1 | Medium | Husky | Supplied route with photos every 10 m | `navigate_and_photograph.xml` | Implemented catalogue |
| M2 | Medium | BlueBoat | Supplied four-station water-temperature route | `blueboat_temperature_logging.xml` | Blocked: BlueBoat implementation |
| M3 | Medium | Husky | Derive three temperature points from the north-zone context | `temperature_logging.xml` | Implemented catalogue |
| C1 | Complex | Husky | Derive field-coverage waypoints with boundary, obstacle, and battery admission constraints | `explore_area.xml` | Implemented catalogue |
| C2 | Complex | BlueBoat | Derive a shoreline temperature route with interval, offset, geofence, exclusion, and duration constraints | `blueboat_temperature_logging.xml` | Blocked: BlueBoat implementation |
| C3 | Complex | BlueBoat | Derive an east-shore route when satellite context is unavailable | `blueboat_temperature_logging.xml` | Blocked: BlueBoat implementation |

## Important design choices

- The platform split is 5 Husky and 4 BlueBoat missions.
- Complexity is based on six predeclared dimensions. A complex mission does not
  need a newly generated complex tree; context and interacting constraints can
  make parametrization complex even when execution uses a compact fixed tree.
- The core set excludes “explore and photograph target objects” because the
  current executable catalogue does not contain one tree that performs both.
- C1 evaluates a static coverage route derived before execution. It does not
  assume online frontier planning, completion detection, or runtime return-home.
- BlueBoat uses one proposed equivalent mission tree across four difficulty
  levels. This keeps platform comparison focused, but the tree and interface
  must exist before the dataset can be frozen.
- Reference routes are acceptable examples, not exact-string targets, except
  where the operator supplied fixed named points and order.

## Decisions required before freeze

1. Confirm that `blueboat_temperature_logging.xml` is the intended BlueBoat
   mission contract, or replace it consistently before implementation.
2. Review the synthetic map geometry and canonical routes.
3. Replace placeholder map images with immutable artifacts and hashes.
4. Confirm the three general-purpose model identifiers.
5. Decide whether S1, M1, C1, S2, and C2 are the final execution subset.
