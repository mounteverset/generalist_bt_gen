# Core Mission Review Sheet

This page is a human-readable view of `core_missions.json`. The JSON file is the
machine-readable source of truth.

| ID | Complexity | Platform | Mission | Expected tree | Current status |
|---|---|---|---|---|---|
| S1 | Simple | Husky | One temperature reading at a supplied map-frame waypoint | `temperature_logging.xml` | Implemented catalogue |
| S2 | Simple | BlueBoat | One water-temperature reading at a supplied WGS84 waypoint | `blueboat_temperature_logging.xml` | Implemented catalogue |
| S3 | Simple | Husky | Supplied three-point temperature route | `temperature_logging.xml` | Implemented catalogue |
| M1 | Medium | Husky | Supplied route with photos every 10 m | `navigate_and_photograph.xml` | Implemented catalogue |
| M2 | Medium | BlueBoat | Supplied four-station water-temperature route | `blueboat_temperature_logging.xml` | Implemented catalogue |
| M3 | Medium | Husky | Derive three temperature points from the north-zone context | `temperature_logging.xml` | Implemented catalogue |
| C1 | Complex | Husky | Derive field-coverage waypoints with boundary and obstacle constraints | `explore_area.xml` | Implemented catalogue |
| C2 | Complex | Husky | Derive a roundtrip around Hollerner See from OSM paths and log temperature every 10 m | `gps_temperature_logging.xml` | Implemented catalogue |
| C3 | Complex | Husky | Visit five tree locations supplied by FindAnything | `find_and_drive_to_nearest_object.xml` | Implemented catalogue |

## Important design choices

- The platform split is 7 Husky and 2 BlueBoat missions.
- E5 executes all nine core missions three times, giving 27 physical trials:
  21 Husky trials and 6 BlueBoat trials.
- Complexity is based on six predeclared dimensions. A complex mission does not
  need a newly generated complex tree; context and interacting constraints can
  make parametrization complex even when execution uses a compact fixed tree.
- The core set excludes “explore and photograph target objects” because the
  current executable catalogue does not contain one tree that performs both.
- C1 evaluates a static coverage route derived before execution. It does not
  assume online frontier planning, completion detection, or runtime return-home.
- BlueBoat uses one mission tree in one simple and one medium case.
- C2 uses a frozen OpenStreetMap path loop around Hollerner See. Its canonical
  route interpolates this geometry at approximately 10 m intervals.
- C3 uses five map-frame tree detections returned by FindAnything before
  Behavior Tree execution.
- Reference routes are acceptable examples, not exact-string targets, except
  where the operator supplied fixed coordinates and order in the instruction.

## Decisions required before freeze

1. **Model identifiers selected**: GPT-5.6-Sol-xhigh → `openai/gpt-5.6-sol` (xhigh reasoning), Gemini 3.8 Flash → `google/gemini-3.8-flash`, Gemma 4 26B → `google/gemma-4-26b-a4b-it`. All route through OpenRouter; verify the Gemini 3.8 provider before freeze.
2. Record three physical M3 executions for every core mission as E5;
   the offline mission contract is now implemented and factory-loadable.
3. Review the synthetic map geometry and canonical routes.
4. Review the immutable synthetic map artifacts and replace them only if the
   study claims real-world imagery.
