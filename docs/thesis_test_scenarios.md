# Thesis Test Scenarios

## Purpose

This document defines the scenarios that should be used to evaluate the thesis:

> How can Large Language Models be used to transform natural-language mission descriptions and multimodal contextual information into a reliable selection and parametrization of existing Behavior Trees for autonomous outdoor robots?

The scenarios below are chosen to test the full chain:

1. natural-language mission understanding
2. context gathering and structuring
3. BT template selection
4. BT parametrization
5. execution robustness in realistic outdoor situations
6. platform independence across ground and water robots

## Design Principles

Each thesis scenario should satisfy these rules:

- It must require more than simple keyword matching.
- It must benefit from multimodal context such as map data, GPS, camera input, or platform capabilities.
- It must be solvable with predefined BT templates and parameters rather than unrestricted BT generation.
- It must produce measurable outputs that can be compared against a baseline.
- It should map cleanly from pre-trials to final end-to-end demonstrations.

## Evaluation Structure

The testing should be split into two layers:

- `Pre-trials`: controlled experiments for isolated capabilities such as BT selection, waypoint generation, or completion detection.
- `Final scenarios`: end-to-end missions that combine interpretation, context use, BT execution, and robustness.

This avoids a weak evaluation where only full demos are shown without understanding which part of the system actually worked.

## Pre-Trials

### 1. Mission-to-BT Selection Baseline

**Goal:** Verify that the LLM selects the correct predefined BT template from natural-language instructions.

**Example prompts**
- "Drive to the inspection point and take one photo."
- "Survey the shoreline and measure water temperature every 50 meters."
- "Explore the marked field and photograph isolated trees."

**What is tested**
- intent recognition
- mapping from language to subtree family
- confidence scoring for ambiguous commands

**Success criteria**
- correct BT template selected
- unsupported missions are rejected or clarified instead of forced into a wrong tree

### 2. Parameter Extraction From Context

**Goal:** Verify that the system derives valid BT parameters from language plus structured context.

**Example prompts**
- "Take photos every 10 meters around the lake."
- "Inspect the northern half of the field."
- "Measure close to the shoreline, but do not enter open water."

**Required context**
- robot pose
- GPS boundary or map geometry
- optional map tiles or satellite imagery

**What is tested**
- extraction of intervals, areas, landmarks, and constraints
- waypoint generation quality
- schema-valid payload creation

**Success criteria**
- generated payload matches the expected schema
- waypoints are inside valid mission areas
- repeated runs stay consistent under equivalent context

### 3. Context Sensitivity and Ambiguity Handling

**Goal:** Show that context improves decisions compared to text-only interpretation.

**Example prompts**
- "Drive around the lake and take photos."
- "Inspect the trees near the path."
- "Search this field for a lost object."

**What is tested**
- ambiguity resolution using maps, robot position, and available sensors
- behavior under different map zoom levels or incomplete context
- whether the system asks for clarification when confidence is too low

**Success criteria**
- context-aware outputs outperform text-only outputs
- confidence drops in ambiguous cases instead of producing overconfident wrong plans

### 4. Open-Ended Completion Detection

**Goal:** Test whether the system can decide when an exploration-style mission is sufficiently complete.

**Example prompts**
- "Explore the area around you."
- "Cover the whole field and look for isolated trees."

**What is tested**
- completion logic based on coverage, detections, or time budget
- operator-in-the-loop stopping decisions
- consistency between planner expectations and execution feedback

**Success criteria**
- missions stop for defensible reasons
- the system avoids both premature stopping and endless execution

## Final Scenarios

### Scenario A: Ground Robot Tree Documentation

**Mission**
- "Explore the marked field and photograph all solitary standing trees."

**Why this scenario matters**
- This is the strongest ground-robot scenario because it requires language interpretation, area understanding, coverage behavior, visual context, and repeated sensing actions.

**Required context**
- mission boundary given in natural language
- robot pose
- camera input
- satellite map or aerial image

**Expected BT capabilities**
- area exploration or coverage
- waypoint following
- tree inspection or photo capture
- mission completion reporting

**Evaluation focus**
- correct selection of exploration/documentation tree
- valid exploration payload
- useful photo capture behavior
- completeness of covered area and detected targets

### Scenario B: Ground Robot Landmark-Based Route Mission

**Mission**
- "Drive around the lake and take photos every 10 meters."

**Why this scenario matters**
- It tests whether the system can ground vague spatial language in map context instead of relying on explicit coordinates.

**Required context**
- GPS pose
- local map or satellite map
- geometry of the lake or drivable perimeter

**Expected BT capabilities**
- route generation around a landmark
- interval-based photo triggering
- navigation recovery if parts of the route are not traversable

**Evaluation focus**
- landmark disambiguation
- waypoint generation quality
- interval correctness
- execution continuity around the target area

### Scenario C: Ground Robot Recovery and Replanning

**Mission**
- Start from Scenario A or B, then inject an obstruction or navigation failure.

**Why this scenario matters**
- The thesis is not only about initial BT selection. It also needs to show that contextual information helps the system react when execution no longer matches the original assumptions.

**Required context**
- failure feedback from BT execution
- local sensor context such as camera, obstacle map, or LiDAR-derived occupancy
- original mission intent

**Expected BT capabilities**
- fallback or recovery subtree selection
- re-routing or alternate viewpoint selection
- graceful failure reporting if recovery is impossible

**Evaluation focus**
- whether failure is correctly classified
- whether a suitable recovery subtree is chosen
- whether the mission continues safely or terminates clearly

### Scenario D: USV Shoreline Temperature Profiling

**Mission**
- "Measure water temperature at regular intervals along the shoreline."

**Why this scenario matters**
- This is the clearest water-platform demonstration of platform-independent BT selection and parametrization.

**Required context**
- GPS pose
- shoreline geometry
- water-safe operating area
- temperature sensor availability

**Expected BT capabilities**
- shoreline-following or waypoint-survey behavior
- interval-based measurement
- logging of sampled values

**Evaluation focus**
- correct BT selection for a water platform
- valid shoreline waypoint generation
- correct sampling intervals
- mission completion with usable measurement data

### Scenario E: Cross-Platform Transfer

**Mission style**
- Use the same high-level command style on both platforms, but with platform-appropriate BT templates and parameters.

**Example pair**
- Ground robot: "Survey the boundary of this area and document points of interest."
- USV: "Survey the shoreline of this area and document points of interest."

**Why this scenario matters**
- The thesis explicitly claims platform-independent context structuring and BT parametrization. That claim needs a direct test, not only two unrelated demos.

**Required context**
- common mission schema
- platform capability metadata
- platform-specific environment geometry

**Expected outcome**
- the same representation pipeline works across both robots
- only the final BT choice and payload details differ by platform

**Evaluation focus**
- reuse of context schema and mission schema
- amount of platform-specific adaptation required
- consistency of operator experience across platforms

## Baselines and Comparisons

Each final scenario should be evaluated against at least one of these baselines:

- manual BT selection and manual parameter entry
- LLM selection without contextual input
- contextual pipeline without recovery logic

This supports an ablation-style evaluation instead of relying only on anecdotal demos.

## Metrics to Record Per Scenario

- BT selection correctness
- payload validity against schema
- waypoint or sampling plan validity
- mission completion rate
- (number of clarification requests)
- time from user command to executable BT payload
- execution time
- number of recoveries needed
- final task quality
  - examples: photos captured, area coverage, temperature samples collected

Scenario Planning
- how many runs per scenario
- success rate per scenario 

