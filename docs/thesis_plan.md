# Master Thesis Plan: Context-Aware Behavior Tree Selection and Parametrization for Outdoor Robots Using LLMs

**Supervisor**: Dr. Lennart Trösken  
**Chair**: Prof. Dr. Alexander Koenig  
**Duration**: 6 months (January - June 2026)

---

## Research Question

> **How can Large Language Models be used to transform natural-language mission descriptions and multimodal contextual information into a reliable selection and parametrization of existing Behavior Trees for autonomous outdoor robots?**

### Sub-Questions
1. How to acquire and structure multimodal context in a platform-independent way?
2. How to interpret natural-language mission descriptions using LLMs?
3. How to select suitable predefined BT templates based on mission and robot capabilities?
4. How to integrate and parametrize context data within ROS 2 Behavior Trees?

---

## Timeline Overview (6 Months)

```
Jan         Feb         Mar         Apr         May         Jun
|-----------|-----------|-----------|-----------|-----------|-----------|
[=== Literature Review ===]
            [=== Architecture & Formats ===]
                        [=== Context Gatherer ===]
                                    [=== LLM Integration ===]
                                                [=== Demonstrator ===]
                                                            [=== Eval ===]
[================ Writing (incremental) ================]
```

---

## Phase 1: State of the Art (Jan 6 - Feb 2, 4 weeks)

### Milestone 1: Literature Review

**Topics**:
- **Behavior Trees**: BT.CPP, BehaviorTree.ROS2, comparison with FSMs/HTNs
- **LLM/VLM Mission Interpretation**: Prompt engineering, structured output, tool calling
- **Multimodal Context Gathering**: Sensor fusion, spatial reasoning, external data sources
- **Platform-Independent Outdoor Robotics**: ROS 2 abstraction, ground vs. water robots

**Deliverables**:
- Annotated bibliography (25-35 papers)
- State of the art chapter draft (15-20 pages)

---

## Phase 2: Architecture & Representation (Feb 3 - Mar 2, 4 weeks)

### Milestone 2: Platform-Independent System Architecture

**Design Goals**:
- Modular architecture applicable to ground robot AND BlueBoat USV
- Clear separation: Context Gatherer ↔ LLM Module ↔ BT Executor
- Platform abstraction layer for sensor inputs

**Components**:
```
┌─────────────────────────────────────────────────────────────┐
│                    Natural Language Input                    │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                     LLM Interface                            │
│  • Mission interpretation                                    │
│  • BT template selection                                     │
│  • Parameter extraction                                      │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                    Context Gatherer                          │
│  • Sensor abstraction (GPS, camera, LiDAR, temperature)     │
│  • External sources (maps, APIs, drone images)              │
│  • Platform-neutral JSON output                              │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                    Mission Coordinator                       │
│  • BT template loading                                       │
│  • Blackboard parametrization                                │
│  • Execution monitoring                                      │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                    BT Executor (BehaviorTree.ROS2)           │
└─────────────────────────────────────────────────────────────┘
```

### Milestone 3: Representation Formats

**JSON Schemas**:
1. **Mission Description Schema**: Natural language + structured metadata
2. **Context Snapshot Schema**: Sensor data, GPS, imagery, external sources
3. **BT Parametrization Schema**: Blackboard entries with types and constraints
4. **BT Template Metadata**: Required context, capabilities, platform compatibility

**Deliverables**:
- Architecture diagram
- JSON/XML schema definitions
- Platform abstraction design document
- Methodology chapter draft (10-15 pages)

---

## Phase 3: Context Gatherer (Mar 3 - Mar 30, 4 weeks)

### Milestone 4: Context Gatherer Implementation

**Already Completed** ✅:
- Sensor subscribers (odometry, RGB, depth, battery)
- Requirement handler pattern
- Image saving with URI generation
- JSON context output

**Remaining Work**:
- [ ] GPS/NavSat integration
- [ ] Temperature sensor support (for BlueBoat)
- [ ] External data sources (map tiles, satellite imagery APIs)
- [ ] Platform abstraction layer (ground robot vs. USV)
- [ ] LiDAR point cloud handling (optional)

**Platform-Specific Adaptations**:

| Context Type | Ground Robot | BlueBoat USV |
|--------------|--------------|--------------|
| Position | GPS + Odometry | GPS |
| Vision | RGB Camera | RGB Camera |
| Environment | LiDAR | Sonar (optional) |
| Measurement | N/A | Temperature probe |

**Deliverables**:
- Platform-independent context_gatherer
- Tested on both robot platforms (or simulators)

---

## Phase 4: LLM Integration (Mar 31 - Apr 27, 4 weeks)

### Milestone 5: LLM-Based Mission Interpretation & BT Parametrization

**Already Completed** ✅:
- SelectBehaviorTree service
- CreatePayload service
- Tree metadata schema
- LangChain integration

**Remaining Work**:
- [ ] Mission parsing from natural language
- [ ] BT template matching based on capabilities
- [ ] Parameter extraction (coordinates, frequencies, areas)
- [ ] Confidence scoring and fallback handling
- [ ] Prompt optimization for reliability

**Example Flow**:
```
User: "Survey the shoreline and measure water temperature every 50 meters"
        ↓
LLM extracts:
  - Mission type: survey + measurement
  - Target: shoreline
  - Measurement: temperature
  - Interval: 50m
        ↓
Selects: "waypoint_survey_with_measurement.xml"
        ↓
Parametrizes:
  - waypoints: [computed from shoreline geometry]
  - measurement_type: "temperature"
  - measurement_interval_m: 50
```

**Deliverables**:
- Robust LLM mission interpretation
- BT selection accuracy benchmarks
- Parametrization validation

---

## Phase 5: Demonstrator (Apr 28 - May 25, 4 weeks)

### Milestone 6: End-to-End Implementation

**Demo Mission A: Tree Photography (Ground Robot)**
> "Photograph all individual trees within the marked area"

- Context: GPS boundary, camera, tree detection (optional VLM)
- BT: Navigate to waypoints, detect trees, capture photos
- Platform: Mobile ground robot (A200 or similar)

**Demo Mission B: Temperature Profiling (BlueBoat USV)**
> "Measure water temperature at regular intervals along the shoreline"

- Context: GPS, shoreline geometry, temperature sensor
- BT: Follow shoreline, stop at intervals, log temperature
- Platform: BlueBoat USV

**Testing Plan**:
1. **Simulation**: Gazebo (ground), custom USV sim (water)
2. **Lab validation**: Indoor with mock sensors
3. **Field trials**: Outdoor with real robots (weather permitting)

**Deliverables**:
- Working demonstrator on both platforms
- Video recordings of missions
- Data logs (JSONL format)

---

## Phase 6: Evaluation (May 26 - Jun 15, 3 weeks)

### Milestone 7: Assessment

**Evaluation Criteria**:

| Aspect | Metrics |
|--------|---------|
| **Context Quality** | Accuracy, completeness, latency |
| **BT Selection** | Precision, recall, confidence calibration |
| **Parametrization** | Validity rate, type correctness |
| **Execution** | Success rate, time to completion |
| **Platform Reusability** | Code sharing %, adaptation effort |
| **Robustness** | Failure recovery, edge case handling |

**Experiments**:
1. **Ablation study**: With/without context, with/without LLM
2. **Baseline comparison**: Manual parametrization vs. LLM-driven
3. **Cross-platform test**: Same mission on ground robot and USV

**Deliverables**:
- Evaluation results with statistical analysis
- Results chapter draft (15-20 pages)

---

## Writing Schedule (Parallel Track)

| Period | Writing Focus | Target Pages |
|--------|---------------|--------------|
| Jan | Introduction, motivation | 5-8 |
| Feb | State of the art | 15-20 |
| Mar | Architecture & methodology | 15-20 |
| Apr | Implementation details | 10-15 |
| May | Results & evaluation | 15-20 |
| Jun 1-15 | Conclusion, revision | 5 + revision |
| Jun 16-23 | Final formatting, submission | - |

**Target Thesis Length**: 70-90 pages

---

## Thesis Structure

1. **Introduction** (8 pages)
   - Motivation: Non-expert users, natural language interfaces
   - Research question and sub-questions
   - Contributions
   - Thesis outline

2. **State of the Art** (18 pages)
   - Behavior Trees in robotics
   - LLM/VLM for robotic task planning
   - Multimodal context gathering
   - Platform-independent outdoor robotics

3. **System Architecture** (15 pages)
   - Overall design
   - Platform abstraction
   - Representation formats (schemas)
   - Component interfaces

4. **Implementation** (15 pages)
   - Context Gatherer
   - LLM integration
   - BT selection and parametrization
   - ROS 2 integration

5. **Demonstrator** (10 pages)
   - Demo missions
   - Platform descriptions
   - Execution examples

6. **Evaluation** (15 pages)
   - Experimental setup
   - Results
   - Discussion

7. **Conclusion** (5 pages)
   - Summary
   - Limitations
   - Future work (v2.0 self-repair as outlook)

---

## Key Milestones & Deadlines

| Date | Milestone | Deliverable |
|------|-----------|-------------|
| **Feb 2** | Literature review complete | State of the art draft |
| **Mar 2** | Architecture finalized | Schemas, design docs |
| **Mar 30** | Context Gatherer complete | Platform-independent module |
| **Apr 27** | LLM integration complete | Reliable BT selection |
| **May 25** | Demonstrator complete | Working demos on 2 platforms |
| **Jun 15** | Evaluation complete | Full thesis draft |
| **Jun 23** | Final submission | Thesis PDF |

---

## Current Progress

### Completed ✅
- [x] Basic system architecture
- [x] Context Gatherer with sensor integration
- [x] CreatePayload service (LLM-based)
- [x] SelectBehaviorTree service
- [x] Tree metadata schema

### In Progress 🔄
- [ ] Platform abstraction for USV
- [ ] GPS/temperature sensor integration
- [ ] BT template library

### Not Started ⏳
- [ ] Literature review documentation
- [ ] Demo mission implementation
- [ ] Field trials
- [ ] Evaluation

---

## Next Steps (This Week)

1. Start literature review: 5 key papers on BT + LLM integration
2. Define JSON schemas for mission and context formats
3. Test current implementation with mock GPS data
4. Set up LaTeX thesis template
