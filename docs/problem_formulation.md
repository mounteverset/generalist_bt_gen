# Title: Context-Aware Behavior Tree Selection and Parametrization for Outdoor Robots Using LLMs
## Type: Master’s Thesis
### Supervisor: Dr. Lennart Trösken
### Time: TODO
### Chair: Prof. Dr. Alexander Koenig

## Problem Formulation
In mobile outdoor robotics, the demand for autonomous systems capable of reliably performing complex monitoring and exploration missions is steadily increasing. These robots are increasingly
used by domain experts who are not trained roboticists - such as biologists, environmental scientists, or geoscientists. For such users, it is essential that missions can be communicated intuitively, e.g., via natural language, rather than through technical parameters or robotics-specific interfaces. Behavior Trees (BTs) provide a modular, transparent architecture for robust mission execution in safety-critical robotic systems. They are well suited for outdoor scenarios due to their clear control flow, reusability, and fault tolerance. However, mission-relevant parameters - such as target coordinates or measurement frequencies—must still be correctly extracted and inserted into the BT, typically by hand.

Modern Large Language Models (LLMs) and Vision-Language Models (VLMs) enable extraction of mission goals from free-text instructions and interpretation of contextual information from maps, GPS data, LiDAR, camera imagery, or external sources (e.g., drone images, web APIs). Making these capabilities useful in BT-based robotic systems requires a Context Gatherer that collects raw data, unifies and semantically interprets them, and transforms them into structured representations suitable for LLM reasoning and BT parametrization—ideally in a platform-independent manner. In contrast to fully generating BTs via LLMs, the key challenge of this thesis is to structure multimodal  context such that an LLM can reliably select, parameterize, and adapt existing BT structures. The goal is to develop a framework that combines natural-language mission descriptions, multimodal context gathering, and LLM-based interpretation to robustly control outdoor robots across platforms using predefined BTs.

Despite recent advances in LLM- and VLM-based interpretation, reliably integrating such models into Behavior Tree–based robotic control systems remains a non-trivial challenge, particularly when multimodal context information and platform-independent deployment are required. Against this background, the central research question of this thesis is:

"How can Large Language Models be used to transform natural-language mission descriptions and multimodal contextual information into a reliable selection and parametrization of existing Behavior Trees for autonomous outdoor robots?"

To address this question, the thesis investigates the following sub-aspects:
• the acquisition and platform-independent structuring of multimodal, mission-relevant contextual information
• the interpretation of natural-language mission descriptions using Large Language Models
• the selection of suitable predefined Behavior Tree templates based on the interpreted mission and available robot capabilities
• the integration and parametrization of derived context data within existing ROS 2 based Behavior Trees

## Milestones
The general structure and main contents of this thesis are:
1. Overview of the state of the art
Analysis of Behavior Trees, LLM/VLM-based mission interpretation, multimodal context gathering, and platform-independent outdoor robotic systems.
2. Platform-Independent System Architecture
Design of a modular architecture consisting of an LLM module, a Context Gatherer, standardized representation formats, and ROS2 BT integration, applicable to both ground and water surface robots.
3. Representation Formats
Development of platform-neutral JSON/XML schemas for mission parameters, context data, and BT parametrization.
4. Context Gatherer
Implementation of a module for multimodal data collection and abstraction (sensors and external sources) and LLM-compatible context structuring.
5. LLM-Based Mission Interpretation & BT Parametrization
Extraction of mission parameters from natural-language input, selection of suitable BT templates and parametrization or light adaptation of existing BT structures.
6. Implementation of a demonstrator
End-to-end implementation on a mobile ground robot and a BlueBoat USV using selected missions, such as:
(a) photographic documentation of individual trees within a defined area
(b) water temperature profiling near the shoreline of a lake
7. Evaluation and robustness
Assessment of context quality, BT execution performance, platform-independent reusability,
