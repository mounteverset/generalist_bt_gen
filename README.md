# generalist_bt_gen

This repository contains a software module which runs a behavior tree for an autonomous mobile robot. 
Whenever the behavior tree executor detects that the BT in its current form has got limitations and can not complete its mission with the available behaviors it will query an LLM in order for it to generate a new subtree which purpose it is to be expand the domain of the robot. The tree then gets relaunched and the robot can finish its task with the new and improved BT guiding its action.

The general idea is to have a given set of BT skills that call ROS 2 services/actions, while a **mission coordinator** node outside the tree talks to the LLM (via `llm_interface`) to decide **which** subtree/entry point should run next. It also triggers tree regeneration when the skill catalog is insufficient. The BT itself remains a pure execution layer.

The way to interact with the system is by chat, either via CLI or via a simple web interface; both talk to the mission coordinator action server.

All “Thinking” (LLM calls, plan validation, regeneration) now happens **outside** the BT. The coordinator gathers context (cameras, GPS, satellite map) via `context_gatherer`, queries LangChain, and then instructs `bt_executor` which subtree to execute. Blackboard updates produced by the LLM are injected before the BT run; no dedicated LLM plugins live inside the tree anymore.

## Dependencies / Tech stack

BehaviorTree.CPP as Behavior Tree framework
ROS 2 Jazzy 
BehaviorTree.ROS2 as behavior tree wrapper
LangChain to use PromptTemplates and Tool Calls / MCP servers
OpenAI API / Gemini API / Claude API / Gemini Robotics 1.5 https://ai.google.dev/gemini-api/docs/robotics-overview
MCP Servers: ROS2 MCP server https://github.com/robotmcp/ros-mcp-server OpenStreetMapMCP https://github.com/wiseman/osm-mcp

## Repo structure

bt_executor -> extends BehaviorTree.ROS2 with needed functionality
robot_actions -> robot specific behavior tree actions like Drive, TakePicture etc.
mission_coordinator -> orchestrates LLM planning + bt_executor goals
llm_interface/context_gatherer -> LangChain

### Possible scenarios where this approach can outperform current methods of behavior tree generation and hence show higher levels of deliberation for the robot:
  - Give commands like
    - "Drive around the lake and take photos every 10m"
      - unclear without context which lake
    - "Explore this field and take pictures of all solitary standing trees."
      - unclear which field, how to explore, finding solitary trees
    - "Cover the whole field and search for a lost key"
      - coverage path planning, search mission
  - Situation specific recoveries like a stuck robot
    - can't proceed to drive with current plan, trying to find an alternative path with visual context of sorrounding and satellite map
  - Can give always suggestions or even create additions to the skill catalogue of composable bt actions
