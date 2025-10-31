# generalist_bt_gen

This repository contains a software module which runs a behavior tree for an autonomous mobile robot. 
Whenever the behavior tree executor detects that the BT in its current form has got limitations and can not complete its mission with the available behaviors it will query an LLM in order for it to generate a new subtree which purpose it is to be expand the domain of the robot. The tree then gets relaunched and the robot can finish its task with the new and improved BT guiding its action.

The general idea is to have a given set of actions that are calling ROS 2 services and actions, and the LLM has to craft a BT that is able to fulfill the commands it receives.

The way to interact with the BT executor and give it tasks is by chat, either via CLI or via a simple web interface. 

Inside of the behavior tree exists a central "Thinking node" which passes the command that the robot should do to a LLM and it will return an enum to choose which subtree to execute. Also this Thinking node is the first line of detection if the robot is capable of exhibiting the desired behavior. If the LLM reasons that from the available subtrees none can accomplish the desired behavior it will exit the subtree and the LLM BT update step is triggered.

The special thing about the BT generation step is that it can gather additional context from the cameras, GPS position and satellite map.

Another special node inside the tree is a LLM querying node to update blackboard values, like to populate lists of waypoints, navigation goals, etc. for the BT subtrees to be used in versatile ways. 

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
   
# How to start the tree
`ros2 run husky_behavior_tree bt_main --ros-args -p tree_xml_file:=$(ros2 pkg prefix husky_behavior_tree)/share/husky_behavior_tree/config/husky_bt.xml`