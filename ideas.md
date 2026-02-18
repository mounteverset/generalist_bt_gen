- inside of the user interface you have a checkmark which lets you decide if you
want to how to play the subtree, did it generate it or let it let the user
confirm the execution -> circling back with the user feedback when rejection happened
- We need a sort of feedback analyzer/completion checker that is constantly
checking the feedback,or maybe sometimes checking the feedback from the
action server which the tree is executed in which then decides when the mission
is finished. For example open-ended missions like "explore the area around you"
may not have a clear cut goal when they are finished but we need to have an
instance that is being able to hold the execution of the tree to say when it is
finished.
- instead of only relying on langchain to be able to make tool calls and generate an answer. If it fails after let's say two tries to generate a suitable tree, let's use an LLM CLI tool
which has a longer runtime but may produce better results because it is able to scrape and dynamically open files in the workspace.
- The XML generator prompt for a new tree should be specifically tasked to NOT create new action nodes and then build the package to get new actions into the tree because in a sense we want to control the actions to makethem be as safe as normally written nodes.
- The payload generator prompt must include to not have nested JSON inside it. Just one key value level.
- Ability to describe parameters and set them via the first user command. Like "just do it dont ask me again" gets interpreted as autorunning the bt. or exploration without asking the user if he is satisfied
- Feedback directly before selecting a subtree to run, if the confidence score is too low to better understand the user mission if the intent is unclear.
- Circle back and ask questions if subtree is unclear
- regularly create up to date catalogues of actions/trees, maybe when starting the whole stack we first run a parser, that updates the catalogue of available trees 
- context_gatherer checks the available ros topics and matches them to the respective requirement
- we need to enforce the outcome generated payload to fit the expected formar by the BT
    - compare llm interface -> to compare the generated payload with the required output format and other requirements, like number of waypoints from the initial prompt
- pre-trials of outputs with specific input data 
    - how well does an llm perform with relation to the goal we give it 
    - can it generate waypoints from gps tile map and gps pose?
    - if something doesnt work we can relate to these pre-trials to start a discussion
- save history of past missions to better e.g. byte_rover, memori
