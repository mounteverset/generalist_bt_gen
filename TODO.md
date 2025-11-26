- Figure out how we handle subtree description saving
    - Who keeps the library of available subtrees? 
    - Are they generated on the fly each time a new request comes? Are they periodically updated?
    - How do we manage to get the description into the parsed XML
    - Also we need an up to date catalogue with very specific details for the available nodes
        - how does the node exepect input strings
        - what is the desired behavior, condition for failures etc. 

- For creating subtrees we need a XML -> JSON parser which is a more LLM friendly format

- How do we populate the payload? How to set the waypoints etc.? 
    - Probably need a context gatherer node pretty soon for this to work
    - How do we manage to use it with MCP Servers?

- Global / local navigation switch implementation on the robot to drive with MAVROS or Nav2

- weighting of different sensor sources -> example: find nearest tree, prefer gps based 

