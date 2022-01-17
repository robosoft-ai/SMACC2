std_msgs/Header header

# The path to this node in the server
string path

# The children of this node
string[] children

# The outcome edges
# Each index across these arrays denote one edge
string[] internal_outcomes
string[] outcomes_from
string[] outcomes_to

# The potential outcomes from this container
string[] container_outcomes
