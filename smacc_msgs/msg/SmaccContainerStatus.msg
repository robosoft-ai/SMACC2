std_msgs/Header header

# The path to this node in the server
string path

# The initial state description
# Effects an arc from the top state to each one
string[] initial_states

# The current state description
string[] active_states

# A pickled user data structure
# i.e. the UserData's internal dictionary
string local_data

# Debugging info string
string info
