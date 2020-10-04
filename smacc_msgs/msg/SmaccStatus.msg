std_msgs/Header header

#returns the list of current states (the complete hierarchy)
#from ancestor active states to descendents active states), ie: [Superstate1, State3]
string[] current_states

string[] global_variable_names
string[] global_variable_values