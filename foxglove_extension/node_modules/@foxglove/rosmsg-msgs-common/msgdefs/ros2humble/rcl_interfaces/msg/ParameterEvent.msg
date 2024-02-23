# This message contains a parameter event.
# Because the parameter event was an atomic update, a specific parameter name
# can only be in one of the three sets.

# The time stamp when this parameter event occurred.
builtin_interfaces/Time stamp

# Fully qualified ROS path to node.
string node

# New parameters that have been set for this node.
Parameter[] new_parameters

# Parameters that have been changed during this event.
Parameter[] changed_parameters

# Parameters that have been deleted during this event.
Parameter[] deleted_parameters

================================================================================
MSG: rcl_interfaces/Parameter
# This is the message to communicate a parameter. It is an open struct with an enum in
# the descriptor to select which value is active.

# The full name of the parameter.
string name

# The parameter's value which can be one of several types, see
# `ParameterValue.msg` and `ParameterType.msg`.
ParameterValue value

================================================================================
MSG: rcl_interfaces/Parameter
# This is the message to communicate a parameter. It is an open struct with an enum in
# the descriptor to select which value is active.

# The full name of the parameter.
string name

# The parameter's value which can be one of several types, see
# `ParameterValue.msg` and `ParameterType.msg`.
ParameterValue value

================================================================================
MSG: rcl_interfaces/Parameter
# This is the message to communicate a parameter. It is an open struct with an enum in
# the descriptor to select which value is active.

# The full name of the parameter.
string name

# The parameter's value which can be one of several types, see
# `ParameterValue.msg` and `ParameterType.msg`.
ParameterValue value

================================================================================
MSG: rcl_interfaces/ParameterValue
# Used to determine which of the next *_value fields are set.
# ParameterType.PARAMETER_NOT_SET indicates that the parameter was not set
# (if gotten) or is uninitialized.
# Values are enumerated in `ParameterType.msg`.

# The type of this parameter, which corresponds to the appropriate field below.
uint8 type

# "Variant" style storage of the parameter value. Only the value corresponding
# the type field will have valid information.

# Boolean value, can be either true or false.
bool bool_value

# Integer value ranging from -9,223,372,036,854,775,808 to
# 9,223,372,036,854,775,807.
int64 integer_value

# A double precision floating point value following IEEE 754.
float64 double_value

# A textual value with no practical length limit.
string string_value

# An array of bytes, used for non-textual information.
byte[] byte_array_value

# An array of boolean values.
bool[] bool_array_value

# An array of 64-bit integer values.
int64[] integer_array_value

# An array of 64-bit floating point values.
float64[] double_array_value

# An array of string values.
string[] string_array_value

