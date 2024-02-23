# This message contains descriptors of a parameter event.
# It was an atomic update.
# A specific parameter name can only be in one of the three sets.

ParameterDescriptor[] new_parameters
ParameterDescriptor[] changed_parameters
ParameterDescriptor[] deleted_parameters

================================================================================
MSG: rcl_interfaces/ParameterDescriptor
# This is the message to communicate a parameter's descriptor.

# The name of the parameter.
string name

# Enum values are defined in the `ParameterType.msg` message.
uint8 type

# Description of the parameter, visible from introspection tools.
string description

# Parameter constraints

# Plain English description of additional constraints which cannot be expressed
# with the available constraints, e.g. "only prime numbers".
#
# By convention, this should only be used to clarify constraints which cannot
# be completely expressed with the parameter constraints below.
string additional_constraints

# If 'true' then the value cannot change after it has been initialized.
bool read_only false

# If true, the parameter is allowed to change type.
bool dynamic_typing false

# If any of the following sequences are not empty, then the constraint inside of
# them apply to this parameter.
#
# FloatingPointRange and IntegerRange are mutually exclusive.

# FloatingPointRange consists of a from_value, a to_value, and a step.
FloatingPointRange[<=1] floating_point_range

# IntegerRange consists of a from_value, a to_value, and a step.
IntegerRange[<=1] integer_range

================================================================================
MSG: rcl_interfaces/ParameterDescriptor
# This is the message to communicate a parameter's descriptor.

# The name of the parameter.
string name

# Enum values are defined in the `ParameterType.msg` message.
uint8 type

# Description of the parameter, visible from introspection tools.
string description

# Parameter constraints

# Plain English description of additional constraints which cannot be expressed
# with the available constraints, e.g. "only prime numbers".
#
# By convention, this should only be used to clarify constraints which cannot
# be completely expressed with the parameter constraints below.
string additional_constraints

# If 'true' then the value cannot change after it has been initialized.
bool read_only false

# If true, the parameter is allowed to change type.
bool dynamic_typing false

# If any of the following sequences are not empty, then the constraint inside of
# them apply to this parameter.
#
# FloatingPointRange and IntegerRange are mutually exclusive.

# FloatingPointRange consists of a from_value, a to_value, and a step.
FloatingPointRange[<=1] floating_point_range

# IntegerRange consists of a from_value, a to_value, and a step.
IntegerRange[<=1] integer_range

================================================================================
MSG: rcl_interfaces/ParameterDescriptor
# This is the message to communicate a parameter's descriptor.

# The name of the parameter.
string name

# Enum values are defined in the `ParameterType.msg` message.
uint8 type

# Description of the parameter, visible from introspection tools.
string description

# Parameter constraints

# Plain English description of additional constraints which cannot be expressed
# with the available constraints, e.g. "only prime numbers".
#
# By convention, this should only be used to clarify constraints which cannot
# be completely expressed with the parameter constraints below.
string additional_constraints

# If 'true' then the value cannot change after it has been initialized.
bool read_only false

# If true, the parameter is allowed to change type.
bool dynamic_typing false

# If any of the following sequences are not empty, then the constraint inside of
# them apply to this parameter.
#
# FloatingPointRange and IntegerRange are mutually exclusive.

# FloatingPointRange consists of a from_value, a to_value, and a step.
FloatingPointRange[<=1] floating_point_range

# IntegerRange consists of a from_value, a to_value, and a step.
IntegerRange[<=1] integer_range

================================================================================
MSG: rcl_interfaces/FloatingPointRange
# Represents bounds and a step value for a floating point typed parameter.

# Start value for valid values, inclusive.
float64 from_value

# End value for valid values, inclusive.
float64 to_value

# Size of valid steps between the from and to bound.
# 
# Step is considered to be a magnitude, therefore negative values are treated
# the same as positive values, and a step value of zero implies a continuous
# range of values.
#
# Ideally, the step would be less than or equal to the distance between the
# bounds, as well as an even multiple of the distance between the bounds, but
# neither are required.
#
# If the absolute value of the step is larger than or equal to the distance
# between the two bounds, then the bounds will be the only valid values. e.g. if
# the range is defined as {from_value: 1.0, to_value: 2.0, step: 5.0} then the
# valid values will be 1.0 and 2.0.
#
# If the step is less than the distance between the bounds, but the distance is
# not a multiple of the step, then the "to" bound will always be a valid value,
# e.g. if the range is defined as {from_value: 2.0, to_value: 5.0, step: 2.0}
# then the valid values will be 2.0, 4.0, and 5.0.
float64 step

================================================================================
MSG: rcl_interfaces/IntegerRange
# Represents bounds and a step value for an integer typed parameter.

# Start value for valid values, inclusive.
int64 from_value

# End value for valid values, inclusive.
int64 to_value

# Size of valid steps between the from and to bound.
#
# A step value of zero implies a continuous range of values. Ideally, the step
# would be less than or equal to the distance between the bounds, as well as an
# even multiple of the distance between the bounds, but neither are required.
#
# If the absolute value of the step is larger than or equal to the distance
# between the two bounds, then the bounds will be the only valid values. e.g. if
# the range is defined as {from_value: 1, to_value: 2, step: 5} then the valid
# values will be 1 and 2.
# 
# If the step is less than the distance between the bounds, but the distance is
# not a multiple of the step, then the "to" bound will always be a valid value,
# e.g. if the range is defined as {from_value: 2, to_value: 5, step: 2} then
# the valid values will be 2, 4, and 5.
uint64 step

