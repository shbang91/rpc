#############################################
# A generic metrics message providing statistics for measurements from different sources. For example,
# measure a system's CPU % for a given window yields the following data points over a window of time:
#
#   - average cpu %
#   - std deviation
#   - min
#   - max
#   - sample count
#
# These are all represented as different 'StatisticDataPoint's.
#############################################

# Name metric measurement source, e.g., node, topic, or process name
string measurement_source_name

# Name of the metric being measured, e.g. cpu_percentage, free_memory_mb, message_age, etc.
string metrics_source

# Unit of measure of the metric, e.g. percent, mb, seconds, etc.
string unit

# Measurement window start time
builtin_interfaces/Time window_start

# Measurement window end time
builtin_interfaces/Time window_stop

# A list of statistics data point, defined in StatisticDataPoint.msg
StatisticDataPoint[] statistics

================================================================================
MSG: statistics_msgs/StatisticDataPoint
#############################################
# This holds the structure of a single data point of a StatisticDataType.
#
# This message is used in MetricsStatisticsMessage, defined in MetricsStatisticsMessage.msg.
#
# Examples of the value of data point are
# - average size of messages received
# - standard deviation of the period of messages published
# - maximum age of messages published
#
# A value of nan represents no data is available.
# One example is that standard deviation is only available when there are two or more data points but there is only one,
# and in this case the value would be nan.
# +inf and -inf are not allowed.
#
#############################################

# The statistic type of this data point, defined in StatisticDataType.msg
# Default value should be StatisticDataType.STATISTICS_DATA_TYPE_UNINITIALIZED (0).
uint8 data_type

# The value of the data point
float64 data

