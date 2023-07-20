# ros2_temperature_tracker
ROS 2 Package to monitor and publish CPU

## Parameters

### publish_cpu_temperature
Enables CPU temperature publishing.

>Note: This works by reading directly from thermal files in the system. It has only been tested on Ubuntu 20.04.


### cpu_type_id
The string value found in /sys/class/thermal/thermal_zone*/type

>Note: Usually "x86_pkg_temp" represents the CPU. This may be different depending on CPU architecture.


### cpu_output_topic

The string value representing the desired output topic name for the CPU temperature to be published on


### publish_rate

Time in seconds for the temperatures to be published
