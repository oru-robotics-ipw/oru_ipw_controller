# ORU IPW controller

Robot controller that performs some basic safety task on top of the Husqvarna
driver node (`am_driver_safe`).

This node:

* Puts the Husqvarna driver into a known good state (loop detection off, manual
  control, ...)
* Performs keep alive handling on cmd_vel for safety.
* Handles collisions.
* Translates idiosyncratic battery & system status messages to more useful
  information for other components.
* Implements functionality to exit the charging station, as that is non-obvious.

## Topics

Subscribed from front end:

* `cmd_vel` (`geometry_msgs/Twist`) - Commanded driving speed. Keep-alive by
  repeated messages required or the robot will automatically stop.

Published to front end:

* `battery/a` (`sensor_msgs/BatteryState`) - Battery state for battery A
* `battery/b` (`sensor_msgs/BatteryState`) - Battery state for battery B
* `battery/status` (`oru_ipw_controller/SimpleBatteryStatus`) - Simplified
  battery status for web GUI.

Subscribed from Husqvarna driver:
* `hrp/battery_status` (`am_driver/BatteryStatus`) - Low level battery status
* `hrp/current_status` (`am_driver/CurrentStatus`) - Low level system status
* `hrp/sensor_status` (`am_driver/SensorStatus`) - Low level system status

Published to Husqvarna driver:
* `hrp/cmd_mode` (`std_msgs/UInt16`) - Mode control
* `hrp/cmd_vel` (`geometry_msgs/Twist`) - Velocity command

## Services

For usage from front end:

* `exit_charging_station` (`std_srvs/Trigger`) - Trigger an attempt to exit the
  charging station.

Called on the Husqvarna driver:

* `hrp/tif_command` (`am_driver_safe/TifCmd`) - Used to control headlights.

## Parameters

* `~drive_timeout` (`double`, default: 0.5) - Stop if no velocity command
  received within this time period [s]
* `~low_battery_level` (`double`, default: 18.0) - Voltage below which batteries
  are considered low.
