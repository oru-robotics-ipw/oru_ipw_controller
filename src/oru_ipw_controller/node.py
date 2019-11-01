"""ROS node that handles safety of cmd_vel"""

from __future__ import (print_function, absolute_import, division)

import events
import rospy
from am_driver.msg import CurrentStatus, Mode, SensorStatus, WheelPower
from am_driver_safe.srv import TifCmd
from dynamic_reconfigure.encoding import Config
from dynamic_reconfigure.server import Server as DynReconfigureServer
from geometry_msgs.msg import Twist
from oru_ipw_controller.cfg import OruIpwControllerConfig
from std_msgs.msg import UInt16
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from oru_ipw_controller.battery_handler import BatteryHandler
from oru_ipw_controller.soft_estop import SoftEstop


class Node(object, events.Events):
    """Main node class"""

    __events__ = ('on_reconfigure',)

    def __init__(self):
        """Constructor"""
        super(Node, self).__init__()
        self._collision = False
        self._cfg = Config(OruIpwControllerConfig.defaults)
        self._driving = False
        self._last_cmd_vel_ts = rospy.Time.now()
        self._drive_timout = rospy.Duration(0.5)
        self._current_status = CurrentStatus()
        self._senor_status = SensorStatus()

        # Estop handling
        self._soft_estop = SoftEstop()
        self._soft_estop.on_stop += self._handle_estop
        self._soft_estop.on_resume += self._handle_resume

        # Battery handling
        self._battery_handler = BatteryHandler()
        self.on_reconfigure += self._battery_handler.handle_reconfigure

        # Dynamic reconfiguration
        self._dyn_reconfigure_srv = DynReconfigureServer(OruIpwControllerConfig, self._dynamic_config_callback)

        # Frontend communication
        self._cmd_vel_sub = rospy.Subscriber('cmd_vel',
                                             Twist,
                                             callback=self._callback_cmd_vel,
                                             queue_size=1)
        self._exit_charging_station_srv = rospy.Service('exit_charging_station', Trigger,
                                                        self.handle_exit_charging_station)

        # Driver communication
        self._current_status_sub = rospy.Subscriber('hrp/current_status',
                                                    CurrentStatus,
                                                    callback=self._callback_current_status,
                                                    queue_size=1)
        self._sensor_status_sub = rospy.Subscriber('hrp/sensor_status',
                                                   SensorStatus,
                                                   callback=self._callback_sensor_status,
                                                   queue_size=1)

        self._cmd_vel_pub = rospy.Publisher('hrp/cmd_vel',
                                            Twist,
                                            latch=False,
                                            queue_size=1)
        self._cmd_mode_pub = rospy.Publisher('hrp/cmd_mode',
                                             UInt16,
                                             latch=False,
                                             queue_size=1)
        self._cmd_power_pub = rospy.Publisher('hrp/cmd_power',
                                              WheelPower,
                                              latch=False,
                                              queue_size=1)
        self._tif_cmd_service_proxy = rospy.ServiceProxy('hrp/tif_command', TifCmd)

        # Cyclic timer for heartbeat functionality
        self._timer = rospy.Timer(rospy.Duration(0.5), self._timer_callback)

    def _dynamic_config_callback(self, config, level):
        """Callback for dynamic reconfigure

        :type config: Config
        :param config: New config
        :type level: int
        :param level: Config change level
        :return:
        """
        self._cfg = config
        self.on_reconfigure(config)
        self._drive_timout = rospy.Duration(config['drive_timeout'])
        return config

    def _timer_callback(self, event):
        """Main timer callback

        :type event: rospy.timer.TimerEvent
        :param event: Timer event
        """
        if self._driving and event.current_real - self._last_cmd_vel_ts > self._drive_timout:
            self.stop()

    def _callback_cmd_vel(self, msg):
        """Callback from cmd_vel

        :type msg: Twist
        """
        self._driving = msg.angular.z != 0 or msg.linear.x != 0
        if not self._collision and not self._soft_estop.stopped:
            self._last_cmd_vel_ts = rospy.Time.now()
            self._cmd_vel_pub.publish(msg)

    def _handle_estop(self):
        """Actions to perform when enabling soft estop"""
        self.stop()
        self._send_mode(Mode.MODE_SOUND_FAULT)

    def _handle_resume(self):
        """Actions to perform when disabling soft estop"""
        self._send_mode(Mode.MODE_SOUND_LONG_BEEP)

    def _callback_current_status(self, msg):
        """Callback from hrp/current_status

        :type msg: CurrentStatus
        """
        self._current_status = msg

    def _callback_sensor_status(self, msg):
        """Callback from hrp/sensor_status

        :type msg: SensorStatus
        """
        self._senor_status = msg
        # Collision handling
        was_collision = self._collision
        self._collision = msg.sensorStatus & SensorStatus.SENSOR_STATUS_COLLISION != 0
        if self._collision:
            self.stop()
        if self._collision != was_collision:

            if self._collision:
                rospy.logerr("Collision")
                self._send_mode(Mode.MODE_SOUND_FAULT)
            else:
                rospy.loginfo("Collision ended")
            self.set_headlights(self._collision)

        # Enforce sensible state for robot
        if msg.sensorStatus & SensorStatus.SENSOR_STATUS_LOOP_ON:
            self._send_mode(Mode.MODE_LOOP_OFF)
            rospy.loginfo("Turned off loop detection")
        if msg.sensorStatus & SensorStatus.SENSOR_STATUS_DISC_ON:
            self._send_mode(Mode.MODE_CUTTING_DISC_OFF)
            rospy.loginfo("Turned off cutting disc")

        # As long as the robot is not in the charging station ensure it is in manual control mode.
        # In the charging station, this value is more complicated, see exit_charging_station() for more details.
        if not (msg.sensorStatus & SensorStatus.SENSOR_STATUS_IN_CS):
            if msg.operationalMode != SensorStatus.OPERATIONAL_MODE_MANUAL:
                self._send_mode(Mode.MODE_MANUAL)
                rospy.loginfo("Switching to manual mode (from %r)" % msg.operationalMode)

    def _send_mode(self, mode_value):
        """Send mode command to robot

        :type mode_value: int
        :param mode_value: Mode value to send
        """
        mode = UInt16()
        mode.data = mode_value
        self._cmd_mode_pub.publish(mode)

    def stop(self):
        """Send command to stop"""
        twist = Twist()
        twist.linear.x = twist.angular.z = 0
        self._cmd_vel_pub.publish(twist)

        wheel_power = WheelPower()
        wheel_power.left = wheel_power.right = 0
        self._cmd_power_pub.publish(wheel_power)
        self._driving = False

    def set_headlights(self, status):
        """Set headlights off/on

        :type status: bool
        :param status: True to on headlights, false to turn them off
        """
        self._tif_cmd_service_proxy(str="SystemSettings.SetHeadlightEnabled(headlight:%d)" % (not status))

    def handle_exit_charging_station(self, req):
        """Exit the charging station

        :type req: TriggerRequest
        :rtype: TriggerResponse
        """
        # We can't exit the charging station on our own. As a workaround we switch to random mode and let it back us
        # out. Note that this will not do anything if the charge is low, in that case we need the user to physically
        # drag the robot out.
        if self._soft_estop.stopped:
            reason = "Soft emergency stop engaged. Not exiting charging station!"
            rospy.logwarn(reason)
            return TriggerResponse(success=False, message=reason)
        if not (self._senor_status.sensorStatus & SensorStatus.SENSOR_STATUS_IN_CS):
            reason = "Exiting charging station requested, but we aren't in it."
            rospy.logwarn(reason)
            return TriggerResponse(success=False, message=reason)
        if self._senor_status.sensorStatus & SensorStatus.SENSOR_STATUS_CHARGING:
            reason = "Can't automatically exit charging station when not fully charged."
            rospy.logerr(reason)
            return TriggerResponse(success=False,
                                   message=reason)
        self._send_mode(Mode.MODE_RANDOM)
        return TriggerResponse(success=True, message="Exiting charging station")


def main():
    """Main program entry point"""
    rospy.init_node("oru_ipw_controller")

    ros_node = Node()

    # Attempt to be safe, though message publication is not guaranteed in on_shutdown
    rospy.on_shutdown(ros_node.stop)

    # Spin on ROS message bus
    rospy.spin()


if __name__ == '__main__':
    main()
