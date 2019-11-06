import events
import rospy
from am_driver.msg import BatteryStatus, SensorStatus
from dynamic_reconfigure.encoding import Config
from oru_ipw_controller.cfg import OruIpwControllerConfig
from oru_ipw_msgs.msg import SimpleBatteryStatus
from sensor_msgs.msg import BatteryState


class BatteryHandler(object, events.Events):
    """Handles the mess that is the battery message types"""

    __events__ = ('on_low_battery',)

    def __init__(self):
        """Constructor"""
        super(BatteryHandler, self).__init__()
        self._cfg = Config(OruIpwControllerConfig.defaults)
        self._battery_status = BatteryStatus()
        self._senor_status = SensorStatus()
        # Frontend communication
        self._battery_a_pub = rospy.Publisher('battery/a',
                                              BatteryState,
                                              latch=True,
                                              queue_size=1)
        self._battery_b_pub = rospy.Publisher('battery/b',
                                              BatteryState,
                                              latch=True,
                                              queue_size=1)
        self._battery_status_pub = rospy.Publisher('battery/status',
                                                   SimpleBatteryStatus,
                                                   latch=True,
                                                   queue_size=1)

        # Driver communication
        self._battery_status_sub = rospy.Subscriber('hrp/battery_status',
                                                    BatteryStatus,
                                                    callback=self._callback_battery_status,
                                                    queue_size=1)
        self._sensor_status_sub = rospy.Subscriber('hrp/sensor_status',
                                                   SensorStatus,
                                                   callback=self._callback_sensor_status,
                                                   queue_size=1)

    def handle_reconfigure(self, config):
        """Handle reconfiguration

        :type config: Config
        """
        self._cfg = config

    def _callback_battery_status(self, msg):
        """Callback from hrp/battery_status

        :type msg: BatteryStatus
        """
        self._battery_status = msg
        low_level = self._cfg['low_battery_level']  # type: float
        a_volt = msg.batteryAVoltage / 1000
        b_volt = msg.batteryBVoltage / 1000
        battery_low = False
        if a_volt < low_level or b_volt < low_level:
            battery_low = True
            self.on_low_battery()

        # Interpret sensor status
        in_cs = self._senor_status.sensorStatus & SensorStatus.SENSOR_STATUS_IN_CS != 0
        charging = self._senor_status.sensorStatus & SensorStatus.SENSOR_STATUS_CHARGING != 0

        # Publish simple battery status for display in web UI
        self._battery_status_pub.publish(SimpleBatteryStatus(
            header=msg.header,
            battery_low=battery_low,
            is_charging=charging,
            is_fully_charged=in_cs and not charging))

        # Publish detailed battery state messages
        supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        if in_cs and charging:
            supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif in_cs and not charging:
            supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif not in_cs:
            supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        def _make_state(location, voltage, current):
            """Create battery state message. Most information is unavailable."""
            nan = float('NaN')
            return BatteryState(
                header=msg.header,
                voltage=voltage,
                current=current,
                charge=nan,
                capacity=nan,
                design_capacity=nan,
                percentage=nan,
                power_supply_status=supply_status,
                power_supply_health=BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN,
                power_supply_technology=BatteryState.POWER_SUPPLY_TECHNOLOGY_LION,
                present=True,
                location=location)

        self._battery_a_pub.publish(_make_state(voltage=a_volt,
                                                current=msg.batteryACurrent / 1000,
                                                location='A'))
        self._battery_b_pub.publish(_make_state(voltage=b_volt,
                                                current=msg.batteryBCurrent / 1000,
                                                location='B'))

    def _callback_sensor_status(self, msg):
        """Callback from hrp/sensor_status

        :type msg: SensorStatus
        """
        self._senor_status = msg
