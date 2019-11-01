import events
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class SoftEstop(object, events.Events):
    """Handles various ways to trigger ESTOP. Actual stopping is handled by event handlers in main Node class."""

    __events__ = ('on_stop', 'on_resume')

    def __init__(self):
        super(SoftEstop, self).__init__()
        self.stopped = False
        # Subscribe to joystick to handle buttons.
        self._joy_sub = rospy.Subscriber('joy/joy',
                                         Joy,
                                         callback=self._callback_joy,
                                         queue_size=1)
        self._estop_pub = rospy.Publisher('soft_estop/get',
                                          Bool,
                                          latch=True,
                                          queue_size=1)
        self._estop_sub = rospy.Subscriber('soft_estop/set',
                                           Bool,
                                           callback=self._callback_estop,
                                           queue_size=1)
        self._estop_pub.publish(Bool(False))

    def _callback_joy(self, msg):
        """Joystick/gamepad callback

        :type msg: Joy
        """
        if len(msg.buttons) >= 2:
            if msg.buttons[1] == 1:
                self._set_estop(True)
            elif msg.buttons[0] == 1 and msg.buttons[6] == 1:
                self._set_estop(False)

    def _callback_estop(self, msg):
        """Estop callback

        :type msg: Bool
        """
        self._set_estop(msg.data)

    def _set_estop(self, enabled):
        """Enable/disable soft estop"""
        if enabled:
            self.stopped = True
            self.on_stop()
            self._estop_pub.publish(Bool(True))
            rospy.logerr("Estop activated")
        else:
            self.stopped = False
            self.on_resume()
            self._estop_pub.publish(Bool(False))
            rospy.loginfo("Estop deactivated")
