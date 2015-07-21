#!/usr/bin/env python
#
# License: BSD
##############################################################################
import rospy
import rapix_bridge
import std_msgs.msg as std_msgs


class RobosemBridge(rapix_bridge.RAPIXBridge):
    def init_publisher(self):
        publishers = {}
        publishers['TouchSensorEvent'] = rospy.Publisher('touch_sensor_event', std_msgs.String, latch=False, queue_size=1)
        self.publishers = publishers

    def init_subscriber(self):
        subscribers = {}
        subscribers['PlayTTS'] = rospy.Subscriber('play_tts', std_msgs.String, self.play_tts)
        subscribers['RobotMotion'] = rospy.Subscriber('robot_motion', std_msgs.String, self.robot_motion)
        self.subscribers = subscribers

    def process_touch_event(self, button_id, status):
        if (button_id is 4 or button_id is 3) and status is 1:
            if button_id is 4:
                button_id = std_msgs.String('right')
            elif button_id is 3:
                button_id = std_msgs.String('left')
            self.publishers['TouchSensorEvent'].publish(button_id)

    def play_tts(self, msg):
        self.loginfo("TTS: %s" % msg.data)
        self._play_tts(msg.data)

    def robot_motion(self, msg):
        self.loginfo("Motion: %s" % msg.data)
        self._robot_motion(msg.data)


##########################################################################
# Main
##########################################################################

if __name__ == '__main__':
    rospy.init_node('robosem_bridge')
    robosem_ip = ""
    robosem_port = ""
    robosem_ip = rospy.get_param('~robot_ip', "localhost")
    robosem_port = rospy.get_param('~robot_port', 6001)

    rb = RobosemBridge(host=robosem_ip, port=robosem_port)
    rb.spin()
