#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy

from uoa_poc2_bridge.emergency_bridge import EmergencyBridge


def main():
    try:
        rospy.init_node('robbot_emergency')
        EmergencyBridge().connect().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
