#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy

from uoa_poc2_bridge.state_bridge import StateBridge


def main():
    try:
        rospy.init_node('robot_state')
        StateBridge().connect().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
