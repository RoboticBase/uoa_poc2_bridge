#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy

from uoa_poc2_bridge.command_bridge import CommandBridge


def main():
    try:
        rospy.init_node('robbot_command')
        CommandBridge().connect().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
