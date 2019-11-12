# -*- coding: utf-8 -*-
import datetime
import json
from threading import Lock

import pytz

import rospy

from delivery_robot.msg import r_state

from uoa_poc2_bridge.mqtt_base import MQTTBase

from uoa_poc2_bridge.logging import getLogger
logger = getLogger(__name__)


class StateBridge(MQTTBase):
    def __init__(self):
        self.__params = rospy.get_param('~')
        super(StateBridge, self).__init__(self.__params)
        self.__mqtt_attrs_topic = '/{}/{}/attrs'.format(self.entity_type, self.entity_id)
        rospy.Subscriber(self.__params['ros']['topic']['state'], r_state, self._on_receive, queue_size=10)
        self.__tz = pytz.timezone(self.__params['timezone'])
        self.__send_delta_ms = self.__params['thresholds']['send_delta_millisec']
        self.__prev_ms = datetime.datetime.now(self.__tz)
        self.__lock = Lock()

    def start(self):
        logger.infof('StateBridge start')
        rospy.spin()
        logger.infof('StateBridge finish')

    def _on_receive(self, state):
        now = datetime.datetime.now(self.__tz)
        if now >= self.__prev_ms + datetime.timedelta(milliseconds=self.__send_delta_ms) and self.__lock.acquire(False):
            self.__prev_ms = now

            message = {
                'time': now.isoformat(),
                'mode': state.mode,
                'errors': [err for err in state.errors if len(err) > 0],
                'pose': {
                    'point': {
                        'x': state.pose.point.x,
                        'y': state.pose.point.y,
                        'z': state.pose.point.z,
                    },
                    'angle': {
                        'roll': state.pose.angle.roll,
                        'pitch': state.pose.angle.pitch,
                        'yaw': state.pose.angle.yaw,
                    },
                },
                'destination': {
                    'point': {
                        'x': state.destination.point.x,
                        'y': state.destination.point.y,
                        'z': state.destination.point.z,
                    },
                    'angle_optional': {
                        'valid': state.destination.angle_optional.valid,
                        'angle': {
                            'roll': state.destination.angle_optional.angle.roll,
                            'pitch': state.destination.angle_optional.angle.pitch,
                            'yaw': state.destination.angle_optional.angle.yaw,
                        }
                    }
                },
                'covariance': [c for c in state.covariance],
                'battery': {
                    'voltage': state.battery.voltage,
                    'current_optional': {
                        'valid': state.battery.current_optional.valid,
                        'current': state.battery.current_optional.current,
                    }
                }
            }

            self.client.publish(self.__mqtt_attrs_topic, json.dumps(message))
            self.__lock.release()
