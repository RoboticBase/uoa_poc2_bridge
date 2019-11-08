# -*- coding: utf-8 -*-
import datetime
import json

import pytz

import rospy

from geometry_msgs.msg import Point

from uoa_poc2_msgs.msg import r_command, r_result, r_pose_optional, r_angle_optional, r_angle

from uoa_poc2_bridge.mqtt_base import MQTTBase

from uoa_poc2_bridge.logging import getLogger
logger = getLogger(__name__)


class CommandBridge(MQTTBase):
    def __init__(self):
        self.__params = rospy.get_param('~')
        super(CommandBridge, self).__init__(self.__params)
        self.__mqtt_cmd_topic = '/{}/{}/cmd'.format(self.entity_type, self.entity_id)
        self.__mqtt_cmdexe_topic = '/{}/{}/cmdexe'.format(self.entity_type, self.entity_id)
        self.__cmd_pub = rospy.Publisher(self.__params['ros']['topic']['cmd'], r_command, queue_size=1)
        rospy.Subscriber(self.__params['ros']['topic']['cmdexe'], r_result, self._on_receive, queue_size=1)
        self.__tz = pytz.timezone(self.__params['timezone'])
        self.__cmd_name = self.__params['roboticbase']['cmd_name']

    def start(self):
        logger.infof('CommandBridge start')
        rospy.spin()
        logger.infof('CommandBridge finish')

    def _on_connect(self, client, userdata, flags, response_code):
        super(CommandBridge, self)._on_connect(client, userdata, flags, response_code)
        client.subscribe(self.__mqtt_cmd_topic)

    def _on_message(self, client, userdata, msg):
        topic = str(msg.topic)
        payload = str(msg.payload)
        logger.infof('received message from {}, payload={}', topic, payload)

        try:
            message = json.loads(payload)
            if self.__cmd_name in message:
                ros_published = self._process_cmd(message[self.__cmd_name])
                logger.infof('processed the command [{}], {}', self.__cmd_name, ros_published)
        except (ValueError, TypeError) as e:
            logger.errorf('invalid payload, topic={}, payload={}', topic, payload)

    def _on_receive(self, result):
        logger.infof('received ros cmdexe message={}', result)
        mqtt_published = self._process_cmdexe(result)
        logger.infof('responded the result to {}, payload={}', self.__mqtt_cmdexe_topic, mqtt_published)

    def _process_cmd(self, body):
        command = r_command()
        command.id = self.entity_id
        command.type = self.entity_type
        command.time = str(body['time'])
        command.cmd = str(body['cmd'])

        for wp in body['waypoints']:
            point = Point()
            point.x = wp['point']['x']
            point.y = wp['point']['y']
            point.z = wp['point']['z']

            angle = r_angle()
            angle_optional = r_angle_optional()

            if wp['angle'] is not None:
                angle.roll = wp['angle']['roll']
                angle.pitch = wp['angle']['pitch']
                angle.yaw = wp['angle']['yaw']
                angle_optional.valid = True
                angle_optional.angle = angle
            else:
                angle.roll = 0.0
                angle.pitch = 0.0
                angle.yaw = 0.0
                angle_optional.valid = False
                angle_optional.angle = angle

            pose_optional = r_pose_optional()
            pose_optional.point = point
            pose_optional.angle_optional = angle_optional

            command.waypoints.append(pose_optional)

        self.__cmd_pub.publish(command)
        return command

    def _process_cmdexe(self, result):
        message = {}
        message[self.__cmd_name] = {
            'time': result.time,
            'received_time': result.received_time,
            'received_cmd': result.received_cmd,
            'received_waypoints': [{
                'point': {'x': w.point.x, 'y': w.point.y, 'z': w.point.z},
                'angle': {
                    'roll': w.angle_optional.angle.roll,
                    'pitch': w.angle_optional.angle.pitch,
                    'yaw': w.angle_optional.angle.yaw,
                } if w.angle_optional.valid else None,
            } for w in result.received_waypoints],
            'result': result.result,
            'errors': [str(e) for e in result.errors],
        }
        payload = json.dumps(message)
        self.client.publish(self.__mqtt_cmdexe_topic, payload)
        return payload
