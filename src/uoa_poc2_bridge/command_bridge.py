# -*- coding: utf-8 -*-
import datetime
import json

import pytz

import rospy

from geometry_msgs.msg import Point

from uoa_poc2_msgs.msg import r_command, r_result

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
            body = message[self.__cmd_name]
            ros_published = self._process_cmd(body)
            logger.infof('processed the command, {}', ros_published)
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
            point.x = wp['x']
            point.y = wp['y']
            point.z = wp['z']
            command.waypoints.append(point)

        self.__cmd_pub.publish(command)
        return command

    def _process_cmdexe(self, result):
        message = {}
        message[self.__cmd_name] = {
            'time': result.time,
            'received_time': result.received_time,
            'received_cmd': result.received_cmd,
            'received_waypoints': [{'x': w.x, 'y': w.y, 'z': w.z} for w in result.received_waypoints],
            'result': result.result,
            'errors': [str(e) for e in result.errors],
        }
        payload = json.dumps(message)
        self.client.publish(self.__mqtt_cmdexe_topic, payload)
        return payload
