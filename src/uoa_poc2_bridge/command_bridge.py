# -*- coding: utf-8 -*-
import datetime
import json

import pytz

import rospy

from geometry_msgs.msg import Point

from uoa_poc2_msgs.msg import r_command

from uoa_poc2_bridge.mqtt_base import MQTTBase

from uoa_poc2_bridge.logging import getLogger
logger = getLogger(__name__)


class CommandBridge(MQTTBase):
    def __init__(self):
        self.__params = rospy.get_param('~')
        super(CommandBridge, self).__init__(self.__params)
        self.__mqtt_cmd_topic = '/{}/{}/cmd'.format(self.entity_type, self.entity_id)
        self.__cmd_pub = rospy.Publisher(self.__params['ros']['topic'], r_command, queue_size=1)
        self.__tz = pytz.timezone(self.__params['timezone'])

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
            key = message.keys()[0]
            body = message[key]

            self._process_cmd(body)
            self._respond_cmdexe(key, body)
        except (ValueError, TypeError) as e:
            logger.errorf('invalid payload, topic={}, payload={}', topic, payload)

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
        logger.infof('processed the command, {}', command)

    def _respond_cmdexe(self, key, body):
        mqtt_cmdexe_topic = '/{}/{}/cmdexe'.format(self.entity_type, self.entity_id)

        result = {}
        result[key] = {
            'time': datetime.datetime.now(self.__tz).isoformat(),
            'received_time': str(body['time']),
            'received_cmd': str(body['cmd']),
            'received_waypoints': body['waypoints'],
            'result': 'ack',
            'errors': []
        }

        payload = json.dumps(result)
        self.client.publish(mqtt_cmdexe_topic, payload)
        logger.infof('responded the result to {}, payload={}', mqtt_cmdexe_topic, payload)
