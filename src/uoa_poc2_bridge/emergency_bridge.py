# -*- coding: utf-8 -*-
import datetime
import json

import pytz

import rospy

from geometry_msgs.msg import Point

from uoa_poc2_msgs.msg import r_emergency_command, r_emergency_result

from uoa_poc2_bridge.mqtt_base import MQTTBase

from uoa_poc2_bridge.logging import getLogger
logger = getLogger(__name__)


class EmergencyBridge(MQTTBase):
    def __init__(self):
        self.__params = rospy.get_param('~')
        super(EmergencyBridge, self).__init__(self.__params)
        self.__mqtt_cmd_topic = '/{}/{}/cmd'.format(self.entity_type, self.entity_id)
        self.__mqtt_cmdexe_topic = '/{}/{}/cmdexe'.format(self.entity_type, self.entity_id)
        self.__emg_pub = rospy.Publisher(self.__params['ros']['topic']['emg'], r_emergency_command, queue_size=1)
        rospy.Subscriber(self.__params['ros']['topic']['emgexe'], r_emergency_result, self._on_receive, queue_size=1)
        self.__tz = pytz.timezone(self.__params['timezone'])
        self.__emg_name = self.__params['roboticbase']['emg_name']

    def start(self):
        logger.infof('EmergencyBridge start')
        rospy.spin()
        logger.infof('EmgergencyBridge finish')

    def _on_connect(self, client, userdata, flags, response_code):
        super(EmergencyBridge, self)._on_connect(client, userdata, flags, response_code)
        client.subscribe(self.__mqtt_cmd_topic)

    def _on_message(self, client, userdata, msg):
        topic = str(msg.topic)
        payload = str(msg.payload)
        logger.infof('received message from {}, payload={}', topic, payload)

        try:
            message = json.loads(payload)
            if self.__emg_name in message:
                ros_published = self._process_emg(message[self.__emg_name])
                logger.infof('processed the command [{}], {}', self.__emg_name, ros_published)
        except (ValueError, TypeError) as e:
            logger.errorf('invalid payload, topic={}, payload={}', topic, payload)

    def _on_receive(self, result):
        logger.infof('received ros emgexe message={}', result)
        mqtt_published = self._process_emgexe(result)
        logger.infof('responded the result to {}, payload={}', self.__mqtt_cmdexe_topic, mqtt_published)

    def _process_emg(self, body):
        command = r_emergency_command()
        command.id = self.entity_id
        command.type = self.entity_type
        command.time = str(body['time'])
        command.emergency_cmd = str(body['emergency_cmd'])

        self.__emg_pub.publish(command)
        return command

    def _process_emgexe(self, result):
        message = {}
        message[self.__emg_name] = {
            'time': result.time,
            'received_time': result.received_time,
            'received_emergency_cmd': result.received_emergency_cmd,
            'result': result.result,
            'errors': [str(e) for e in result.errors],
        }
        payload = json.dumps(message)
        self.client.publish(self.__mqtt_cmdexe_topic, payload)
        return payload
