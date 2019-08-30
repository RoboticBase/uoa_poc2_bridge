# -*- coding: utf-8 -*-
import os
import ssl

import rospy

import paho.mqtt.client as mqtt

from uoa_poc2_bridge.logging import getLogger
logger = getLogger(__name__)


class MQTTBase(object):
    def __init__(self, params):
        self.__client = None
        self.__params = params

    @property
    def client(self):
        return self.__client

    @property
    def entity_type(self):
        return self.__params['roboticbase']['entity_type']

    @property
    def entity_id(self):
        return self.__params['roboticbase']['entity_id']

    def connect(self):
        logger.infof('try to Connect mqtt broker, host={}', self.__params['mqtt']['host'])
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.on_connect = self._on_connect
        self.__client.on_message = self._on_message

        if 'use_ca' in self.__params['mqtt'] and self.__params['mqtt']['use_ca'] and 'cafile' in self.__params['mqtt']:
            cafile = self.__params['mqtt']['cafile'].strip()
            if len(cafile) > 0 and os.path.isfile(cafile):
                self.__client.tls_set(cafile, tls_version=ssl.PROTOCOL_TLSv1_2)

        if 'username' in self.__params['mqtt'] and 'password' in self.__params['mqtt']:
            username = self.__params['mqtt']['username'].strip()
            password = self.__params['mqtt']['password'].strip()
            if len(username) > 0 and len(password) > 0:
                self.__client.username_pw_set(username, password)

        self.__client.connect(self.__params['mqtt']['host'], port=self.__params['mqtt']['port'], keepalive=60)
        self.__client.loop_start()

        rospy.on_shutdown(self._on_shutdown)
        return self

    def _on_shutdown(self):
        if self.__client:
            self.__client.loop_stop()
            self.__client.disconnect()

    def _on_connect(self, client, userdata, flags, response_code):
        logger.infof('connected to mqtt broker, status={}', response_code)

    def _on_message(self, client, userdata, msg):
        pass
