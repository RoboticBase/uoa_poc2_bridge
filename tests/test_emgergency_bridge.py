# -*- coding: utf-8 -*-
import json
from mock import call

import pytest
import lazy_import

from . import Message, r_emergency_command, r_emergency_result

emergency_bridge = lazy_import.lazy_module('uoa_poc2_bridge.emergency_bridge')
mqtt_base = lazy_import.lazy_module('uoa_poc2_bridge.mqtt_base')


@pytest.fixture(scope='function', autouse=True)
def reload_mock(mocker):
    mqtt_base.mqtt = mocker.MagicMock()
    mqtt_base.logger = mocker.MagicMock()
    emergency_bridge.rospy = mocker.MagicMock()
    emergency_bridge.logger = mocker.MagicMock()
    emergency_bridge.r_emergency_command = r_emergency_command
    emergency_bridge.r_emergency_result = r_emergency_result
    yield


@pytest.fixture(scope='function', autouse=True)
def set_params():
    params = {
        'roboticbase': {
            'entity_type': 'test_type',
            'entity_id': 'test_id',
            'emg_name': 'emg_name',
        },
        'mqtt': {
            'host': 'mqtt://mqtt.example.com',
            'port': 1234,
        },
        'ros': {
            'topic': {
                'emg': '/ros/topic/cmd',
                'emgexe': '/ros/topic/cmdexe'
            },
        },
        'timezone': 'Asia/Tokyo',
    }
    emergency_bridge.rospy.get_param.return_value = params


class TestEmergencyBridge(object):

    def test_init(self):
        bridge = emergency_bridge.EmergencyBridge()
        assert bridge.entity_type == 'test_type'
        assert bridge.entity_id == 'test_id'

        assert emergency_bridge.rospy.Publisher.call_count == 1
        assert emergency_bridge.rospy.Publisher.call_args == call('/ros/topic/cmd', r_emergency_command, queue_size=1)

        assert emergency_bridge.rospy.Subscriber.call_count == 1
        assert emergency_bridge.rospy.Subscriber.call_args == call(
            '/ros/topic/cmdexe', r_emergency_result, bridge._on_receive, queue_size=1)

        assert mqtt_base.mqtt.Client.call_count == 0
        assert emergency_bridge.rospy.spin.call_count == 0

    def test_start(self):
        bridge = emergency_bridge.EmergencyBridge()
        bridge.start()

        assert bridge.client is None
        assert mqtt_base.mqtt.Client.call_count == 0
        assert emergency_bridge.rospy.spin.call_count == 1

    def test_connect_start(self):
        bridge = emergency_bridge.EmergencyBridge()
        bridge.connect().start()

        assert bridge.client is not None
        assert mqtt_base.mqtt.Client.call_count == 1
        assert bridge.client.connect.call_count == 1
        assert bridge.client.loop_start.call_count == 1
        assert emergency_bridge.rospy.spin.call_count == 1

    def test_on_connect(self):
        bridge = emergency_bridge.EmergencyBridge()
        bridge.connect()

        assert mqtt_base.logger.infof.call_count == 1
        assert mqtt_base.logger.infof.call_args_list[0] == call(
            'try to Connect mqtt broker, host={}', 'mqtt://mqtt.example.com')

        assert bridge.client is not None
        bridge._on_connect(bridge.client, 'userdata', 'flags', 0)

        assert mqtt_base.logger.infof.call_count == 2
        assert mqtt_base.logger.infof.call_args_list[1] == call('connected to mqtt broker, status={}', 0)

        assert bridge.client.subscribe.call_count == 1
        assert bridge.client.subscribe.call_args == call('/test_type/test_id/cmd')

    @pytest.mark.parametrize('topic', [
        '/test', '', 1, 1e-1, True, None, [], {},
    ])
    @pytest.mark.parametrize('payload, published', [
        ({
            'emg_name': {
                'time': '2020-01-02T03:04:05.678Z',
                'emergency_cmd': 'stop',
            },
        }, r_emergency_command(id='test_id', type='test_type', time='2020-01-02T03:04:05.678Z', emergency_cmd='stop')),
    ])
    def test_on_message_process_emg(self, topic, payload, published):
        msg = Message(topic=topic, payload=json.dumps(payload))

        bridge = emergency_bridge.EmergencyBridge()
        bridge.connect()
        bridge._on_message(bridge.client, 'userdata', msg)

        assert emergency_bridge.rospy.Publisher.return_value.publish.call_count == 1
        assert emergency_bridge.rospy.Publisher.return_value.publish.call_args == call(published)

        assert emergency_bridge.logger.infof.call_count == 2
        assert emergency_bridge.logger.infof.call_args_list[0] == call(
            'received message from {}, payload={}', str(topic), json.dumps(payload))
        assert emergency_bridge.logger.infof.call_args_list[1] == call(
            'processed the command [{}], {}', 'emg_name', published)
        assert emergency_bridge.logger.debugf.call_count == 0
        assert emergency_bridge.logger.errorf.call_count == 0
        assert bridge.client.publish.call_count == 0

    @pytest.mark.parametrize('topic', [
        '/test', '', 1, 1e-1, True, None, [], {},
    ])
    @pytest.mark.parametrize('payload', [
        '{}', '{"other_cmd": {}}', 1, 1e-1, '[]'
    ])
    def test_on_message_invalid_emg_name(self, topic, payload):
        msg = Message(topic=topic, payload=payload)

        bridge = emergency_bridge.EmergencyBridge()
        bridge.connect()
        bridge._on_message(bridge.client, 'userdata', msg)

        assert emergency_bridge.rospy.Publisher.return_value.publish.call_count == 0

        assert emergency_bridge.logger.infof.call_count == 1
        assert emergency_bridge.logger.infof.call_args == call(
            'received message from {}, payload={}', str(topic), str(payload))

        assert emergency_bridge.logger.debugf.call_count == 1
        assert emergency_bridge.logger.debugf.call_args == call(
            'ignore this command, topic={}, payload={}', str(topic), str(payload))

        assert emergency_bridge.logger.errorf.call_count == 0
        assert bridge.client.publish.call_count == 0

    @pytest.mark.parametrize('topic', [
        '/test', '', 1, 1e-1, True, None, [], {},
    ])
    @pytest.mark.parametrize('payload, errmsg', [
        ('{', 'Expecting object: line 1 column 1 (char 0)'),
        ('[1, 2', 'Expecting object: line 1 column 5 (char 4)'),
        ('}', 'No JSON object could be decoded'),
        ('1, 2]', 'Extra data: line 1 column 2 - line 1 column 6 (char 1 - 5)'),
        ('', 'No JSON object could be decoded'),
        ('invalid', 'No JSON object could be decoded'),
        (True, 'No JSON object could be decoded'),
        (None, 'No JSON object could be decoded'),
    ])
    def test_on_message_invalid_json(self, topic, payload, errmsg):
        msg = Message(topic=topic, payload=payload)

        bridge = emergency_bridge.EmergencyBridge()
        bridge.connect()
        bridge._on_message(bridge.client, 'userdata', msg)

        assert emergency_bridge.rospy.Publisher.return_value.publish.call_count == 0

        assert emergency_bridge.logger.infof.call_count == 1
        assert emergency_bridge.logger.infof.call_args == call(
            'received message from {}, payload={}', str(topic), str(payload))

        assert emergency_bridge.logger.debugf.call_count == 0

        assert emergency_bridge.logger.errorf.call_count == 1
        assert emergency_bridge.logger.errorf.call_args == call(
            'invalid payload, topic={}, payload={}, error={}', str(topic), str(payload), errmsg)
        assert bridge.client.publish.call_count == 0

    @pytest.mark.parametrize('result, expected', [
        (r_emergency_result(time='2020-11-12T13:14:15.987+09:00',
                            received_time='2020-01-02T03:04:05.678+09:00',
                            received_emergency_cmd='stop',
                            result='success',
                            errors=[],
                            ),
         {
             'emg_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_emergency_cmd': 'stop',
                 'result': 'success',
                 'errors': [],
             },
         }),
        (r_emergency_result(time='2020-11-12T13:14:15.987+09:00',
                            received_time='2020-01-02T03:04:05.678+09:00',
                            received_emergency_cmd='stop',
                            result='success',
                            errors=['error 1'],
                            ),
         {
             'emg_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_emergency_cmd': 'stop',
                 'result': 'success',
                 'errors': ['error 1'],
             },
         }),
        (r_emergency_result(time='2020-11-12T13:14:15.987+09:00',
                            received_time='2020-01-02T03:04:05.678+09:00',
                            received_emergency_cmd='stop',
                            result='success',
                            errors=['error 1', None, True, 1, 1e-1],
                            ),
         {
             'emg_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_emergency_cmd': 'stop',
                 'result': 'success',
                 'errors': ['error 1', 'None', 'True', '1', '0.1'],
             },
         }),
    ])
    def test_on_receive_process_emgexe(self, result, expected):
        bridge = emergency_bridge.EmergencyBridge()
        bridge.connect()
        bridge._on_receive(result)

        assert bridge.client.publish.call_count == 1
        assert bridge.client.publish.call_args == call('/test_type/test_id/cmdexe', json.dumps(expected))

        assert emergency_bridge.logger.infof.call_count == 2
        assert emergency_bridge.logger.infof.call_args_list[0] == call(
            'received ros emgexe message={}', result)
        assert emergency_bridge.logger.infof.call_args_list[1] == call(
            'responded the result to {}, payload={}', '/test_type/test_id/cmdexe', json.dumps(expected))
