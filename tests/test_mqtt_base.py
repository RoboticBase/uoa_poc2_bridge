# -*- coding: utf-8 -*-
import ssl

from mock import call

import pytest
import lazy_import

mqtt_base = lazy_import.lazy_module('uoa_poc2_bridge.mqtt_base')


@pytest.fixture(scope='function', autouse=True)
def reload_mock(mocker):
    mqtt_base.rospy = mocker.MagicMock()
    mqtt_base.logger = mocker.MagicMock()

    mocked_mqtt = mocker.MagicMock()
    protocol = mocker.PropertyMock()
    protocol.return_value = 'MQTTv311'
    type(mocked_mqtt).MQTTv311 = protocol
    mqtt_base.mqtt = mocked_mqtt
    yield


@pytest.fixture(scope='function')
def base_params():
    return {
        'mqtt': {
            'host': 'mqtt://mqtt.example.com',
            'port': 1234,
        }
    }


class TestMQTTBase(object):

    def test_init(self):
        params = {
            'roboticbase': {
                'entity_type': 'test_type',
                'entity_id': 'test_id',
            }
        }

        mqttBase = mqtt_base.MQTTBase(params)
        assert mqttBase.client is None
        assert mqttBase.entity_type == 'test_type'
        assert mqttBase.entity_id == 'test_id'

        mqttBase._on_shutdown()
        assert mqtt_base.logger.warnf.call_count == 1
        assert mqtt_base.logger.warnf.call_args == call('_on_shutdown is ignored')

    def test_connect(self, base_params):
        mqttBase = mqtt_base.MQTTBase(base_params)
        mqttBase.connect()
        assert mqttBase.client is not None
        assert mqtt_base.logger.infof.call_count == 1
        assert mqtt_base.logger.infof.call_args == call('try to Connect mqtt broker, host={}', 'mqtt://mqtt.example.com')

        assert mqtt_base.mqtt.Client.call_count == 1
        assert mqtt_base.mqtt.Client.call_args == call(protocol='MQTTv311')

        assert mqttBase.client.on_connect == mqttBase._on_connect
        assert mqttBase.client.on_message == mqttBase._on_message

        assert mqttBase.client.tls_set.call_count == 0

        assert mqttBase.client.username_pw_set.call_count == 0

        assert mqttBase.client.connect.call_count == 1
        assert mqttBase.client.connect.call_args == call('mqtt://mqtt.example.com', port=1234, keepalive=60)

        assert mqttBase.client.loop_start.call_count == 1
        assert mqttBase.client.loop_start.call_args == call()

        assert mqtt_base.rospy.on_shutdown.call_count == 1
        assert mqtt_base.rospy.on_shutdown.call_args == call(mqttBase._on_shutdown)

        mqttBase._on_shutdown()
        assert mqttBase.client.loop_stop.call_count == 1
        assert mqttBase.client.loop_stop.call_args == call()
        assert mqttBase.client.disconnect.call_count == 1
        assert mqttBase.client.disconnect.call_args == call()

    @pytest.mark.parametrize('ca_params, is_tls_set', [
        ({'use_ca': True, 'cafile': 'cafile'}, True),
        ({'use_ca': 1, 'cafile': 'cafile'}, True),
        ({'use_ca': 'TRUE', 'cafile': 'cafile'}, True),
        ({'cafile': 'cafile'}, False),
        ({'use_ca': False, 'cafile': 'cafile'}, False),
        ({'use_ca': None, 'cafile': 'cafile'}, False),
        ({'use_ca': True, 'cafile': ''}, False),
        ({'use_ca': True, 'cafile': '/tmp/foo/bar'}, False),
        ({'use_ca': True, 'cafile': None}, False),
        ({'use_ca': True, 'cafile': True}, False),
        ({}, False),
    ])
    def test_ca(self, tmpdir, base_params, ca_params, is_tls_set):
        base_params['mqtt'].update(ca_params)
        if 'cafile' in ca_params and ca_params['cafile'] == 'cafile':
            f = tmpdir.join('cafile')
            f.write('ca')
            base_params['mqtt']['cafile'] = str(f)

        mqttBase = mqtt_base.MQTTBase(base_params)
        mqttBase.connect()
        assert mqttBase.client is not None

        assert mqtt_base.mqtt.Client.call_count == 1
        assert mqtt_base.mqtt.Client.call_args == call(protocol='MQTTv311')

        if is_tls_set:
            assert mqttBase.client.tls_set.call_count == 1
            assert mqttBase.client.tls_set.call_args == call(str(f), tls_version=ssl.PROTOCOL_TLSv1_2)
        else:
            assert mqttBase.client.tls_set.call_count == 0

        assert mqttBase.client.username_pw_set.call_count == 0

        assert mqttBase.client.connect.call_count == 1
        assert mqttBase.client.connect.call_args == call('mqtt://mqtt.example.com', port=1234, keepalive=60)

        assert mqttBase.client.loop_start.call_count == 1
        assert mqttBase.client.loop_start.call_args == call()

        assert mqtt_base.rospy.on_shutdown.call_count == 1
        assert mqtt_base.rospy.on_shutdown.call_args == call(mqttBase._on_shutdown)

    @pytest.mark.parametrize('auth_params, is_username_pw_set', [
        ({'username': 'testuser', 'password': 'testpassword'}, True),
        ({'username': '', 'password': 'testpassword'}, False),
        ({'username': 'testuser', 'password': ''}, False),
        ({'username': 'testuser'}, False),
        ({'password': 'testpassword'}, False),
        ({'username': None, 'password': 'testpassword'}, False),
        ({'username': True, 'password': 'testpassword'}, False),
        ({'username': 1, 'password': 'testpassword'}, False),
        ({'username': 'testuser', 'password': None}, False),
        ({'username': 'testuser', 'password': True}, False),
        ({'username': 'testuser', 'password': 1}, False),
        ({}, False),
    ])
    def test_auth(self, base_params, auth_params, is_username_pw_set):
        base_params['mqtt'].update(auth_params)
        mqttBase = mqtt_base.MQTTBase(base_params)
        mqttBase.connect()
        assert mqttBase.client is not None

        assert mqtt_base.mqtt.Client.call_count == 1
        assert mqtt_base.mqtt.Client.call_args == call(protocol='MQTTv311')

        assert mqttBase.client.tls_set.call_count == 0

        if is_username_pw_set:
            assert mqttBase.client.username_pw_set.call_count == 1
            assert mqttBase.client.username_pw_set.call_args == call(auth_params['username'], auth_params['password'])
        else:
            assert mqttBase.client.username_pw_set.call_count == 0

        assert mqttBase.client.connect.call_count == 1
        assert mqttBase.client.connect.call_args == call('mqtt://mqtt.example.com', port=1234, keepalive=60)

        assert mqttBase.client.loop_start.call_count == 1
        assert mqttBase.client.loop_start.call_args == call()

        assert mqtt_base.rospy.on_shutdown.call_count == 1
        assert mqtt_base.rospy.on_shutdown.call_args == call(mqttBase._on_shutdown)

    def test_on_connect(self, base_params):
        mqttBase = mqtt_base.MQTTBase(base_params)
        mqttBase.connect()

        mqttBase.client.on_connect(mqttBase.client, 'userdata', 'flags', 0)
        assert mqtt_base.logger.infof.call_count == 2
        assert mqtt_base.logger.infof.call_args_list[0] == call(
            'try to Connect mqtt broker, host={}', 'mqtt://mqtt.example.com')
        assert mqtt_base.logger.infof.call_args_list[1] == call('connected to mqtt broker, status={}', 0)

    def test_on_message(self, base_params):
        mqttBase = mqtt_base.MQTTBase(base_params)
        mqttBase.connect()

        mqttBase.client.on_message(mqttBase.client, 'userdata', 'test message')
        assert mqtt_base.logger.infof.call_count == 2
        assert mqtt_base.logger.infof.call_args_list[0] == call(
            'try to Connect mqtt broker, host={}', 'mqtt://mqtt.example.com')
        assert mqtt_base.logger.infof.call_args == call('received message from mqtt broker, msg={}', 'test message')
