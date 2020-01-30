# -*- coding: utf-8 -*-
import json
from mock import call

import pytest
import lazy_import
import freezegun

from . import Point, r_state, r_angle, r_angle_optional, r_pose, r_pose_optional, r_battery, r_current_optional

state_bridge = lazy_import.lazy_module('uoa_poc2_bridge.state_bridge')
mqtt_base = lazy_import.lazy_module('uoa_poc2_bridge.mqtt_base')


@pytest.fixture(scope='function', autouse=True)
def reload_mock(mocker):
    mqtt_base.mqtt = mocker.MagicMock()
    mqtt_base.logger = mocker.MagicMock()
    state_bridge.rospy = mocker.MagicMock()
    state_bridge.logger = mocker.MagicMock()
    state_bridge.r_state = r_state
    yield


@pytest.fixture(scope='function', autouse=True)
def set_params():
    params = {
        'roboticbase': {
            'entity_type': 'test_type',
            'entity_id': 'test_id',
        },
        'mqtt': {
            'host': 'mqtt://mqtt.example.com',
            'port': 1234,
        },
        'ros': {
            'topic': {
                'state': '/ros/topic/state',
            },
        },
        'timezone': 'Asia/Tokyo',
        'thresholds': {
            'send_delta_millisec': 500
        }
    }
    state_bridge.rospy.get_param.return_value = params


class TestStateBridge(object):

    def test_init(self):
        bridge = state_bridge.StateBridge()
        assert bridge.entity_type == 'test_type'
        assert bridge.entity_id == 'test_id'

        assert state_bridge.rospy.Publisher.call_count == 0

        assert state_bridge.rospy.Subscriber.call_count == 1
        assert state_bridge.rospy.Subscriber.call_args == call(
            '/ros/topic/state', r_state, bridge._on_receive, queue_size=10)

        assert mqtt_base.mqtt.Client.call_count == 0
        assert state_bridge.rospy.spin.call_count == 0

    def test_start(self):
        bridge = state_bridge.StateBridge()
        bridge.start()

        assert bridge.client is None
        assert mqtt_base.mqtt.Client.call_count == 0
        assert state_bridge.rospy.spin.call_count == 1

    def test_connect_start(self):
        bridge = state_bridge.StateBridge()
        bridge.connect().start()

        assert bridge.client is not None
        assert mqtt_base.mqtt.Client.call_count == 1
        assert bridge.client.connect.call_count == 1
        assert bridge.client.loop_start.call_count == 1
        assert state_bridge.rospy.spin.call_count == 1

    @pytest.mark.parametrize('state, expected', [
        (r_state(mode='navi',
                 errors=[],
                 pose=r_pose(point=Point(x=0.1, y=0.2, z=0.3), angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3)),
                 destination=r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                     valid=True, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                 covariance=[],
                 battery=r_battery(voltage=10.0, current_optional=r_current_optional(valid=True, current=11.0))
                 ),
         {
             'time': '2020-01-02T03:04:05.678000+09:00',
             'mode': 'navi',
             'errors': [],
             'pose': {
                 'point': {
                     'x': 0.1,
                     'y': 0.2,
                     'z': 0.3,
                 },
                 'angle': {
                     'roll': -0.1,
                     'pitch': -0.2,
                     'yaw': -0.3,
                 },
             },
             'destination': {
                 'point': {
                     'x': 1.1,
                     'y': 1.2,
                     'z': 1.3,
                 },
                 'angle': {
                     'roll': -1.1,
                     'pitch': -1.2,
                     'yaw': -1.3,
                 },
             },
             'covariance': [],
             'battery': {
                 'voltage': 10.0,
                 'current': 11.0,
             },
         }),
        (r_state(mode='navi',
                 errors=[],
                 pose=r_pose(point=Point(x=0.1, y=0.2, z=0.3), angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3)),
                 destination=r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                     valid=False, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                 covariance=[],
                 battery=r_battery(voltage=10.0, current_optional=r_current_optional(valid=True, current=11.0))
                 ),
         {
             'time': '2020-01-02T03:04:05.678000+09:00',
             'mode': 'navi',
             'errors': [],
             'pose': {
                 'point': {
                     'x': 0.1,
                     'y': 0.2,
                     'z': 0.3,
                 },
                 'angle': {
                     'roll': -0.1,
                     'pitch': -0.2,
                     'yaw': -0.3,
                 },
             },
             'destination': {
                 'point': {
                     'x': 1.1,
                     'y': 1.2,
                     'z': 1.3,
                 },
                 'angle': None,
             },
             'covariance': [],
             'battery': {
                 'voltage': 10.0,
                 'current': 11.0,
             },
         }),
        (r_state(mode='navi',
                 errors=[],
                 pose=r_pose(point=Point(x=0.1, y=0.2, z=0.3), angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3)),
                 destination=r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                     valid=True, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                 covariance=[],
                 battery=r_battery(voltage=10.0, current_optional=r_current_optional(valid=False, current=11.0))
                 ),
         {
             'time': '2020-01-02T03:04:05.678000+09:00',
             'mode': 'navi',
             'errors': [],
             'pose': {
                 'point': {
                     'x': 0.1,
                     'y': 0.2,
                     'z': 0.3,
                 },
                 'angle': {
                     'roll': -0.1,
                     'pitch': -0.2,
                     'yaw': -0.3,
                 },
             },
             'destination': {
                 'point': {
                     'x': 1.1,
                     'y': 1.2,
                     'z': 1.3,
                 },
                 'angle': {
                     'roll': -1.1,
                     'pitch': -1.2,
                     'yaw': -1.3,
                 },
             },
             'covariance': [],
             'battery': {
                 'voltage': 10.0,
                 'current': None,
             },
         }),
        (r_state(mode='navi',
                 errors=['error', '', None, True, 1, 1e-1],
                 pose=r_pose(point=Point(x=0.1, y=0.2, z=0.3), angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3)),
                 destination=r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                     valid=True, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                 covariance=['dummy', '', None, True, 1, 1e-1],
                 battery=r_battery(voltage=10.0, current_optional=r_current_optional(valid=False, current=11.0))
                 ),
         {
             'time': '2020-01-02T03:04:05.678000+09:00',
             'mode': 'navi',
             'errors': ['error'],
             'pose': {
                 'point': {
                     'x': 0.1,
                     'y': 0.2,
                     'z': 0.3,
                 },
                 'angle': {
                     'roll': -0.1,
                     'pitch': -0.2,
                     'yaw': -0.3,
                 },
             },
             'destination': {
                 'point': {
                     'x': 1.1,
                     'y': 1.2,
                     'z': 1.3,
                 },
                 'angle': {
                     'roll': -1.1,
                     'pitch': -1.2,
                     'yaw': -1.3,
                 },
             },
             'covariance': ['dummy', '', None, True, 1, 1e-1],
             'battery': {
                 'voltage': 10.0,
                 'current': None,
             },
         }),
    ])
    def test_on_receive_publish(self, state, expected):
        with freezegun.freeze_time('2020-01-02T03:04:05.000+09:00'):
            bridge = state_bridge.StateBridge()

        bridge.connect()

        with freezegun.freeze_time('2020-01-02T03:04:05.678+09:00'):
            bridge._on_receive(state)

        assert bridge.client.publish.call_count == 1
        assert bridge.client.publish.call_args == call('/test_type/test_id/attrs', json.dumps(expected))

        assert bridge._StateBridge__prev_ms.isoformat() == '2020-01-02T03:04:05.678000+09:00'

    @pytest.mark.parametrize('prev_ms, current_ms, is_publish', [
        ('2020-01-02T03:04:05.000001+09:00', '2020-01-02T03:04:05.500001+09:00', True),
        ('2020-01-02T03:04:05.000001+09:00', '2020-01-02T03:04:05.500000+09:00', False),
    ])
    def test_on_receive_check_timedelta(self, prev_ms, current_ms, is_publish):
        state = r_state(mode='navi',
                        errors=[],
                        pose=r_pose(point=Point(x=0.1, y=0.2, z=0.3), angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3)),
                        destination=r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                            valid=True, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                        covariance=[],
                        battery=r_battery(voltage=10.0, current_optional=r_current_optional(valid=True, current=11.0)))

        with freezegun.freeze_time(prev_ms):
            bridge = state_bridge.StateBridge()

        bridge.connect()

        with freezegun.freeze_time(current_ms):
            bridge._on_receive(state)

        if is_publish:
            assert bridge.client.publish.call_count == 1
            assert bridge._StateBridge__prev_ms.isoformat() == current_ms
        else:
            assert bridge.client.publish.call_count == 0
            assert bridge._StateBridge__prev_ms.isoformat() == prev_ms

    @pytest.mark.parametrize('is_lock, is_publish', [
        (True, True),
        (False, False),
    ])
    def test_on_receive_check_lock(self, mocker, is_lock, is_publish):
        state = r_state(mode='navi',
                        errors=[],
                        pose=r_pose(point=Point(x=0.1, y=0.2, z=0.3), angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3)),
                        destination=r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                            valid=True, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                        covariance=[],
                        battery=r_battery(voltage=10.0, current_optional=r_current_optional(valid=True, current=11.0)))

        state_bridge.Lock = mocker.MagicMock()
        state_bridge.Lock.return_value.acquire.return_value = is_lock

        with freezegun.freeze_time('2020-01-02T03:04:05.000+09:00'):
            bridge = state_bridge.StateBridge()

        bridge.connect()

        with freezegun.freeze_time('2020-01-02T03:04:05.678+09:00'):
            bridge._on_receive(state)

        assert state_bridge.Lock.return_value.acquire.call_count == 1
        assert state_bridge.Lock.return_value.acquire.call_args == call(False)

        if is_publish:
            assert bridge.client.publish.call_count == 1
            assert state_bridge.Lock.return_value.release.call_count == 1
            assert state_bridge.Lock.return_value.release.call_args == call()
        else:
            assert bridge.client.publish.call_count == 0
            assert state_bridge.Lock.return_value.release.call_count == 0
