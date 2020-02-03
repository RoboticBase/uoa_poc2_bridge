# -*- coding: utf-8 -*-
import json
from mock import call

import pytest
import lazy_import
import freezegun

from . import Message, Point, r_command, r_angle, r_angle_optional, r_pose_optional, r_result

command_bridge = lazy_import.lazy_module('uoa_poc2_bridge.command_bridge')
mqtt_base = lazy_import.lazy_module('uoa_poc2_bridge.mqtt_base')


@pytest.fixture(scope='function', autouse=True)
def reload_mock(mocker):
    mqtt_base.mqtt = mocker.MagicMock()
    mqtt_base.logger = mocker.MagicMock()
    command_bridge.rospy = mocker.MagicMock()
    command_bridge.logger = mocker.MagicMock()
    command_bridge.r_command = r_command
    command_bridge.r_result = r_result
    yield


@pytest.fixture(scope='function', autouse=True)
def set_params():
    params = {
        'roboticbase': {
            'entity_type': 'test_type',
            'entity_id': 'test_id',
            'cmd_name': 'cmd_name',
            'auto_cmdexe': False,
        },
        'mqtt': {
            'host': 'mqtt://mqtt.example.com',
            'port': 1234,
        },
        'ros': {
            'topic': {
                'cmd': '/ros/topic/cmd',
                'cmdexe': '/ros/topic/cmdexe'
            },
        },
        'timezone': 'Asia/Tokyo',
    }
    command_bridge.rospy.get_param.return_value = params


class TestCommandBridge(object):

    def test_init(self):
        bridge = command_bridge.CommandBridge()
        assert bridge.client is None
        assert bridge.entity_type == 'test_type'
        assert bridge.entity_id == 'test_id'

        assert command_bridge.rospy.Publisher.call_count == 1
        assert command_bridge.rospy.Publisher.call_args == call('/ros/topic/cmd', r_command, queue_size=1)

        assert command_bridge.rospy.Subscriber.call_count == 1
        assert command_bridge.rospy.Subscriber.call_args == call(
            '/ros/topic/cmdexe', r_result, bridge._on_receive, queue_size=1)

        assert mqtt_base.mqtt.Client.call_count == 0
        assert command_bridge.rospy.spin.call_count == 0

    def test_start(self):
        bridge = command_bridge.CommandBridge()
        bridge.start()

        assert bridge.client is None
        assert mqtt_base.mqtt.Client.call_count == 0
        assert command_bridge.rospy.spin.call_count == 1

    def test_connect_start(self):
        bridge = command_bridge.CommandBridge()
        bridge.connect().start()

        assert bridge.client is not None
        assert mqtt_base.mqtt.Client.call_count == 1
        assert bridge.client.connect.call_count == 1
        assert bridge.client.loop_start.call_count == 1
        assert command_bridge.rospy.spin.call_count == 1

    def test_on_connect(self):
        bridge = command_bridge.CommandBridge()
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
    @pytest.mark.parametrize('auto_cmdexe', [
        False, True, "", None
    ])
    @pytest.mark.parametrize('payload, published', [
        ({
            'cmd_name': {
                'time': '2020-01-02T03:04:05.678Z',
                'cmd': 'navi',
                'waypoints': [],
            },
        }, r_command(id='test_id', type='test_type', time='2020-01-02T03:04:05.678Z', cmd='navi')),
        ({
            'cmd_name': {
                'time': '2020-01-02T03:04:05.678Z',
                'cmd': 'navi',
                'waypoints': [
                    {
                        'point': {
                            'x': 0.1,
                            'y': 0.2,
                            'z': 0.3,
                        },
                        'angle': None,
                    },
                ],
            },
        }, r_command(id='test_id', type='test_type', time='2020-01-02T03:04:05.678Z', cmd='navi',
                     waypoints=[
                         r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                             valid=False, angle=r_angle(roll=0.0, pitch=0.0, yaw=0.0))),
                     ])),
        ({
            'cmd_name': {
                'time': '2020-01-02T03:04:05.678Z',
                'cmd': 'navi',
                'waypoints': [
                    {
                        'point': {
                            'x': 0.1,
                            'y': 0.2,
                            'z': 0.3,
                        },
                        'angle': None,
                    },
                    {
                        'point': {
                            'x': 1.1,
                            'y': 1.2,
                            'z': 1.3,
                        },
                        'angle': None,
                    },
                ],
            },
        }, r_command(id='test_id', type='test_type', time='2020-01-02T03:04:05.678Z', cmd='navi',
                     waypoints=[
                         r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                             valid=False, angle=r_angle(roll=0.0, pitch=0.0, yaw=0.0))),
                         r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                             valid=False, angle=r_angle(roll=0.0, pitch=0.0, yaw=0.0))),
                     ])),
        ({
            'cmd_name': {
                'time': '2020-01-02T03:04:05.678Z',
                'cmd': 'navi',
                'waypoints': [
                    {
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
                ],
            },
        }, r_command(id='test_id', type='test_type', time='2020-01-02T03:04:05.678Z', cmd='navi',
                     waypoints=[
                         r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                             valid=True, angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3))),
                     ])),
        ({
            'cmd_name': {
                'time': '2020-01-02T03:04:05.678Z',
                'cmd': 'navi',
                'waypoints': [
                    {
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
                    {
                        'point': {
                            'x': 1.1,
                            'y': 1.2,
                            'z': 1.3,
                        },
                        'angle': None,
                    },
                ],
            },
        }, r_command(id='test_id', type='test_type', time='2020-01-02T03:04:05.678Z', cmd='navi',
                     waypoints=[
                         r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                             valid=True, angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3))),
                         r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                             valid=False, angle=r_angle(roll=0.0, pitch=0.0, yaw=0.0))),
                     ])),
        ({
            'cmd_name': {
                'time': '2020-01-02T03:04:05.678Z',
                'cmd': 'navi',
                'waypoints': [
                    {
                        'point': {
                            'x': 0.1,
                            'y': 0.2,
                            'z': 0.3,
                        },
                        'angle': None,
                    },
                    {
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
                ],
            },
        }, r_command(id='test_id', type='test_type', time='2020-01-02T03:04:05.678Z', cmd='navi',
                     waypoints=[
                         r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                             valid=False, angle=r_angle(roll=0.0, pitch=0.0, yaw=0.0))),
                         r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                             valid=True, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                     ])),
        ({
            'cmd_name': {
                'time': '2020-01-02T03:04:05.678Z',
                'cmd': 'navi',
                'waypoints': [
                    {
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
                    {
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
                ],
            },
        }, r_command(id='test_id', type='test_type', time='2020-01-02T03:04:05.678Z', cmd='navi',
                     waypoints=[
                         r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                             valid=True, angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3))),
                         r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                             valid=True, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                     ])),
    ])
    def test_on_message_process_cmd(self, topic, auto_cmdexe, payload, published):
        msg = Message(topic=topic, payload=json.dumps(payload))

        bridge = command_bridge.CommandBridge()
        bridge._CommandBridge__is_auto_cmdexe = auto_cmdexe
        bridge.connect()
        with freezegun.freeze_time('2020-11-12T13:14:15.100001+09:00'):
            bridge._on_message(bridge.client, 'userdata', msg)

        assert command_bridge.rospy.Publisher.return_value.publish.call_count == 1
        assert command_bridge.rospy.Publisher.return_value.publish.call_args == call(published)

        if auto_cmdexe:
            cmdexe = {
                'cmd_name': {
                    'time': '2020-11-12T13:14:15.100001+09:00',
                    'received_time': published.time,
                    'received_cmd': published.cmd,
                    'received_waypoints': [{
                        'point': {'x': w.point.x, 'y': w.point.y, 'z': w.point.z},
                        'angle': {
                            'roll': w.angle_optional.angle.roll,
                            'pitch': w.angle_optional.angle.pitch,
                            'yaw': w.angle_optional.angle.yaw,
                        } if w.angle_optional.valid else None,
                    } for w in published.waypoints],
                    'result': 'ack',
                    'errors': [],
                }
            }

            assert command_bridge.logger.infof.call_count == 3
            assert command_bridge.logger.infof.call_args_list[2] == call(
                'auto responded [{}], {}', 'cmd_name', json.dumps(cmdexe))
            assert bridge.client.publish.call_count == 1
            assert bridge.client.publish.call_args == call('/test_type/test_id/cmdexe', json.dumps(cmdexe))
        else:
            assert command_bridge.logger.infof.call_count == 2
            assert bridge.client.publish.call_count == 0

        assert command_bridge.logger.infof.call_args_list[0] == call(
            'received message from {}, payload={}', str(topic), json.dumps(payload))
        assert command_bridge.logger.infof.call_args_list[1] == call(
            'processed the command [{}], {}', 'cmd_name', published)

        assert command_bridge.logger.debugf.call_count == 0
        assert command_bridge.logger.errorf.call_count == 0

    @pytest.mark.parametrize('topic', [
        '/test', '', 1, 1e-1, True, None, [], {},
    ])
    @pytest.mark.parametrize('payload', [
        '{}', '{"other_cmd": {}}', 1, 1e-1, '[]'
    ])
    def test_on_message_invalid_cmd_name(self, topic, payload):
        msg = Message(topic=topic, payload=payload)

        bridge = command_bridge.CommandBridge()
        bridge.connect()
        bridge._on_message(bridge.client, 'userdata', msg)

        assert command_bridge.rospy.Publisher.return_value.publish.call_count == 0

        assert command_bridge.logger.infof.call_count == 1
        assert command_bridge.logger.infof.call_args == call(
            'received message from {}, payload={}', str(topic), str(payload))

        assert command_bridge.logger.debugf.call_count == 1
        assert command_bridge.logger.debugf.call_args == call(
            'ignore this command, topic={}, payload={}', str(topic), str(payload))

        assert command_bridge.logger.errorf.call_count == 0
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

        bridge = command_bridge.CommandBridge()
        bridge.connect()
        bridge._on_message(bridge.client, 'userdata', msg)

        assert command_bridge.rospy.Publisher.return_value.publish.call_count == 0

        assert command_bridge.logger.infof.call_count == 1
        assert command_bridge.logger.infof.call_args == call(
            'received message from {}, payload={}', str(topic), str(payload))

        assert command_bridge.logger.debugf.call_count == 0

        assert command_bridge.logger.errorf.call_count == 1
        assert command_bridge.logger.errorf.call_args == call(
            'invalid payload, topic={}, payload={}, error={}', str(topic), str(payload), errmsg)
        assert bridge.client.publish.call_count == 0

    @pytest.mark.parametrize('result, expected', [
        (r_result(time='2020-11-12T13:14:15.987+09:00',
                  received_time='2020-01-02T03:04:05.678+09:00',
                  received_cmd='navi',
                  received_waypoints=[],
                  result='success',
                  errors=[],
                  ),
         {
             'cmd_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_cmd': 'navi',
                 'received_waypoints': [],
                 'result': 'success',
                 'errors': [],
             },
         }),
        (r_result(time='2020-11-12T13:14:15.987+09:00',
                  received_time='2020-01-02T03:04:05.678+09:00',
                  received_cmd='navi',
                  received_waypoints=[
                    r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                        valid=False, angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3))),
                  ],
                  result='success',
                  errors=[],
                  ),
         {
             'cmd_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_cmd': 'navi',
                 'received_waypoints': [
                     {
                         'point': {
                             'x': 0.1,
                             'y': 0.2,
                             'z': 0.3,
                         },
                         'angle': None,
                     },
                 ],
                 'result': 'success',
                 'errors': [],
             },
         }),
        (r_result(time='2020-11-12T13:14:15.987+09:00',
                  received_time='2020-01-02T03:04:05.678+09:00',
                  received_cmd='navi',
                  received_waypoints=[
                    r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                        valid=True, angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3))),
                  ],
                  result='success',
                  errors=[],
                  ),
         {
             'cmd_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_cmd': 'navi',
                 'received_waypoints': [
                     {
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
                 ],
                 'result': 'success',
                 'errors': [],
             },
         }),
        (r_result(time='2020-11-12T13:14:15.987+09:00',
                  received_time='2020-01-02T03:04:05.678+09:00',
                  received_cmd='navi',
                  received_waypoints=[
                    r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                        valid=False, angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3))),
                    r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                        valid=False, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                  ],
                  result='success',
                  errors=[],
                  ),
         {
             'cmd_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_cmd': 'navi',
                 'received_waypoints': [
                     {
                         'point': {
                             'x': 0.1,
                             'y': 0.2,
                             'z': 0.3,
                         },
                         'angle': None,
                     },
                     {
                         'point': {
                             'x': 1.1,
                             'y': 1.2,
                             'z': 1.3,
                         },
                         'angle': None,
                     },
                 ],
                 'result': 'success',
                 'errors': [],
             },
         }),
        (r_result(time='2020-11-12T13:14:15.987+09:00',
                  received_time='2020-01-02T03:04:05.678+09:00',
                  received_cmd='navi',
                  received_waypoints=[
                    r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                        valid=True, angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3))),
                    r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                        valid=False, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                  ],
                  result='success',
                  errors=[],
                  ),
         {
             'cmd_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_cmd': 'navi',
                 'received_waypoints': [
                     {
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
                     {
                         'point': {
                             'x': 1.1,
                             'y': 1.2,
                             'z': 1.3,
                         },
                         'angle': None,
                     },
                 ],
                 'result': 'success',
                 'errors': [],
             },
         }),
        (r_result(time='2020-11-12T13:14:15.987+09:00',
                  received_time='2020-01-02T03:04:05.678+09:00',
                  received_cmd='navi',
                  received_waypoints=[
                    r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                        valid=False, angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3))),
                    r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                        valid=True, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                  ],
                  result='success',
                  errors=[],
                  ),
         {
             'cmd_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_cmd': 'navi',
                 'received_waypoints': [
                     {
                         'point': {
                             'x': 0.1,
                             'y': 0.2,
                             'z': 0.3,
                         },
                         'angle': None,
                     },
                     {
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
                 ],
                 'result': 'success',
                 'errors': [],
             },
         }),
        (r_result(time='2020-11-12T13:14:15.987+09:00',
                  received_time='2020-01-02T03:04:05.678+09:00',
                  received_cmd='navi',
                  received_waypoints=[
                    r_pose_optional(point=Point(x=0.1, y=0.2, z=0.3), angle_optional=r_angle_optional(
                        valid=True, angle=r_angle(roll=-0.1, pitch=-0.2, yaw=-0.3))),
                    r_pose_optional(point=Point(x=1.1, y=1.2, z=1.3), angle_optional=r_angle_optional(
                        valid=True, angle=r_angle(roll=-1.1, pitch=-1.2, yaw=-1.3))),
                  ],
                  result='success',
                  errors=[],
                  ),
         {
             'cmd_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_cmd': 'navi',
                 'received_waypoints': [
                     {
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
                     {
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
                 ],
                 'result': 'success',
                 'errors': [],
             },
         }),
        (r_result(time='2020-11-12T13:14:15.987+09:00',
                  received_time='2020-01-02T03:04:05.678+09:00',
                  received_cmd='navi',
                  received_waypoints=[],
                  result='success',
                  errors=['error 1'],
                  ),
         {
             'cmd_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_cmd': 'navi',
                 'received_waypoints': [],
                 'result': 'success',
                 'errors': ['error 1'],
             },
         }),
        (r_result(time='2020-11-12T13:14:15.987+09:00',
                  received_time='2020-01-02T03:04:05.678+09:00',
                  received_cmd='navi',
                  received_waypoints=[],
                  result='success',
                  errors=['error 1', None, True, 1, 1e-1],
                  ),
         {
             'cmd_name': {
                 'time': '2020-11-12T13:14:15.987+09:00',
                 'received_time': '2020-01-02T03:04:05.678+09:00',
                 'received_cmd': 'navi',
                 'received_waypoints': [],
                 'result': 'success',
                 'errors': ['error 1', 'None', 'True', '1', '0.1'],
             },
         }),
    ])
    def test_on_receive_process_cmdexe(self, result, expected):
        bridge = command_bridge.CommandBridge()
        bridge.connect()
        bridge._on_receive(result)

        assert bridge.client.publish.call_count == 1
        assert bridge.client.publish.call_args == call('/test_type/test_id/cmdexe', json.dumps(expected))

        assert command_bridge.logger.infof.call_count == 2
        assert command_bridge.logger.infof.call_args_list[0] == call(
            'received ros cmdexe message={}', result)
        assert command_bridge.logger.infof.call_args_list[1] == call(
            'responded the result to {}, payload={}', '/test_type/test_id/cmdexe', json.dumps(expected))
