# -*- coding: utf-8 -*-
import os
import sys

from mock import MagicMock

import pytest


@pytest.fixture(scope='session', autouse=True)
def append_libpath():
    sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))
    yield


@pytest.fixture(scope='session', autouse=True)
def mock_rospy():
    sys.modules['rospy'] = MagicMock()
    sys.modules['paho'] = MagicMock()
    sys.modules['paho.mqtt'] = MagicMock()
    sys.modules['paho.mqtt.client'] = MagicMock()
    sys.modules['geometry_msgs'] = MagicMock()
    sys.modules['geometry_msgs.msg'] = MagicMock()
    sys.modules['uoa_poc2_msgs'] = MagicMock()
    sys.modules['uoa_poc2_msgs.msg'] = MagicMock()
    yield
