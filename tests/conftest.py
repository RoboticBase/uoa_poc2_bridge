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
    yield
