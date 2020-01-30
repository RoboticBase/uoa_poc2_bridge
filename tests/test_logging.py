# -*- coding: utf-8 -*-
from mock import call

import pytest
import lazy_import

logging = lazy_import.lazy_module('uoa_poc2_bridge.logging')


@pytest.fixture(scope='function', autouse=True)
def reload_mock(mocker):
    logging.rospy = mocker.MagicMock()
    yield


class TestLogger(object):
    def test_debugf(self):
        logger = logging.getLogger('dummy module')
        logger.debugf('test debugf message')
        assert logging.rospy.logdebug.call_count == 1
        assert logging.rospy.logdebug.call_args == call('[dummy module:TestLogger.test_debugf] test debugf message')
        assert logging.rospy.loginfo.call_count == 0
        assert logging.rospy.logwarn.call_count == 0
        assert logging.rospy.logerr.call_count == 0
        assert logging.rospy.logfatal.call_count == 0

    def test_infof(self):
        logger = logging.getLogger('dummy module')
        logger.infof('test infof message')
        assert logging.rospy.logdebug.call_count == 0
        assert logging.rospy.loginfo.call_count == 1
        assert logging.rospy.loginfo.call_args == call('[dummy module:TestLogger.test_infof] test infof message')
        assert logging.rospy.logwarn.call_count == 0
        assert logging.rospy.logerr.call_count == 0
        assert logging.rospy.logfatal.call_count == 0

    def test_warnf(self):
        logger = logging.getLogger('dummy module')
        logger.warnf('test warnf message')
        assert logging.rospy.logdebug.call_count == 0
        assert logging.rospy.loginfo.call_count == 0
        assert logging.rospy.logwarn.call_count == 1
        assert logging.rospy.logwarn.call_args == call('[dummy module:TestLogger.test_warnf] test warnf message')
        assert logging.rospy.logerr.call_count == 0
        assert logging.rospy.logfatal.call_count == 0

    def test_errorf(self):
        logger = logging.getLogger('dummy module')
        logger.errorf('test errorf message')
        assert logging.rospy.logdebug.call_count == 0
        assert logging.rospy.loginfo.call_count == 0
        assert logging.rospy.logwarn.call_count == 0
        assert logging.rospy.logerr.call_count == 1
        assert logging.rospy.logerr.call_args == call('[dummy module:TestLogger.test_errorf] test errorf message')
        assert logging.rospy.logfatal.call_count == 0

    def test_fatalf(self):
        logger = logging.getLogger('dummy module')
        logger.fatalf('test fatalf message')
        assert logging.rospy.logdebug.call_count == 0
        assert logging.rospy.loginfo.call_count == 0
        assert logging.rospy.logwarn.call_count == 0
        assert logging.rospy.logerr.call_count == 0
        assert logging.rospy.logfatal.call_count == 1
        assert logging.rospy.logfatal.call_args == call('[dummy module:TestLogger.test_fatalf] test fatalf message')

    def test_invalid(self, mocker):
        class DummyRospy(object):
            def __init__(self):
                pass

        logging.rospy = DummyRospy()
        logger = logging.getLogger('dummy module')
        logger.debugf('test invalid func')

    def test_keyerror(self, mocker):
        class DummyCode(object):
            def __init__(self):
                self.co_name = 'co_name'

        class DummyFrame(object):
            def __init__(self):
                self.f_locals = dict()
                self.f_code = DummyCode()

        logging.inspect = mocker.MagicMock()
        logging.inspect.stack.return_value = [None, None, [DummyFrame()]]

        logger = logging.getLogger('dummy module')
        logger.debugf('test debugf message')
        assert logging.rospy.logdebug.call_count == 1
        assert logging.rospy.logdebug.call_args == call('[dummy module:co_name] test debugf message')
