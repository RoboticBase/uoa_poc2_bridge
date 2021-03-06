# -*- coding: utf-8 -*-
from pydataclasses import DataClass


class Message(DataClass):
    topic = None
    payload = None


class Point(DataClass):
    x = None
    y = None
    z = None


class r_command(DataClass):
    id = None
    type = None
    time = None
    cmd = None
    waypoints = []


class r_angle(DataClass):
    roll = None
    pitch = None
    yaw = None


class r_angle_optional(DataClass):
    valid = None
    angle = None


class r_pose_optional(DataClass):
    point = None
    angle_optional = None


class r_result(DataClass):
    time = None
    received_time = None
    received_cmd = None
    received_waypoints = None
    result = None
    errors = None


class r_emergency_command(DataClass):
    id = None
    type = None
    time = None
    emergency_cmd = None


class r_emergency_result(DataClass):
    time = None
    received_time = None
    received_emergency_cmd = None
    result = None
    errors = None


class r_state(DataClass):
    mode = None
    errors = None
    pose = None
    destination = None
    covariance = None
    battery = None


class r_pose(DataClass):
    point = None
    angle = None


class r_battery(DataClass):
    voltage = None
    current_optional = None


class r_current_optional(DataClass):
    valid = None
    current = None
