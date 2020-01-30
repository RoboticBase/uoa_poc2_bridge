# uoa\_poc2\_bridge
This ros package acts as a bridge between [FIWARE](https://www.fiware.org/) and ROS through MQTT.

[![TravisCI Status](https://travis-ci.org/RoboticBase/uoa_poc2_bridge.svg?branch=master)](https://travis-ci.org/RoboticBase/uoa_poc2_bridge/)

## Description
This application bridges a ROS autonomous mobile robot and [RoboticBase](https://github.com/RoboticBase/).

This application was used in a PoC demonstrated by TIS and UoA in November 2019.

## requirements
* ROS [kinetic Kame](http://wiki.ros.org/kinetic)

## prepare

### install dependency
```
rosdep install --from-paths src --ignore-src -r -y
```

## environment variables

|environment variable|description|
|:--|:--|
|MQTT\_HOST|IP address or FQDN of MQTT Broker|
|MQTT\_PORT|Port number of MQTT Broker|
|MQTT\_USERNAME|Username to connect MQTT Broker|
|MQTT\_PASSWORD|Password to connect MQTT Broker|
|MQTT\_USE\_CA|If "true", connect MQTT Broker by using mqtt/tls|
|ENTITY\_TYPE|Type of RoboticBase's Entity|
|ENTITY\_ID|ID of RoboticBase's Entity|
|CMD\_NAME|the command name of RoboticBase's iotagent-json|

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2019 [TIS Inc.](https://www.tis.co.jp/)
