# uoa_poc2_bridge


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
