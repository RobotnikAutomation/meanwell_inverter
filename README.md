# meanwell_inverter
ROS package to control MeanWell Inverter

# Dependencies

* [robotnik_msgs](https://github.com/RobotnikAutomation/robotnik_msgs.git)

# Available params

* port: serial used for the connection

# Launch the node

```
$ rosrun meanwell_inverter meanwell_inverter_node.py
```

# Topics

## State

To read the state of the node

```
~state (type robotnik_msgs/State)

```

## InverterStatus

To read the status of the MeanWell Inverter

```
~inverter_status (type robotnik_msgs/InverterStatus)

```

# Services

## To switch on/off the MeanWell Inverter

```
~toggle (type std_srvs/SetBool)

```


