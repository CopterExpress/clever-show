# clever-show drone client

Application for remote synchronized control of drones and failsafe drone protection module.

* `client.py` is the main drone communication and control module
* `failsafe.py` is the drone protection module, which creates `/emergency_land` service and monitors the state of the drone in accordance with the logic specified in `[FAILSAFE]` item of `client.ini` settings file

## Requirements

* Ubuntu Bionic or Debian Stretch
* ROS Melodic
* Clover ROS package
* Python 2.7
* Time synchronization client

Can be used with built-in NTP client or with external package for time synchronization like `chrony` on Linux systems.

## Installation

```cmd
pip install -r requirements.txt
```

## Usage

Start roscore with clover package on the drone or on the PC with drone simulator.

Start client:

```cmd
python client.py
```

If you want to start failsafe module, execute in the other terminal:

```cmd
python failsafe.py
```

## Documentation

* English
* [Russian](../docs/ru/client.md)
