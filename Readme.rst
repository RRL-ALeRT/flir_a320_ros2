FLIR ROS2 image publisher
=========================

Installation
------------
Install spinnaker sdk in default location (/opt)

* Tested on Ubuntu 22.04 with ROS2 Humble

Run
---

.. code-block:: bash

    ros2 run flir_a320_ros2 flir_node

Run mac address specified (Serial no. isn't visible for our device)
-------------------------------------------------------------------

.. code-block:: bash

    ros2 run flir_a320_ros2 flir_node --ros-args -p mac_address:="00:11:1C:01:97:XX"
