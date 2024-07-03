# ros2_webots_sandbox

A webots simulation environment with ros2 interface for simulating planetary exploration scenarios. Based on [wolfgang_webots_sim](https://github.com/bit-bots/bitbots_main/tree/main/bitbots_wolfgang/wolfgang_webots_sim) by bit-bots.

## Dependencies

This project requires the following ros2 packages to run:

* [bitbots_msgs](https://github.com/bit-bots/bitbots_main/tree/main/bitbots_msgs)
* [bitbots_docs](https://github.com/bit-bots/bitbots_main/tree/main/bitbots_misc/bitbots_docs)

The project also expects some robot models in the proto format. These models cannot be included here due to licensing reasons, but can be generated from their corresponding URDF files by using the [urdf2webots](https://github.com/cyberbotics/urdf2webots) converter by cyberbotics. Currently, the following robot models are being used:

* [LEO Rover](https://github.com/LeoRover/leo_common)
* [Unitree Go2 Edu](https://github.com/unitreerobotics/unitree_ros/tree/master)
* [ExoMy Rover](https://github.com/esa-prl/ExoMy_Model)

Finally, the Webots simulator needs to be installed as well. This code has been tested with Webots R2023b. Information on how to install Webots can be found [here](https://cyberbotics.com/).

## Usage

The code in this repository has been tested on Ros2 rolling. To start the simulation, after building, simply type

```ros2 launch ros2_webots_sandbox --multi_robot:=[true|false]```

The multi_robot parameter switches the scenarios between a single LEO Rover, and all robots.