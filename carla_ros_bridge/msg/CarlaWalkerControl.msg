#
# Copyright (c) 2019 Intel Coporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# This represents a walker control message sent to CARLA simulator

Header header

# The CARLA Walker control data

geometry_msgs/Vector3 direction
float32 speed
bool jump

