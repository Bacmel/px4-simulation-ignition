# !/bin/bash

path=$(readlink -f ${BASH_SOURCE:-$0})
DIR_PATH=$(dirname $path)

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$IGN_GAZEBO_SYSTEM_PLUGIN_PATH:$DIR_PATH/build
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$DIR_PATH/models
export IGN_DESCRIPTOR_PATH=$IGN_DESCRIPTOR_PATH:$DIR_PATH/build