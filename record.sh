#!/bin/bash

rosbag record -o ${1:-recording} \
  /robot/joint_states \
  /joy \
  /robot/limb/right/command_twist_stamped