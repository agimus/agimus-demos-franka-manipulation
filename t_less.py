# Copyright 2023 CNRS
# Author: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

class TLess:
    template =\
    '''<?xml version='1.0' encoding='utf-8'?>
    <robot name="{name}">
    <link name="base_link">
      <inertial>
        <mass value="0.05"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0"
	         iyy="0.001" iyz="0.0"
	         izz="0.001" />
      </inertial>
      <visual>
        <geometry>
          <mesh filename="package://agimus_demos/franka/manipulation/meshes/t-less/obj_{obj_id}.ply" scale="0.001 0.001 0.001"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <mesh filename="package://agimus_demos/franka/manipulation/meshes/t-less/obj_{obj_id}.ply" scale="0.001 0.001 0.001"/>
        </geometry>
      </collision>
    </link>
    </robot>
    '''

    def __init__(self, vf, name, obj_id):
        import os
        ros_paths = os.getenv("ROS_PACKAGE_PATH").split(":")
        for p in ros_paths:
            filename = p + f"/agimus_demos/franka/manipulation/srdf/t-less/obj_{obj_id}.srdf"
            if os.path.exists(filename):
                break
            else:
                filename = None

        if filename is None:
            raise RuntimeError('Failed to find file ' +
                f'"package://agimus_demos/franka/manipulation/srdf/t-less/obj_{obj_id}.srdf"')
        with open(filename) as f:
            srdfString=f.read()
        urdfString = self.template.format(name = name, obj_id = obj_id)
        vf.loadRobotModelFromString(f"obj_{obj_id}", "freeflyer", urdfString,
                                    srdfString)
