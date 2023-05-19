# Copyright 2022 CNRS
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

from math import pi, sqrt
from hpp.corbaserver import loadServerPlugin, shrinkJointRange
from hpp.corbaserver.manipulation import Robot, \
    createContext, newProblem, ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Rule, Constraints, CorbaClient, SecurityMargins
from hpp.gepetto.manipulation import ViewerFactory
from agimus_demos.tools_hpp import RosInterface
from hpp.corbaserver import wrap_delete
from create_graph import Factory
from t_less import TLess
from hpp.corbaserver.bin_picking import Client as BpClient
from tools_hpp import displayHandle, generateTargetConfig, shootPartInBox

connectedToRos = False

try:
    import rospy
    Robot.urdfString = rospy.get_param('robot_description')
    print("reading URDF from ROS param")
    connectedToRos = True
except:
    print("reading generic URDF")
    from hpp.rostools import process_xacro, retrieve_resource
    Robot.urdfString = process_xacro\
      ("package://agimus_demos/franka/manipulation/urdf/demo.urdf.xacro")
Robot.srdfString = ""

class Box:
    urdfFilename="package://agimus_demos/franka/manipulation/urdf/big_box.urdf"
    srdfFilename="package://agimus_demos/franka/manipulation/srdf/big_box.srdf"
    rootJointType = "freeflyer"

defaultContext = "corbaserver"
loadServerPlugin(defaultContext, "manipulation-corba.so")
loadServerPlugin(defaultContext, "bin_picking.so")
newProblem()

robot = Robot("robot", "pandas", rootJointType="anchor")
shrinkJointRange(robot, [f'pandas/panda2_joint{i}' for i in range(1,8)],0.95)
ps = ProblemSolver(robot)

ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")
ps.setParameter('SimpleTimeParameterization/order', 2)
ps.setParameter('SimpleTimeParameterization/maxAcceleration', .5)
ps.setParameter('SimpleTimeParameterization/safety', 0.95)

# Add path projector to avoid discontinuities
ps.selectPathProjector ("Progressive", .05)
ps.selectPathValidation("Graph-Progressive", 0.01)
vf = ViewerFactory(ps)
part = TLess(vf, name="part", obj_id="01")
vf.loadRobotModel (Box, "box")
robot.setJointBounds('part/root_joint', [-1., 1., -1., 1., -0.8, 1.5])
robot.setJointBounds('box/root_joint', [-1., 1., -1., 1., -0.8, 1.5])

print("Part and box loaded")

robot.client.manipulation.robot.insertRobotSRDFModel\
    ("pandas", "package://agimus_demos/franka/manipulation/srdf/demo.srdf")

# Discretize handles
handles = list()
nh = 16
bpc = BpClient()

for h in ['part/lateral_top', 'part/lateral_bottom', 'part/top',
          'part/bottom' ]:
    bpc.bin_picking.discretizeHandle(h, nh)
    handles += ['%s_%03d' % (h, i) for i in range(nh)]

# Create a handle and a gripper for the goal position of the object
# Create specific graph with vertical preplace motions
graph = ConstraintGraph(robot, 'graph')
# create preplacement and placement constraints for the object
ps.createTransformationConstraint("place_part", "", "part/root_joint",
    [0,0,0,0,0,0,1], 6*[False])
ps.createLockedJoint("place_part/complement", "part/root_joint",
    [0,0,0,0,0,0,1])
ps.setConstantRightHandSide("place_part/complement", False)
ps.createTransformationConstraint("preplace_part", "box/root_joint",
    "part/root_joint", [0, 0, .30, 0, 0, 0, 1],
    [False, False, True, False, False, False])
ps.createLockedJoint("place_box/complement", "box/root_joint",
    [0,0,0,0,0,0,1])
ps.setConstantRightHandSide("place_box/complement", False)
ps.createTransformationConstraint("vertical_part", "",
            f"part/root_joint", [0,0,0,0,0,0,1],
            [True, True, False, True, True, True])
ps.setConstantRightHandSide("vertical_part", False)

factory = ConstraintGraphFactory(graph)
factory.constraints.removeEmptyConstraints = False
factory.setGrippers(['pandas/panda2_gripper'])
factory.setObjects(["part"], [handles], [[]])
factory.generate()
for edge in graph.edges.keys():
    graph.addConstraints(edge = edge,
        constraints = Constraints(numConstraints = ["place_box/complement"]))
gripper = "pandas/panda2_gripper"; ig = 0
for ih, h in enumerate(handles):
    edge = f"{gripper} > {h} | f_23"
    graph.addConstraints(edge = edge,
        constraints = Constraints(numConstraints = ["vertical_part"]))
    edge = f"{gripper} < {h} | {ig}-{ih}_32"
    graph.addConstraints(edge = edge,
        constraints = Constraints(numConstraints = ["vertical_part"]))

q0 = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4, 0.035, 0.035,
      0, 0, 1.2, 0, 0, 0, 1,
      0, 0, 0.761, 0, 0, 0, 1]
# Lock gripper in open position.
ps.createLockedJoint('locked_finger_1', 'pandas/panda2_finger_joint1', [0.035])
ps.createLockedJoint('locked_finger_2', 'pandas/panda2_finger_joint2', [0.035])
ps.setConstantRightHandSide('locked_finger_1', True)
ps.setConstantRightHandSide('locked_finger_2', True)
graph.addConstraints(graph=True,
                     constraints = Constraints(numConstraints =
                        ['locked_finger_1', 'locked_finger_2']))
graph.initialize()


if connectedToRos:
    ri = RosInterface(robot)
    q = ri.getCurrentConfig(q0)
else:
    q = q0[:]

# Create effector
edge = 'pandas/panda2_gripper > part/lateral_top_000 | f_01'
bpc.bin_picking.createEffector('effector', 'pandas/panda2_gripper', q0,
                             graph.edges[edge])
for i in range(5):
    bpc.bin_picking.addObstacleToEffector('effector', f'box/base_link_{i}', 0.)

found = False
while not found:
    q = shootPartInBox(robot, q0)
    found, msg = robot.isConfigValid(q)

freeHandles = list()

for handle in handles:
    res, msg = bpc.bin_picking.collisionTest('effector', handle, q)
    if not res:
        freeHandles.append(handle)

freeGrasps = list()

for handle in freeHandles:
    edge = f"pandas/panda2_gripper > {handle} | f_01"
    res, q1 = generateTargetConfig(robot, graph, edge, q)
    if not res: continue
    edge = f"pandas/panda2_gripper > {handle} | f_12"
    res, q2 = generateTargetConfig(robot, graph, edge, q1)
    if not res: continue
    edge = f"pandas/panda2_gripper > {handle} | f_23"
    res, q3 = generateTargetConfig(robot, graph, edge, q2)
    if not res: continue
    freeGrasps.append((q1, q2, q3))
