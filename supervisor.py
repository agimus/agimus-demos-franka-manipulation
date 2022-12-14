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

import time
import numpy as np
from dynamic_graph.ros import RosPublish, RosSubscribe
from agimus_sot.action import Action
from franka_gripper.msg import GraspActionGoal, MoveActionGoal

Action.maxControlSqrNorm = 20


class OpenGripper(object):
    """
    Preaction that opens the gripper by publishing on ros topic
    /franka_gripper/move/goal
    """
    # Timeout for opening the gripper
    timeout = 5
    # Tolerance to consider that the gripper is open
    eps = 0.001
    def __init__(self, robot, moveActionGoal):
        self.robot = robot
        self.moveActionGoal = moveActionGoal
        self.ros_publish = RosPublish("pub_OpenGripper")
        signal_name = 'sout'
        self.ros_publish.add('vector', signal_name, '/agimus/sot/open_gripper')
        self.signal = self.ros_publish.signal(signal_name)

    def __call__(self):
        # Send command
        self.signal.value = np.array([self.moveActionGoal.goal.width,
                                      self.moveActionGoal.goal.speed])
        t = self.robot.device.state.time
        self.ros_publish.signal("trigger").recompute(t)
        # wait for 3 seconds
        time.sleep(3.)
        return True, ""

class CloseGripper(object):
    """
    Preaction that opens the gripper by publishing on ros topic
    /franka_gripper/move/goal
    """
    # Tolerance to consider that the gripper is open
    eps = 0.001
    def __init__(self, robot, graspActionGoal):
        self.robot = robot
        self.graspActionGoal = graspActionGoal
        self.ros_publish = RosPublish("pub_CloseGripper")
        signal_name = 'sout'
        self.ros_publish.add('vector', signal_name, '/agimus/sot/close_gripper')
        self.signal = self.ros_publish.signal(signal_name)

    def __call__(self):
        self.signal.value = np.array([self.graspActionGoal.goal.width,
        self.graspActionGoal.goal.epsilon.inner,
        self.graspActionGoal.goal.epsilon.outer,
        self.graspActionGoal.goal.speed,
        self.graspActionGoal.goal.force,])
        t = self.robot.device.state.time
        self.ros_publish.signal("trigger").recompute(t)
        time.sleep(3.)
        return True, ""


def makeSupervisorWithFactory(robot):
    from agimus_sot import Supervisor
    from agimus_sot.factory import Factory, Affordance
    from agimus_sot.srdf_parser import parse_srdf, attach_all_to_link
    import pinocchio
    from rospkg import RosPack
    rospack = RosPack()

    srdf = {}
    # retrieve objects from ros param
    robotDict = globalDemoDict["robots"]
    if len(robotDict) != 1:
        raise RuntimeError("One and only one robot is supported for now.")
    objectDict = globalDemoDict["objects"]
    objects = list(objectDict.keys())
    # parse robot and object srdf files
    srdfDict = dict()
    for r, data in robotDict.items():
        srdfDict[r] = parse_srdf(srdf = data["srdf"]["file"],
                                 packageName = data["srdf"]["package"],
                                 prefix=r)
    for o, data in objectDict.items():
        objectModel = pinocchio.buildModelFromUrdf\
                      (rospack.get_path(data["urdf"]["package"]) + "/" +
                       data["urdf"]["file"])
        srdfDict[o] = parse_srdf(srdf = data["srdf"]["file"],
                                 packageName = data["srdf"]["package"],
                                 prefix=o)
        attach_all_to_link(objectModel, "base_link", srdfDict[o])

    grippers = list(globalDemoDict["grippers"])
    handlesPerObjects = list()
    contactPerObjects = list()
    for o in objects:
        handlesPerObjects.append(sorted(list(srdfDict[o]["handles"].keys())))
        contactPerObjects.append(sorted(list(srdfDict[o]["contacts"].keys())))

    for w in ["grippers", "handles","contacts"]:
        srdf[w] = dict()
        for k, data in srdfDict.items():
            srdf[w].update(data[w])

    envContactSurfaces = list(globalDemoDict["contact surfaces"])
    print (f"env contact surfaces: {envContactSurfaces}")
    supervisor = Supervisor(robot, prefix=list(robotDict.keys())[0])
    factory = Factory(supervisor)
    factory.parameters["period"] = 0.01  # TODO soon: robot.getTimeStep()
    factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
    factory.parameters["addTracerToAdmittanceController"] = False
    # factory.parameters["addTimerToSotControl"] = True
    factory.setGrippers(grippers)
    factory.setObjects(objects, handlesPerObjects, contactPerObjects)
    factory.environmentContacts(envContactSurfaces)

    from hpp.corbaserver.manipulation import Rule
    factory.setupFrames(srdf["grippers"], srdf["handles"], robot)
    for k in handlesPerObjects[0]:
        factory.handleFrames[k].hasVisualTag = False
    factory.setupContactFrames(srdf["contacts"])
    factory.generate()

    moveActionGoal = MoveActionGoal()
    moveActionGoal.goal.width = 0.08
    moveActionGoal.goal.speed = 0.1
    openGripper = OpenGripper(robot, moveActionGoal)
    graspActionGoal = GraspActionGoal()
    graspActionGoal.goal.width = 0.05
    graspActionGoal.goal.epsilon.inner = 0.005
    graspActionGoal.goal.epsilon.outer = 0.005
    graspActionGoal.goal.speed = 0.1
    graspActionGoal.goal.force = 200.
    closeGripper = CloseGripper(robot, graspActionGoal)
    ig = 0; g = factory.grippers[ig]
    for ih, h in enumerate(factory.handles):
        # Add preaction to open the gripper
        transitionName_12 = f'{g} > {h} | f_12'
        supervisor.actions[transitionName_12].preActions.append(openGripper)
        transitionName_23 = f'{g} > {h} | f_23'
        supervisor.actions[transitionName_23].preActions.append(closeGripper)
        transitionName_21 = f'{g} < {h} | {ig}-{ih}_21'
        supervisor.actions[transitionName_21].preActions.append(openGripper)
    supervisor.makeInitialSot()
    return factory, supervisor

factory, supervisor = makeSupervisorWithFactory(robot)

supervisor.plugTopicsToRos()
supervisor.plugSot("")
