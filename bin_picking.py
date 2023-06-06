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

from hpp.corbaserver import Client, loadServerPlugin, shrinkJointRange, \
    wrap_delete
from hpp.corbaserver.manipulation import Client as ManipClient, ProblemSolver
from hpp.corbaserver.manipulation import Constraints, Robot, Rule
from hpp.corbaserver.bin_picking import Client as BpClient
from agimus_demos import InStatePlanner
from create_graph import makeGraph
from agimus_demos.tools_hpp import concatenatePaths
from tools_hpp import displayGripper, displayHandle, generateTargetConfig, \
    shootPartInBox

## This class defines a bin-picking problem.
#
class BinPicking(object):
    objects = list()
    """
    List of object names. The part to grasp is the first one.
    """
    robotGrippers = list()
    """
    List of robot grippers to grasp the object
    """
    goalGrippers = list()
    """
    List of gripper that define goal poses of the object, ranked in order of
    priority. Using only one robot gripper, it may not be possible to release
    the object always in the same pose.
    """
    nh = 16
    """
    Number of handles to produce when discretizing
    """
    handles = list()
    """
    List of handles of the object to grasp
    """
    handlesToDiscretize = list()
    """
    List of handles to discretize
    """
    goalHandles = list()
    """
    List of handles that define the goal poses of the object. mapped one to
    one with goalGrippers.
    """
    graphConstraints = list()
    """
    List of constraints to be added to the constraint graph: quasi-static
    equilibrium, gripper open...
    """
    goalConfigs = dict()
    """
    List of precomputed goal configurations to release the object
    """
    q_goal = None
    """
    Configuration with which goal configurations are precomputed.
    This configuration defines the poses of objects other than the part.
    """
    def wd(self, o):
        return wrap_delete(o, self.ps.client.basic._tools)

    def __init__(self, ps):
        self.ps = ps
        self.robot = ps.robot
        self.bpc = BpClient()
        # Store pregrasp configurations for releasing the object
        # self.goalConfig["pregrasp"][gripper][handle] = config
        # where config is the configuration to reach when the object is
        # grasped by (gripper, handle)
        self.goalConfigs["pregrasp"] = dict()
        # Store grasp configurations for releasing the object
        self.goalConfigs["grasp"] = dict()
        # Store preplace configurations for releasing the object
        self.goalConfigs["preplace"] = dict()
        # store which pair (goalGripper, goalHandle) corresponds to the
        # goal configuration when the object is grasped by (gripper, handle)
        # self.goalGrasps[gripper][handle] = i, where
        # goalGripper = self.goalGrippers[i]
        # goalHandle = self.goalHandles[i]
        self.goalGrasps = dict()


    def _rules(self):
        """
        Build rules for the constraint graph factory
        """
        rules = list()
        # robot grippers should not grasp goal handles
        for gripper in self.robotGrippers:
            for handle in self.goalHandles:
                rules += [ Rule(grippers=[gripper], handles = [handle],
                                link=False),]
        # Each goal gripper can only grasp one goal handle
        if len(self.goalGrippers) != len(self.goalHandles):
            raise RuntimeError(
                f"number of goal grippers ({len(self.goalGrippers)})"
                + " should be the same as " +
                f"number of goal handles ({len(self.goalHandles)})"
            )
        for ig, gripper in enumerate(self.goalGrippers):
            for ih, handle in enumerate(self.goalHandles):
                if ig != ih:
                    rules  += [
                        Rule(grippers=[gripper], handles = [handle], link=False)
                        ]
        # goal grippers should not grasp regular part handles
        for gripper in self.goalGrippers:
            for handle in self.initialHandles:
                rules += [
                    Rule(grippers=[gripper], handles=[handle], link=False)
                    ]
            for handle in self.handlesToDiscretize:
                rules += [
                    Rule(grippers=[gripper], handles=[handle + "*"], link=False)
                    ]
        rules += [
            Rule(grippers=self.robotGrippers + self.goalGrippers,
                 handles = [".*", "part/center1", "part/center2"],
                 link=False),
            Rule(grippers=[".*"], handles=[".*"], link=True)
        ]
        return rules

    def buildGraph(self):
        """
        Build the constraint graph
          - discretize handles,
        """
        # store list of not discretized handles
        self.initialHandles = self.handles[:]
        # Discretize handles
        for h in self.handlesToDiscretize:
            self.bpc.bin_picking.discretizeHandle(h, self.nh)
            self.handles += ['%s_%03d' % (h, i) for i in range(self.nh)]
        # build graph
        self.factory, self.graph = makeGraph(
            self.ps, self.robot, self.robotGrippers + self.goalGrippers,
            self.objects, [self.handles + self.goalHandles, []], self._rules())
        self.graph.addConstraints(graph=True,
            constraints = Constraints(numConstraints = self.graphConstraints))
        self.graph.initialize()
        # Make sure previous instance is destroyed if any before creating
        # a new instance. This will avoid troubles with undesired CORBA server
        # destruction.
        self.inStatePlanner = None
        self.inStatePlanner = InStatePlanner(self.ps, self.graph)
        self.inStatePlanner.timeOutPathPlanning = 3.
        self.inStatePlanner.optimizerTypes = ["EnforceTransitionSemantic",
                                              "SimpleTimeParameterization"]
        self.inStatePlanner.parameters['SimpleTimeParameterization/order'] = 2
        self.inStatePlanner.parameters\
            ['SimpleTimeParameterization/maxAcceleration'] = 2.
        self.inStatePlanner.parameters\
            ['SimpleTimeParameterization/safety'] = .95
    def buildEffectors(self, obstacles, q):
        """
        build and effector to test collision of grasps
          - name: name of the effector,
          - gripper: name of the gripper,
          - obstacles: list of obstacle name with which the effector with
                       be tested for collision,
          - q: any configuration where the various geometries of the gripper are
               in the right relative pose.
        """
        for gripper in self.robotGrippers:
            # find any edge that grasps the object
            edge = None
            for e in self.graph.edges.keys():
                if e.startswith(gripper + " > ") and e.endswith("12"):
                    edge = e
                    break
            if edge is None:
                raise RuntimeError(
                    f"Did not find any edge where {gripper} grasps naything")
            self.bpc.bin_picking.createEffector(gripper, gripper, q,
                                                self.graph.edges[edge])
        for o in obstacles:
            self.bpc.bin_picking.addObstacleToEffector(gripper, o, 0.)

    def generateGoalConfigs(self, q):
        """
        Precompute goal configurations for the part hold by the robot
          - q configuration that provides the pose of other objects
        """
        self.q_goal = q[:]
        nrg = len(self.robotGrippers)
        nh = len(self.handles)
        for irg, robotGripper in enumerate(self.robotGrippers):
            self.goalConfigs["pregrasp"][robotGripper] = dict()
            self.goalConfigs["grasp"][robotGripper] = dict()
            self.goalConfigs["preplace"][robotGripper] = dict()
            self.goalGrasps[robotGripper] = dict()

            for ih, handle in enumerate(self.handles):
                for igg, (goalGripper, goalHandle) in enumerate(
                        zip(self.goalGrippers, self.goalHandles)):
                    edge = f"{goalGripper} > {goalHandle} | {irg}-{ih}_01"
                    assert(edge in self.graph.edges.keys())
                    res, q1 = generateTargetConfig(self.robot, self.graph,
                                                   edge, q)
                    if not res: continue
                    edge = f"{goalGripper} > {goalHandle} | {irg}-{ih}_12"
                    assert(edge in self.graph.edges.keys())
                    res, q2 = generateTargetConfig(self.robot, self.graph,
                                                   edge, q1)
                    if not res: continue
                    edge = f"{robotGripper} < {handle} | {irg}-{ih}:" + \
                        f"{igg+nrg}-{igg+nh}_21"
                    assert(edge in self.graph.edges.keys())
                    res, q3 = generateTargetConfig(self.robot, self.graph,
                                                   edge, q2)
                    self.goalConfigs["pregrasp"][robotGripper][handle] = q1
                    self.goalConfigs["grasp"][robotGripper][handle] = q2
                    self.goalConfigs["preplace"][robotGripper][handle] = q3
                    self.goalGrasps[robotGripper][handle] = igg
                    if res: break

    def checkObjectPoses(self, q):
        """
        Check that
          - the initial configuration is in state "free",
          - each object is in the same pose as in the configuration
            that was used to precompute the goal configurations. Index 0
            corresponds to the part and thus is skipped.
        """
        if self.graph.getNode(q) != "free":
            raise RuntimeError(
                f"Initial configuration {q} not in state 'free'.")
        if not self.q_goal:
            raise RuntimeError(
                "You need to call method generateGoalConfigs first.")
        for o in self.objects[1:]:
            r = self.robot.rankInConfiguration[f"{o}/root_joint"]
            if q[r:r+7] != self.q_goal[r:r+7]:
                raise RuntimeError(f"Object {o} is in pose {q[r:r+7]} but " +
                    "was in pose {self.q_goal[r:r+7]} when pre-computing" +
                    " goal configurations.")

    def computeFreeGrasps(self, q):
        """
        For each gripper, compute list of object handles that are not in
        collision in the given configuration.

        Result is a dictionary of keys the robot grippers and with values
        lists of handles.
        """
        self._freeGrasps = dict()
        res = False
        for gripper in self.robotGrippers:
            self._freeGrasps[gripper] = list()
            for handle in self.handles:
                col, msg = self.bpc.bin_picking.collisionTest(
                    gripper, handle, q)
                if not col:
                    self._freeGrasps[gripper].append(handle)
                    res = True
        return True

    def selectGrasp(self, q):
        """
        Select a grasp among all possible collision-free grasps
          - q initial configuration of the system,
          - return gripper, handle, and 6 waypoint configurations corresponding
              to - pregrasp, grasp, preplace configurations to grasp the object,
                 - pregrasp, grasp, preplace configurations to release the
                   object.
        """
        for gripper in self.robotGrippers:
            for handle in self._freeGrasps[gripper]:
                # check that goal configuration exists for this grasp
                q4 = self.goalConfigs["pregrasp"][gripper].get(handle)
                if q4 is None: continue
                q5 = self.goalConfigs["grasp"][gripper].get(handle)
                if q5 is None: continue
                q6 = self.goalConfigs["preplace"][gripper].get(handle)
                if q6 is None: continue
                # generate pregrasp, grasp and preplace configuration to
                # grasp the object
                edge = f"{gripper} > {handle} | f_01"
                res, q1 = generateTargetConfig(self.robot, self.graph, edge, q)
                if not res: continue
                edge = f"{gripper} > {handle} | f_12"
                res, q2 = generateTargetConfig(self.robot, self.graph, edge, q1)
                if not res: continue
                edge = f"{gripper} > {handle} | f_23"
                res, q3 = generateTargetConfig(self.robot, self.graph, edge, q2)
                if not res: continue
                if res:
                    return (gripper, handle, q1, q2, q3, q4, q5, q6)
        return 8*(None,)


    def solve(self, q):
        """
        Compute a trajectory to grasp and release the object
          - q initial configuration of the robot and objects
        """
        self.checkObjectPoses(q)
        res = self.computeFreeGrasps(q)
        if not res:
            msg = "End effector in collision for all possible grasps."
            return False, msg
        gripper, handle, q1, q2, q3, q4, q5, q6 = self.selectGrasp(q)
        if gripper is None:
            msg = "Failed to generate collision-free grasp with valid goal " +\
                "configurations"
            return False, msg
        # Plan paths between waypoint configurations
        edge = "Loop | f"
        self.inStatePlanner.setEdge(edge)
        try:
            p1 = self.inStatePlanner.computePath(q, [q1,], resetRoadmap = True)
        except Exception as exc:
            raise RuntimeError(f"Failed to connect {q} and {q1}: {exc}")
        edge = f"{gripper} > {handle} | f_12"
        self.inStatePlanner.setEdge(edge)
        res, p2, msg = self.inStatePlanner.directPath(q1, q2, False)
        assert(res)
        p2 = self.inStatePlanner.timeParameterization(p2.asVector())
        edge = f"{gripper} > {handle} | f_23"
        self.inStatePlanner.setEdge(edge)
        res, p3, msg = self.inStatePlanner.directPath(q2, q3, False)
        assert(res)
        p3 = self.inStatePlanner.timeParameterization(p3.asVector())
        ig = self.robotGrippers.index(gripper)
        ih = self.handles.index(handle)
        edge = f"Loop | {ig}-{ih}"
        self.inStatePlanner.setEdge(edge)
        try:
            p4 = self.inStatePlanner.computePath(q3, [q4,], resetRoadmap = True)
        except Exception as exc:
            raise RuntimeError(f"Failed to connect {q3} and {q4}: {exc}")
        igg = self.goalGrasps[gripper][handle]
        ng = len(self.robotGrippers); nh = len(self.handles)
        goalGripper = self.goalGrippers[igg]
        goalHandle = self.goalHandles[igg]
        edge = f"{goalGripper} > {goalHandle} | {ig}-{ih}_12"
        self.inStatePlanner.setEdge(edge)
        res, p5, msg = self.inStatePlanner.directPath(q4, q5, False)
        assert(res)
        p5 = self.inStatePlanner.timeParameterization(p5.asVector())
        edge = f"{gripper} < {handle} | {ig}-{ih}:{ng+igg}-{nh+igg}_21"
        self.inStatePlanner.setEdge(edge)
        res, p6, msg = self.inStatePlanner.directPath(q5, q6, False)
        assert(res)
        p6 = self.inStatePlanner.timeParameterization(p6.asVector())
        edge = "Loop | f"
        self.inStatePlanner.setEdge(edge)
        # Return to initial configuration
        q7 = q[:]
        r = self.robot.rankInConfiguration[f"{self.objects[0]}/root_joint"]
        q7[r:r+7] = q6[r:r+7]
        try:
            p7 = self.inStatePlanner.computePath(q6, [q7,], resetRoadmap = True)
        except Exception as exc:
            raise RuntimeError(f"Failed to connect {q6} and {q7}: {exc}")
        return True, concatenatePaths([p1,p2,p3,p4,p5,p6,p7])

if __name__ == "__main__":
    from hpp.rostools import process_xacro, retrieve_resource
    Robot.urdfString = process_xacro\
        ("package://agimus_demos/franka/manipulation/urdf/demo.urdf.xacro")
    Robot.srdfString = ""

    problems = ["default", "learning"]
    defaultContext = "corbaserver"
    loadServerPlugin (defaultContext, "manipulation-corba.so")
    cl = ManipClient()
    for p in problems:
        res = cl.problem.selectProblem(p)
        if res: print(f'created problem "{p}"')
        else: print(f'did not create problem "{p}"')
        cl.problem.resetProblem()

    cl.problem.selectProblem("default")
    robot = Robot("robot", "pandas", rootJointType="anchor")
    shrinkJointRange(robot, [f'pandas/panda2_joint{i}' for i in range(1,8)],
                     0.95)
    ps = ProblemSolver(robot)
