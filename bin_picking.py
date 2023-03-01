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
from hpp.corbaserver.manipulation import Robot

## This class defines a bin-picking problem.
#
class BinPicking(object):
    maxIter = 100
    def wd(self, o):
        return wrap_delete(o, self.ps.client.basic._tools)

    def __init__(self, ps, graph):
        self.ps = ps
        self.graph = graph
        self.robot = ps.robot
        self.cproblem = self.wd(ps.hppcorba.problem.getProblem())
        self.cgraph = self.wd(self.cproblem.getConstraintGraph())
        self.pathValidations = dict()
        for e in self.graph.edges.keys():
            cedge = self.wd(self.cgraph.get(self.graph.edges[e]))
            self.pathValidations[e] = self.wd(cedge.getPathValidation())

    # Generates N pairs of configurations in pregrasp and grasp
    def generatePregrasps(self, q0, gripper, handle, N):
        nIter = 0
        configs = list()
        while len(configs) < N:
            if nIter > self.maxIter:
                raise RuntimeError("Failed to generate grasp configurations")
            nIter += 1
            # Generate pregrasp configuration
            e = f"{gripper} > {handle} | f_01"
            q_rand = self.robot.shootRandomConfig()
            if nIter == 0: q_rand = q0
            cvalidation = self.pathValidations[e]
            res, qpg, err = self.graph.generateTargetConfig(e, q0, q_rand)
            if not res: continue
            res, msg = cvalidation.validateConfiguration(qpg)
            if not res: continue
            # Generate grasp configuration
            e = f"{gripper} > {handle} | f_12"
            cvalidation = self.pathValidations[e]
            res, qg, err = self.graph.generateTargetConfig(e, qpg, qpg)
            if not res: continue
            res, msg = cvalidation.validateConfiguration(qg)
            if not res: continue
            # Generate preplace configuration
            e = f"{gripper} > {handle} | f_23"
            cvalidation = self.pathValidations[e]
            res, qpp, err = self.graph.generateTargetConfig(e, qg, qg)
            if not res: continue
            res, msg = cvalidation.validateConfiguration(qg)
            if not res: continue
            configs.append((qpg, qg, qpp))
        return configs

if __name__ == "__main__":
    from hpp.rostools import process_xacro, retrieve_resource
    Robot.urdfString = process_xacro\
        ("package://agimus_demos/franka/manipulation/urdf/demo.urdf.xacro",
         "calibration:=false")
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
