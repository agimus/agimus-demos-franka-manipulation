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

import numpy as np
from hpp import Transform

## Randomly shoot a configuration where the part is in the box
def shootPartInBox(robot, q0):
    robot.setCurrentConfig(q0)
    r = robot.rankInConfiguration['box/root_joint']
    Mb = Transform(q0[r:r+7])
    q = robot.shootRandomConfig()
    r = robot.rankInConfiguration['part/root_joint']
    q[r+0] = .56 * np.random.uniform() - .28
    q[r+1] = .36 * np.random.uniform() - .18
    q[r+2] = .20 * np.random.uniform()
    print(q[r:r+7])
    Mp = Mb * Transform(q[r:r+7])
    qres = q0[:]
    qres[r:r+7] = list(Mp)
    return qres

## Display a frame in gepetto-gui corresponding to a handle
# \param robot instance of class hpp.corbaserver.robot.Robot
# \param name name of the handle.
def displayHandle(viewer, name):
    robot = viewer.robot
    joint, pose = robot.getHandlePositionInJoint(name)
    link = robot.getLinkNames('part/root_joint')[0]
    hname = 'handle__' + name.replace('/', '_')
    viewer.client.gui.addXYZaxis(hname, [0, 1, 0, 1], 0.005, 0.015)
    viewer.client.gui.addToGroup(hname, robot.name + '/' + link)
    viewer.client.gui.applyConfiguration(hname, pose)

# Generate target config from randomly sampled configurations

def generateTargetConfig(robot, graph, edge, q, Nsamples = 20):
    res = False
    for i in range(Nsamples + 1):
        if i == 0:
            qrand = q[:]
        else:
            qrand = robot.shootRandomConfig()
            r = robot.rankInConfiguration['box/root_joint']
            qrand[r:r+7] = q[r:r+7]
        res, q1, err = graph.generateTargetConfig(edge, q, qrand)
        if not res: continue
        res, msg = robot.isConfigValid(q1)
        if res: break
    if res:
        return True, q1
    else:
        return False, q
