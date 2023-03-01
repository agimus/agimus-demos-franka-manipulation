#!/usr/bin/env python
#
# Copyright (c) 2023 CNRS
# Author: Florent Lamiraux
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

from hpp.corbaserver.manipulation import Constraints

## Class that creates graphs with simple grasps
#
class Factory(object):

    def __init__(self, ps, graph, obj):
        self.ps = ps
        self.graph = graph
        self.obj = obj
        self.startGraph()

    def startGraph(self):
        # Constraint that keeps the object static
        self.static_obj = f"place_{self.obj}/complement"
        self.ps.createLockedJoint(self.static_obj, f"{self.obj}/root_joint",
            [0,0,0,0,0,0,1])
        self.ps.setConstantRightHandSide(self.static_obj, False)
        # Constraint that moves the object vertically
        self.vertical_obj = "vertical_obj"
        self.ps.createTransformationConstraint(self.vertical_obj, "",
            f"{self.obj}/root_joint", [0,0,0,0,0,0,1],
            [True, True, False, True, True, True])
        self.ps.setConstantRightHandSide(self.vertical_obj, False)

        self.graph.createNode("free", False, 0)
        self.graph.createEdge("free", "free", "Loop | f", weight=0,
                              isInNode="free")
        self.graph.addConstraints(edge="Loop | f", constraints = Constraints(
            numConstraints = [self.static_obj]))

    def addGrasp(self, gripper, handle, ig, ih):
        grasp = f"{gripper} grasps {handle}"
        pregrasp = f"{gripper} pregrasps {handle}"
        grasp_comp = f"{gripper} grasps {handle}/complement"
        grasp_hold = f"{gripper} grasps {handle}/hold"
        self.graph.createGrasp(grasp, gripper, handle)
        self.graph.createPreGrasp(pregrasp, gripper, handle)
        self.graph.createNode(grasp, False, 1)
        self.graph.addConstraints(node = grasp, constraints = Constraints(
            numConstraints = [grasp]))
        loop_name = "Loop | {ig}-{ih}"
        self.graph.createEdge(f"{gripper} grasps {handle}",
                              f"{gripper} grasps {handle}", loop_name,
                              weight=0, isInNode=f"{gripper} grasps {handle}")
        self.graph.addConstraints(edge = loop_name, constraints = Constraints(
            numConstraints = [grasp_hold]))
        pregrasp_node = f"{gripper} > {handle} | f_pregrasp"
        self.graph.createNode(pregrasp_node, True)
        self.graph.addConstraints(node = pregrasp_node, constraints =
            Constraints(numConstraints = [pregrasp]))
        intersect_name = f"{gripper} > {handle} | f_intersec"
        self.graph.createNode(intersect_name, True)
        self.graph.addConstraints(node = intersect_name, constraints =
            Constraints(numConstraints=[grasp]))
        preplace_name = f"{gripper} > {handle} | f_preplace"
        self.ps.createTransformationConstraint(preplace_name, "",
            f"{self.obj}/root_joint", [0, 0, .9, 0, 0, 0, 1],
            [False, False, True, False, False, False])
        self.graph.createNode(preplace_name, True)
        self.graph.addConstraints(node=preplace_name, constraints = Constraints(
            numConstraints = [preplace_name, grasp]))
        self.graph.createWaypointEdge("free", f"{gripper} grasps {handle}",
            f"{gripper} > {handle} | f", 3, automaticBuilder=False)
        self.graph.createWaypointEdge(f"{gripper} grasps {handle}", "free",
            f"{gripper} < {handle} | {ig}-{ih}", 3, automaticBuilder=False)
        self.graph.createEdge("free", f"{gripper} > {handle} | f_pregrasp",
            f"{gripper} > {handle} | f_01", -1, "free")
        self.graph.addConstraints(edge = f"{gripper} > {handle} | f_01",
            constraints = Constraints(numConstraints=[self.static_obj]))
        self.graph.createEdge(f"{gripper} > {handle} | f_pregrasp", "free",
            f"{gripper} < {handle} | {ig}-{ih}_10", -1, "free")
        self.graph.addConstraints(edge = f"{gripper} < {handle} | {ig}-{ih}_10",
            constraints = Constraints(numConstraints=[self.static_obj]))
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} > {handle} | f"], 0, self.graph.edges[
            f"{gripper} > {handle} | f_01"], self.graph.nodes[
            f"{gripper} > {handle} | f_pregrasp"],)
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}"], 3, self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}_10"], self.graph.nodes["free"],)
        self.graph.createEdge(f"{gripper} > {handle} | f_pregrasp",
            f"{gripper} > {handle} | f_intersec",
            f"{gripper} > {handle} | f_12", -1, "free")
        self.graph.addConstraints(edge = f"{gripper} > {handle} | f_12",
            constraints = Constraints(numConstraints = [self.static_obj]))
        self.graph.createEdge(f"{gripper} > {handle} | f_intersec",
            f"{gripper} > {handle} | f_pregrasp",
            f"{gripper} < {handle} | {ig}-{ih}_21", -1, "free")
        self.graph.addConstraints(edge = f"{gripper} < {handle} | {ig}-{ih}_21",
            constraints = Constraints(numConstraints = [self.static_obj]))
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} > {handle} | f"], 1, self.graph.edges[
            f"{gripper} > {handle} | f_12"], self.graph.nodes[
            f"{gripper} > {handle} | f_intersec"],)
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}"], 2, self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}_21"], self.graph.nodes[
            f"{gripper} > {handle} | f_pregrasp"],)
        # This edge is not really in "free", but as we associate the hold
        # constraint to it, it is better to put it in an unconstrained state
        self.graph.createEdge(f"{gripper} > {handle} | f_intersec",
            f"{gripper} > {handle} | f_preplace",
            f"{gripper} > {handle} | f_23", -1, "free")
        self.graph.addConstraints(edge = f"{gripper} > {handle} | f_23",
            constraints = Constraints(numConstraints = [grasp_hold,
                                                        self.vertical_obj]))
        # This edge is not really in "free", but as we associate the hold
        # constraint to it, it is better to put it in an unconstrained state
        self.graph.createEdge(f"{gripper} > {handle} | f_preplace",
            f"{gripper} > {handle} | f_intersec",
            f"{gripper} < {handle} | {ig}-{ih}_32", -1, "free")
        self.graph.addConstraints(edge = f"{gripper} < {handle} | {ig}-{ih}_32",
            constraints = Constraints(numConstraints = [grasp_hold,
                                                        self.vertical_obj]))
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} > {handle} | f"], 2, self.graph.edges[
            f"{gripper} > {handle} | f_23"], self.graph.nodes[
            f"{gripper} > {handle} | f_preplace"],)
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}"], 1, self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}_32"], self.graph.nodes[
            f"{gripper} > {handle} | f_intersec"],)
        self.graph.createEdge(f"{gripper} > {handle} | f_preplace",
            f"{gripper} grasps {handle}", f"{gripper} > {handle} | f_34", -1,
            "free")
        self.graph.addConstraints(edge = f"{gripper} > {handle} | f_34",
            constraints = Constraints(numConstraints = [grasp_hold]))
        self.graph.createEdge(f"{gripper} grasps {handle}",
            f"{gripper} > {handle} | f_preplace",
            f"{gripper} < {handle} | {ig}-{ih}_43", -1, "free")
        self.graph.addConstraints(edge = f"{gripper} < {handle} | {ig}-{ih}_43",
            constraints = Constraints(numConstraints = [grasp_hold]))
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} > {handle} | f"], 3, self.graph.edges[
            f"{gripper} > {handle} | f_34"], self.graph.nodes[
            f"{gripper} grasps {handle}"],)
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}"], 0, self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}_43"], self.graph.nodes[
            f"{gripper} > {handle} | f_preplace"],)
