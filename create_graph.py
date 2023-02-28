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

## Class that creates graphs with simple grasps
#
class Factory(object):

    def __init__(self, graph):
        self.graph = graph
        
    def startGraph(self):
        self.graph.createNode("free", False, 0)
        self.graph.createEdge("free", "free", Loop | f, weight=0,
                              isInNode="free")

    def addGrasp(self, gripper, handle, ig, ih):
        self.graph.createNode(f"{gripper} grasps {handle}", False, 1)
        self.graph.createEdge(f"{gripper} grasps {handle}",
                              f"{gripper} grasps {handle}", "Loop | {ig}-{ih}",
                              weight=0, isInNode=f"{gripper} grasps {handle}")
        self.graph.createNode(f"{gripper} > {handle} | f_pregrasp", True)
        self.graph.createNode(f"{gripper} > {handle} | f_intersec", True)
        self.graph.createNode(f"{gripper} > {handle} | f_preplace", True)
        self.graph.createWaypointEdge(f"{gripper} grasps {handle}", "free",
            f"{gripper} < {handle} | {ig}-{ih}", 3, automaticBuilder=False)
        self.graph.createEdge("free", f"{gripper} > {handle} | f_pregrasp",
            f"{gripper} > {handle} | f_01", -1)
        self.graph.createEdge(f"{gripper} > {handle} | f_pregrasp", "free",
            f"{gripper} < {handle} | {ig}-{ih}_10", -1)
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} > {handle} | f"], 0, self.graph.edges[
            f"{gripper} > {handle} | f_01"], self.graph.nodes[
            f"{gripper} > {handle} | f_pregrasp"],)
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}"], 3, self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}_10"], self.graph.nodes["free"],)
        self.graph.createEdge(f"{gripper} > {handle} | f_pregrasp",
            f"{gripper} > {handle} | f_intersec",
            f"{gripper} > {handle} | f_12", -1)
        self.graph.createEdge(f"{gripper} > {handle} | f_intersec",
            f"{gripper} > {handle} | f_pregrasp",
            f"{gripper} < {handle} | {ig}-{ih}_21", -1)
        self.graph.graph.setWaypoint(self.graph.edges[
            "{gripper} > {handle} | f"], 1, self.graph.edges[
            f"{gripper} > {handle} | f_12"], self.graph.nodes[
            f"{gripper} > {handle} | f_intersec"],)
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}"], 2, self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}_21"], self.graph.nodes[
            f"{gripper} > {handle} | f_pregrasp"],)
        self.graph.createEdge(f"{gripper} > {handle} | f_intersec",
            f"{gripper} > {handle} | f_preplace",
            f"{gripper} > {handle} | f_23", -1)
        self.graph.createEdge(f"{gripper} > {handle} | f_preplace",
            f"{gripper} > {handle} | f_intersec",
            f"{gripper} < {handle} | {ig}-{ih}_32", -1)
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} > {handle} | f"], 2, self.graph.edges[
            f"{gripper} > {handle} | f_23"], self.graph.nodes[
            f"{gripper} > {handle} | f_preplace"],)
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}"], 1, self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}_32"], self.graph.nodes[
            f"{gripper} > {handle} | f_intersec"],)
        self.graph.createEdge(f"{gripper} > {handle} | f_preplace",
            f"{gripper} grasps {handle}", f"{gripper} > {handle} | f_34", -1)
        self.graph.createEdge(f"{gripper} grasps {handle}",
            f"{gripper} > {handle} | f_preplace",
            f"{gripper} < {handle} | {ig}-{ih}_43", -1)
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} > {handle} | f"], 3, self.graph.edges[
            f"{gripper} > {handle} | f_34"], self.graph.nodes[
            f"{gripper} grasps {handle}"],)
        self.graph.graph.setWaypoint(self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}"], 0, self.graph.edges[
            f"{gripper} < {handle} | {ig}-{ih}_43"], self.graph.nodes[
            f"{gripper} > {handle} | f_preplace"],)
