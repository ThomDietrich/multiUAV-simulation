//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 2015 OpenSim Ltd.
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

//
// A mobile node that follows a track defined by a set of points
//
simple UAVNode extends MobileNode
{
    parameters:
        @class(UAVNode);
        @display("i=block/broadcast");
        double startX @unit("m") = default(0m);          // the starting coordinates in meter
        double startY @unit("m") = default(0m);          // the starting coordinates in meter
        double startZ @unit("m") = default(2m);          // the starting coordinates in meter
        int replacementMethod = default(0);              // 0: latest opportunity heuristic
                                                         // 1: shortest return heuristic
                                                         // 2: utilization quotient heuristic
}

//
// mod->getName()       - uav
// mod->getClassName()  - UAVNode
// mod->getFullName()   - uav[0]
// mod->getFullPath()   - OSgEarthNet.uav[0]
//