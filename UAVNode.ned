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
        double speed @unit("mps") = default(10mps);  // the speed of the mobile nodes
        double startX @unit("m") = default(0m);      // the starting coordinates in meter
        double startY @unit("m") = default(0m);      // the starting coordinates in meter
        string trackFile;                            // a file with longitude latitude pairs
}
