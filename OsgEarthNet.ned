//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 2015 OpenSim Ltd.
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

//
// The osgEarth Demo network.
//
network OsgEarthNet
{
    parameters:
        @display("bgb=$playgroundWidth,$playgroundHeight;bgi=background/terrain,s");
        double playgroundLatitude; // geographic position of the playground's north-west corner
        double playgroundLongitude; // geographic position of the playground's north-west corner
        double playgroundWidth @unit("m") = default(300m);  // the E-W size of playground
        double playgroundHeight @unit("m") = default(300m); // the N-S size of playground
        int numUAVs = default(0);  // the number of UAVs in the field

    submodules:
        osgEarthScene: OsgEarthScene {
            @display("p=74,31");
        }
        channelController: ChannelController {
            @display("p=243,31");
        }
        uav[numUAVs]: UAVNode {
            @display("p=243,150");
        }
}
