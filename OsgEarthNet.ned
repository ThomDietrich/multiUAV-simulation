//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
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
        int numCSs = default(0);  // the number of charging stations in the field

    types:
        channel Channel extends ned.DelayChannel {
            delay = 1ms;
        }
    submodules:
        osgEarthScene: OsgEarthScene {
            @display("p=74,31");
        }
        missionControl: MissionControl {
            @display("p=74,150");
        }
        channelController: ChannelController {
            @display("p=243,31");
        }
        uav[numUAVs]: UAVNode {
            @display("p=243,150");
        }
        cs[numCSs]: ChargingNode {
            @display("p=150,100");
        }

    connections:
        for i=0..numUAVs-1 {
            missionControl.gate++ <--> Channel <--> uav[i].gate++;
        }
        for x=0..numUAVs-1, for y=0..numCSs-1 {
            uav[x].gate++ <--> Channel <--> cs[y].gate++;
        }
}
