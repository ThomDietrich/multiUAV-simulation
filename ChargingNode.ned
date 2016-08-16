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
// A node that serves as a charging station
//
simple ChargingNode extends GenericNode
{
    parameters:
        @class(ChargingNode);
        @display("i=block/plug");
        string modelURL;
        // decorations and annotations
        string labelColor = default("#ffff00ff");    // the color of the model label in hex RRGGBBAA format or "" to turn off labels
        string label2Color = default("#cccc00ff");   // the color of the second model label in hex RRGGBBAA format or "" to turn off labels
        string trailColor = default("#000000ff");    // the color of the trail indicator in hex RRGGBBAA format
        string rangeColor = default("#ff000040");    // the color of the range indicator in hex RRGGBBAA format
        int trailLength = default(0);	             // the length of the trail in time steps showing the past movement of the node. 0 disables the trail
        bool showTxRange = default(false);           // whether to show the transmission range around the nodes
        double txRange @unit("m") = default(200m);   // the transmission range of the mobile node's radio
        double timeStep @unit("s") = default(33ms);  // the time granularity of movement calculation (for 30fps)
        double startTime @unit("s") = default(0s);   // time when the movement starts

        double batteryCapacity @unit("mAh") = default(2200mAh); // the battery capacity in mAh, "0" for infinite energy storage
}
