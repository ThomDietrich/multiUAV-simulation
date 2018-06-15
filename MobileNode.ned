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
// A mobile node showing a 3D model. Able to display
// its position, trail, transmission range
//
simple MobileNode extends GenericNode
{
    parameters:
        @class(MobileNode);
        @display("i=block/broadcast");

        // decorations and annotations
        int trailLength = default(0);                      // the length of the trail in time steps showing the past movement of the node. 0 disables the trail        
        string trailColor = default("#000000ff");          // the color of the trail indicator in hex RRGGBBAA format
        int commandCount = default(3);                     // the length of the waypoint movement in time steps showing the planned movement of the node. 0 shows all
        string commandPreviewColor = default("#00000055"); // the color of the command preview in hex RRGGBBAA format
        bool commandPreviewEnabled = default(true);        // decides whether or not to display command preview

        //energy consumption
        int batteryCapacity @unit("mAh") = default(2200mAh);  // the battery capacity (in mAh, "0" for infinite energy storage)
        int batteryRemaining @unit("mAh") = default(2200mAh); // the battery capacity remaining on simulation start (in mAh, must be equal or less than batteryCapacity)
}
