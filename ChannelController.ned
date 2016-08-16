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
// This module is responsible for tracking the distance of mobile nodes,
// and visualizing the connectivity graph via KML.
//
simple ChannelController
{
    @display("i=block/network2");
    bool showConnections = default(true); // whether to show the connection graph (all nodes within range are connected by a line)
    string connectionColor = default(""); // the color of the connection graph in hex BBGGRR format or "" for random color
}
