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
        double posX @unit("m") = default(0m);            // the starting coordinates in meter
        double posY @unit("m") = default(0m);            // the starting coordinates in meter
        double chargeCurrent @unit("A") = default(1.0A); // the charging current, assumed as constant 
        int spotsWaiting = default(0);                   // the amount of spots a incoming node can wait on	
        int spotsCharging = default(0);                  // the amount of spots a node is charged on
}
