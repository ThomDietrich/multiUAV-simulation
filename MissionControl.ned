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
// the deciding and commanding entity
//
simple MissionControl
{
    parameters:
        @display("i=block/table2");
        double startTime @unit("s") = default(2s);   // time when the mission scheduling to UAVs starts
        string missionFiles = default("BostonParkCircle.waypoints"); // comma separated string with path(s) to file(s) from which missions shall be loaded
        int replacementSearchMethod = default(0); // 0: Closest
                                                  // 1: HighestChargeAtReplacement

    gates:
        inout gate[];
}
