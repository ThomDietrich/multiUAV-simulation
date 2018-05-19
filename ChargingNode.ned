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
        double batteryCapacity @unit("mAh") = default(0mAh); // the capacity of the ChargingStation in mAh
        double chargeEffectivenessPercentage = default(100.0); // the effectiveness of the battery in the Charging Station (100% - effectiveness = how much energy is wasted during charging)
        bool prioritizeFastCharge = default(false);      // when true fast charged is prioritized over full charge
        double chargeCurrent @unit("A") = default(1.0A); // the charging current, assumed as constant 
        int spotsWaiting = default(0);                   // the amount of spots a incoming node can wait on (0 - unlimited)
        int spotsCharging = default(0);                  // the amount of spots a node is charged on (0 - unlimited)
        // values for ChargingAlgorithm
		double linearGradient = default(1.0);
		double expGradient = default(0.001);
		double nonLinearPhaseStartPercentage = default(90.0);
		double nonLinearPhaseLimitPercentage = default(101.0);	
}
