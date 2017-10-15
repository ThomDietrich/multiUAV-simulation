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

#ifndef CHARGINGNODESPOTELEMENT_H_
#define CHARGINGNODESPOTELEMENT_H_

#include "MobileNode.h"

/*
 * Class ChargingNodeSpotElement
 * Represents one element on a waiting or charging spot of the charging station.
 * Is used to carry further information before and during the charging process.
 *
 */
class ChargingNodeSpotElement {
protected:
    MobileNode* node;
    double targetCapacityPercentage;
    double estimatedChargeDuration;
    double estimatedWaitingDuration;
    simtime_t pointInTimeWhenDone;
    simtime_t pointInTimeWhenChargingStarted;
public:
    /*
     * Constructor with target capacity
     */
    ChargingNodeSpotElement(MobileNode* node, double estimatedChargeDuration, double estimatedWaitingDuration, double targetCapacityPercentage = 100.0);

    virtual ~ChargingNodeSpotElement();

    /*
     * Getter and Setter
     */
    MobileNode* getNode() const;
    double getTargetCapacityPercentage() const;
    double getEstimatedChargeDuration() const;
    double getEstimatedWaitingDuration() const;
    simtime_t getPointInTimeWhenDone() const;
    simtime_t getPointInTimeWhenChargingStarted() const;
    void setPointInTimeWhenChargingStarted(simtime_t pointInTimeWhenChargingStarted);
};

#endif /* CHARGINGNODESPOTELEMENT_H_ */
