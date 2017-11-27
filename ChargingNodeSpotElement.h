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
    simtime_t estimatedArrival;
    simtime_t reservationTime;

public:
    /*
     * Constructor with target capacity
     */
    ChargingNodeSpotElement(MobileNode* node, double estimatedChargeDuration, double estimatedWaitingDuration, double targetCapacityPercentage = 100.0);

    virtual ~ChargingNodeSpotElement();

    double getEstimatedChargeDuration() const
    {
        return estimatedChargeDuration;
    }

    double getEstimatedWaitingDuration() const
    {
        return estimatedWaitingDuration;
    }

    MobileNode* getNode()
    {
        return node;
    }

    const simtime_t& getPointInTimeWhenChargingStarted() const
    {
        return pointInTimeWhenChargingStarted;
    }

    void setPointInTimeWhenChargingStarted(const simtime_t& pointInTimeWhenChargingStarted)
    {
        this->pointInTimeWhenChargingStarted = pointInTimeWhenChargingStarted;
    }

    const simtime_t& getPointInTimeWhenDone() const
    {
        return pointInTimeWhenDone;
    }

    double getTargetCapacityPercentage() const
    {
        return targetCapacityPercentage;
    }

    void setEstimatedArrival(const simtime_t& estimatedArrival)
    {
        this->estimatedArrival = estimatedArrival;
    }

    const simtime_t& getEstimatedArrival() const
    {
        return estimatedArrival;
    }

    void setReservationTime(const simtime_t& reservationTime)
    {
        this->reservationTime = reservationTime;
    }

    const simtime_t& getReservationTime() const
    {
        return reservationTime;
    }
};

#endif /* CHARGINGNODESPOTELEMENT_H_ */
