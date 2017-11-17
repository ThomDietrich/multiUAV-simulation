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

#ifndef MSGS_RESERVESPOTMSG_H_
#define MSGS_RESERVESPOTMSG_H_

#include <omnetpp.h>
#include "../MobileNode.h"

using namespace omnetpp;

/*
 * ReserveSpotMsg provides a wrapper for cMessage class.
 * Intended to reserve a waiting (or charging spot) at a Charging Station.
 * Due to this information the Charging Station will be able to give more accurate forecast information.
 */

class ReserveSpotMsg : public cMessage {
public:
    ReserveSpotMsg(simtime_t estimatedArrival, double estimatedConsumptionTillArrival, MobileNode* mobileNode);
    virtual ~ReserveSpotMsg();

protected:
    void addParameters(simtime_t estimatedArrival, double estimatedConsumptionTillArrival, MobileNode* mobileNode);
};

#endif /* MSGS_RESERVESPOTMSG_H_ */
