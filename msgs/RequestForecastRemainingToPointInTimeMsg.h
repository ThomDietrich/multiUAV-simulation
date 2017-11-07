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

#ifndef MSGS_REQUESTFORECASTREMAININGTOPOINTINTIMEMSG_H_
#define MSGS_REQUESTFORECASTREMAININGTOPOINTINTIMEMSG_H_

#include <omnetpp.h>

using namespace omnetpp;

/*
 * The class RequestForecastRemainingToTargetMsg provides a wrapper for cMessage class,
 * intended to make generation of particular messages as easy as possible.
 */
class RequestForecastRemainingToPointInTimeMsg : public cMessage{
public:
    RequestForecastRemainingToPointInTimeMsg(double remaining, double capacity, simtime_t pointInTime);
    virtual ~RequestForecastRemainingToPointInTimeMsg();

protected:
    void addParameters(double remaining, double capacity, simtime_t pointInTime);
};

#endif /* MSGS_REQUESTFORECASTREMAININGTOPOINTINTIMEMSG_H_ */
