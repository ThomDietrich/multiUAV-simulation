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

#ifndef MSGS_REQUESTFORECASTREMAININGTOTARGETMSG_H_
#define MSGS_REQUESTFORECASTREMAININGTOTARGETMSG_H_

#include <omnetpp.h>

using namespace omnetpp;

/*
 * The class RequestForecastRemainingToTargetMsg provides a wrapper for cMessage class,
 * intended to make generation of particular messages as easy as possible.
 */
class RequestForecastRemainingToTargetMsg : public cMessage {
public:
    RequestForecastRemainingToTargetMsg(double remaining, double capacity, double targetPercentage = 100.0);
    virtual ~RequestForecastRemainingToTargetMsg();

protected:
    void addParameters(double remaining, double capacity, double targetPercentage);
};

#endif /* MSGS_REQUESTFORECASTREMAININGTOTARGETMSG_H_ */
