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

#ifndef MSGS_RESPONSEFORECASTMSG_H_
#define MSGS_RESPONSEFORECASTMSG_H_

#include <omnetpp.h>

using namespace omnetpp;

/*
 * The class ResponseForecastMsg provides a wrapper for cMessage class,
 * Intended to respond to forecast requests.
 * It will always contain a point in time and the reached percentage for that point in time.
 */
class ResponseForecastMsg : public cMessage {
public:
    ResponseForecastMsg(simtime_t pointInTime, double reachedPercentage = 100.0);
    virtual ~ResponseForecastMsg();

protected:
    void addParameters(simtime_t pointInTime, double reachedPercentage);
};

#endif /* MSGS_RESPONSEFORECASTMSG_H_ */
