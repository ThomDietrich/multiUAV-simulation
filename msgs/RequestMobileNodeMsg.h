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

#ifndef MSGS_REQUESTMOBILENODEMSG_H_
#define MSGS_REQUESTMOBILENODEMSG_H_

#include <omnetpp.h>

using namespace omnetpp;

/*
 * RequestMobileNodeMsg provides a wrapper for cMessage class.
 * Intended to request a mobile node which fits the given current requirement.
 */
class RequestMobileNodeMsg : public cMessage {
public:
    RequestMobileNodeMsg(double remaining);
    virtual ~RequestMobileNodeMsg();

protected:
    void addParameters(double remaining);
};

#endif /* MSGS_RESPONSEFORECASTMSG_H_ */
