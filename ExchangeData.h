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

#ifndef EXCHANGEDATA_H_
#define EXCHANGEDATA_H_

#include <omnetpp.h>

//TODO Erzeugt Fehler wegen Ring-include
//#include "UAVNode.h"
class UAVNode;

using namespace omnetpp;

class ExchangeData {
public:
    ExchangeData();
    virtual ~ExchangeData();
    simtime_t timestamp;
    double x, y, z;
    UAVNode* nodeToExchange;
//private:
};

#endif /* EXCHANGEDATA_H_ */
