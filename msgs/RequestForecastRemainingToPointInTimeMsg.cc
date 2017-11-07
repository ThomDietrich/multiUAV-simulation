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

#include "RequestForecastRemainingToPointInTimeMsg.h"

RequestForecastRemainingToPointInTimeMsg::RequestForecastRemainingToPointInTimeMsg(double remaining,  double capacity, simtime_t pointInTime)
{
    this->setName("requestForecastRemainingToPointInTime");
    this->addParameters(remaining, capacity, pointInTime);
}

RequestForecastRemainingToPointInTimeMsg::~RequestForecastRemainingToPointInTimeMsg()
{
    // TODO Auto-generated destructor stub
}

void RequestForecastRemainingToPointInTimeMsg::addParameters(double remaining, double capacity, simtime_t pointInTime)
{
    cMsgPar *remainingPar = new cMsgPar("remaining");
    remainingPar->setDoubleValue(remaining);
    cMsgPar *pointInTimePar = new cMsgPar("pointInTime");
    pointInTimePar->setDoubleValue(pointInTime.dbl());

    this->addPar(remainingPar);
    this->addPar(pointInTimePar);
}
