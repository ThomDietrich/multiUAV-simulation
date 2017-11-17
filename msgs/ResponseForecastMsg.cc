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

#include "ResponseForecastMsg.h"

ResponseForecastMsg::ResponseForecastMsg(simtime_t pointInTime, double reachedPercentage)
{
    this->setName("responseForecast");
    this->addParameters(pointInTime, reachedPercentage);
}

ResponseForecastMsg::~ResponseForecastMsg()
{
    // TODO Auto-generated destructor stub
}

void ResponseForecastMsg::addParameters(simtime_t pointInTime, double reachedPercentage)
{
    cMsgPar *pointInTimePar = new cMsgPar("pointInTime");
    pointInTimePar->setDoubleValue(pointInTime.dbl());
    cMsgPar *reachedPercentagePar = new cMsgPar("reachedPercentage");
    reachedPercentagePar->setDoubleValue(reachedPercentage);

    this->addPar(pointInTimePar);
    this->addPar(reachedPercentagePar);
}

