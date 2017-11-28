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

#include "RequestForecastRemainingToTargetMsg.h"

RequestForecastRemainingToTargetMsg::RequestForecastRemainingToTargetMsg(double remaining, double capacity, double targetPercentage)
{
    this->setName("requestForecastRemainingToTarget");
    this->addParameters(remaining, capacity, targetPercentage);
}

RequestForecastRemainingToTargetMsg::~RequestForecastRemainingToTargetMsg()
{
    // TODO Auto-generated destructor stub
}

void RequestForecastRemainingToTargetMsg::addParameters(double remaining, double capacity, double targetPercentage)
{
    cMsgPar *remainingPar = new cMsgPar("remaining");
    remainingPar->setDoubleValue(remaining);
    cMsgPar *capacityPar = new cMsgPar("capacity");
    capacityPar->setDoubleValue(capacity);
    cMsgPar *targetPercentagePar = new cMsgPar("targetPercentage");
    targetPercentagePar->setDoubleValue(targetPercentage);

    this->addPar(remainingPar);
    this->addPar(capacityPar);
    this->addPar(targetPercentagePar);
}

