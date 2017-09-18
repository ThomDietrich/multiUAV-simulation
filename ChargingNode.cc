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

#ifdef WITH_OSG
#include "ChargingNode.h"

Define_Module(ChargingNode);

ChargingNode::ChargingNode()
{
}

ChargingNode::~ChargingNode()
{
}

void ChargingNode::initialize(int stage)
{
    GenericNode::initialize(stage);
    switch (stage) {
        case 0:
        this->spotsLanding = 5;
        this->spotsCharging = 1;
        this->chargingCurrent = 2000;
        this->x = par("posX");
        this->y = par("posY");
        this->z = 2;
        this->pitch = 0;
        this->yaw = 0;
        break;

        case 1:
        this->labelNode->setText("");
        this->sublabelNode->setText("");
        break;
    }
}

/// TODO: Decouple Charging Node from GenericNode

ReplacementData* ChargingNode::endOfOperation()
{
    return 0;
}

void ChargingNode::loadCommands(CommandQueue commands)
{
}

void ChargingNode::clearCommands()
{
}

void ChargingNode::refreshDisplay() const
{
    GenericNode::refreshDisplay();
}

void ChargingNode::handleMessage(cMessage* msg)
{
//    GenericNode::handleMessage(msg);
    EV_INFO << msg->getFullName() << endl;
    if (msg->isName("startCharge")) {
        EV_INFO << "UAV is ready to get charged" << endl;
    } else if (msg->isName("onMyWay")) {
        EV_INFO << "UAV is on the way to CS, reserve spot" << endl;

    }
}

void ChargingNode::selectNextCommand()
{
}

void ChargingNode::initializeState()
{
}

void ChargingNode::updateState()
{
}

bool ChargingNode::commandCompleted()
{
    return false;
}

double ChargingNode::nextNeededUpdate()
{
    return 100.0;
}

void appendToWaitingQueue()
{

}

#endif // WITH_OSG
