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
#include "MobileNode.h";

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
        this->spotsWaiting = 5;
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
        cModule* mod = msg->getSenderModule();
        EV_DEBUG << "Class name: " << mod->getClassName() << endl;
        MobileNode *mn = check_and_cast<MobileNode*>(msg->getSenderModule());
        this->appendToObjectsWaiting(mn);
        // ToDo: switch to selfmessages
        this->updateState();
    } else if (msg->isName("onMyWay")) {
        EV_INFO << "UAV is on the way to CS, reserve spot" << endl;
    } else if (msg->isName("update")) {
        this->updateState();
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
    EV_DEBUG << "Update ChargingStation State!" << endl;
    this->fillSpots();
    if(this->objectsCharging.size() == 0) {
        return;
    }
    this->charge();
    this->scheduleAt(simTime()+10, new cMessage("update"));
}

bool ChargingNode::commandCompleted()
{
    return false;
}

double ChargingNode::nextNeededUpdate()
{
    return 100.0;
}

void ChargingNode::appendToObjectsWaiting(MobileNode* mn)
{
    if(this->objectsWaiting.size() >= this->spotsWaiting) {
        EV_INFO << "All spots for waiting ("<< spotsWaiting <<") are already taken." << endl;
        return;
    }
    if(std::find(this->objectsWaiting.begin(), this->objectsWaiting.end(), mn) != this->objectsWaiting.end()) {
        EV_INFO << "The module is already in the queue and therefore not appended again." << endl;
        return;
    }
    this->objectsWaiting.push_back(mn);
    EV_INFO << "The module got appended to the queue." << endl;
}

void ChargingNode::fillSpots()
{
    while(this->spotsCharging > this->objectsCharging.size()
            && this->objectsWaiting.size() > 0) {
        this->objectsCharging.push_back(this->objectsWaiting.front());
        this->objectsWaiting.erase(this->objectsWaiting.begin());
    }
}

void ChargingNode::charge()
{
    for(unsigned int i = 0; i < this->spotsCharging; i++) {
        if(this->objectsCharging[i]->battery.isFull()) {
            EV_INFO << "UAV in slot " << i <<" is fully charged." << endl;
            send(new cMessage("nextCommand"), this->getOutputGateTo(this->objectsCharging[i]));
            // ToDo remove the right element and not the first one.
            this->objectsCharging.erase(this->objectsCharging.begin());
            break;
        }
        EV_INFO << "UAV in slot " << i <<" is currently getting charged." << endl;
        this->objectsCharging[i]->battery.charge(this->calculateChargeAmount(this->objectsCharging[i]->battery, (simTime() - this->lastUpdate).dbl()));
    }
}

float ChargingNode::calculateChargeAmount(Battery battery, double seconds)
{
    if(battery.isFull()) {
        return 0;
    }
    if(battery.getRemainingPercentage() < 90) {
        // linear
        float chargeAmountLinear = 1*seconds;
        if(battery.getCapacity()*0.9 < battery.getRemaining() + chargeAmountLinear) {
             // the amount charged will exceed the linear component
        }
    } else {
        // non linear
//        float
        float chargeAmountNonLinear = 0.5*seconds;

    }

    return 100;
}

// ToDo Implement real values.
float ChargingNode::calculateChargeAmountLinear(double seconds) {
    return seconds * 1;
}

/*
 * Returns the time (in seconds) in which the charging process will be linear
 */
double ChargingNode::calculateMaximumChargeTimeLinear(Battery battery) {
    if(battery.getCapacity()*0.9 < battery.getRemaining()) {
        return 0;
    }
    return (battery.getCapacity()*0.9 - battery.getRemaining()) / this->calculateChargeAmountLinear(1);
}


float ChargingNode::calculateChargeAmountNonLinear(double seconds) {
    return seconds *0.5;
}

#endif // WITH_OSG
