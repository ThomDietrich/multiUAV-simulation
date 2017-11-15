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
            // spotsWaiting = 0 -> unlimited spots.
            // if limit is insufficient an info will be printed and the simulation possibly breaks down
            this->spotsWaiting = 0;
            this->spotsCharging = 2;
            this->chargingCurrent = 2000;
            this->x = par("posX");
            this->y = par("posY");
            this->z = 2;
            this->pitch = 0;
            this->yaw = 0;
            this->usedPower = 0;
            this->chargedUAVs = 0;
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
    GenericNode::handleMessage(msg);

    if (msg->isName("startCharge")) {
        EV_INFO << "UAV is ready to get charged" << endl;
        MobileNode *mn = check_and_cast<MobileNode*>(msg->getSenderModule());
        this->appendToObjectsWaiting(mn);

        this->scheduleAt(simTime(), new cMessage("update"));

    } else if (msg->isName("onMyWay")) {
        //ToDo add arrival time and "reserve" a spot.
        EV_INFO << "UAV is on the way to CS, reserve spot" << endl;

    } else if (msg->isName("requestForecastRemainingToTarget")) {
        double remaining = stod(msg->getParListPtr()->get("remaining")->str());
        double capacity = stod(msg->getParListPtr()->get("capacity")->str());
        double targetPercentage = stod(msg->getParListPtr()->get("targetPercentage")->str());
        double forecastDuration = this->getForecastRemainingToTarget(remaining, capacity, targetPercentage);
        simtime_t forecastPointInTime = simTime() + forecastDuration;

        cMessage *response = new ResponseForecastMsg(forecastPointInTime, targetPercentage);
        GenericNode *sender = check_and_cast<GenericNode*>(msg->getSenderModule());
        this->send(response, this->getOutputGateTo(sender));

    } else if (msg->isName("requestForecastRemainingToPointInTime")) {
        double remaining = stod(msg->getParListPtr()->get("remaining")->str());
        double capacity = stod(msg->getParListPtr()->get("capacity")->str());
        simtime_t pointInTime = (simtime_t)stod(msg->getParListPtr()->get("pointInTime")->str());
        double forecastPercentage = this->getForecastRemainingToPointInTime(remaining, capacity, pointInTime);

        cMessage *response = new ResponseForecastMsg(pointInTime, forecastPercentage);
        GenericNode *sender = check_and_cast<GenericNode*>(msg->getSenderModule());
        this->send(response, this->getOutputGateTo(sender));

    } else if(msg->isName("requestMobileNode")) {
        double remaining = stod(msg->getParListPtr()->get("remaining")->str());
        MobileNode* sufficientNode = this->getSufficientlyChargedNode(remaining);

        cMessage *response = new ResponseMobileNodeMsg(sufficientNode);
        GenericNode *sender = check_and_cast<GenericNode*>(msg->getSenderModule());
        this->send(response, this->getOutputGateTo(sender));

    } else {
        // Message is unknown for Charging Node, child classes may handle those messages
        return;
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
    this->fillSpots();
    if(this->objectsCharging.size() == 0) {
        return;
    }
    this->charge();
}

bool ChargingNode::commandCompleted()
{
    return false;
}

/**
 * Calculates the seconds till the next event (a node being done with charging).
 * When there is no next event a placeholder value (10 seconds) is returned.
 *
 * @return double, seconds till next event
 */
double ChargingNode::nextNeededUpdate()
{
    simtime_t currentTime = simTime();
    double nextEvent = -1;
    for(unsigned int i = 0; i < this->objectsCharging.size(); i++) {
        if(this->objectsCharging[i].getPointInTimeWhenDone().dbl()-currentTime.dbl() < nextEvent
                || nextEvent == -1) {
            nextEvent = this->objectsCharging[i].getPointInTimeWhenDone().dbl()-currentTime.dbl();
        }
    }

    if(nextEvent == -1) {
        // ToDo Review minimum time without a upcoming event
        return std::max(timeStep, 10.0);
    } else {
        return nextEvent;
    }
}

/**
 * @return double, seconds till X -> Y charged
 */
double ChargingNode::getForecastRemainingToTarget(double remaining, double capacity, double targetPercentage) {
    // ToDo inlcude targetPercentage
    return this->calculateChargeTime(remaining, capacity);
}

/**
 * @return double, charge percentage for a given point in time
 */
double ChargingNode::getForecastRemainingToPointInTime(double remaining, double capacity, simtime_t pointInTime) {
    double chargeAmount = this->calculateChargeAmount(remaining, capacity, (pointInTime - simTime() - this->getEstimatedWaitingSeconds()).dbl());
    return capacity / (remaining + chargeAmount);
}

/**
 * ToDo: Refactor according to DRY
 * Won't work as good as possible when nodes have different energy consumption.
 * @return MobileNode*|nullptr, node with lowest sufficent remaining current, if there is no suitable nullptr
 */
MobileNode* ChargingNode::getSufficientlyChargedNode(double current)
{
    MobileNode* sufficientlyChargedNode = nullptr;
    for(unsigned int i = 0; i < this->objectsFinished.size(); i++) {
        if(sufficientlyChargedNode == nullptr) {
            sufficientlyChargedNode = this->objectsFinished[i];
        }
        if(sufficientlyChargedNode->getBattery()->getRemaining() > current
                && sufficientlyChargedNode->getBattery()->getRemaining() < sufficientlyChargedNode->getBattery()->getRemaining()) {
            sufficientlyChargedNode = this->objectsFinished[i];
        }
    }
    for(unsigned int i = 0; i < this->objectsWaiting.size(); i++) {
        if(sufficientlyChargedNode == nullptr) {
            sufficientlyChargedNode = this->objectsWaiting[i].getNode();
        }
        if(sufficientlyChargedNode->getBattery()->getRemaining() > current
                && sufficientlyChargedNode->getBattery()->getRemaining() < sufficientlyChargedNode->getBattery()->getRemaining()) {
            sufficientlyChargedNode = this->objectsWaiting[i].getNode();
        }
    }
    for(unsigned int i = 0; i < this->objectsCharging.size(); i++) {
        if(sufficientlyChargedNode == nullptr) {
            sufficientlyChargedNode = this->objectsCharging[i].getNode();
        }
        if(sufficientlyChargedNode->getBattery()->getRemaining() > current
                && sufficientlyChargedNode->getBattery()->getRemaining() < sufficientlyChargedNode->getBattery()->getRemaining()) {
            sufficientlyChargedNode = this->objectsCharging[i].getNode();
        }
    }
    return sufficientlyChargedNode;
}

/**
 * Appends a MobileNode to the waiting queue.
 * Estimated wait and charge duration get calculated and appended.
 */
void ChargingNode::appendToObjectsWaiting(MobileNode* mn)
{
    if(this->objectsWaiting.size() >= this->spotsWaiting
            && this->spotsWaiting != 0) {
        EV_INFO << "All spots for waiting ("<< spotsWaiting <<") are already taken." << endl;
        return;
    }
    // generate a new waiting element with estimated charge and waiting times
    double chargeTimeLinear = this->calculateChargeTimeLinear(mn->getBattery()->getRemaining(), mn->getBattery()->getCapacity());
    double chargeTimeNonLinear = this->calculateChargeTimeNonLinear(mn->getBattery()->getRemaining(), mn->getBattery()->getCapacity());
    ChargingNodeSpotElement element = ChargingNodeSpotElement(mn,
            chargeTimeLinear + chargeTimeNonLinear,
            this->getEstimatedWaitingSeconds());
    EV_DEBUG << "waiting time: " << element.getEstimatedWaitingDuration() << endl;
    EV_DEBUG << "charging: " << element.getEstimatedChargeDuration() << endl;
//    EV_DEBUG << "battery missing: " << element.getNode()->getBattery()->getMissing() << endl;
//    EV_DEBUG << "charging amount lin: " << chargeTimeLinear << endl;
//    EV_DEBUG << "charging amount non-lin: " << chargeTimeNonLinear << endl;

    this->objectsWaiting.push_back(element);
    EV_INFO << "The module got appended to a waiting spot." << endl;
}

/**
 * Moves MobileNodes from the waiting queue to the charging spots if possible.
 */
void ChargingNode::fillSpots()
{
    while(this->spotsCharging > this->objectsCharging.size()
            && this->objectsWaiting.size() > 0) {
        ChargingNodeSpotElement nextWaitingElement = this->objectsWaiting.front();
        nextWaitingElement.setPointInTimeWhenChargingStarted(simTime());
        this->objectsCharging.push_back(nextWaitingElement);
        this->objectsWaiting.erase(this->objectsWaiting.begin());
    }
}

/**
 * Charges the nodes placed on the charging spots depending on the last update.
 * Afterwards updates the last update timestamp and adds the used power to the statistics.
 */
void ChargingNode::charge()
{
    for(unsigned int i = 0; i < this->objectsCharging.size(); i++) {
        EV_INFO << "UAV in slot " << i << " is currently getting charged. Currently Remaining: " << this->objectsCharging[i].getNode()->getBattery()->getRemaining()  << endl;

        if(this->objectsCharging[i].getNode()->getBattery()->isFull()) {
            EV_INFO << "UAV in slot " << i <<" is fully charged." << endl;
            // Send wait message to node
            send(new cMessage("wait"), this->getOutputGateTo(this->objectsCharging[i].getNode()));
            // Send a message to the node which signalizes that the charge process is finished
            send(new cMessage("nextCommand"), this->getOutputGateTo(this->objectsCharging[i].getNode()));
            // Push fully charged nodes to the corresponding list
            this->objectsFinished.push_back(this->objectsCharging[i].getNode());
            // ToDo remove the right element and not the first one.
            this->objectsCharging.erase(this->objectsCharging.begin());

            // increment the statistics value
            this->chargedUAVs++;
            break;
        }
        simtime_t currentTime = simTime();
        double chargeAmount = this->calculateChargeAmount(
                this->objectsCharging[i].getNode()->getBattery()->getRemaining(),
                this->objectsCharging[i].getNode()->getBattery()->getCapacity(),
                (currentTime - std::max(this->lastUpdate, this->objectsCharging[i].getPointInTimeWhenChargingStarted())).dbl()
        );
        this->objectsCharging[i].getNode()->getBattery()->charge(chargeAmount);
        this->usedPower += chargeAmount;
        this->lastUpdate = currentTime;
    }
}

/**
 * Calculates the charge amount for a given amount of time depending on the remaining current and the capacity.
 * Combines the linear and the non-linear values.
 * ToDo: Adapt to real values.
 * @return float, charge amount for X seconds
 */
float ChargingNode::calculateChargeAmount(double remaining, double capacity, double seconds)
{
    if(remaining == capacity) {
        return 0;
    }
    // linear
    // either use the given seconds or the maximum possible linear charge time
    double linearChargeTime = std::min(seconds, this->calculateChargeTimeLinear(remaining, capacity));
    float chargeAmountLinear = this->calculateChargeAmountLinear(linearChargeTime);
    seconds = seconds - linearChargeTime;

    // non linear
    // either use the remaining seconds (given - linear) or the maximum possible non-linear charge time
    double nonLinearChargeTime = std::min(seconds, this->calculateChargeTimeNonLinear(remaining, capacity));
    float chargeAmountNonLinear = this->calculateChargeAmountNonLinear(nonLinearChargeTime, remaining / capacity);

    return chargeAmountLinear + chargeAmountNonLinear;
}

/**
 * ToDo Implement real (linear) values.
 * @return float, charge amount for given seconds
 */
float ChargingNode::calculateChargeAmountLinear(double seconds) {
    return seconds * 10;
}

/**
 * ToDo Implement real (nonlinear) values.
 * @return float, charge amount for given seconds
 */
float ChargingNode::calculateChargeAmountNonLinear(double seconds, double remainingPercentage) {
    return seconds * 5;
}

/**
 * ToDo Add targetCapacity
 * @return double, seconds till fully charged
 */
double ChargingNode::calculateChargeTime(double remaining, double capacity) {
    return this->calculateChargeTimeLinear(remaining, capacity) + this->calculateChargeTimeNonLinear(remaining, capacity);
}

/*
 * ToDo Adapt to real values
 * @return double, seconds till linear charging process is finished (currently 90%)
 */
double ChargingNode::calculateChargeTimeLinear(double remaining, double capacity) {
    if(capacity*0.9 < remaining) {
        return 0;
    }
    return (capacity*0.9 - remaining) / this->calculateChargeAmountLinear(1);
}

/*
 * ToDo Adapt to real values
 * @return double, seconds till non linear charging process is finished (90% to 100%)
 */
double ChargingNode::calculateChargeTimeNonLinear(double remaining, double capacity) {
    if(remaining == capacity) {
        return 0;
    }
    double missing = capacity - remaining;
    if(missing > capacity*0.1) {
        missing = capacity*0.1;
    }
    return (missing / this->calculateChargeAmountNonLinear(1, remaining / capacity));
}

/*
 * @return double, seconds to wait before a newly added node would enter a charging spot
 */
double ChargingNode::getEstimatedWaitingSeconds() {
    // initialize an Array witht he size of chargingSpots with 0's
    double waitingTimes[objectsCharging.size()] = {0.0};

    for(unsigned int c = 0; c < this->objectsCharging.size(); c++) {
        // set array values to the remaining seconds needed for currently charged objects
        waitingTimes[c] = (this->objectsCharging[c].getPointInTimeWhenDone()-simTime()).dbl();
    }
    for(unsigned int w = 0; w < this->objectsWaiting.size(); w++) {
        // add the estimated charge duration of the next waiting object to "spot" with the smallest duration
        *std::min_element(waitingTimes, waitingTimes+this->objectsCharging.size()) += this->objectsWaiting[w].getEstimatedChargeDuration();
    }
    return *std::min_element(waitingTimes, waitingTimes+this->objectsCharging.size());
}

#endif // WITH_OSG
