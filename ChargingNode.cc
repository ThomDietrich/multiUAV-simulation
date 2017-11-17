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
        EV_INFO << "MobileNode is ready to get charged" << endl;
        MobileNode *mn = check_and_cast<MobileNode*>(msg->getSenderModule());
        this->appendToObjectsWaiting(mn);

        this->scheduleAt(simTime(), new cMessage("update"));

    } else if (msg->isName("reserveSpot")) {
        double estimatedArrival = stod(msg->getParListPtr()->get("estimatedArrival")->str());
        double estimatedConsumption = stod(msg->getParListPtr()->get("estimatedConsumption")->str());
        MobileNode *mn = check_and_cast<MobileNode*>(msg->getParListPtr()->get("mobileNode"));

        EV_INFO << "Mobile Node is on the way to CS, reserve spot, estimated Arrival: " << estimatedArrival << endl;

        this->appendToObjectsWaiting(mn, simTime(), estimatedArrival, estimatedConsumption);

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
    this->scheduleChargingSpots();
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
 * Calculates the seconds till the next event (a node being done with charging or the arrival time of an reservation is met).
 * When there is no next event and the timeStep is set to 0 a placeholder value (10 seconds) is returned.
 *
 * @return double, seconds till next event || timeStep if not 0 || 10 seconds as fallback
 */
double ChargingNode::nextNeededUpdate()
{
    simtime_t currentTime = simTime();
    double nextEvent = -1;
    // get time when the next object is successfully charged
    for(unsigned int i = 0; i < this->objectsCharging.size(); i++) {
        if(this->objectsCharging[i]->getPointInTimeWhenDone().dbl()-currentTime.dbl() < nextEvent
                || nextEvent == -1) {
            nextEvent = this->objectsCharging[i]->getPointInTimeWhenDone().dbl()-currentTime.dbl();
        }
    }

    // get next (furure) arrival time for reservations
    for(unsigned int i = 0; i < this->objectsWaiting.size(); i++) {
        if((this->objectsWaiting[i]->getEstimatedArrival() < nextEvent
                && this->objectsWaiting[i]->getEstimatedArrival() >= simTime())
                || nextEvent == -1) {
            nextEvent = this->objectsCharging[i]->getPointInTimeWhenDone().dbl()-currentTime.dbl();
        }
    }

    if(nextEvent == -1) {
        if(timeStep == 0) {
            return 10;
        } else {
            return timeStep;
        }
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
            sufficientlyChargedNode = this->objectsWaiting[i]->getNode();
        }
        if(sufficientlyChargedNode->getBattery()->getRemaining() > current
                && sufficientlyChargedNode->getBattery()->getRemaining() < sufficientlyChargedNode->getBattery()->getRemaining()) {
            sufficientlyChargedNode = this->objectsWaiting[i]->getNode();
        }
    }
    for(unsigned int i = 0; i < this->objectsCharging.size(); i++) {
        if(sufficientlyChargedNode == nullptr) {
            sufficientlyChargedNode = this->objectsCharging[i]->getNode();
        }
        if(sufficientlyChargedNode->getBattery()->getRemaining() > current
                && sufficientlyChargedNode->getBattery()->getRemaining() < sufficientlyChargedNode->getBattery()->getRemaining()) {
            sufficientlyChargedNode = this->objectsCharging[i]->getNode();
        }
    }
    return sufficientlyChargedNode;
}

/**
 * Appends a MobileNode to the waiting queue.
 * Estimated wait and charge duration get calculated and appended.
 * Default values, take place when there is no reservation and the object is already present (object appeared via startCharge):
 *  estimatedArrival    -> 0
 *  reservationTime     -> 0
 *  consumption         -> 0
 */
void ChargingNode::appendToObjectsWaiting(MobileNode* mobileNode, simtime_t reservationTime, simtime_t estimatedArrival, double consumption)
{
    // check if the waiting queue size would be exceeded
    if(this->objectsWaiting.size() >= this->spotsWaiting
            && this->spotsWaiting != 0) {
        EV_INFO << "All spots for waiting ("<< spotsWaiting <<") are already taken." << endl;
        return;
    }
    // check if given object is already in waiting queue
    if(this->isInWaitingQueue(mobileNode)) {
        EV_INFO << "Mobile Node is already in waiting Queue." << endl;
        return;
    }

    // generate a new waiting element with estimated charge and waiting times
    // substract consumption which will occur between reservation and the charging process
    double chargeTimeLinear = this->calculateChargeTimeLinear(mobileNode->getBattery()->getRemaining()-consumption, mobileNode->getBattery()->getCapacity());
    double chargeTimeNonLinear = this->calculateChargeTimeNonLinear(mobileNode->getBattery()->getRemaining()-consumption, mobileNode->getBattery()->getCapacity());
    ChargingNodeSpotElement* element = new ChargingNodeSpotElement(mobileNode,
            chargeTimeLinear + chargeTimeNonLinear,
            this->getEstimatedWaitingSeconds());

    // set estimatedArrival and reservationTime if not 0, otherwise simTime() will be used as default value
    if(!estimatedArrival.isZero()) {
        element->setEstimatedArrival(estimatedArrival);
    }
    if(!reservationTime.isZero()) {
        element->setReservationTime(reservationTime);
    }

    this->objectsWaiting.push_back(element);
    EV_INFO << "MobileNode got appended to a waiting spot." << endl;
}

bool ChargingNode::isInWaitingQueue(MobileNode* mobileNode)
{
    std::deque<ChargingNodeSpotElement*>::iterator objectWaiting = this->objectsWaiting.begin();
    while(objectWaiting != this->objectsWaiting.end()) {
        if((*objectWaiting)->getNode() == mobileNode) {
            return true;
        }
        objectWaiting++;
    }
    return false;
}

/**
 * Elements in the waiting queue get prioritized by their reservationTime.
 * Furthermore they need to be physically at the ChargingNode.
 * @return std::deque<ChargingNodeSpotElement*>::iterator to the next element in waiting queue which is physically present
 */
std::deque<ChargingNodeSpotElement*>::iterator ChargingNode::getNextWaitingObjectIterator()
{
    std::deque<ChargingNodeSpotElement*>::iterator next = this->objectsWaiting.begin();
    std::deque<ChargingNodeSpotElement*>::iterator objectWaitingIt = this->objectsWaiting.begin();
    while(objectWaitingIt != this->objectsWaiting.end()) {
        if((*objectWaitingIt)->getReservationTime() < (*next)->getReservationTime()
                && (*objectWaitingIt)->getEstimatedArrival() <= simTime()) {
            if(!this->isPhysicallyPresent((*objectWaitingIt)->getNode())) {
                throw "Mobile Node should be physically present (arrivalTime is in the past)!";
            }
            next = objectWaitingIt;
        }
        objectWaitingIt++;
    }
    return next;
}

bool ChargingNode::isPhysicallyPresent(MobileNode* mobileNode) {
    return (mobileNode->getX() == this->getX()
            && mobileNode->getY() == this->getY()
            && mobileNode->getZ() == this->getZ());
}

/**
 * Populates the charging nodes.
 * Will push back nodes from charging spots to waiting spots when needed due to reservation.
 */
void ChargingNode::scheduleChargingSpots()
{
    // when there are no waiting objects, the method does nothing
    if(this->objectsWaiting.size() == 0) {
        return;
    }

    // get the next waiting object (see referenced method for details)
    // variable gets used through both loops and always rewritten when the object was appended to a charging spot
    std::deque<ChargingNodeSpotElement*>::iterator nextWaitingObject = this->getNextWaitingObjectIterator();

    // loop through currently used spots and check for earlier reservations
    // when an earlier reservation time occurs, throw out the currently charged node and push it back to the waiting objects
    std::deque<ChargingNodeSpotElement*>::iterator objectChargingIt = this->objectsCharging.begin();
    while(objectChargingIt != this->objectsCharging.end()) {
        if((*objectChargingIt)->getReservationTime() > (*nextWaitingObject)->getReservationTime()) {
            this->appendToObjectsWaiting((*objectChargingIt)->getNode(), (*objectChargingIt)->getReservationTime());
            this->objectsCharging.erase(objectChargingIt);
            (*nextWaitingObject)->setPointInTimeWhenChargingStarted(simTime());
            this->objectsCharging.push_back(*nextWaitingObject);
            this->objectsWaiting.erase(nextWaitingObject);
            nextWaitingObject = this->getNextWaitingObjectIterator();
        }
        objectChargingIt++;
    }

    // loop through empty charging spots and fill them with waiting objects
    while(this->spotsCharging > this->objectsCharging.size()
            && this->objectsWaiting.size() > 0) {
        (*nextWaitingObject)->setPointInTimeWhenChargingStarted(simTime());
        this->objectsCharging.push_back(*nextWaitingObject);
        this->objectsWaiting.erase(nextWaitingObject);
        nextWaitingObject = this->getNextWaitingObjectIterator();
    }
}

/**
 * Charges the nodes placed on the charging spots depending on the last update.
 * Afterwards updates the last update timestamp and adds the used power to the statistics.
 */
void ChargingNode::charge()
{
    for(unsigned int i = 0; i < this->objectsCharging.size(); i++) {
        EV_INFO << "UAV in slot " << i << " is currently getting charged. Currently Remaining: " << this->objectsCharging[i]->getNode()->getBattery()->getRemaining()  << endl;

        if(this->objectsCharging[i]->getNode()->getBattery()->isFull()) {
            EV_INFO << "UAV in slot " << i <<" is fully charged." << endl;
            // Send wait message to node
            MobileNode* mobileNode = this->objectsCharging[i]->getNode();
            send(new cMessage("wait"), this->getOutputGateTo(mobileNode));
            // Send a message to the node which signalizes that the charge process is finished
            send(new cMessage("nextCommand"), this->getOutputGateTo(mobileNode));
            // Push fully charged nodes to the corresponding list
            this->objectsFinished.push_back(this->objectsCharging[i]->getNode());
            this->objectsCharging.erase(this->objectsCharging.begin());

            // increment the statistics value
            this->chargedUAVs++;
            break;
        }
        simtime_t currentTime = simTime();
        double chargeAmount = this->calculateChargeAmount(
                this->objectsCharging[i]->getNode()->getBattery()->getRemaining(),
                this->objectsCharging[i]->getNode()->getBattery()->getCapacity(),
                (currentTime - std::max(this->lastUpdate, this->objectsCharging[i]->getPointInTimeWhenChargingStarted())).dbl()
        );
        this->objectsCharging[i]->getNode()->getBattery()->charge(chargeAmount);
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
float ChargingNode::calculateChargeAmountLinear(double seconds)
{
    return seconds * 10;
}

/**
 * ToDo Implement real (nonlinear) values.
 * @return float, charge amount for given seconds
 */
float ChargingNode::calculateChargeAmountNonLinear(double seconds, double remainingPercentage)
{
    return seconds * 5;
}

/**
 * ToDo Add targetCapacity
 * @return double, seconds till fully charged
 */
double ChargingNode::calculateChargeTime(double remaining, double capacity)
{
    return this->calculateChargeTimeLinear(remaining, capacity) + this->calculateChargeTimeNonLinear(remaining, capacity);
}

/*
 * ToDo Adapt to real values
 * @return double, seconds till linear charging process is finished (currently 90%)
 */
double ChargingNode::calculateChargeTimeLinear(double remaining, double capacity)
{
    if(capacity*0.9 < remaining) {
        return 0;
    }
    return (capacity*0.9 - remaining) / this->calculateChargeAmountLinear(1);
}

/*
 * ToDo Adapt to real values
 * @return double, seconds till non linear charging process is finished (90% to 100%)
 */
double ChargingNode::calculateChargeTimeNonLinear(double remaining, double capacity)
{
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
double ChargingNode::getEstimatedWaitingSeconds()
{
    // initialize an Array witht he size of chargingSpots with 0's
    double waitingTimes[objectsCharging.size()] = {0.0};

    for(unsigned int c = 0; c < this->objectsCharging.size(); c++) {
        // set array values to the remaining seconds needed for currently charged objects
        waitingTimes[c] = (this->objectsCharging[c]->getPointInTimeWhenDone()-simTime()).dbl();
    }
    for(unsigned int w = 0; w < this->objectsWaiting.size(); w++) {
        // add the estimated charge duration of the next waiting object to "spot" with the smallest duration
        *std::min_element(waitingTimes, waitingTimes+this->objectsCharging.size()) += this->objectsWaiting[w]->getEstimatedChargeDuration();
    }
    return *std::min_element(waitingTimes, waitingTimes+this->objectsCharging.size());
}

#endif // WITH_OSG
