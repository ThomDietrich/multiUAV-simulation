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
            this->spotsWaiting = par("spotsWaiting");
            this->spotsCharging = par("spotsCharging");
            this->x = par("posX");
            this->y = par("posY");
            this->z = 2;
            this->pitch = 0;
            this->yaw = 0;
            break;
        case 1:
            double linearGradient = par("linearGradient");
            double expGradient = par("expGradient");
            double nonLinearPhaseStartPercentage = par("nonLinearPhaseStartPercentage");
            double nonLinearPhaseLimitPercentage = par("nonLinearPhaseLimitPercentage");
            this->chargeAlgorithm = new ChargeAlgorithmCCCV(linearGradient, expGradient, nonLinearPhaseStartPercentage, nonLinearPhaseLimitPercentage);

            this->labelNode->setText("");
            this->sublabelNode->setText("");
            par("stateSummary").setStringValue("");
			WATCH(usedPower);
            WATCH(chargedUAVs);
            break;
    }
}

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
        appendToObjectsWaiting(mn);

        scheduleAt(simTime(), new cMessage("update"));

    }
    else if (msg->isName("reserveSpot")) {
        double estimatedArrival = stod(msg->getParListPtr()->get("estimatedArrival")->str());
        double estimatedConsumption = stod(msg->getParListPtr()->get("estimatedConsumption")->str());
        MobileNode *mn = check_and_cast<MobileNode*>(msg->getParListPtr()->get("mobileNode"));

        EV_INFO << "Mobile Node is on the way to CS, reserve spot, estimated Arrival: " << estimatedArrival << endl;

        appendToObjectsWaiting(mn, simTime(), estimatedArrival, estimatedConsumption);

    }
    else if (msg->isName("requestForecastRemainingToTarget")) {
        double remaining = stod(msg->getParListPtr()->get("remaining")->str());
        double capacity = stod(msg->getParListPtr()->get("capacity")->str());
        double targetPercentage = stod(msg->getParListPtr()->get("targetPercentage")->str());
        double forecastDuration = getForecastRemainingToTarget(remaining, capacity, targetPercentage);
        simtime_t forecastPointInTime = simTime() + forecastDuration;

        cMessage *response = new ResponseForecastMsg(forecastPointInTime, targetPercentage);
        GenericNode *sender = check_and_cast<GenericNode*>(msg->getSenderModule());
        send(response, getOutputGateTo(sender));

    }
    else if (msg->isName("requestForecastRemainingToPointInTime")) {
        double remaining = stod(msg->getParListPtr()->get("remaining")->str());
        double capacity = stod(msg->getParListPtr()->get("capacity")->str());
        simtime_t pointInTime = (simtime_t) stod(msg->getParListPtr()->get("pointInTime")->str());
        double forecastPercentage = getForecastRemainingToPointInTime(remaining, capacity, pointInTime);

        cMessage *response = new ResponseForecastMsg(pointInTime, forecastPercentage);
        GenericNode *sender = check_and_cast<GenericNode*>(msg->getSenderModule());
        send(response, getOutputGateTo(sender));

    }
    else if (msg->isName("requestMobileNode")) {
        double remaining = stod(msg->getParListPtr()->get("remaining")->str());
        MobileNode* sufficientNode = getSufficientlyChargedNode(remaining);

        cMessage *response = new ResponseMobileNodeMsg(sufficientNode);
        GenericNode *sender = check_and_cast<GenericNode*>(msg->getSenderModule());
        send(response, getOutputGateTo(sender));

    }
    else {
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
    scheduleChargingSpots();
    if (objectsCharging.size() == 0) {
        return;
    }
    charge();
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
    for (unsigned int i = 0; i < objectsCharging.size(); i++) {
        if (objectsCharging[i]->getPointInTimeWhenDone().dbl() - currentTime.dbl() < nextEvent || nextEvent == -1) {
            nextEvent = objectsCharging[i]->getPointInTimeWhenDone().dbl() - currentTime.dbl();
        }
    }

    // get next (furure) arrival time for reservations
    for (unsigned int i = 0; i < objectsWaiting.size(); i++) {
        if ((objectsWaiting[i]->getEstimatedArrival() < nextEvent && objectsWaiting[i]->getEstimatedArrival() >= simTime()) || nextEvent == -1) {
            nextEvent = objectsCharging[i]->getPointInTimeWhenDone().dbl() - currentTime.dbl();
        }
    }

    return (nextEvent != -1) ? nextEvent : (timeStep ? timeStep : 10);
}

/**
 * @return double, seconds till X -> Y charged
 */
double ChargingNode::getForecastRemainingToTarget(double remaining, double capacity, double targetPercentage)
{
    // ToDo include targetPercentage
    return chargeAlgorithm->calculateChargeTime(remaining, capacity, targetPercentage);
}

/**
 * @return double, charge percentage for a given point in time
 */
double ChargingNode::getForecastRemainingToPointInTime(double remaining, double capacity, simtime_t pointInTime)
{
    double chargeAmount = chargeAlgorithm->calculateChargeAmount(remaining, capacity, (pointInTime - simTime() - getEstimatedWaitingSeconds()).dbl());
    return capacity / (remaining + chargeAmount);
}

/**
 * ToDo: Refactor according to DRY
 * Won't work as good as possible when nodes have different energy consumption.
 * @return MobileNode*|nullptr, node with lowest sufficent remaining current, if there is no suitable: nullptr
 */
MobileNode* ChargingNode::getSufficientlyChargedNode(double current)
{
    MobileNode* sufficientlyChargedNode = nullptr;
    for (unsigned int i = 0; i < objectsFinished.size(); i++) {
        if (sufficientlyChargedNode == nullptr) {
            sufficientlyChargedNode = objectsFinished[i];
        }
        if (sufficientlyChargedNode->getBattery()->getRemaining() > current
                && sufficientlyChargedNode->getBattery()->getRemaining() < sufficientlyChargedNode->getBattery()->getRemaining()) {
            sufficientlyChargedNode = objectsFinished[i];
        }
    }
    for (unsigned int i = 0; i < objectsWaiting.size(); i++) {
        if (sufficientlyChargedNode == nullptr) {
            sufficientlyChargedNode = objectsWaiting[i]->getNode();
        }
        if (sufficientlyChargedNode->getBattery()->getRemaining() > current
                && sufficientlyChargedNode->getBattery()->getRemaining() < sufficientlyChargedNode->getBattery()->getRemaining()) {
            sufficientlyChargedNode = objectsWaiting[i]->getNode();
        }
    }
    for (unsigned int i = 0; i < objectsCharging.size(); i++) {
        if (sufficientlyChargedNode == nullptr) {
            sufficientlyChargedNode = objectsCharging[i]->getNode();
        }
        if (sufficientlyChargedNode->getBattery()->getRemaining() > current
                && sufficientlyChargedNode->getBattery()->getRemaining() < sufficientlyChargedNode->getBattery()->getRemaining()) {
            sufficientlyChargedNode = objectsCharging[i]->getNode();
        }
    }
    return sufficientlyChargedNode;
}

/**
 * Appends a MobileNode to the waiting queue.
 * Estimated wait and charge duration get calculated and appended.
 * Default values take place when there is no reservation and the object is already present (object appeared with startCharge message):
 */
void ChargingNode::appendToObjectsWaiting(MobileNode* mobileNode, simtime_t reservationTime, simtime_t estimatedArrival, double consumption)
{
    // check if the waiting queue size would be exceeded
    if (objectsWaiting.size() >= spotsWaiting && spotsWaiting != 0) {
        EV_INFO << "All spots for waiting (" << spotsWaiting << ") are already taken." << endl;
        return;
    }
    // check if given object is already in waiting queue
    if (isInWaitingQueue(mobileNode)) {
        EV_INFO << "Mobile Node is already in waiting Queue." << endl;
        return;
    }

    // generate a new waiting element with estimated charge and waiting times
    // substract consumption which will occur between reservation and the charging process
    // ToDo include targetPercentage
    ChargingNodeSpotElement* element = new ChargingNodeSpotElement(mobileNode,
            chargeAlgorithm->calculateChargeTime(mobileNode->getBattery()->getRemaining() - consumption, mobileNode->getBattery()->getCapacity(), 100),
            getEstimatedWaitingSeconds());

    // set estimatedArrival and reservationTime if not 0, otherwise simTime() will be used as default value
    if (!estimatedArrival.isZero()) {
        element->setEstimatedArrival(estimatedArrival);
    }
    if (!reservationTime.isZero()) {
        element->setReservationTime(reservationTime);
    }

    objectsWaiting.push_back(element);
    EV_INFO << "MobileNode got appended to a waiting spot." << endl;
}

/*
 * @return bool true when the given mobileNode is already in the waitingQueue
 */
bool ChargingNode::isInWaitingQueue(MobileNode* mobileNode)
{
    std::deque<ChargingNodeSpotElement*>::iterator objectWaiting = objectsWaiting.begin();
    while (objectWaiting != objectsWaiting.end()) {
        if ((*objectWaiting)->getNode() == mobileNode) {
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
    std::deque<ChargingNodeSpotElement*>::iterator next = objectsWaiting.begin();
    std::deque<ChargingNodeSpotElement*>::iterator objectWaitingIt = objectsWaiting.begin();
    while (objectWaitingIt != objectsWaiting.end()) {
        if ((*objectWaitingIt)->getReservationTime() < (*next)->getReservationTime() && (*objectWaitingIt)->getEstimatedArrival() <= simTime()) {
            if (not isPhysicallyPresent((*objectWaitingIt)->getNode())) {
                throw "Mobile Node should be physically present (arrivalTime is in the past)!";
            }
            next = objectWaitingIt;
        }
        objectWaitingIt++;
    }
    return next;
}

bool ChargingNode::isPhysicallyPresent(MobileNode* mobileNode)
{
    return (mobileNode->getX() == getX() && mobileNode->getY() == getY() && mobileNode->getZ() == getZ());
}

/**
 * Populates the charging nodes.
 * Will push back nodes from charging spots to waiting spots when needed due to reservation.
 */
void ChargingNode::scheduleChargingSpots()
{
    // when there are no waiting objects, the method does nothing
    if (objectsWaiting.size() == 0) {
        return;
    }

    // get the next waiting object (see referenced method for details)
    // variable gets used through both loops and always rewritten when the object was appended to a charging spot
    std::deque<ChargingNodeSpotElement*>::iterator nextWaitingObject = getNextWaitingObjectIterator();

    // loop through currently used spots and check for earlier reservations
    // when an earlier reservation time occurs, throw out the currently charged node and push it back to the waiting objects
    std::deque<ChargingNodeSpotElement*>::iterator objectChargingIt = objectsCharging.begin();
    while (objectChargingIt != objectsCharging.end()) {
        if ((*objectChargingIt)->getReservationTime() > (*nextWaitingObject)->getReservationTime()) {
            appendToObjectsWaiting((*objectChargingIt)->getNode(), (*objectChargingIt)->getReservationTime());
            objectsCharging.erase(objectChargingIt);
            (*nextWaitingObject)->setPointInTimeWhenChargingStarted(simTime());
            objectsCharging.push_back(*nextWaitingObject);
            objectsWaiting.erase(nextWaitingObject);
            nextWaitingObject = getNextWaitingObjectIterator();
        }
        objectChargingIt++;
    }

    // loop through empty charging spots and fill them with waiting objects
    while (spotsCharging > objectsCharging.size() && objectsWaiting.size() > 0) {
        (*nextWaitingObject)->setPointInTimeWhenChargingStarted(simTime());
        objectsCharging.push_back(*nextWaitingObject);
        objectsWaiting.erase(nextWaitingObject);
        nextWaitingObject = getNextWaitingObjectIterator();
    }
}

/**
 * Charges the nodes placed on the charging spots depending on the last update.
 * Afterwards updates the last update timestamp and adds the used power to the statistics.
 */
void ChargingNode::charge()
{
    for (unsigned int i = 0; i < objectsCharging.size(); i++) {
        EV_INFO << "UAV in slot " << i << " is currently getting charged. Currently Remaining: " << objectsCharging[i]->getNode()->getBattery()->getRemaining()
                << endl;

        if (objectsCharging[i]->getNode()->getBattery()->isFull()) {
            EV_INFO << "UAV in slot " << i << " is fully charged." << endl;
            // Send wait message to node
            MobileNode* mobileNode = objectsCharging[i]->getNode();
            send(new cMessage("wait"), getOutputGateTo(mobileNode));
            // Send a message to the node which signalizes that the charge process is finished
            send(new cMessage("nextCommand"), getOutputGateTo(mobileNode));
            // Push fully charged nodes to the corresponding list
            objectsFinished.push_back(objectsCharging[i]->getNode());
            objectsCharging.erase(objectsCharging.begin());

            // increment the statistics value
            chargedUAVs++;
            break;
        }
        simtime_t currentTime = simTime();
        double chargeAmount = chargeAlgorithm->calculateChargeAmount(objectsCharging[i]->getNode()->getBattery()->getRemaining(),
                objectsCharging[i]->getNode()->getBattery()->getCapacity(),
                (currentTime - std::max(lastUpdate, objectsCharging[i]->getPointInTimeWhenChargingStarted())).dbl());
        objectsCharging[i]->getNode()->getBattery()->charge(chargeAmount);
        usedPower += chargeAmount;
        lastUpdate = currentTime;
    }
}

/*
 * @return double, seconds to wait before a newly added node would enter a charging spot
 */
double ChargingNode::getEstimatedWaitingSeconds()
{
    // initialize an Array witht he size of chargingSpots with 0's
    double waitingTimes[objectsCharging.size()] = { 0.0 };

    for (unsigned int c = 0; c < objectsCharging.size(); c++) {
        // set array values to the remaining seconds needed for currently charged objects
        waitingTimes[c] = (objectsCharging[c]->getPointInTimeWhenDone() - simTime()).dbl();
    }
    for (unsigned int w = 0; w < objectsWaiting.size(); w++) {
        // add the estimated charge duration of the next waiting object to "spot" with the smallest duration
        *std::min_element(waitingTimes, waitingTimes + objectsCharging.size()) += objectsWaiting[w]->getEstimatedChargeDuration();
    }
    return *std::min_element(waitingTimes, waitingTimes + objectsCharging.size());
}

#endif // WITH_OSG
