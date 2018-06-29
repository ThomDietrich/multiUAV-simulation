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

#include "msgs/ForecastPointInTimeRequest_m.h"
#include "msgs/ForecastTargetRequest_m.h"
#include "msgs/ForecastResponse_m.h"
#include "msgs/UpdateChargingMsg_m.h"
#include "msgs/MobileNodeRequest_m.h"
#include "msgs/MobileNodeResponse_m.h"
#include "msgs/ReserveSpotMsg_m.h"

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
            this->z = par("posZ");
            this->pitch = 0;
            this->yaw = 0;
            break;
        case 1:
            //Initialize energy storage
            int capacity = int(par("batteryCapacity"));
            (capacity == 0) ? battery = Battery() : battery = Battery(capacity);

            //Initialize charge parameters
            chargeEffectivenessPercentage = double(par("chargeEffectivenessPercentage")) / 100;
            prioritizeFastCharge = par("prioritizeFastCharge").boolValue();

            //Initialize chargeAlgorithm
            double chargeCurrent = double(par("chargeCurrent"));
            double nonLinearPhaseStartPercentage = double(par("nonLinearPhaseStartPercentage"));
            //double linearGradient = double(par("linearGradient"));
            //this->chargeAlgorithm = new ChargeAlgorithmCCCV(linearGradient, chargeCurrent, nonLinearPhaseStartPercentage);
            this->chargeAlgorithm = new ChargeAlgorithmCCCVCurrent(chargeCurrent, nonLinearPhaseStartPercentage);

            this->labelNode->setText(getFullName());
            this->sublabelNode->setText("");
            par("stateSummary").setStringValue("");

            //WATCH statistical values
            WATCH(usedPower);
            WATCH(chargedPower);
            WATCH(chargedMobileNodes);
            WATCH(reservations);
            break;
    }
}

ReplacementData* ChargingNode::endOfOperation()
{
    return 0;
}

void ChargingNode::loadCommands(CommandQueue commands, bool isMission)
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
    if (msg->isName("startCharge")) {
        EV_INFO << "MobileNode is ready to get charged" << endl;
        MobileNode *mn = check_and_cast<MobileNode*>(msg->getSenderModule());
        appendToObjectsWaiting(mn, 100.0);

        if (not active) {
            msg->setName("update");
            scheduleAt(simTime(), msg);
            active = true;
        }
    }
    else if (msg->isName("reserveSpot")) {
        ReserveSpotMsg *rsmsg = check_and_cast<ReserveSpotMsg*>(msg);
        MobileNode *mn = check_and_cast<MobileNode*>(msg->getSenderModule());
        appendToObjectsWaiting(mn, rsmsg->getTargetPercentage(), simTime(), rsmsg->getEstimatedArrival(), rsmsg->getConsumptionTillArrival());
        reservations++;
        EV_INFO << "MobileNode " << mn->getFullName() << " is on the way to CS. Spot reserved for: " << rsmsg->getEstimatedArrival() << endl;

        if (not active) {
            msg->setName("update");
            scheduleAt(simTime(), msg);
            active = true;
        }
        else {
            delete msg;
            msg = nullptr;
        }
    }
    else if (msg->isName("forecastTargetRequest")) {
        ForecastTargetRequest *ftmsg = check_and_cast<ForecastTargetRequest *>(msg);
        double forecastDuration = getForecastRemainingToTarget(ftmsg->getRemaining(), ftmsg->getCapacity(), ftmsg->getTargetPercentage());

        ForecastResponse *frmsg = new ForecastResponse("forecastResponse");
        frmsg->setPointInTime(simTime() + forecastDuration);
        frmsg->setReachedPercentage(ftmsg->getTargetPercentage());
        send(frmsg, getOutputGateTo(frmsg->getSenderModule()));

        delete msg;
        msg = nullptr;
    }
    else if (msg->isName("forecastPointInTimeRequest")) {
        ForecastPointInTimeRequest *fpitmsg = check_and_cast<ForecastPointInTimeRequest *>(msg);
        double forecastPercentage = getForecastRemainingToPointInTime(fpitmsg->getRemaining(), fpitmsg->getCapacity(), fpitmsg->getPointInTime());

        ForecastResponse *frmsg = new ForecastResponse("forecastResponse");
        frmsg->setPointInTime(fpitmsg->getPointInTime());
        frmsg->setReachedPercentage(forecastPercentage);
        send(frmsg, getOutputGateTo(frmsg->getSenderModule()));

        delete msg;
        msg = nullptr;
    }
    else if (msg->isName("mobileNodeRequest")) {
        MobileNodeRequest *mnmsg = check_and_cast<MobileNodeRequest *>(msg);
        updateState();
        MobileNode* sufficientNode = getSufficientlyChargedNode(mnmsg->getRemaining());

        MobileNodeResponse *answerMsg = new MobileNodeResponse("mobileNodeResponse");
        if (sufficientNode != nullptr) {
            answerMsg->setNodeFound(true);
            answerMsg->setMobileNodeIndex(sufficientNode->getIndex());
            answerMsg->setCapacity(sufficientNode->getBattery()->getCapacity());
            answerMsg->setRemaining(sufficientNode->getBattery()->getRemaining());
        }
        else {
            answerMsg->setNodeFound(false);
        }

        send(answerMsg, getOutputGateTo(msg->getSenderModule()));

        delete msg;
        msg = nullptr;
    }
    else if (msg->isName("mobileNodeExit")) {
        MobileNode* sender = check_and_cast<MobileNode*>(msg->getSenderModule());
        removeFromChargingNode(sender);
        updateState();

        delete msg;
        msg = nullptr;
    }
    else {
        GenericNode::handleMessage(msg);
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
    if (battery.isEmpty()) {
        EV_WARN << "The battery of the Charging Station is exhausted!";
        return;
    }

    chargeAllChargingSpots();
    clearChargingSpots();
    fillChargingSpots();
    rearrangeChargingSpots();

    UpdateChargingMsg* updateMsg = new UpdateChargingMsg("chargingUpdate");
    std::string update("");

    for (auto it = objectsCharging.cbegin(); it != objectsCharging.cend(); ++it) {
        MobileNode* node = (*it)->getNode();
        Battery* battery = node->getBattery();
        update.append(std::to_string(node->getIndex()));
        update.append(",");
        update.append(std::to_string(battery->getRemaining()));
        update.append(",");
        update.append(std::to_string(battery->getCapacity()));
        update.append(";");
    }
    if (update.empty()) {
        delete updateMsg;
    }
    else {
        updateMsg->setUpdate(update.c_str());
        send(updateMsg, "gate$o", 0);
    }
}

bool ChargingNode::isCommandCompleted()
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

    // get next (future) arrival time for reservations
    for (unsigned int i = 0; i < objectsWaiting.size(); i++) {
        if ((objectsWaiting[i]->getEstimatedArrival() < nextEvent && objectsWaiting[i]->getEstimatedArrival() >= simTime()) || nextEvent == -1) {
            nextEvent = objectsWaiting[i]->getPointInTimeWhenDone().dbl() - currentTime.dbl();
        }
    }

    return (nextEvent > 0) ? nextEvent : (timeStep ? timeStep : 10);
}

void ChargingNode::collectStatistics()
{
}

/**
 * @return double, seconds till X -> Y charged
 */
double ChargingNode::getForecastRemainingToTarget(double remaining, double capacity, double targetPercentage)
{
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
 * Won't work as good as possible when nodes have different energy consumption.
 * @return MobileNode*|nullptr, node with lowest sufficent remaining current, if there is no suitable it returns the one with most remaining energy
 */
MobileNode* ChargingNode::getSufficientlyChargedNode(double current)
{
    MobileNode* sufficientlyChargedNode = nullptr;
    MobileNode* highestChargedNode = nullptr;
    for (unsigned int i = 0; i < objectsFinished.size(); i++) {
        if (checkForSufficientlyChargedNode(objectsFinished[i], sufficientlyChargedNode, current)) {
            sufficientlyChargedNode = objectsFinished[i];
        }
        if (checkForHighestChargedNode(objectsFinished[i], highestChargedNode)) {
            highestChargedNode = objectsFinished[i];
        }
    }
    for (unsigned int i = 0; i < objectsWaiting.size(); i++) {
        if (checkForSufficientlyChargedNode(objectsWaiting[i]->getNode(), sufficientlyChargedNode, current)) {
            sufficientlyChargedNode = objectsWaiting[i]->getNode();
        }
        if (checkForHighestChargedNode(objectsWaiting[i]->getNode(), highestChargedNode)) {
            highestChargedNode = objectsWaiting[i]->getNode();
        }
    }
    for (unsigned int i = 0; i < objectsCharging.size(); i++) {
        if (checkForSufficientlyChargedNode(objectsCharging[i]->getNode(), sufficientlyChargedNode, current)) {
            sufficientlyChargedNode = objectsCharging[i]->getNode();
        }
        if (checkForHighestChargedNode(objectsCharging[i]->getNode(), highestChargedNode)) {
            highestChargedNode = objectsCharging[i]->getNode();
        }
    }
    if (sufficientlyChargedNode) {
        return (sufficientlyChargedNode->getBattery()->getRemaining() > current) ? sufficientlyChargedNode : highestChargedNode;
    }
    return nullptr;
}

bool ChargingNode::checkForSufficientlyChargedNode(MobileNode* nextNode, MobileNode* sufficientlyChargedNode, double current)
{
    if (sufficientlyChargedNode == nullptr) {
        return true;
    }
    if (nextNode->getBattery()->getRemaining() > current && nextNode->getBattery()->getRemaining() < sufficientlyChargedNode->getBattery()->getRemaining()) {
        return true;
    }
    return false;
}

bool ChargingNode::checkForHighestChargedNode(MobileNode* nextNode, MobileNode* highestChargedNode)
{
    if (highestChargedNode == nullptr) {
        return true;
    }
    if (nextNode->getBattery()->getRemaining() > highestChargedNode->getBattery()->getRemaining()) {
        return true;
    }
    return false;
}

/**
 * Removes all connections to the given MobileNode.
 * This needs to be executed before a MobileNode can safely leave the ChargingNode.
 */
void ChargingNode::removeFromChargingNode(MobileNode* mobileNode)
{
    for (unsigned int i = 0; i < objectsFinished.size(); i++) {
        if (objectsFinished[i] == mobileNode) {
            objectsFinished.erase(objectsFinished.begin() + i);
        }
    }
    for (unsigned int i = 0; i < objectsWaiting.size(); i++) {
        if (objectsWaiting[i]->getNode() == mobileNode) {
            objectsWaiting.erase(objectsWaiting.begin() + i);
        }
    }
    for (unsigned int i = 0; i < objectsCharging.size(); i++) {
        if (objectsCharging[i]->getNode() == mobileNode) {
            objectsCharging.erase(objectsCharging.begin() + i);
        }
    }
}

/**
 * Appends a MobileNode to the waiting queue.
 * Estimated wait and charge duration get calculated and appended.
 * Default values take place when there is no reservation and the object is already present (object appeared with startCharge message):
 */
void ChargingNode::appendToObjectsWaiting(MobileNode* mobileNode, double targetPercentage, simtime_t reservationTime, simtime_t estimatedArrival,
        double consumption)
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
    double chargeTime = chargeAlgorithm->calculateChargeTime(mobileNode->getBattery()->getRemaining() - consumption, mobileNode->getBattery()->getCapacity(),
            targetPercentage);
    ASSERT(chargeTime > 0);
    ChargingNodeSpotElement* element = new ChargingNodeSpotElement(mobileNode, chargeTime, getEstimatedWaitingSeconds(), targetPercentage);

    // set estimatedArrival and reservationTime if not 0, otherwise simTime() will be used as default value
    if (!estimatedArrival.isZero()) {
        element->setEstimatedArrival(estimatedArrival);
    }
    if (!reservationTime.isZero()) {
        element->setReservationTime(reservationTime);
    }

    objectsWaiting.push_back(element);
    EV_INFO << "MobileNode " << mobileNode->getFullName() << " got appended to a waiting spot." << endl;
}

/*
 * @return bool true when the given mobileNode is already in the waiting queue
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
 * When fastCharge is enbabled the top priority is that the object has less energy then the chargeAlgorithm is advertising as fastCharge.
 * Furthermore they need to be physically at the ChargingNode.
 * @return std::deque<ChargingNodeSpotElement*>::iterator to the next element in waiting queue which is physically present
 */
std::deque<ChargingNodeSpotElement*>::iterator ChargingNode::getNextWaitingObjectIterator(bool fastCharge)
{
    std::deque<ChargingNodeSpotElement*>::iterator next = objectsWaiting.end();
    std::deque<ChargingNodeSpotElement*>::iterator objectWaitingIt = objectsWaiting.begin();
    while (objectWaitingIt != objectsWaiting.end()) {
        if (not isPhysicallyPresent((*objectWaitingIt)->getNode())) {
            objectWaitingIt++;
            continue;
        }
        if (fastCharge
                && static_cast<double>((*objectWaitingIt)->getNode()->getBattery()->getRemainingPercentage())
                        > chargeAlgorithm->getFastChargePercentage((*objectWaitingIt)->getNode()->getBattery()->getCapacity())) {
            objectWaitingIt++;
            continue;
        }
        if (next == objectsWaiting.end()
                || ((*objectWaitingIt)->getReservationTime() < (*next)->getReservationTime() && (*objectWaitingIt)->getEstimatedArrival() <= simTime())) {
            next = objectWaitingIt;
        }
        objectWaitingIt++;
    }
    if (fastCharge && next == objectsWaiting.end()) {
        return getNextWaitingObjectIterator(false);
    }
    return next;
}

bool ChargingNode::isPhysicallyPresent(MobileNode* mobileNode)
{
    return (round(mobileNode->getX()) == round(getX()) && round(mobileNode->getY()) == round(getY()) && round(mobileNode->getZ()) == round(getZ()));
}

int ChargingNode::numberWaitingAndPhysicallyPresent()
{
    int result = 0;
    if (objectsWaiting.size() == 0) {
        return result;
    }
    std::deque<ChargingNodeSpotElement*>::iterator objectWaitingIt = objectsWaiting.begin();
    while (objectWaitingIt != objectsWaiting.end()) {
        if (isPhysicallyPresent((*objectWaitingIt)->getNode())) {
            result++;
        }
        objectWaitingIt++;
    }
    return result;
}

/**
 * Calculates the seconds for the charging process for the given MobileNode.
 * The next event is either fully charged or the fastChargePercentage depending on the configuration.
 */
double ChargingNode::calculateSecondsToNextEvent(MobileNode* mn, bool prioritizeFastCharge)
{
    double remaining = mn->getBattery()->getRemaining();
    double capacity = mn->getBattery()->getCapacity();
    double fastChargePercentage = chargeAlgorithm->getFastChargePercentage(capacity);
    double targetPercentage = fastChargePercentage;
    if (!prioritizeFastCharge || remaining / capacity * 100 >= fastChargePercentage) {
        targetPercentage = 100.0;
    }
    return chargeAlgorithm->calculateChargeTime(remaining, capacity, targetPercentage);
}

/**
 * Populates the charging nodes.
 */
void ChargingNode::fillChargingSpots()
{
    // when there are no waiting objects, the method does nothing
    int availableNodes = numberWaitingAndPhysicallyPresent();
    if (availableNodes == 0) {
        return;
    }

    // get the next waiting object
    std::deque<ChargingNodeSpotElement*>::iterator nextWaitingObject = getNextWaitingObjectIterator(prioritizeFastCharge);

    // loop through empty charging spots and fill them with waiting objects
    while (spotsCharging > objectsCharging.size() && availableNodes > 0) {
        EV_INFO << (*nextWaitingObject)->getNode()->getFullName() << " is added to charging spot." << endl;
        (*nextWaitingObject)->setPointInTimeWhenChargingStarted(simTime());
        // set the point in time when the next event needs to be executed
        double secondsToNextEvent = calculateSecondsToNextEvent((*nextWaitingObject)->getNode(), prioritizeFastCharge);
        (*nextWaitingObject)->setPointInTimeWhenDone(simTime() + secondsToNextEvent);
        objectsCharging.push_back(*nextWaitingObject);
        objectsWaiting.erase(nextWaitingObject);
        nextWaitingObject = getNextWaitingObjectIterator(prioritizeFastCharge);
        availableNodes = numberWaitingAndPhysicallyPresent();
    }
}

/*
 * Remove nodes from charging spot when done
 */
void ChargingNode::clearChargingSpots()
{
    for (unsigned int i = 0; i < objectsCharging.size(); i++) {
        if (not this->isPhysicallyPresent(objectsCharging[i]->getNode())) {
            EV_INFO << objectsCharging[i]->getNode()->getFullName() << " is removed from charging spot - not physically present anymore." << endl;
            objectsCharging.erase(objectsCharging.begin() + i);
            continue;
        }
        if (objectsCharging[i]->getNode()->getBattery()->getRemainingPercentage() > objectsCharging[i]->getTargetCapacityPercentage()
                || objectsCharging[i]->getNode()->getBattery()->isFull()) {
            EV_INFO << objectsCharging[i]->getNode()->getFullName() << " is removed from charging spot - charged to target: "
                    << objectsCharging[i]->getNode()->getBattery()->getRemainingPercentage() << "/" << objectsCharging[i]->getTargetCapacityPercentage() << "%"
                    << endl;
            // Push fully charged nodes to the corresponding list
            objectsFinished.push_back(objectsCharging[i]->getNode());
            objectsCharging.erase(objectsCharging.begin() + i);
            // increment the statistics value
            chargedMobileNodes++;
        }
    }
}

/*
 * Exchange waiting spots when needed due to earlier reservation or the fast charge mechanismn
 */
void ChargingNode::rearrangeChargingSpots()
{
    // this method does nothing when either there is no object charged currently or there is no available waiting object
    if (objectsCharging.size() < spotsCharging || numberWaitingAndPhysicallyPresent() == 0) {
        return;
    }

    // get the next waiting object
    std::deque<ChargingNodeSpotElement*>::iterator nextWaitingObject = getNextWaitingObjectIterator(prioritizeFastCharge);

    // loop through currently used spots and check for earlier reservations
    // when an earlier reservation time occurs, throw out the currently charged node and push it back to the waiting objects
    std::deque<ChargingNodeSpotElement*>::iterator objectChargingIt = objectsCharging.begin();
    while (objectChargingIt != objectsCharging.end()) {
        simtime_t chargingObjResTime = (*objectChargingIt)->getReservationTime();
        simtime_t waitingObjResTime = (*nextWaitingObject)->getReservationTime();
        double waitingObjRemainingP = static_cast<double>((*nextWaitingObject)->getNode()->getBattery()->getRemainingPercentage());
        double chargingObjRemainingP = static_cast<double>((*objectChargingIt)->getNode()->getBattery()->getRemainingPercentage());
        double waitingObjFastChargeP = getChargeAlgorithm()->getFastChargePercentage((*nextWaitingObject)->getNode()->getBattery()->getCapacity());
        double chagingObjFastChargeP = getChargeAlgorithm()->getFastChargePercentage((*objectChargingIt)->getNode()->getBattery()->getCapacity());
        //        EV_DEBUG << "chargingObjResTime " << chargingObjRevTime <<  endl;
        //        EV_DEBUG << "waitingObjResTime " << waitingObjRevTime <<  endl;
        if ((chargingObjResTime > waitingObjResTime && (not prioritizeFastCharge || waitingObjRemainingP < waitingObjFastChargeP))
                || (prioritizeFastCharge && waitingObjRemainingP < waitingObjFastChargeP && chargingObjRemainingP >= chagingObjFastChargeP)) {
            ChargingNodeSpotElement* temp = *nextWaitingObject;
            *nextWaitingObject = *objectChargingIt;
            *objectChargingIt = temp;
            (*objectChargingIt)->setPointInTimeWhenChargingStarted(simTime());
            // set the point in time when the next event needs to be executed
            double secondsToNextEvent = calculateSecondsToNextEvent((*objectChargingIt)->getNode(), prioritizeFastCharge);
            (*objectChargingIt)->setPointInTimeWhenDone(simTime() + secondsToNextEvent);

            EV_INFO << "MobileNode ID(" << (*nextWaitingObject)->getNode()->getId() << ") charge spot exchanged with ID("
                    << (*objectChargingIt)->getNode()->getId() << ") waiting spot." << endl;

            nextWaitingObject = getNextWaitingObjectIterator(prioritizeFastCharge);
        }
        objectChargingIt++;
    }
}

/**
 * Charges the nodes placed on the charging spots depending on the last update.
 */
void ChargingNode::chargeAllChargingSpots()
{
    for (unsigned int i = 0; i < objectsCharging.size(); i++) {
        if (not this->isPhysicallyPresent(objectsCharging[i]->getNode())) {
            continue;
        }
        double durationSeconds = (simTime() - std::max(lastUpdate, objectsCharging[i]->getPointInTimeWhenChargingStarted())).dbl();
        ASSERT(durationSeconds >= 0);
        if (durationSeconds < 1.e-10) continue;

        double chargeAmount = chargeAlgorithm->calculateChargeAmount(objectsCharging[i]->getNode()->getBattery()->getRemaining(),
                objectsCharging[i]->getNode()->getBattery()->getCapacity(), durationSeconds);
        double chargeMeanCurrent = chargeAmount * 3600 / durationSeconds / 1000;
        EV_INFO << objectsCharging[i]->getNode()->getFullName() << " charging: " << durationSeconds << "s * " << chargeMeanCurrent << "A = " << chargeAmount
                << "mAh (now " << objectsCharging[i]->getNode()->getBattery()->getRemainingPercentage() << "%)" << endl;
        objectsCharging[i]->getNode()->getBattery()->charge(chargeAmount);
        objectsCharging[i]->getNode()->getCommandExecEngine()->setConsumptionPerSecond((-1) * chargeMeanCurrent);
        battery.discharge(chargeAmount / this->chargeEffectivenessPercentage);
        usedPower += chargeAmount / this->chargeEffectivenessPercentage;
        chargedPower += chargeAmount;
    }
}

/*
 * @return double, seconds to wait before a newly added node would enter a charging spot
 */
double ChargingNode::getEstimatedWaitingSeconds()
{
    if (objectsCharging.empty()) return 0;

    std::vector<double> waitingTimes(objectsCharging.size(), 0.0);

    for (unsigned int c = 0; c < objectsCharging.size(); c++) {
        // set array values to the remaining seconds needed for currently charged objects
        waitingTimes[c] = (objectsCharging[c]->getPointInTimeWhenDone() - simTime()).dbl();
    }
    for (unsigned int w = 0; w < objectsWaiting.size(); w++) {
        // add the estimated charge duration of the next waiting object to "spot" with the smallest duration
        *std::min_element(waitingTimes.begin(), waitingTimes.end()) += objectsWaiting[w]->getEstimatedChargeDuration();
    }
    return *std::min_element(waitingTimes.begin(), waitingTimes.end());
}

#endif // WITH_OSG
