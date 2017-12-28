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

#ifndef CHARGINGNODE_H_
#define CHARGINGNODE_H_

#include <omnetpp.h>
#include "Battery.h"
#include "ChargeAlgorithmCCCV.h"
#include "ChargingNodeSpotElement.h"
#include "Command.h"
#include "CommandExecEngine.h"
#include "GenericNode.h"
#include "IChargeAlgorithm.h"
#include "MobileNode.h"
#include "msgs/ForecastPointInTimeRequest_m.h"
#include "msgs/ForecastTargetRequest_m.h"
#include "msgs/ForecastResponse_m.h"
#include "msgs/MobileNodeRequest_m.h"
#include "msgs/MobileNodeResponse_m.h"
#include "msgs/ReserveSpotMsg_m.h"

using namespace omnetpp;

class ChargingNodeSpotElement;

class ChargingNode : public GenericNode {
private:
    double usedPower = 0;
    int chargedUAVs = 0;
protected:
    Battery battery;
    double chargeCurrent;
    double chargeEffectivenessPercentage;
    unsigned int spotsWaiting;
    unsigned int spotsCharging;
    std::deque<ChargingNodeSpotElement*> objectsWaiting;
    std::deque<ChargingNodeSpotElement*> objectsCharging;
    std::deque<MobileNode*> objectsFinished;
    IChargeAlgorithm* chargeAlgorithm;
    bool active;
    bool prioritizeFastCharge;
public:
    ChargingNode();
    virtual ~ChargingNode();
    virtual void selectNextCommand() override;
    virtual void initializeState() override;
    virtual void loadCommands(CommandQueue commands, bool isMission = true) override;
    virtual void clearCommands() override;
    virtual void updateState() override;
    virtual bool commandCompleted() override;
    virtual double nextNeededUpdate() override;
    virtual void collectStatistics() override;
    virtual ReplacementData* endOfOperation() override;
    // Could be moved to private methods, there functionality is externally available via messages
    double getForecastRemainingToTarget(double remaining, double capacity, double targetPercentage = 100.0);
    double getForecastRemainingToPointInTime(double remaining, double capacity, simtime_t pointInTime);
    MobileNode* getSufficientlyChargedNode(double current);
    // Getters
    unsigned int getSpotsCharging() const
    {
        return spotsCharging;
    }

    unsigned int getSpotsWaiting() const
    {
        return spotsWaiting;
    }

    IChargeAlgorithm* getChargeAlgorithm() const
    {
        return chargeAlgorithm;
    }

    double getUsedPower() const
    {
        return usedPower;
    }

    int getChargedUaVs() const
    {
        return chargedUAVs;
    }

protected:
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void refreshDisplay() const override;
    void appendToObjectsWaiting(MobileNode* mobileNode, double targetPercentage, simtime_t reservationTime = 0, simtime_t estimatedArrival = 0, double consumption = 0);
    bool isInWaitingQueue(MobileNode* mobileNode);
    std::deque<ChargingNodeSpotElement*>::iterator getNextWaitingObjectIterator(bool fastCharge);
    int numberWaitingAndPhysicallyPresent();
    bool isPhysicallyPresent(MobileNode* mobileNode);
    void fillChargingSpots();
    void clearChargingSpots();
    void rearrangeChargingSpots();
    void charge();
    double getEstimatedWaitingSeconds();
};

#endif /* CHARGINGNODE_H_ */
