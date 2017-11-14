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
#include "GenericNode.h"
#include "ChargingNodeSpotElement.h"
#include "Battery.h"
#include "msgs/ResponseForecastMsg.h"
#include "Command.h"
#include "CommandExecEngine.h"

using namespace omnetpp;

class ChargingNodeSpotElement;

class ChargingNode : public GenericNode {
protected:
    unsigned int spotsWaiting;
    unsigned int spotsCharging;
    std::deque<ChargingNodeSpotElement> objectsWaiting;
    std::deque<ChargingNodeSpotElement> objectsCharging;
    std::deque<MobileNode*> objectsFinished;
    double chargingCurrent;
    double usedPower;
    int chargedUAVs;
public:
    ChargingNode();
    virtual ~ChargingNode();
    virtual void selectNextCommand() override;
    virtual void initializeState() override;
    virtual void loadCommands(CommandQueue commands) override;
    virtual void clearCommands() override;
    virtual void updateState() override;
    virtual bool commandCompleted() override;
    virtual double nextNeededUpdate() override;
    virtual ReplacementData* endOfOperation() override;
    double getForecastRemainingToTarget(double remaining, double capacity, double targetPercentage = 100.0);
    double getForecastRemainingToPointInTime(double remaining, double capacity, simtime_t pointInTime);
    // Getters
    double getChargingCurrent() const
    {
        return chargingCurrent;
    }

    unsigned int getSpotsCharging() const
    {
        return spotsCharging;
    }

    unsigned int getSpotsWaiting() const
    {
        return spotsWaiting;
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
    void appendToObjectsWaiting(MobileNode* module);
    void fillSpots();
    void charge();
    float calculateChargeAmount(double remaining, double capacity, double seconds);
    float calculateChargeAmountLinear(double seconds);
    float calculateChargeAmountNonLinear(double seconds, double remainingPercentage);
    double calculateMaximumChargeTime(double remaining, double capacity);
    double calculateChargeTime(double remaining, double capacity);
    double calculateChargeTimeLinear(double remaining, double capacity);
    double calculateChargeTimeNonLinear(double remaining, double capacity);
    double getEstimatedWaitingSeconds();
    simtime_t getPointInTimeWhenDone(ChargingNodeSpotElement spotElement);

};

#endif /* CHARGINGNODE_H_ */
