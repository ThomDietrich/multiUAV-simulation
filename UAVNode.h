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

#ifndef __UAVNODE_H__
#define __UAVNODE_H__

#include <vector>
#include <string>
#include <omnetpp.h>
#include "MobileNode.h"
#include "msgs/MissionMsg_m.h"
#include "msgs/ExchangeCompletedMsg_m.h"
#include <boost/math/distributions/normal.hpp>
#include "UAVSoloEmpiricData.h"

using namespace omnetpp;

/**
 * A mobile node that follows a predefined track.
 */
class UAVNode : public MobileNode {

    friend class CommandExecEngine;
    friend class WaypointCEE;
    friend class HoldPositionCEE;
    friend class TakeoffCEE;
    friend class ChargeCEE;
    friend class ExchangeCEE;
    friend class IdleCEE;

public:
    UAVNode();
    virtual ~UAVNode();
    virtual void loadCommands(CommandQueue commands, bool isMission = true) override;
    virtual double estimateCommandsDuration();
    static float getHoverConsumption(float duration, int fromMethod = 0, float quantile = 0.95);
    static float getMovementConsumption(float angle, float duration, int fromMethod = 0, float quantile = 0.95);
    static float getSpeed(float angle, int fromMethod = 1, float quantile = 0.95);

    //TODO part of hack111 to make the replacing node known to the Exchange command
    GenericNode* replacingNode = nullptr;
    double replacementX = DBL_MAX, replacementY = DBL_MAX, replacementZ = DBL_MAX;
    simtime_t replacementTime = 0;

protected:
    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual int numInitStages() const override
    {
        return 2;
    }
    virtual void selectNextCommand() override;
    virtual void initializeState() override;
    virtual void updateState() override;
    virtual bool isCommandCompleted() override;
    virtual double nextNeededUpdate() override;
    virtual void collectStatistics() override;
    virtual ReplacementData* endOfOperation() override;
    virtual float energyToNearestCN(double fromX, double fromY, double fromZ) override;

    bool exchangeAfterCurrentCommand = false;

    //not needed
    virtual void move();

    void transferMissionDataTo(UAVNode* node);

private:
    bool cmpCoord(const Command& cmd, const double X, const double Y, const double Z);
    bool cmpCoord(const Command& cmd1, const Command& cmd2);
    float energyForCEE(CommandExecEngine* cee);
    float estimateEnergy(double fromX, double fromY, double fromZ, double toX, double toY, double toZ);
    double estimateDuration(double fromX, double fromY, double fromZ, double toX, double toY, double toZ);
    float quantile = 0.95;
    bool receivedMission_valid = false;
    int receivedMission_missionId;
    bool receivedMission_commandsRepeat;
    CommandQueue receivedMission_missionCommands;
};

#endif
