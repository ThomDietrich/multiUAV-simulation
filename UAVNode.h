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

public:
    UAVNode();
    virtual ~UAVNode();
    virtual void loadCommands(CommandQueue commands) override;
    static double getSpeedFromAngle(double angle);
    static double getCurrentFromAngle(double angle);
    static double getCurrentHover();

protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override
    {
        return 2;
    }
    virtual void selectNextCommand() override;
    virtual void initializeState() override;
    virtual void updateState() override;
    virtual bool commandCompleted() override;
    virtual double nextNeededUpdate() override;
    virtual ExchangeData* endOfOperation() override;
    virtual float energyToNearestCN(double fromX, double fromY, double fromZ) override;

    //not needed
    virtual void move();
};

#endif
