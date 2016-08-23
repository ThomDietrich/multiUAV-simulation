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

#ifndef COMMANDEXECENGINE_H_
#define COMMANDEXECENGINE_H_

#include <omnetpp.h>
#include "Command.h"

using namespace omnetpp;

//make MobileNode known to be reverse reference type
class MobileNode;

enum ceeType {
    WAYPOINT, TAKEOFF, HOLDPOSITION
};

/**
 * Abstract Command Execution Engine
 * To be derived for specific Commands -> CEEs
 */
class CommandExecEngine {
protected:
    //SubclassNode *node;
    //SpecializedCommand *command;
    ceeType type;
    void setType(ceeType type);
public:
    //CommandExecEngine(SubclassNode &node, SpecializedCommand &command) { };
    virtual ~CommandExecEngine() {
    }
    ;

    virtual bool commandCompleted() = 0;

    virtual void initializeState() = 0;

    /**
     * Update physical and logical state of the node.
     *
     * @param stepSize simulation time step in seconds
     */
    virtual void updateState(double stepSize) = 0;

    virtual double getRemainingTime() = 0;

    /**
     * For the energy consumption calculation and prediction
     * The current is dependent on the specific command and command-related parameters (speed etc.)
     *
     * @return statistical mean current flow (mA)
     */
    virtual double getCurrent() = 0;

    /**
     * Predict the overall consumption for the full command execution procedure
     * @return to be consumed energy in mAh
     */
    virtual double predictConsumption() = 0;

    ceeType getCeeType() {
        return type;
    }
    ;
};

//make UAVNode known to be reverse reference type
class UAVNode;

/**
 * Waypoint Command Execution Engine
 */
class WaypointCEE : public CommandExecEngine {
protected:
    UAVNode *node;
    WaypointCommand *command;
public:
    WaypointCEE(UAVNode &node, WaypointCommand &command);
    bool commandCompleted() override;
    void initializeState() override;
    void updateState(double stepSize) override;
    double getRemainingTime() override;
    double getCurrent() override;
    double predictConsumption() override;
};

/**
 * Takeoff Command Execution Engine
 */
class TakeoffCEE : public CommandExecEngine {
protected:
    UAVNode *node;
    TakeoffCommand *command;
public:
    TakeoffCEE(UAVNode &node, TakeoffCommand &command);
    bool commandCompleted() override;
    void initializeState() override;
    void updateState(double stepSize) override;
    double getRemainingTime() override;
    double getCurrent() override;
    double predictConsumption() override;
};

/**
 * HoldPosition Command Execution Engine
 */
class HoldPositionCEE : public CommandExecEngine {
protected:
    UAVNode *node;
    HoldPositionCommand *command;
    simtime_t holdPositionTill;
public:
    HoldPositionCEE(UAVNode &node, HoldPositionCommand &command);
    bool commandCompleted() override;
    void initializeState() override;
    void updateState(double stepSize) override;
    double getRemainingTime() override;
    double getCurrent() override;
    double predictConsumption() override;
};

#endif /* COMMANDEXECENGINE_H_ */
