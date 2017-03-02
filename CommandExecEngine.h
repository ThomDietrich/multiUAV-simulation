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
    WAYPOINT, TAKEOFF, HOLDPOSITION, CHARGE
};

/**
 * Abstract Command Execution Engine
 * To be derived for specific Commands -> CEEs
 */
class CommandExecEngine {
protected:
    //SubclassNode *node;
    Command *command;
    ceeType type;
    void setType(ceeType type);

    /// coordinates from where the command will be executed
    double x0, y0, z0;
    /// coordinates to where the command is going to
    double x1, y1, z1;
    /// movement data
    double yaw = 0, pitch = 0, climbAngle = 0, speed = 0;

public:
    //CommandExecEngine(SubclassNode &boundNode, SpecializedCommand &command) { };
    
    virtual ~CommandExecEngine() {
    }
    ;

    /**
     * Adopt the coordinates the command will be executed from, needed for consumption prediction.
     * These will be the current position of the Node by default.
     * TODO It's undecided what should be done if a Node should do if this is different to the current position.
     *
     * @param x
     * @param y
     * @param z
     */
    void setFromCoordinates(double x, double y, double z) {
        this->x0 = x;
        this->y0 = y;
        this->z0 = z;
    }
    void setToCoordinates(double x, double y, double z) {
        this->x1 = x;
        this->y1 = y;
        this->z1 = z;
    }
    
    double getX0() {
        return x0;
    }
    double getY0() {
        return y0;
    }
    double getZ0() {
        return z0;
    }
    double getX1() {
        return x1;
    }
    double getY1() {
        return y1;
    }
    double getZ1() {
        return z1;
    }
    
    virtual bool commandCompleted() = 0;

    virtual void initializeCEE() = 0;

    virtual void setNodeParameters() = 0;

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
     * @return statistical mean current flow (A)
     */
    virtual double getCurrent() {
        return 0;
    }
    
    /**
     * Predict the overall consumption for the full command execution procedure
     * @return to be consumed energy in mAh
     */
    virtual double predictConsumption() {
        return 0;
    }
    
    ceeType getCeeType() {
        return type;
    }
    virtual char* getCeeTypeString() = 0;
};

//make UAVNode known to be reverse reference type
class UAVNode;

/**
 * Waypoint Command Execution Engine
 *
 * Go from (x0,y0,z0) to (x1,y1,z1) on a straight line
 */
class WaypointCEE : public CommandExecEngine {
protected:
    UAVNode *node;
    WaypointCommand *command;
public:
    WaypointCEE(UAVNode &boundNode, WaypointCommand &command);
    bool commandCompleted() override;
    void initializeCEE() override;
    void setNodeParameters() override;
    void updateState(double stepSize) override;
    double getRemainingTime() override;
    double getCurrent() override;
    double predictConsumption() override;
    char* getCeeTypeString() override;
};

/**
 * Takeoff Command Execution Engine
 *
 * Go straight up or straight down.
 * x and y are NOT considered!
 */
class TakeoffCEE : public CommandExecEngine {
protected:
    UAVNode *node;
    TakeoffCommand *command;
public:
    TakeoffCEE(UAVNode &boundNode, TakeoffCommand &command);
    bool commandCompleted() override;
    void initializeCEE() override;
    void setNodeParameters() override;
    void updateState(double stepSize) override;
    double getRemainingTime() override;
    double getCurrent() override;
    double predictConsumption() override;
    char* getCeeTypeString() override;
};

/**
 * HoldPosition Command Execution Engine
 *
 * Stay at (x,y,z) until {@link holdPositionTill}.
 */
class HoldPositionCEE : public CommandExecEngine {
protected:
    UAVNode *node;
    HoldPositionCommand *command;
    simtime_t holdPositionTill;
public:
    HoldPositionCEE(UAVNode &boundNode, HoldPositionCommand &command);
    bool commandCompleted() override;
    void initializeCEE() override;
    void setNodeParameters() override;
    void updateState(double stepSize) override;
    double getRemainingTime() override;
    double getCurrent() override;
    double predictConsumption() override;
    char* getCeeTypeString() override;
};

/**
 * Charging Command Execution Engine
 */
class ChargeCEE : public CommandExecEngine {
protected:
    UAVNode *node;
    ChargeCommand *command;
public:
    ChargeCEE(UAVNode &boundNode, ChargeCommand &command);
    bool commandCompleted() override;
    void initializeCEE() override;
    void setNodeParameters() override;
    void updateState(double stepSize) override;
    double getRemainingTime() override;
    double getCurrent() override;
    char* getCeeTypeString() override;
};

#endif /* COMMANDEXECENGINE_H_ */
