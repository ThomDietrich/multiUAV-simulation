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

enum class CeeType {
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
    CeeType type;
    void setType(CeeType type);

    int commandId;

    /// coordinates from where the command will be executed
    double x0, y0, z0;
    /// coordinates to where the command is going to
    double x1, y1, z1;
    /// movement data
    double yaw = 0, pitch = 0, climbAngle = 0, speed = 0;

    /// consumption for this CEE (drawn from stochastic distribution)
    /// Normalized to mAh/s
    double consumptionPerSecond = 0;

public:
    //CommandExecEngine(SubclassNode &boundNode, SpecializedCommand &command) { };

    virtual ~CommandExecEngine()
    {
    }
    ;

    int getCommandId()
    {
        return commandId;
    }
    void setCommandId(int commandId)
    {
        this->commandId = commandId;
    }

    /**
     * Adopt the coordinates the command will be executed from, needed for consumption prediction.
     * These will be the current position of the Node by default.
     * TODO It's undecided what should be done if a Node should do if this is different to the current position.
     *
     * @param x
     * @param y
     * @param z
     */
    void setFromCoordinates(double x, double y, double z)
    {
        this->x0 = x;
        this->y0 = y;
        this->z0 = z;
    }
    void setToCoordinates(double x, double y, double z)
    {
        this->x1 = x;
        this->y1 = y;
        this->z1 = z;
    }

    double getX0()
    {
        return x0;
    }
    double getY0()
    {
        return y0;
    }
    double getZ0()
    {
        return z0;
    }
    double getX1()
    {
        return x1;
    }
    double getY1()
    {
        return y1;
    }
    double getZ1()
    {
        return z1;
    }

    /**
     * Check whether or not the current CEE has reached its completion.
     * Depending on the command compares the current position and state of the node with the abort criterion of the command.
     */
    virtual bool commandCompleted() = 0;

    /**
     * Initialize the CEE itself.
     * The method does not modify the befriended node, see setNodeParameters().
     */
    virtual void initializeCEE() = 0;

    /**
     * Transfer CEE parameters to the befriended node.
     * Call this method (after initializeCEE()) if a CEE should actually be executed by the node.
     * Do not call if you just want to pre-simulate effects (e.g. consumption) of the CEE.
     */
    virtual void setNodeParameters() = 0;

    /**
     * Update physical and logical state of the node.
     *
     * @param stepSize simulation time step in seconds
     */
    virtual void updateState(double stepSize) = 0;

    /**
     * Get the overall duration expected for the CEE.
     */
    virtual double getDuration() = 0;

    /**
     * Get the remaining time the CEE is still active.
     * Is the same as getDuration() before execution of the CEE.
     */
    virtual double getRemainingTime() = 0;

    /**
     * Generate one probable energy consumption by the CEE.
     * The consumption dependents on the specific command and command-related parameters (speed etc.)
     * The result is drawn from a stochastic distribution.
     *
     * @param normalized Decide if the full consumption should be returned (false) or a normalized energy amount per second (true)
     * @return statistical consumption, in [mAh] (normalized == false) or [mAh/s] (normalized == true)
     */
    virtual double getProbableConsumption(bool normalized = true, float percentile = NAN) = 0;

    /**
     * Predict the overall consumption for the full command execution procedure
     *
     * @param percentile The percentile to apply to the inverse normal distribution function, in the range 0..1
     * @return The energy to be consumed for the whole CEE, in [mAh]
     */
    virtual double predictFullConsumption(float percentile)
    {
        return getProbableConsumption(false, percentile);
    }

    /**
     *
     */
    CeeType getCeeType()
    {
        return type;
    }

    /**
     *
     */
    virtual char* getCeeTypeString() = 0;

    /**
     *
     * @return normalized consumption, in [mAh/s]
     */
    double getConsumptionPerSecond()
    {
        return consumptionPerSecond;
    }
};

/**
 * A set of command exec engines to be execute by the Node
 */
typedef std::deque<CommandExecEngine*> CEEQueue;

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
    double getDuration() override;
    double getRemainingTime() override;
    double getProbableConsumption(bool normalized = true, float percentile = NAN) override;
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
    double getDuration() override;
    double getRemainingTime() override;
    double getProbableConsumption(bool normalized = true, float percentile = NAN) override;
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
    double getDuration() override;
    double getRemainingTime() override;
    double getProbableConsumption(bool normalized = true, float percentile = NAN) override;
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
    double getDuration() override;
    double getRemainingTime() override;
    double getProbableConsumption(bool normalized = true, float percentile = NAN) override;
    char* getCeeTypeString() override;
};

#endif /* COMMANDEXECENGINE_H_ */
