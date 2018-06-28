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
    WAYPOINT, TAKEOFF, HOLDPOSITION, CHARGE, EXCHANGE, IDLE
};

/**
 * Abstract Command Execution Engine
 * To be derived for specific Commands -> CEEs
 */
class CommandExecEngine {
protected:
    CeeType type;
    void setType(CeeType type);

    int commandId = -1;

    /// coordinates from where the command will be executed
    double x0, y0, z0;
    /// coordinates to where the command is going to
    double x1, y1, z1;
    /// movement data
    double yaw = 0, pitch = 0, climbAngle = 0, speed = 0;

    /// consumption for this CEE (drawn from stochastic distribution)
    /// Negative for charging processes
    /// Normalized to mAh/s
    double consumptionPerSecond = 0;

    bool commandCompleted = false;

    /// CEEs can be part of a mission or special inserted commands, e.g. for maintenance
    bool partOfMission = true;

    /// if set to false, this cee will be executed without a check for a replacement node
    bool replacementNeeded = true;

    simtime_t timeExecutionStart = 0;

public:
    //CommandExecEngine(SubclassNode *boundNode, SpecializedCommand *command) { };

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

    virtual Command* extractCommand() const = 0;

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

    double getX0() const
    {
        return x0;
    }
    double getY0() const
    {
        return y0;
    }
    double getZ0() const
    {
        return z0;
    }
    double getX1() const
    {
        return x1;
    }
    double getY1() const
    {
        return y1;
    }
    double getZ1() const
    {
        return z1;
    }

    /**
     * Check whether or not the current CEE has reached its completion.
     * Depending on the command compares the current position and state of the node with the abort criterion of the command.
     */
    virtual bool isCommandCompleted() = 0;

    void setCommandCompleted()
    {
        this->commandCompleted = true;
    }

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
     * No replacement node is requested for this CEE.
     */
    virtual void setNoReplacementNeeded()
    {
        replacementNeeded = false;
    }

    /**
     * Returns whether or not a replacement node has to be requested for this CEE.
     */
    virtual bool isReplacementNeeded() const
    {
        return replacementNeeded;
    }

    /**
     *
     */
    bool isActive() const
    {
        return timeExecutionStart != 0;
    }

    /**
     * Not all commands have a predefined or deterministic ending time that can be calculated.
     * This function has to be called before getDuration() or getRemainingTime().
     * @return true if duration/deadline is known
     */
    virtual bool hasDeterminedDuration() const
    {
        return true;
    }

    /**
     * Get the overall duration for the CEE in seconds.
     */
    virtual double getOverallDuration() const = 0;

    /**
     * Get the duration so far, for an CEE in execution
     */
    double getDuration() const
    {
        if (timeExecutionStart == 0) throw cRuntimeError("getDuration(): CEE not yet started");
        return (simTime() - timeExecutionStart).dbl();
    }

    /**
     * Get the remaining time the CEE is still active.
     * Is the same as getOverallDuration() before execution of the CEE.
     */
    virtual double getRemainingTime() const = 0;

    /**
     * Generate one probable energy consumption by the CEE.
     * The consumption dependents on the specific command and command-related parameters (speed etc.)
     * The result is drawn from a stochastic distribution.
     *
     * @param normalized Decide if the full consumption should be returned (false) or a normalized energy amount per second (true)
     * @param fromMethod 0: random  1: mean  2: predictionQuantile
     * @return statistical consumption, in [mAh] (normalized == false) or [mAh/s] (normalized == true)
     */
    virtual double getProbableConsumption(bool normalized = true, int fromMethod = 2) const = 0;

    /**
     * Predict the overall consumption for the full command execution procedure
     *
     * @param fromMethod 0: random  1: mean  2: predictionQuantile
     * @return The energy to be consumed for the whole CEE, in [mAh]
     */
    virtual double predictFullConsumption(int fromMethod = 2) const
    {
        return getProbableConsumption(false, fromMethod);
    }

    virtual double predictFullConsumptionQuantile() const
    {
        return getProbableConsumption(false, 2);
    }
    virtual double predictFullConsumptionMean() const
    {
        return getProbableConsumption(false, 1);
    }
    virtual double predictFullConsumptionRandom() const
    {
        return getProbableConsumption(false, 0);
    }
    virtual double predictNormConsumptionRandom() const
    {
        return getProbableConsumption(true, 0);
    }

    /**
     *
     */
    CeeType getCeeType() const
    {
        return type;
    }

    bool isCeeType(CeeType ceeType) const
    {
        return (this->type == ceeType);
    }

    /**
     *
     */
    virtual char* getCeeTypeString() const = 0;

    /**
     *
     * @return normalized consumption, in [mAh/s]
     */
    double getConsumptionPerSecond() const
    {
        return consumptionPerSecond;
    }

    /**
     * Note: This method is only provided for the Charging Station and is not supposed to be used otherwise.
     *
     * @param a new value for normalized consumption, in [mAh/s]
     */
    void setConsumptionPerSecond(double newValue)
    {
        consumptionPerSecond = newValue;
    }

    /**
     * CEEs can be part of a mission or special inserted commands, e.g. for maintenance
     */
    bool isPartOfMission() const
    {
        return partOfMission;
    }

    /**
     * CEEs can be part of a mission or special inserted commands, e.g. for maintenance
     */
    void setPartOfMission(bool partOfMission = true)
    {
        this->partOfMission = partOfMission;
    }

    /**
     * Actions to perform when the CEE execution starts
     */
    virtual void performEntryActions()
    {
    }

    /**
     * Actions to perform after the CEE was executed and before it is replaced by another one
     */
    virtual void performExitActions()
    {
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
    WaypointCEE(UAVNode *boundNode, WaypointCommand *command);
    bool isCommandCompleted() override;
    void initializeCEE() override;
    void setNodeParameters() override;
    void updateState(double stepSize) override;
    double getOverallDuration() const override;
    double getRemainingTime() const override;
    double getProbableConsumption(bool normalized = true, int fromMethod = 2) const override;
    char* getCeeTypeString() const override;

    WaypointCommand* extractCommand() const override
    {
        return command;
    }
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
    TakeoffCEE(UAVNode *boundNode, TakeoffCommand *command);
    bool isCommandCompleted() override;
    void initializeCEE() override;
    void setNodeParameters() override;
    void updateState(double stepSize) override;
    double getOverallDuration() const override;
    double getRemainingTime() const override;
    double getProbableConsumption(bool normalized = true, int fromMethod = 2) const override;
    char* getCeeTypeString() const override;

    TakeoffCommand* extractCommand() const override
    {
        return command;
    }
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
    HoldPositionCEE(UAVNode *boundNode, HoldPositionCommand *command);
    bool isCommandCompleted() override;
    void initializeCEE() override;
    void setNodeParameters() override;
    void updateState(double stepSize) override;
    double getOverallDuration() const override;
    double getRemainingTime() const override;
    double getProbableConsumption(bool normalized = true, int fromMethod = 2) const override;
    char* getCeeTypeString() const override;

    HoldPositionCommand* extractCommand() const override
    {
        return command;
    }
};

/**
 * Charging Command Execution Engine
 */
class ChargeCEE : public CommandExecEngine {
protected:
    UAVNode *node;
    ChargeCommand *command;
public:
    ChargeCEE(UAVNode *boundNode, ChargeCommand *command);
    bool isCommandCompleted() override;
    void initializeCEE() override;
    void setNodeParameters() override;
    void updateState(double stepSize) override;
    double getOverallDuration() const override;
    double getRemainingTime() const override;
    double getProbableConsumption(bool normalized = true, int fromMethod = 2) const override;
    char* getCeeTypeString() const override;

    ChargeCommand* extractCommand() const override
    {
        return command;
    }
    bool hasDeterminedDuration() const override
    {
        return false;
    }
};

/**
 * Exchange Command Execution Engine
 */
class ExchangeCEE : public CommandExecEngine {
protected:
    UAVNode *node;
    ExchangeCommand *command;
public:
    ExchangeCEE(UAVNode *boundNode, ExchangeCommand *command);
    bool isCommandCompleted() override;
    void initializeCEE() override;
    void setNodeParameters() override;
    void updateState(double stepSize) override;
    double getOverallDuration() const override;
    double getRemainingTime() const override;
    double getProbableConsumption(bool normalized = true, int fromMethod = 2) const override;
    char* getCeeTypeString() const override;

    void performEntryActions() override;
    void performExitActions() override;
    GenericNode *getOtherNode();

    bool dataTransferPerformed = false;

    ExchangeCommand* extractCommand() const override
    {
        return command;
    }
    bool hasDeterminedDuration() const override
    {
        return false;
    }
};

/**
 * Wait Command Execution Engine
 */
class IdleCEE : public CommandExecEngine {
protected:
    MobileNode *node;
    IdleCommand *command;
public:
    IdleCEE(MobileNode *boundNode, IdleCommand *command);
    bool isCommandCompleted() override;
    void initializeCEE() override;
    void setNodeParameters() override;
    void updateState(double stepSize) override;
    double getOverallDuration() const override;
    double getRemainingTime() const override;
    double getProbableConsumption(bool normalized = true, int fromMethod = 2) const override;
    char* getCeeTypeString() const override;

    IdleCommand* extractCommand() const override
    {
        return command;
    }
    bool hasDeterminedDuration() const override
    {
        return false;
    }
};

#endif /* COMMANDEXECENGINE_H_ */
