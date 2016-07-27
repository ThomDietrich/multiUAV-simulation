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

/**
 * Abstract Command Execution Engine
 * To be derived for specific Commands -> CEEs
 */
class CommandExecEngine {
protected:
    //SubclassNode *node;
    //SpecializedCommand *command;
public:
    //CommandExecEngine(SubclassNode &node, SpecializedCommand &command) { };
    virtual ~CommandExecEngine() {
    }
    ;
    virtual bool commandCompleted() = 0;
    virtual void initializeState() = 0;
    virtual void updateState(double stepSize) = 0;
};

//make UAVNode known to be reverse reference type
class UAVNode;

/**
 * Waypoint Command Execution Engine
 */
class WaypointCEE: public CommandExecEngine {
protected:
    UAVNode *node;
    WaypointCommand *command;
public:
    WaypointCEE(UAVNode &node, WaypointCommand &command);
    bool commandCompleted() override;
    void initializeState() override;
    void updateState(double stepSize) override;
};

/**
 * HoldPosition Command Execution Engine
 */
class HoldPositionCEE: public CommandExecEngine {
protected:
    UAVNode *node;
    HoldPositionCommand *command;
public:
    HoldPositionCEE(UAVNode &node, HoldPositionCommand &command);
    bool commandCompleted() override;
    void initializeState() override;
    void updateState(double stepSize) override;
};

/**
 * Takeoff Command Execution Engine
 */
class TakeoffCEE: public CommandExecEngine {
protected:
    UAVNode *node;
    TakeoffCommand *command;
public:
    TakeoffCEE(UAVNode &node, TakeoffCommand &command);
    bool commandCompleted() override;
    void initializeState() override;
    void updateState(double stepSize) override;
};

#endif /* COMMANDEXECENGINE_H_ */
