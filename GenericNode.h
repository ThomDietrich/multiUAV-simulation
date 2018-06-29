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

#ifndef __GENERICNODE_H__
#define __GENERICNODE_H__

#include <omnetpp.h>

#include <osg/PositionAttitudeTransform>
#include <osgEarth/MapNode>
#include <osgEarth/GeoTransform>
#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthUtil/ObjectLocator>

#include "OsgEarthScene.h"

#include "Command.h"
#include "CommandExecEngine.h"
#include "msgs/MissionMsg_m.h"
#include "msgs/CmdCompletedMsg_m.h"
#include "ReplacementData.h"
//#include "ChargingNode.h"

using namespace omnetpp;

class ReplacementData;
class ChargingNode;

/**
 * Interface to be implemented by nodes to be able to register in ChannelController.
 */
class IGenericNode {
public:
    virtual ~IGenericNode()
    {
    }
    virtual double getX() const = 0;
    virtual double getY() const = 0;
    virtual double getZ() const = 0;
    virtual double getLatitude() const = 0;
    virtual double getLongitude() const = 0;
    virtual double getAltitude() const = 0;
    virtual double getTxRange() const = 0;
    virtual double getYaw() const = 0;
    virtual double getPitch() const = 0;
};

/**
 *
 */
class GenericNode : public cSimpleModule, public IGenericNode {
protected:
    // configuration
    double timeStep;
    osgEarth::Style labelStyle;
    std::string labelColor;
    std::string label2Color;
    std::string rangeColor;
    std::string modelURL;
    bool showTxRange;
    double txRange;

    /// Timestamp when the last time-related node state update happened
    simtime_t lastUpdate = 0;

    /// Contains future Command Execution Engines
    CEEQueue cees;

    /// Instance of CEE subclass, contains current command
    CommandExecEngine *commandExecEngine = nullptr;

    /// Repeat commands, rotate queue
    bool commandsRepeat = false;

    /// Unique identifier for a mission
    /// Poor man's hack: -1=noMission, -2=Idle
    int missionId = -1;

    /// If true the node is actively executing commands.
    /// If false the node is not performing any actions, doesn't move or consume energy.
    bool activeInField = false;

    /// state
    double x, y, z; // relative to playground origin (top left) in meters


    /**
     * yaw/horizontal orientation in degrees
     * 0° = oriented to the right of the playground
     * 90° = oriented to the bottom of the playground
     * 180° = oriented to the left of the playground
     * 270° = oriented to the top of the playground
     */
    double yaw = 0;

    /**
     * pitch/vertical orientation in degrees
     * -90° = downwards
     * 0° = forward (in yaw direction)
     * 90° = upwards
     */
    double pitch = 0;

    /**
     * climb angle (pitch heading) in degrees
     * -90° = downwards
     * 0° = forward (in yaw direction)
     * 90° = upwards
     */
    double climbAngle = 0;

    // the node containing the osgEarth data
    osg::observer_ptr<osgEarth::MapNode> mapNode = nullptr;
    // osgEarth node for 3D visualization
    osg::ref_ptr<osgEarth::Util::ObjectLocatorNode> locatorNode = nullptr;
    // range indicator node
    osg::ref_ptr<osgEarth::Annotation::CircleNode> rangeNode = nullptr;
    // label visible next to node
    osg::ref_ptr<osgEarth::Annotation::LabelNode> labelNode = nullptr;
    // second label beneath labelNode
    osg::ref_ptr<osgEarth::Annotation::LabelNode> sublabelNode = nullptr;
    // trail annotation

public:
    GenericNode();
    virtual ~GenericNode();

    double getX() const override
    {
        return x;
    }
    double getY() const override
    {
        return y;
    }
    double getZ() const override
    {
        return z;
    }
    double getLatitude() const override
    {
        return OsgEarthScene::getInstance()->toLatitude(getY());
    }
    double getLongitude() const override
    {
        return OsgEarthScene::getInstance()->toLongitude(getX());
    }
    double getAltitude() const override
    {
        return getZ();
    }
    double getTxRange() const override
    {
        return txRange;
    }
    double getYaw() const override
    {
        return yaw;
    }
    double getPitch() const override
    {
        return pitch;
    }
    double getClimbAngle() const
    {
        return climbAngle;
    }
    CommandExecEngine* getCommandExecEngine() const
    {
        return commandExecEngine;
    }
    virtual bool hasCommandsInQueue();
    virtual void loadCommands(CommandQueue commands, bool isMission = true) = 0;
    virtual void clearCommands();
    virtual CommandQueue* extractCommands();
    virtual CommandQueue* extractAllCommands();

    virtual cGate* getOutputGateTo(cModule *cMod);

protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override
    {
        return 2;
    }
    virtual void handleMessage(cMessage *msg) override;
    virtual void refreshDisplay() const override;
    virtual void updateState() = 0;
    virtual bool isCommandCompleted() = 0;
    virtual void selectNextCommand() = 0;
    virtual void collectStatistics() = 0;

    /**
     * Execute once after new command was selected.
     * Initializes state of the Node based on the new command.
     */
    virtual void initializeState() = 0;
    virtual double nextNeededUpdate() = 0;
    virtual ReplacementData* endOfOperation() = 0;
};

#endif
