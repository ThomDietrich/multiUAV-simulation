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

#include <osgEarth/MapNode>
#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthUtil/ObjectLocator>

#include <omnetpp.h>
#include "OsgEarthScene.h"
#include "Command.h"
#include "CommandExecEngine.h"

using namespace omnetpp;

/**
 * Interface to be implemented by nodes to be able to register in ChannelController.
 */
class IGenericNode {
public:
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

typedef std::deque<Command*> CommandQueue;

class GenericNode : public cSimpleModule, public IGenericNode {
    
protected:
    // configuration
    double timeStep;
    simtime_t lastUpdate;
    std::string labelColor;
    std::string label2Color;
    std::string rangeColor;
    std::string modelURL;
    bool showTxRange;
    double txRange;

    //queue, contains future commands
    CommandQueue commands;
    //instance of CEE subclass, contains current command
    CommandExecEngine *commandExecEngine = nullptr;

    // state
    double x, y, z; // relative to playground origin in meters
    double yaw; // horizontal orientation in degrees
    double pitch; // vertical orientation in degrees. be aware, that yaw and pitch do not directly result in heading
    
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

    double getX() const override {
        return x;
    }
    double getY() const override {
        return y;
    }
    double getZ() const override {
        return z;
    }
    double getLatitude() const override {
        return OsgEarthScene::getInstance()->toLatitude(getY());
    }
    double getLongitude() const override {
        return OsgEarthScene::getInstance()->toLongitude(getX());
    }
    double getAltitude() const override {
        return getZ();
    }
    double getTxRange() const override {
        return txRange;
    }
    double getYaw() const override {
        return yaw;
    }
    double getPitch() const override {
        return pitch;
    }
    
protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override {
        return 2;
    }
    virtual void handleMessage(cMessage *msg) override;
    virtual void refreshDisplay() const override;
    virtual void updateState() = 0;
    virtual bool commandCompleted() = 0;
    virtual void selectNextCommand() = 0;

    /**
     * Execute once after new command was selected.
     * Initializes state of the Node based on the new command.
     */
    virtual void initializeState() = 0;
    virtual double nextNeededUpdate() = 0;
};

#endif
