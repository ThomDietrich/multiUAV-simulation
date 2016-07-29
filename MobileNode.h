//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 2015 OpenSim Ltd.
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifndef __MOBILENODE_H__
#define __MOBILENODE_H__

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
 * Interface to be implemented by mobile nodes to be able to
 * register in ChannelController.
 */
class IMobileNode {
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
    virtual double getSpeed() const = 0;
};

typedef std::deque<Command*> CommandQueue;

/**
 * A mobile node (with a 3D model) moving around. A range indicator, and the
 * model's track can be shown along with its label.
 */
class MobileNode: public cSimpleModule, public IMobileNode {
    friend class CommandExecEngine;
    friend class WaypointCEE;
    friend class HoldPositionCEE;
    friend class TakeoffCEE;
protected:
    // configuration
    double timeStep;
    simtime_t lastUpdate;
    unsigned int trailLength;
    std::string labelColor;
    std::string rangeColor;
    std::string trailColor;
    std::string modelURL;
    bool showTxRange;
    double txRange;

    //queue, contains future commands
    CommandQueue commands;
    //instance of CEE childclass, contains current command
    CommandExecEngine *commandExecEngine = nullptr;

    // state
    double x, y, z;  // in meters, relative to playground origin
    double yaw;  // in degrees
    double pitch;  // in degrees
    double speed; // in meters per second
    //osg::Vec3d currentPath;

    // the node containing the osgEarth data
    osg::observer_ptr<osgEarth::MapNode> mapNode = nullptr;
    // osgEarth node for 3D visualization
    osg::ref_ptr<osgEarth::Util::ObjectLocatorNode> locatorNode = nullptr;
    // range indicator node
    osg::ref_ptr<osgEarth::Annotation::CircleNode> rangeNode = nullptr;
    // label visible next to node
    osg::ref_ptr<osgEarth::Annotation::LabelNode> labelNode = nullptr;
    // trail annotation
    osg::ref_ptr<osgEarth::Annotation::FeatureNode> trailNode = nullptr;
    osgEarth::Style trailStyle;
    osgEarth::Vec3dVector trail;  // recently visited points

public:
    MobileNode();
    virtual ~MobileNode();

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
        return OsgEarthScene::getInstance()->toLatitude(y);
    }
    double getLongitude() const override {
        return OsgEarthScene::getInstance()->toLongitude(x);
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
    double getSpeed() const override {
        return speed;
    }

protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override {
        return 2;
    }
    virtual void handleMessage(cMessage *msg) override;
    virtual void refreshDisplay() const override;
    virtual void updateState() = 0;
    virtual void updateState(double stepSize) = 0;
    virtual bool commandCompleted() = 0;
    virtual void loadNextCommand() = 0;
    virtual void initializeState() = 0;
    virtual double nextNeededUpdate() = 0;
};

#endif
