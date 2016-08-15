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
#include "GenericNode.h"
#include "Battery.h"

using namespace omnetpp;

/**
 * A mobile node (with a 3D model) moving around. A range indicator, and the
 * model's track can be shown along with its label.
 */
class MobileNode : public GenericNode {
protected:
    unsigned int trailLength;
    std::string trailColor;
    bool showTxRange;
    double txRange;

    //Energy storage
    Battery battery;

    //state
    double speed; // in meters per second

    // trail annotation
    osg::ref_ptr<osgEarth::Annotation::FeatureNode> trailNode = nullptr;
    osgEarth::Style trailStyle;
    osgEarth::Vec3dVector trail;  // recently visited points

public:
    MobileNode();
    virtual ~MobileNode();

    double getSpeed() const {
        return speed;
    }

protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override {
        return 2;
    }
    virtual void handleMessage(cMessage *msg) override;
    virtual void refreshDisplay() const override;
};

#endif
