//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 2015 OpenSim Ltd.
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifdef WITH_OSG
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osgEarth/Capabilities>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthSymbology/Geometry>
#include <osgEarthFeatures/Feature>

#include "MobileNode.h"
#include "OsgEarthScene.h"
#include "ChannelController.h"

using namespace omnetpp;

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;

MobileNode::MobileNode()
{
}

MobileNode::~MobileNode()
{
}

void MobileNode::initialize(int stage)
{
    switch (stage) {
        case 0:
            timeStep = par("timeStep");
            trailLength = par("trailLength");
            modelURL = par("modelURL").stringValue();
            showTxRange = par("showTxRange");
            txRange = par("txRange");
            labelColor = par("labelColor").stringValue();
            label2Color = par("label2Color").stringValue();
            rangeColor = par("rangeColor").stringValue();
            trailColor = par("trailColor").stringValue();
            speed = par("speed");
            break;

        case 1:
            ChannelController::getInstance()->addMobileNode(this);

            // scene is initialized in stage 0 so we have to do our init in stage 1
            auto scene = OsgEarthScene::getInstance()->getScene();
            mapNode = osgEarth::MapNode::findMapNode(scene);

            // build up the node representing this module
            // an ObjectLocatorNode allows positioning a model using world coordinates
            locatorNode = new osgEarth::Util::ObjectLocatorNode(mapNode->getMap());
            auto modelNode = osgDB::readNodeFile(modelURL);
            if (!modelNode)
                throw cRuntimeError("Model file \"%s\" not found", modelURL.c_str());

            // disable shader and lighting on the model so textures are correctly shown
            modelNode->getOrCreateStateSet()->setAttributeAndModes(new osg::Program(), osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
            modelNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

            const char *modelColor = par("modelColor");
            if (*modelColor != '\0') {
                auto color = osgEarth::Color(modelColor);
                auto material = new osg::Material();
                material->setAmbient(osg::Material::FRONT_AND_BACK, color);
                material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
                material->setAlpha(osg::Material::FRONT_AND_BACK, 1.0);
                modelNode->getOrCreateStateSet()->setAttribute(material, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
            }

            auto objectNode = new cObjectOsgNode(this); // make the node selectable in Qtenv
            objectNode->addChild(modelNode);
            locatorNode->addChild(objectNode);

            // set the name label if the color is specified
            if (!labelColor.empty()) {
                Style labelStyle;
                labelStyle.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_TOP;
                labelStyle.getOrCreate<TextSymbol>()->declutter() = true;
                labelStyle.getOrCreate<TextSymbol>()->pixelOffset() = osg::Vec2s(0, 40);
                labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color(labelColor);
                labelNode = new LabelNode(getFullName(), labelStyle);
                labelNode->setDynamic(true);
                locatorNode->addChild(labelNode);

                labelStyle.getOrCreate<TextSymbol>()->pixelOffset() = osg::Vec2s(0, 20);
                labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color(label2Color);
                labelStyle.getOrCreate<TextSymbol>()->size() = 12;
                sublabelNode = new LabelNode("---", labelStyle);
                sublabelNode->setDynamic(true);
                locatorNode->addChild(sublabelNode);
            }

            // create a node showing the transmission range
            if (showTxRange) {
                Style rangeStyle;
                rangeStyle.getOrCreate<PolygonSymbol>()->fill()->color() = osgEarth::Color(rangeColor);
                rangeStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
                rangeStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
                rangeNode = new CircleNode(mapNode.get(), GeoPoint::INVALID, Linear(txRange, Units::METERS), rangeStyle);
                locatorNode->addChild(rangeNode);
            }

            // create a node containing a track showing the past trail of the model
            if (trailLength > 0) {
                trailStyle.getOrCreate<LineSymbol>()->stroke()->color() = osgEarth::Color(trailColor);
                trailStyle.getOrCreate<LineSymbol>()->stroke()->width() = 50.0f;
                trailStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
                trailStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
                auto geoSRS = mapNode->getMapSRS()->getGeographicSRS();
                trailNode = new FeatureNode(mapNode.get(), new Feature(new LineString(), geoSRS));
                locatorNode->addChild(trailNode);
            }

            // add the locator node to the scene
            mapNode->getModelLayerGroup()->addChild(locatorNode);

            // schedule start of the mission for each uav (may be delayed by ned parameter)
            cMessage *timer = new cMessage("startMission");
            scheduleAt(par("startTime"), timer);
            break;
    }
}

void MobileNode::refreshDisplay() const
{
    auto geoSRS = mapNode->getMapSRS()->getGeographicSRS();
    double modelheading = fmod((360 + 90 + yaw), 360) - 180;
    double longitude = getLongitude();
    double latitude = getLatitude();
    double altitude = getAltitude();

    // update the 3D position of the model node
    locatorNode->getLocator()->setPosition(osg::Vec3d(longitude, latitude, altitude));
    locatorNode->getLocator()->setOrientation(osg::Vec3d(modelheading, 0, pitch));

    // re-position the range indicator node
    if (showTxRange)
        rangeNode->setPosition(GeoPoint(geoSRS, longitude, latitude));

    // if we are showing the model's track, update geometry in the trackNode
    if (trailNode) {
        // create and assign a new feature containing the updated geometry
        // representing the movement trail as continuous line segments
        auto trailFeature = new Feature(new LineString(&trail), geoSRS, trailStyle);
        trailFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
        trailNode->setFeature(trailFeature);
    }

    // update the position on the 2D canvas, too
    getDisplayString().setTagArg("p", 0, x);
    getDisplayString().setTagArg("p", 1, y);
}

void MobileNode::handleMessage(cMessage *msg)
{
    if (msg->isName("startMission")) {
        //only at the beginning in the simulation, after a delayed 'startTime' in ned file
        loadNextCommand();
        lastUpdate = simTime();
        initializeState();
        msg->setName("update");
        EV_INFO << "UAV #" << this->getIndex() << " initialized and on its way." << endl;
    }
    else if (msg->isName("update")) {
        // update current position to represent position at simTime()
        updateState();
    }
    else {
        throw cRuntimeError("Unknown message name encountered");
    }

    if (commandCompleted()) {
        EV_INFO << "UAV #" << this->getIndex() << " completed its current command. Selecting next command." << endl;
        loadNextCommand();
        initializeState();
    }

    // update the trail data based on the new position
    if (trailNode) {
        // store the new position to be able to create a track later
        trail.push_back(osg::Vec3d(getLongitude(), getLatitude(), getAltitude()));

        // if trail is at max length, remove the oldest point to keep it at "trailLength"
        if (trail.size() > trailLength)
            trail.erase(trail.begin());
    }

    // let the node decide when the next simulation step should happen
    double stepSize = nextNeededUpdate();
    stepSize = (timeStep == 0 || stepSize < timeStep) ? stepSize : timeStep;

    // schedule next update
    lastUpdate = simTime();
    scheduleAt(lastUpdate + stepSize, msg);
}

#endif // WITH_OSG
