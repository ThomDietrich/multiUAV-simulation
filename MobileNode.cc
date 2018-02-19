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

#ifdef WITH_OSG
#include <osg/Node>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgEarth/Capabilities>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthSymbology/Geometry>
#include <osgEarthFeatures/Feature>

#include "MobileNode.h"
#include "OsgEarthScene.h"

using namespace omnetpp;

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;

MobileNode::MobileNode()
{
    // Ignore Warning: members are initialized in "initialize(int stage)"
}

MobileNode::~MobileNode()
{
}

void MobileNode::initialize(int stage)
{
    GenericNode::initialize(stage);
    switch (stage) {
        case 0:
            trailLength = par("trailLength");
            trailColor = par("trailColor").stringValue();
            waypointLength = par("waypointLength");
            waypointsShown = par("waypointsShown").boolValue();
            waypointColor = osgEarth::Color(par("waypointColor").stringValue());
            break;

        case 1:
            // create a node containing a track showing the past trail of the model
            if (trailLength > 0) {
                trailStyle.getOrCreate<LineSymbol>()->stroke()->color() = osgEarth::Color(trailColor);
                trailStyle.getOrCreate<LineSymbol>()->stroke()->width() = 50.0f;
                trailStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
                trailStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
                auto geoSRS = mapNode->getMapSRS(); //->getGeographicSRS();
                trailNode = new FeatureNode(mapNode.get(), new Feature(new LineString(), geoSRS));
                locatorNode->addChild(trailNode);
            }
            if (waypointsShown) {
                waypointStyle.getOrCreate<osgEarth::LineSymbol>()->stroke()->color() = waypointColor;
                waypointStyle.getOrCreate<osgEarth::LineSymbol>()->stroke()->width() = 5.0f;
                auto geoSRS = mapNode->getMapSRS();
                waypointsNode = new FeatureNode(mapNode.get(), new Feature(new LineString(), geoSRS));
                mapNode->getModelLayerGroup()->addChild(waypointsNode);
            }

            osg::Group* coordsArrows = new osg::Group();
            const double heading = 1.5708; // (in radians)

            osg::Group* arrowX = new osg::Group();
            createArrow(arrowX, osgEarth::Color::Red);
            auto transformArrowX = new osg::PositionAttitudeTransform();
            transformArrowX->addChild(arrowX);
            transformArrowX->setPosition(osg::Vec3d(1.55, 0, 4.7));
            transformArrowX->setAttitude(osg::Quat(heading, osg::Vec3d(0, 1, 0)));
            coordsArrows->addChild(transformArrowX);

            osg::Group* arrowY = new osg::Group();
            createArrow(arrowY, osgEarth::Color::Blue);
            auto transformArrowY = new osg::PositionAttitudeTransform();
            transformArrowY->addChild(arrowY);
            transformArrowY->setPosition(osg::Vec3d(0, 1.55, 4.7));
            transformArrowY->setAttitude(osg::Quat(-heading, osg::Vec3d(1, 0, 0)));
            coordsArrows->addChild(transformArrowY);

            osg::Group* arrowZ = new osg::Group();
            createArrow(arrowZ, osgEarth::Color::Green);
            auto transformArrowZ = new osg::PositionAttitudeTransform();
            transformArrowZ->addChild(arrowZ);
            transformArrowZ->setPosition(osg::Vec3d(0, 0, 6));
            coordsArrows->addChild(transformArrowZ);

            locatorNode->addChild(coordsArrows);

            //Initialize Energy storage
            int capacity = int(par("batteryCapacity"));
            battery = Battery(capacity, capacity / 2 + rand() % capacity / 2);
            WATCH(utilizationSecMission);
            WATCH(utilizationSecMaintenance);
            WATCH(utilizationSecIdle);
            WATCH(utilizationEnergyMission);
            WATCH(utilizationEnergyMaintenance);
            break;
    }
}

void MobileNode::createArrow(osg::Group* arrow, osgEarth::Color& color)
{
    const float coneRadius = 1;
    const float coneHeight = 2;
    auto cone = new osg::Cone(osg::Vec3(0, 0, 0), coneRadius, coneHeight);
    auto coneDrawable = new osg::ShapeDrawable(cone);
    auto coneGeode = new osg::Geode;
    auto coneStateSet = coneGeode->getOrCreateStateSet();
    auto coneMaterial = new osg::Material();
    coneMaterial->setAmbient(osg::Material::FRONT_AND_BACK, color);
    coneMaterial->setDiffuse(osg::Material::FRONT_AND_BACK, color);
    coneMaterial->setAlpha(osg::Material::FRONT_AND_BACK, 1.0);
    coneStateSet->setAttribute(coneMaterial, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    coneGeode->addDrawable(coneDrawable);
    auto transformConeNode = new osg::PositionAttitudeTransform();
    transformConeNode->addChild(coneGeode);
    transformConeNode->setPosition(osg::Vec3d(0, 0, 2));
    arrow->addChild(transformConeNode);
    const float lineThickness = 0.4;
    const float lineLength = 3;
    auto line = new osg::Box(osg::Vec3(0, 0, 0), lineThickness, lineThickness, lineLength);
    auto lineDrawable = new osg::ShapeDrawable(line);
    auto lineGeode = new osg::Geode;
    auto lineStateSet = lineGeode->getOrCreateStateSet();
    auto lineMaterial = new osg::Material();
    lineMaterial->setAmbient(osg::Material::FRONT_AND_BACK, osgEarth::Color::Black);
    lineMaterial->setDiffuse(osg::Material::FRONT_AND_BACK, osgEarth::Color::Black);
    lineMaterial->setAlpha(osg::Material::FRONT_AND_BACK, 1.0);
    lineStateSet->setAttribute(lineMaterial, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    lineGeode->addDrawable(lineDrawable);
    arrow->addChild(lineGeode);
}

void MobileNode::refreshDisplay() const
{
    GenericNode::refreshDisplay();
    auto geoSRS = mapNode->getMapSRS(); //->getGeographicSRS();
    // if we are showing the model's track, update geometry in the trackNode
    if (trailNode) {
        // create and assign a new feature containing the updated geometry
        // representing the movement trail as continuous line segments
        auto trailFeature = new Feature(new LineString(&trail), geoSRS, trailStyle);
        trailFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
        trailNode->setFeature(trailFeature);
    }
    if (waypointsShown) {
        auto waypointsFeature = new Feature(new LineString(&waypoints), geoSRS, waypointStyle);
        waypointsFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
        waypointsNode->setFeature(waypointsFeature);
    }
}

void MobileNode::handleMessage(cMessage *msg)
{
    double stepSize = 0;
    if (msg->isName("wait")) {
        EV_INFO << __func__ << "(): wait message received" << endl;

        // Prepare Wait Command and WaitCEE for finished node
        WaitCommand *command = new WaitCommand();
        WaitCEE *cee = new WaitCEE(this, command);
        cees.push_front(cee);

        delete msg;
        msg = nullptr;
    }
    else if (msg->isName("nextCommand")) {
        GenericNode::handleMessage(msg);
        msg = nullptr;
        if (waypointsShown) {
            if (not waypoints.empty()) waypoints.clear();
            waypoints.push_back(osg::Vec3d(getLongitude(), getLatitude(), getAltitude()));

            Command* extractedCommand = commandExecEngine->extractCommand();
            if (0 != strcmp("charge", extractedCommand->getMessageName())) {
                waypoints.push_back(osg::Vec3d(                                              // pretty
                        OsgEarthScene::getInstance()->toLongitude(extractedCommand->getX()), // set X
                        OsgEarthScene::getInstance()->toLatitude(extractedCommand->getY()),  // set Y
                        extractedCommand->getZ()                                             // set Z
                        ));
            }
            if (not cees.empty()) {
                for (std::deque<CommandExecEngine*>::iterator it = cees.begin(); it != cees.end(); ++it) {
                    if (waypointLength > 0 && waypoints.size() >= waypointLength) break;
                    extractedCommand = (*it)->extractCommand();
                    waypoints.push_back(osg::Vec3d(                                              // pretty
                            OsgEarthScene::getInstance()->toLongitude(extractedCommand->getX()), // set X
                            OsgEarthScene::getInstance()->toLatitude(extractedCommand->getY()),  // set Y
                            extractedCommand->getZ()                                             // set Z
                            ));
                }
            }
        }
    }
    else if (msg->isName("mobileNodeExit")) {
        ChargeCEE *cee = check_and_cast<ChargeCEE *>(commandExecEngine);
        ChargingNode *cn = cee->extractCommand()->getChargingNode();
        delete msg;
        msg = nullptr;
        send(new cMessage("mobileNodeExit"), getOutputGateTo(cn));
    }
    else {
        GenericNode::handleMessage(msg);
        msg = nullptr;
    }

    if (msg != nullptr) {
        scheduleAt(simTime() + stepSize, msg);
    }

    evaluateBatteryCharge();

    // update the trail data based on the new position
    if (trailNode) {
        // store the new position to be able to create a track later
        trail.push_back(osg::Vec3d(getLongitude(), getLatitude(), getAltitude()));

        // if trail is at max length, remove the oldest point to keep it at "trailLength"
        if (trail.size() > trailLength) trail.erase(trail.begin());
    }
}

void inline MobileNode::evaluateBatteryCharge()
{
    switch (getBattery()->getRemainingPercentage() / 20) {
        case 0:
            labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color::Red;
            labelStyle.getOrCreate<TextSymbol>()->halo()->color() = osgEarth::Color::DarkGray;
            break;
        case 1:
            labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color::Orange;
            labelStyle.getOrCreate<TextSymbol>()->halo()->color() = osgEarth::Color::DarkGray;
            break;
        case 2:
            labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color::Yellow;
            labelStyle.getOrCreate<TextSymbol>()->halo()->color() = osgEarth::Color::DarkGray;
            break;
        case 3:
            labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color::Green;
            labelStyle.getOrCreate<TextSymbol>()->halo()->color() = osgEarth::Color::Gray;
            break;
        case 4:
            labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color::Blue;
            labelStyle.getOrCreate<TextSymbol>()->halo()->color() = osgEarth::Color::Gray;
            break;
        case 5:
            labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color::Black;
            labelStyle.getOrCreate<TextSymbol>()->halo()->color() = osgEarth::Color::Gray;
            break;
        default:
            labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color::White;
            labelStyle.getOrCreate<TextSymbol>()->halo()->color() = osgEarth::Color::DarkGray;
            break;
    }
    sublabelNode.get()->setStyle(labelStyle);
}

ChargingNode* MobileNode::findNearestCN(double nodeX, double nodeY, double nodeZ)
{
    //cModule* osgEarthNet = getModuleByPath("OsgEarthNet");
    //cModule* osgEarthNet = getModuleByPath("OsgEarthNet.cs");
    //EV_INFO << "MobileNode #" << this->getIndex() << " name: " << this->getFullName() << " parent name: " << osgEarthNet->getFullName() << endl;
    double minDistance = DBL_MAX;
    ChargingNode *nearest;
    cModule *network = cSimulation::getActiveSimulation()->getSystemModule();
    for (SubmoduleIterator it(network); !it.end(); ++it) {
        cModule *mod = *it;
        if (mod->isName("cs")) {
            //EV << "Module " << mod->getName() << mod->getFullName() << mod->getFullPath() << endl;
            ChargingNode *cs = dynamic_cast<ChargingNode *>(mod);
            double distanceSum = fabs(cs->getX() - nodeX) + fabs(cs->getY() - nodeY) + fabs(cs->getZ() - nodeZ);
            //EV << "ChargingNode " << cs->getFullName() << " distanceSum=" << distanceSum << endl;
            if (distanceSum < minDistance) {
                minDistance = distanceSum;
                nearest = cs;
            }
        }
    }
    //EV << "ChargingNode selected: " << nearest->getFullName() << endl;
    return nearest;
}

Battery* MobileNode::getBattery()
{
    return &battery;
}

#endif // WITH_OSG
