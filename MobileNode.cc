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

            //Initialize Energy storage
            int capacity = int(par("batteryCapacity"));
            battery = (capacity == 0) ? Battery() : Battery(capacity, capacity / 2 + rand() % capacity / 2);
            WATCH(utilizationSecMission);
            WATCH(utilizationSecMaintenance);
            WATCH(utilizationSecIdle);
            WATCH(utilizationEnergyMission);
            WATCH(utilizationEnergyMaintenance);
            break;
    }
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

    // update the trail data based on the new position
    if (trailNode) {
        // store the new position to be able to create a track later
        trail.push_back(osg::Vec3d(getLongitude(), getLatitude(), getAltitude()));

        // if trail is at max length, remove the oldest point to keep it at "trailLength"
        if (trail.size() > trailLength) trail.erase(trail.begin());
    }
}

ChargingNode* MobileNode::findNearestCN(double nodeX, double nodeY, double nodeZ)
{
    //cModule* osgEarthNet = getModuleByPath("OsgEarthNet");
    //cModule* osgEarthNet = getModuleByPath("OsgEarthNet.cs");
    //EV_INFO << "MobileNode #" << this->getIndex() << " name: " << this->getFullName() << " parent name: " << osgEarthNet->getFullName() << endl;
    double minDistance = DBL_MAX;
    ChargingNode *nearest = nullptr;
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
