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
#include <osg/Texture2D>
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
            commandPreviewCommandCount = par("commandPreviewCommandCount");
            commandPreviewEnabled = par("commandPreviewEnabled").boolValue();
            commandPreviewMissionColor = osgEarth::Color(par("commandPreviewMissionColor").stringValue());
            commandPreviewMaintenanceColor = osgEarth::Color(par("commandPreviewMaintenanceColor").stringValue());
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

            if (commandPreviewEnabled) {
                auto geoSRS = mapNode->getMapSRS();
                waypointStyle.getOrCreate<osgEarth::LineSymbol>()->stroke()->color() = commandPreviewMissionColor;
                waypointStyle.getOrCreate<osgEarth::LineSymbol>()->stroke()->width() = 5.0f;
                waypointsNode = new FeatureNode(mapNode.get(), new Feature(new LineString(), geoSRS));
                mapNode->getModelLayerGroup()->addChild(waypointsNode);

                waypointMaintStyle.getOrCreate<osgEarth::LineSymbol>()->stroke()->color() = commandPreviewMaintenanceColor;
                waypointMaintStyle.getOrCreate<osgEarth::LineSymbol>()->stroke()->width() = 3.0f;
                waypointsMaintNode = new FeatureNode(mapNode.get(), new Feature(new LineString(), geoSRS));
                mapNode->getModelLayerGroup()->addChild(waypointsMaintNode);
            }

            //Initialize Energy storage
            int capacity = int(par("batteryCapacity"));
            int charge = int(par("batteryRemaining"));
            battery = (capacity == 0) ? Battery() : Battery(capacity, charge);
            //
            WATCH(utilizationSecMission);
            WATCH(utilizationSecMaintenance);
            WATCH(utilizationSecCharge);
            WATCH(utilizationSecIdle);
            //
            WATCH(utilizationEnergyMission);
            WATCH(utilizationEnergyMaintenance);
            WATCH(utilizationEnergyCharge);
            //
            WATCH(utilizationEnergyOverdrawMission);
            WATCH(utilizationEnergyOverdrawMaintenance);
            //
            WATCH(utilizationCountMissions);
            WATCH(utilizationCountManeuversMission);
            WATCH(utilizationCountManeuversMaintenance);
            WATCH(utilizationCountChargeState);
            WATCH(utilizationCountOverdrawnAfterMission);
            WATCH(utilizationCountIdleState);
            //
            WATCH(utilizationFail);
            break;
    }
}

void MobileNode::finish()
{
    recordScalar("utilizationSecMission", utilizationSecMission);
    recordScalar("utilizationSecMaintenance", utilizationSecMaintenance);
    recordScalar("utilizationSecCharge", utilizationSecCharge);
    recordScalar("utilizationSecIdle", utilizationSecIdle);
    //
    recordScalar("utilizationEnergyMission", utilizationEnergyMission);
    recordScalar("utilizationEnergyMaintenance", utilizationEnergyMaintenance);
    recordScalar("utilizationEnergyCharge", utilizationEnergyCharge);
    //
    recordScalar("utilizationEnergyOverdrawMission", utilizationEnergyOverdrawMission);
    recordScalar("utilizationEnergyOverdrawMaintenance", utilizationEnergyOverdrawMaintenance);
    //
    recordScalar("utilizationCountMissions", utilizationCountMissions);
    recordScalar("utilizationCountManeuversMission", utilizationCountManeuversMission);
    recordScalar("utilizationCountManeuversMaintenance", utilizationCountManeuversMaintenance);
    recordScalar("utilizationCountChargeState", utilizationCountChargeState);
    recordScalar("utilizationCountOverdrawnAfterMission", utilizationCountOverdrawnAfterMission);
    recordScalar("utilizationCountIdleState", utilizationCountIdleState);
    //
    recordScalar("utilizationFail", utilizationFail);
    //
    recordScalar("simulationTime", simTime().dbl());
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
}

void MobileNode::handleMessage(cMessage *msg)
{
    double stepSize = 0;
    if (msg->isName("mobileNodeExit")) {
        ChargeCEE *cee = check_and_cast<ChargeCEE *>(commandExecEngine);
        ChargingNode *cn = cee->extractCommand()->getChargingNode();
        delete msg;
        msg = nullptr;
        send(new cMessage("mobileNodeExit"), getOutputGateTo(cn));
    }
    else {
        bool commandPreview = commandPreviewEnabled && (msg->isName("nextCommand") || msg->isName("startProvision") || msg->isName("startMission"));

        GenericNode::handleMessage(msg);
        msg = nullptr;

        if (commandPreview && getEnvir()->isGUI()) drawCommandPreview();
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

/**
 * Adjusts the sublabel color according to current battery charge.
 */
void inline MobileNode::evaluateBatteryCharge()
{
    double remainingPercentage = getBattery()->getRemainingPercentage();
    double h = 300 * (remainingPercentage / 100);
    const double s = 1;
    const double v = 1;
    osg::Vec4f colorVec = hsv2rgb(h, s, v);
    labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color(colorVec);
    labelStyle.getOrCreate<TextSymbol>()->halo()->color() = osgEarth::Color::Black;
    sublabelNode.get()->setStyle(labelStyle);
}

/**
 * Converts HSV coloring format into RGB with alpha = 1.0f. Hue (h) must be given in range [0,360], whereas saturation (s) and value (v) must be in range [0,1].
 * For more information about the math, see https://www.rapidtables.com/convert/color/hsl-to-rgb.html
 */
osg::Vec4f MobileNode::hsv2rgb(double h, double s, double v)
{
    double c = v * s;
    double x = c * (1 - abs(fmod((double) ((h / 60)), (double) (2)) - 1));
    double m = v - c;
    osg::Vec4f colorVec;
    switch ((int) ((h / 60))) {
        case 0:
            colorVec.set(c, x, 0, 1);
            break;
        case 1:
            colorVec.set(x, c, 0, 1);
            break;
        case 2:
            colorVec.set(0, c, x, 1);
            break;
        case 3:
            colorVec.set(0, x, c, 1);
            break;
        case 4:
            colorVec.set(x, 0, c, 1);
            break;
        case 5:
            colorVec.set(c, 0, x, 1);
            break;
        default:
            colorVec.set(0, 0, 0, 1);
            break;
    }
    colorVec.x() = (colorVec.x() + m);
    colorVec.y() = (colorVec.y() + m);
    colorVec.z() = (colorVec.z() + m);
    return colorVec;
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

void MobileNode::drawCommandPreview()
{
    if (not waypoints.empty()) waypoints.clear();
    if (not waypointsMaint.empty()) waypointsMaint.clear();
    if (not holdCommandNodes.empty()) {
        for (std::vector<osg::ref_ptr<osgEarth::Util::ObjectLocatorNode>>::iterator it = holdCommandNodes.begin(); it != holdCommandNodes.end(); ++it) {
            mapNode->getModelLayerGroup()->removeChild(*it);
        }
        holdCommandNodes.clear();
    }

    // add current location
    unsigned short countDrawnCommands = 0;
    waypoints.push_back(osg::Vec3d(getLongitude(), getLatitude(), getAltitude()));

    CEEQueue currentAndFutureCEEs = cees;
    currentAndFutureCEEs.push_front(commandExecEngine);

    for (auto it = currentAndFutureCEEs.cbegin(); it != currentAndFutureCEEs.cend(); ++it) {
        if (commandPreviewCommandCount != 0 && countDrawnCommands == commandPreviewCommandCount) break;

        CommandExecEngine* cee = *it;

        if (cee->isCeeType(CeeType::IDLE)) {
            // do nothing
        }
        else if (cee->isCeeType(CeeType::CHARGE)) {
            // do nothing
        }
        else if (cee->isCeeType(CeeType::HOLDPOSITION)) {
            auto sphere = new osg::Sphere(osg::Vec3(0, 0, 0), 5);
            auto sphereDrawable = new osg::ShapeDrawable(sphere);
            sphereDrawable->setColor(commandPreviewMissionColor);
            sphereDrawable->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
            auto sphereGeode = new osg::Geode();
            sphereGeode->addDrawable(sphereDrawable);
            osg::ref_ptr<osgEarth::Util::ObjectLocatorNode> node = new osgEarth::Util::ObjectLocatorNode(mapNode->getMap());
            node->addChild(sphereGeode);
            node->getLocator()->setPosition(osg::Vec3d( //
                    OsgEarthScene::getInstance()->toLongitude(cee->getX1()), //
                    OsgEarthScene::getInstance()->toLatitude(cee->getY1()), //
                    cee->getZ1()));
            mapNode->getModelLayerGroup()->addChild(node);
            holdCommandNodes.push_back(node);
        }
        else if (cee->isCeeType(CeeType::EXCHANGE)) {
            auto sphere = new osg::Sphere(osg::Vec3(0, 0, 0), 5);
            auto sphereDrawable = new osg::ShapeDrawable(sphere);
            sphereDrawable->setColor(commandPreviewMissionColor);
            sphereDrawable->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
            auto sphereGeode = new osg::Geode();
            sphereGeode->addDrawable(sphereDrawable);
            osg::ref_ptr<osgEarth::Util::ObjectLocatorNode> node = new osgEarth::Util::ObjectLocatorNode(mapNode->getMap());
            node->addChild(sphereGeode);
            node->getLocator()->setPosition(osg::Vec3d( //
                    OsgEarthScene::getInstance()->toLongitude(cee->getX1()), //
                    OsgEarthScene::getInstance()->toLatitude(cee->getY1()), //
                    cee->getZ1()));
            mapNode->getModelLayerGroup()->addChild(node);
            holdCommandNodes.push_back(node);
        }
        else {
            if (cee->isPartOfMission()) {
                waypoints.push_back(osg::Vec3d( //
                        OsgEarthScene::getInstance()->toLongitude(cee->getX1()), //
                        OsgEarthScene::getInstance()->toLatitude(cee->getY1()), //
                        cee->getZ1()));
            }
            else {
                waypointsMaint.push_back(osg::Vec3d( //
                        OsgEarthScene::getInstance()->toLongitude(cee->getX0()), //
                        OsgEarthScene::getInstance()->toLatitude(cee->getY0()), //
                        cee->getZ0()));
                waypointsMaint.push_back(osg::Vec3d( //
                        OsgEarthScene::getInstance()->toLongitude(cee->getX1()), //
                        OsgEarthScene::getInstance()->toLatitude(cee->getY1()), //
                        cee->getZ1()));
            }
        }

        countDrawnCommands++;
    }

    auto geoSRS = mapNode->getMapSRS();
    auto waypointsFeature = new Feature(new LineString(&waypoints), geoSRS, waypointStyle);
    waypointsFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
    waypointsNode->setFeature(waypointsFeature);
    auto waypointsMaintFeature = new Feature(new LineString(&waypointsMaint), geoSRS, waypointMaintStyle);
    waypointsMaintFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
    waypointsMaintNode->setFeature(waypointsMaintFeature);
}

#endif // WITH_OSG
