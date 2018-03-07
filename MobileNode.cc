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
            commandCount = par("commandCount");
            commandPreviewEnabled = par("commandPreviewEnabled").boolValue();
            commandPreviewColor = osgEarth::Color(par("commandPreviewColor").stringValue());
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
                waypointStyle.getOrCreate<osgEarth::LineSymbol>()->stroke()->color() = commandPreviewColor;
                waypointStyle.getOrCreate<osgEarth::LineSymbol>()->stroke()->width() = 5.0f;
                auto geoSRS = mapNode->getMapSRS();
                waypointsNode = new FeatureNode(mapNode.get(), new Feature(new LineString(), geoSRS));
                mapNode->getModelLayerGroup()->addChild(waypointsNode);
            }

            //Initialize Energy storage
            int capacity = int(par("batteryCapacity"));
            int charge = int(par("batteryRemaining"));
            battery = (capacity == 0) ? Battery() : Battery(capacity, charge);
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

        if (commandPreviewEnabled) {
            if (not waypoints.empty()) waypoints.clear();
            if (not holdCommandNodes.empty()) {
                for (std::vector<osg::ref_ptr<osgEarth::Util::ObjectLocatorNode>>::iterator it = holdCommandNodes.begin(); it != holdCommandNodes.end(); ++it) {
                    mapNode->getModelLayerGroup()->removeChild(*it);
                }
                holdCommandNodes.clear();
            }

            // add current location
            unsigned short count = 1;
            waypoints.push_back(osg::Vec3d(getLongitude(), getLatitude(), getAltitude()));

            Command* extractedCommand = commandExecEngine->extractCommand();
            if (0 == strcmp("charge", extractedCommand->getMessageName())) {
                // do nothing
            }
            else if (0 == strcmp("holdPosition", extractedCommand->getMessageName())) {
                auto sphere = new osg::Sphere(osg::Vec3(0, 0, 0), 5);
                auto sphereDrawable = new osg::ShapeDrawable(sphere);
                sphereDrawable->setColor(commandPreviewColor);
                sphereDrawable->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
                auto sphereGeode = new osg::Geode();
                sphereGeode->addDrawable(sphereDrawable);
                osg::ref_ptr<osgEarth::Util::ObjectLocatorNode> node = new osgEarth::Util::ObjectLocatorNode(mapNode->getMap());
                node->addChild(sphereGeode);
                node->getLocator()->setPosition(osg::Vec3d(                                  // pretty
                        OsgEarthScene::getInstance()->toLongitude(extractedCommand->getX()), // set X
                        OsgEarthScene::getInstance()->toLatitude(extractedCommand->getY()),  // set Y
                        extractedCommand->getZ()                                             // set Z
                        ));
                mapNode->getModelLayerGroup()->addChild(node);
                holdCommandNodes.push_back(node);
            }
            /* TODO: check implementation later on
             else if (0 == strcmp("exchange", extractedCommand->getMessageName())) {

             auto image = osgDB::readImageFile("EXCHANGE.png");
             auto texture = new osg::Texture2D();
             texture->setImage(image);
             auto icon = osg::createTexturedQuadGeometry(osg::Vec3(0.0, 0.0, 0.0), osg::Vec3(image->s(), 0.0, 0.0), osg::Vec3(0.0, image->t(), 0.0), 0.0,
             0.0, 1.0, 1.0);
             icon->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture);
             icon->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
             icon->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
             icon->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
             auto geode = new osg::Geode();
             geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
             geode->addDrawable(icon);
             auto autoTransform = new osg::AutoTransform();
             autoTransform->setAutoScaleToScreen(true);
             autoTransform->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
             autoTransform->addChild(geode);
             osg::ref_ptr<osgEarth::Util::ObjectLocatorNode> node = new osgEarth::Util::ObjectLocatorNode(mapNode->getMap());
             node->addChild(autoTransform);
             mapNode->getModelLayerGroup()->addChild(node);
             holdCommandNodes.push_back(node);
             }
             //*/
            else {
                waypoints.push_back(osg::Vec3d(                                              // pretty
                        OsgEarthScene::getInstance()->toLongitude(extractedCommand->getX()), // set X
                        OsgEarthScene::getInstance()->toLatitude(extractedCommand->getY()),  // set Y
                        extractedCommand->getZ()                                             // set Z
                        ));
            }
            count += 1;
            if (not cees.empty()) {
                for (std::deque<CommandExecEngine*>::iterator it = cees.begin(); it != cees.end(); ++it) {
                    if (commandCount > 0 && count >= commandCount) break;
                    extractedCommand = (*it)->extractCommand();
                    if (0 == strcmp("charge", extractedCommand->getMessageName())) {
                        // do nothing
                    }
                    else if (0 == strcmp("holdPosition", extractedCommand->getMessageName())) {
                        auto sphere = new osg::Sphere(osg::Vec3(0, 0, 0), 5);
                        auto sphereDrawable = new osg::ShapeDrawable(sphere);
                        sphereDrawable->setColor(commandPreviewColor);
                        sphereDrawable->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
                        auto sphereGeode = new osg::Geode();
                        sphereGeode->addDrawable(sphereDrawable);
                        osg::ref_ptr<osgEarth::Util::ObjectLocatorNode> node = new osgEarth::Util::ObjectLocatorNode(mapNode->getMap());
                        node->addChild(sphereGeode);
                        node->getLocator()->setPosition(osg::Vec3d(                                  // pretty
                                OsgEarthScene::getInstance()->toLongitude(extractedCommand->getX()), // set X
                                OsgEarthScene::getInstance()->toLatitude(extractedCommand->getY()),  // set Y
                                extractedCommand->getZ()                                             // set Z
                                ));
                        mapNode->getModelLayerGroup()->addChild(node);
                        holdCommandNodes.push_back(node);
                    }
                    /* TODO: check implementation later on
                     else if (0 == strcmp("exchange", extractedCommand->getMessageName())) {

                     auto image = osgDB::readImageFile("EXCHANGE.png");
                     auto texture = new osg::Texture2D();
                     texture->setImage(image);
                     auto icon = osg::createTexturedQuadGeometry(osg::Vec3(0.0, 0.0, 0.0), osg::Vec3(image->s(), 0.0, 0.0), osg::Vec3(0.0, image->t(), 0.0), 0.0,
                     0.0, 1.0, 1.0);
                     icon->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture);
                     icon->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
                     icon->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
                     icon->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                     auto geode = new osg::Geode();
                     geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
                     geode->addDrawable(icon);
                     auto autoTransform = new osg::AutoTransform();
                     autoTransform->setAutoScaleToScreen(true);
                     autoTransform->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
                     autoTransform->addChild(geode);
                     osg::ref_ptr<osgEarth::Util::ObjectLocatorNode> node = new osgEarth::Util::ObjectLocatorNode(mapNode->getMap());
                     node->addChild(autoTransform);
                     mapNode->getModelLayerGroup()->addChild(node);
                     holdCommandNodes.push_back(node);
                     }
                     //*/
                    else {
                        waypoints.push_back(osg::Vec3d(                                              // pretty
                                OsgEarthScene::getInstance()->toLongitude(extractedCommand->getX()), // set X
                                OsgEarthScene::getInstance()->toLatitude(extractedCommand->getY()),  // set Y
                                extractedCommand->getZ()                                             // set Z
                                ));
                    }
                    count += 1;
                }
            }
            auto geoSRS = mapNode->getMapSRS();
            auto waypointsFeature = new Feature(new LineString(&waypoints), geoSRS, waypointStyle);
            waypointsFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
            waypointsNode->setFeature(waypointsFeature);
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

#endif // WITH_OSG
