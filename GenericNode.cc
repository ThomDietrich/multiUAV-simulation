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

#if defined(WITH_OSG) && defined(WITH_OSGEARTH)
#include "GenericNode.h"
#include "OsgEarthScene.h"
#include "ChannelController.h"

#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osgEarth/Capabilities>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthSymbology/Geometry>
#include <osgEarthFeatures/Feature>

using namespace omnetpp;

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;

GenericNode::GenericNode()
{
    // Ignore Warning: members are initialized in "initialize(int stage)"
}

GenericNode::~GenericNode()
{
}

void GenericNode::initialize(int stage)
{
    switch (stage) {
        case 0:
            timeStep = par("timeStep");
            modelURL = par("modelURL").stringValue();
            showTxRange = par("showTxRange");
            txRange = par("txRange");
            labelColor = par("labelColor").stringValue();
            label2Color = par("label2Color").stringValue();
            rangeColor = par("rangeColor").stringValue();
            break;

        case 1:
            ChannelController::getInstance()->addGenericNode(this);

            // scene is initialized in stage 0 so we have to do our init in stage 1
            auto scene = OsgEarthScene::getInstance()->getScene();
            mapNode = osgEarth::MapNode::findMapNode(scene);

            // build up the node representing this module
            // an ObjectLocatorNode allows positioning a model using world coordinates
            locatorNode = new osgEarth::Util::ObjectLocatorNode(mapNode->getMap());
            auto modelNode = osgDB::readNodeFile(modelURL);
            if (!modelNode) throw cRuntimeError("Model file \"%s\" not found", modelURL.c_str());

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
                labelStyle.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_TOP;
                labelStyle.getOrCreate<TextSymbol>()->declutter() = true;
                labelStyle.getOrCreate<TextSymbol>()->pixelOffset() = osg::Vec2s(0, 40);
                labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color(labelColor);
                labelStyle.getOrCreate<TextSymbol>()->halo()->color() = osgEarth::Color::DarkGray;
                labelStyle.getOrCreate<TextSymbol>()->haloOffset() = 0.2;
                labelNode = new LabelNode(getFullName(), labelStyle);
                labelNode->setDynamic(true);
                locatorNode->addChild(labelNode);

                labelStyle.getOrCreate<TextSymbol>()->pixelOffset() = osg::Vec2s(0, 20);
                labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osgEarth::Color(label2Color);
                labelStyle.getOrCreate<TextSymbol>()->size() = 12;
                sublabelNode = new LabelNode(par("stateSummary"), labelStyle);
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

            // add the locator node to the scene
            mapNode->getModelLayerGroup()->addChild(locatorNode);

            // schedule start of the mission for each node (may be delayed by ned parameter)
            //cMessage *timer = new cMessage("startMission");
            //scheduleAt(par("startTime"), timer);
            break;
    }
}

void GenericNode::refreshDisplay() const
{
    auto geoSRS = mapNode->getMapSRS(); //->getGeographicSRS();
    double longitude = getLongitude();
    double latitude = getLatitude();
    double altitude = getAltitude();

    // update the 3D position of the model node
    locatorNode->getLocator()->setPosition(osg::Vec3d(longitude, latitude, altitude));
    locatorNode->getLocator()->setOrientation(osg::Vec3d(yaw, 0, pitch));

    // re-position the range indicator node
    if (showTxRange) rangeNode->setPosition(GeoPoint(geoSRS, longitude, latitude));

    // update the position on the 2D canvas, too
    getDisplayString().setTagArg("p", 0, x);
    getDisplayString().setTagArg("p", 1, y);
}

void GenericNode::handleMessage(cMessage *msg)
{
    double stepSize = 0;
    if (msg->isName("startProvision")) {
        MissionMsg *mmmsg = check_and_cast<MissionMsg *>(msg);
        if (not mmmsg->getMission().empty()) loadCommands(mmmsg->getMission(), false);
        selectNextCommand();
        initializeState();
        EV_INFO << "UAV initialized for provisioning and on its way." << endl;
        msg->setName("update");
        stepSize = 0;
    }
    else if (msg->isName("startMission")) {
        activeInField = true;
        MissionMsg *mmmsg = check_and_cast<MissionMsg *>(msg);
        if (not mmmsg->getMission().empty()) loadCommands(mmmsg->getMission());
        commandsRepeat = mmmsg->getMissionRepeat();
        missionId = mmmsg->getMissionId();
        selectNextCommand();
        initializeState();
        EV_INFO << "UAV initialized and on its way." << endl;
        msg->setName("update");
        stepSize = 0;
    }
    else if (msg->isName("update")) {
        updateState();
        stepSize = nextNeededUpdate();
        stepSize = (timeStep == 0 || stepSize < timeStep) ? stepSize : timeStep;
        if (commandCompleted()) {
            msg->setName("nextCommand");
            stepSize = 0;
        }
    }
    else if (msg->isName("nextCommand")) {

        if (commandExecEngine != nullptr) commandExecEngine->performExitActions();

        // Check if further commands are available
        if (not hasCommandsInQueue()) {
            EV_ERROR << commandExecEngine->extractCommand()->getMessageName() << " command completed. Queue empty. This should not happen!" << endl;
            delete msg;
            msg = nullptr;
            //TODO: The node has to do something. Insert Hovering Command?
            return;
        }

        // Build and Send a Command Completed Message to Mission Control
        CmdCompletedMsg *ccmsg = new CmdCompletedMsg("commandCompleted");
        ccmsg->setSourceNodeIndex(this->getIndex());
        ReplacementData *replacementData = endOfOperation();
        if (replacementData != nullptr) {
            ccmsg->setReplacementData(*replacementData);
        }
        else {
            ccmsg->setReplacementDataAvailable(false);
        }
        send(ccmsg, "gate$o", 0);

        // Prepare next command to execute
        EV_INFO << commandExecEngine->extractCommand()->getMessageName() << " command completed. Collecting statistics." << endl;
        collectStatistics();
        selectNextCommand();
        initializeState();
        msg->setName("update");
        stepSize = 0;
    }
    else {
        // Message is unknown for Generic Node and all child classes the super call originated from
        std::string message = "Unknown message name encountered: ";
        message += msg->getFullName();
        throw cRuntimeError(message.c_str());
        delete msg;
        msg = nullptr;
        return;
    }

    lastUpdate = simTime();

    // schedule next update
    if (msg != nullptr) {
        scheduleAt(simTime() + stepSize, msg);
    }
}

/**
 * Check if the Node has Commands to execute
 *
 * @return 'true' if commands are available
 */
bool GenericNode::hasCommandsInQueue()
{
    return (not cees.empty());
}

/**
 * Delete the current commands/CEEs from nodes memory
 */
void GenericNode::clearCommands()
{
    //if (activeInField and not cees.empty()) EV_INFO << __func__ << "(): Pre-existing CEEs removed from node." << endl;
    cees.clear();
}

/**
 * Extracts commands of the current CEEs loaded.
 * Removes non-Mission commands and keeps the current order in place.
 *
 * @return An execution neutral list of commands
 */
CommandQueue* GenericNode::extractCommands()
{
    CommandQueue* commands = new CommandQueue();
    for (auto it = cees.begin(); it != cees.end(); it++) {
        CommandExecEngine *cee = *it;
        if (cee->isPartOfMission()) {
            commands->push_back(cee->extractCommand());
        }
    }
    return commands;
}

/**
 * Find and return the cGate pointing to another cModule.
 *
 * @param cMod
 * @return cGate*, 'nullptr' if no gate found
 */
cGate* GenericNode::getOutputGateTo(cModule *cMod)
{
    for (int i = 0; i < this->gateCount(); i++) {
        cGate *gate = this->gateByOrdinal(i);
        if (gate->getType() == cGate::Type::OUTPUT) {
            cModule *gateOwner = gate->getPathEndGate()->getOwnerModule();
            if (gateOwner == cMod) {
                return gate;
            }
            //EV_INFO << "Node is connected to " << gateOwner->getFullName() << " through gate at index " << i << ": " << gate->getFullName() << endl;
        }
    }
    return nullptr;
}

#endif // WITH_OSG
