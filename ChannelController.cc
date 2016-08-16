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
#include <osg/PositionAttitudeTransform>
#include <osgEarthUtil/ObjectLocator>

#include "ChannelController.h"

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;

Define_Module(ChannelController);

ChannelController *ChannelController::instance = nullptr;

ChannelController::ChannelController() {
    if (instance) throw cRuntimeError("There can be only one ChannelController instance in the network");
    instance = this;
}

ChannelController::~ChannelController() {
    instance = nullptr;
}

ChannelController *ChannelController::getInstance() {
    if (!instance) throw cRuntimeError("ChannelController::getInstance(): there is no ChannelController module in the network");
    return instance;
}

int ChannelController::findGenericNode(IGenericNode *p) {
    for (int i = 0; i < (int) nodeList.size(); i++)
        if (nodeList[i] == p) return i;

    return -1;
}

void ChannelController::addGenericNode(IGenericNode *p) {
    if (findGenericNode(p) == -1) nodeList.push_back(p);
}

void ChannelController::removeGenericNode(IGenericNode *p) {
    int k = findGenericNode(p);
    if (k != -1) nodeList.erase(nodeList.begin() + k);
}

void ChannelController::initialize(int stage) {
    switch (stage) {
        case 0:
            playgroundLat = getSystemModule()->par("playgroundLatitude");
            playgroundLon = getSystemModule()->par("playgroundLongitude");
            connectionColor = par("connectionColor").stringValue();
            showConnections = par("showConnections").boolValue();
            break;
        case 1:
            auto scene = OsgEarthScene::getInstance()->getScene(); // scene is initialized in stage 0 so we have to do our init in stage 1
            mapNode = osgEarth::MapNode::findMapNode(scene);
            connectionStyle.getOrCreate<LineSymbol>()->stroke()->color() = osgEarth::Color(connectionColor);
            connectionStyle.getOrCreate<LineSymbol>()->stroke()->width() = 5.0f;
            connectionStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_ABSOLUTE;
            connectionStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

            if (showConnections) {
                auto geoSRS = mapNode->getMapSRS()->getGeographicSRS();
                connectionGraphNode = new FeatureNode(mapNode.get(), new Feature(new LineString(), geoSRS));
                connectionGraphNode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                mapNode->getModelLayerGroup()->addChild(connectionGraphNode);
            }
            break;
    }
}

void ChannelController::refreshDisplay() const {
    if (!showConnections) return;

    auto geoSRS = mapNode->getMapSRS()->getGeographicSRS();
    auto connectionGeometry = new osgEarth::Symbology::MultiGeometry();

    for (int i = 0; i < (int) nodeList.size(); ++i) {
        for (int j = i + 1; j < (int) nodeList.size(); ++j) {
            IGenericNode *pi = nodeList[i];
            IGenericNode *pj = nodeList[j];
            double ix = pi->getX(), iy = pi->getY(), iz = pi->getZ();
            double jx = pj->getX(), jy = pj->getY(), jz = pj->getZ();
            if (pi->getTxRange() * pi->getTxRange() > (ix - jx) * (ix - jx) + (iy - jy) * (iy - jy) + (iz - jz) * (iz - jz)) {
                auto ls = new LineString(2);  // a single line geometry
                ls->push_back(osg::Vec3d(pi->getLongitude(), pi->getLatitude(), pi->getAltitude()));
                ls->push_back(osg::Vec3d(pj->getLongitude(), pj->getLatitude(), pj->getAltitude()));
                connectionGeometry->add(ls);
            }
        }
    }

    auto cgraphFeature = new Feature(connectionGeometry, geoSRS, connectionStyle);
    cgraphFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
    connectionGraphNode->setFeature(cgraphFeature);
}

void ChannelController::handleMessage(cMessage *msg) {
    throw cRuntimeError("This module does not process messages");
}

#endif // WITH_OSG
