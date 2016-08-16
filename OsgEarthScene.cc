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
#include <osgDB/ReadFile>
#include <osgEarth/Viewpoint>
#include <osgEarth/MapNode>
#include <osgEarth/Capabilities>
#include <osgEarthAnnotation/RectangleNode>
#include "OsgEarthScene.h"

using namespace omnetpp;
using namespace osgEarth;
using namespace osgEarth::Annotation;

Define_Module(OsgEarthScene);

OsgEarthScene *OsgEarthScene::instance = nullptr;

OsgEarthScene::OsgEarthScene() {
    if (instance) throw cRuntimeError("There can be only one OsgEarthScene instance in the network");
    instance = this;
}

OsgEarthScene::~OsgEarthScene() {
    instance = nullptr;
}

void OsgEarthScene::initialize() {
    scene = osgDB::readNodeFile(par("scene"));
    if (!scene) throw cRuntimeError("Could not read scene file \"%s\"", par("scene").stringValue());

    playgroundLat = getSystemModule()->par("playgroundLatitude");
    playgroundLon = getSystemModule()->par("playgroundLongitude");
    playgroundHeight = getSystemModule()->par("playgroundHeight");
    playgroundWidth = getSystemModule()->par("playgroundWidth");
    double centerLongitude = toLongitude(playgroundWidth / 2);
    double centerLatitude = toLatitude(playgroundHeight / 2);

    cOsgCanvas *builtinOsgCanvas = getParentModule()->getOsgCanvas();

    auto mapNode = MapNode::findMapNode(scene);
    ASSERT(mapNode != nullptr);

    // set up viewer
    const SpatialReference *geoSRS = mapNode->getMapSRS()->getGeographicSRS();
    builtinOsgCanvas->setViewerStyle(cOsgCanvas::STYLE_EARTH);
    // and move the initial view right above it
    builtinOsgCanvas->setEarthViewpoint(osgEarth::Viewpoint("home", centerLongitude, centerLatitude, 50, 0, -22, playgroundHeight * 2));
    // fine tune the ZLimits (clipping) to better fit this scenario
    builtinOsgCanvas->setZLimits(1, 100000);
    builtinOsgCanvas->setScene(scene);

    // set up an annotation to show the playground area
    Style rectStyle;
    rectStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Black, 0.05);
    rectStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    rectStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
    RectangleNode *rect = new RectangleNode(mapNode, GeoPoint(geoSRS, centerLongitude, centerLatitude), Linear(playgroundWidth, Units::METERS),
            Linear(playgroundHeight, Units::METERS), rectStyle);
    mapNode->getModelLayerGroup()->addChild(rect);
}

OsgEarthScene *OsgEarthScene::getInstance() {
    if (!instance) throw cRuntimeError("OsgEarthScene::getInstance(): there is no OsgEarthScene module in the network");
    return instance;
}

void OsgEarthScene::handleMessage(cMessage *msg) {
    throw cRuntimeError("This module does not handle messages from the outside");
}

#endif // WITH_OSG
