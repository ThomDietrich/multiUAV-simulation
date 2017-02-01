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

#ifndef __MOBILENODE_H__
#define __MOBILENODE_H__

#include <osgEarth/MapNode>
#include <osgEarthAnnotation/CircleNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthUtil/ObjectLocator>

#include <omnetpp.h>
#include "GenericNode.h"
#include "ChargingNode.h"
#include "Battery.h"

using namespace omnetpp;

class ChargingNode;

/**
 * A mobile node (with a 3D model) moving around. A range indicator, and the
 * model's track can be shown along with its label.
 */
class MobileNode : public GenericNode {
    
protected:
    //trail (recently visited points)
    osg::ref_ptr<osgEarth::Annotation::FeatureNode> trailNode = nullptr;
    osgEarth::Vec3dVector trail;
    unsigned int trailLength;
    osgEarth::Style trailStyle;
    std::string trailColor;

    double speed; //speed (3D) in [m/s]
    Battery battery; //energy storage
    
public:
    MobileNode();
    virtual ~MobileNode();

    ChargingNode* findNearestCN(double nodeX, double nodeY, double nodeZ);

protected:
    virtual void initialize(int stage) override;
    virtual void refreshDisplay() const override;
    virtual void handleMessage(cMessage *msg) override;
};

#endif
