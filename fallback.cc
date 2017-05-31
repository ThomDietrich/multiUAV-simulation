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

#if !defined(WITH_OSG) || !defined(WITH_OSGEARTH)
#include <omnetpp.h>

// fallback code used in case OpenSceneGraph and osgEarth is not present on the system
using namespace omnetpp;

class OsgEarthScene : public cSimpleModule
{
protected:
    virtual void initialize() {throw cRuntimeError("This example requires OpenSceneGraph and osgEarth installed");}
    virtual void handleMessage(cMessage *msg) {};
};

Define_Module(OsgEarthScene);

class GenericNode : public cSimpleModule
{
protected:
    virtual void initialize() {throw cRuntimeError("This example requires OpenSceneGraph and osgEarth installed");}
    virtual void handleMessage(cMessage *msg) {};
};

Define_Module(GenericNode);

class ChannelController : public cSimpleModule
{
protected:
    virtual void initialize() {throw cRuntimeError("This example requires OpenSceneGraph and osgEarth installed");}
    virtual void handleMessage(cMessage *msg) {};
};

Define_Module(ChannelController);

#endif // WITH_OSG , WITH_OSGEARTH
