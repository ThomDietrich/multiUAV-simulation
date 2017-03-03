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

#ifndef __MULTIUAV_MISSIONCONTROL_H_
#define __MULTIUAV_MISSIONCONTROL_H_

#include <omnetpp.h>
#include "OsgEarthScene.h"
#include "Command.h"
#include "UAVNode.h"

using namespace omnetpp;

/**
 * TODO - Generated class
 */
class MissionControl : public cSimpleModule {
protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual CommandQueue loadCommandsFromWaypointsFile(const char *fileName);
    virtual UAVNode* selectUAVNode();
};

#endif
