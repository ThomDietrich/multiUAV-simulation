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

#include <deque>

#include "OsgEarthScene.h"
#include "Command.h"
#include "UAVNode.h"
#include "MobileNode.h"
#include "GenericNode.h"
#include "msgs/MissionMsg_m.h"
#include "msgs/CmdCompletedMsg_m.h"
#include "msgs/ExchangeCompletedMsg_m.h"
#include "msgs/MobileNodeRequest_m.h"
#include "msgs/MobileNodeResponse_m.h"
#include "MissionControlDataMap.h"

using namespace omnetpp;

/**
 *
 */
class MissionControl : public cSimpleModule {
private:
    ManagedNodeShadows managedNodeShadows;
    std::deque<CommandQueue> missionQueue;
protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual CommandQueue loadCommandsFromWaypointsFile(const char *fileName);
    virtual void handleReplacementMessage(ReplacementData replData);
    virtual void requestChargedNodesInformation(double remainingBattery);
    virtual cGate* getOutputGateTo(cModule *cMod);
};

#endif
