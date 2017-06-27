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

#ifndef MISSIONCONTROLDATAMAP_H_
#define MISSIONCONTROLDATAMAP_H_

#include <omnetpp.h>

#include <map>

#include "GenericNode.h"
#include "ReplacementData.h"

using namespace omnetpp;

/**
 * The overall status of a node, as seen by the MissionControl.
 */
enum class NodeStatus {
    IDLE, RESERVED, PROVISIONING, MISSION, MAINTENANCE, CHARGING, DEAD
};

/**
 * A summarized view on a node needed by the MissionControl for node management.
 */
class NodeShadow {
private:
    int index;
    GenericNode* node;
    NodeStatus status = NodeStatus::IDLE;
    ReplacementData* replacementData = nullptr;
    cMessage* replacementMsg = nullptr;
public:
    NodeShadow(GenericNode* node);
    virtual ~NodeShadow();
    int getIndex() const
    {
        return index;
    }
    GenericNode* getNode() const
    {
        return node;
    }

    NodeStatus getStatus() const
    {
        return status;
    }

    bool hasReplacementData() const
    {
        return (this->replacementData != nullptr);
    }

    ReplacementData* getReplacementData() const
    {
        if (not hasReplacementData()) throw cRuntimeError("No replacementData available, this method should not be called here");
        return replacementData;
    }

    bool isStatus(NodeStatus status) const
    {
        return this->status == status;
    }

    bool isStatusIdle() const
    {
        return isStatus(NodeStatus::IDLE);
    }

    bool isStatusMission() const
    {
        return isStatus(NodeStatus::MISSION);
    }

    bool isStatusProvisioning() const
    {
        return isStatus(NodeStatus::PROVISIONING);
    }

    bool hasReplacementMsg() const
    {
        return (this->replacementMsg != nullptr);
    }

    cMessage* getReplacementMsg() const
    {
        if (not hasReplacementMsg()) throw cRuntimeError("No replacementMsg available, this method should not be called here");
        return replacementMsg;
    }

    bool compareReplacementMsg(cMessage* replMsg) const
    {
        if (not hasReplacementMsg()) throw cRuntimeError("No replacementMsg available, this method should not be called here");
        return this->replacementMsg == replMsg;
    }

    bool hasReplacingNode() const
    {
        return (this->replacementData != nullptr && this->replacementData->replacingNode != nullptr);
    }

    GenericNode* getReplacingNode() const
    {
        if (not hasReplacingNode()) throw cRuntimeError("No replacingNode available, this method should not be called here");
        return this->replacementData->replacingNode;
    }

    int getReplacingNodeIndex() const
    {
        if (not hasReplacingNode()) throw cRuntimeError("No replacingNode available, this method should not be called here");
        return this->replacementData->replacingNode->getIndex();
    }

    simtime_t getReplacementTime() const
    {
        if (not hasReplacementData()) throw cRuntimeError("No replacementData available, this method should not be called here");
        return this->replacementData->timeOfReplacement;
    }

    void setStatus(NodeStatus status);
    void setReplacementData(ReplacementData* replacementData);
    void setReplacementMsg(cMessage* replacementMsg);
    void clearReplacementMsg();
    void setReplacingNode(GenericNode* replacingNode);
};

/**
 * A map of all nodes managed by the MissionControl.
 * key: the nodeIndex of a GenericNode
 */
typedef std::map<int, NodeShadow*> NodeDataMap;

#endif /* MISSIONCONTROLDATAMAP_H_ */
