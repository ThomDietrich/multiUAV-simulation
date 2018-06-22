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
#include "Battery.h"

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
    Battery* knownBattery = nullptr;
public:
    NodeShadow(GenericNode* node);
    virtual ~NodeShadow();
    int getNodeIndex() const
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

    const char* getStatusString() const
    {
        switch (status) {
            case NodeStatus::DEAD:
                return "DEAD";
            case NodeStatus::IDLE:
                return "IDLE";
            case NodeStatus::MAINTENANCE:
                return "MAINTENANCE";
            case NodeStatus::MISSION:
                return "MISSION";
            case NodeStatus::PROVISIONING:
                return "PROVISIONING";
            case NodeStatus::RESERVED:
                return "RESERVED";
            case NodeStatus::CHARGING:
                return "CHARGING";
            default:
                return "UNKNOWN!!!";
        }
    }

    bool hasReplacementData() const
    {
        return (this->replacementData != nullptr);
    }

    ReplacementData* getReplacementData() const
    {
        if (not hasReplacementData())  {
            std::string error_msg = std::string(node->getFullName()) + ": no replacementData available, NodeShadow::getReplacementData() should not be called.";
            throw cRuntimeError(error_msg.c_str());
        }
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

    bool isStatusReserved() const
    {
        return isStatus(NodeStatus::RESERVED);
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
        if (not hasReplacementMsg())  {
            std::string error_msg = std::string(node->getFullName()) + ": no replacementMsg available, NodeShadow::getReplacementMsg() should not be called.";
            throw cRuntimeError(error_msg.c_str());
        }
        return replacementMsg;
    }

    bool compareReplacementMsg(cMessage* replMsg) const
    {
        //if (not hasReplacementMsg()) throw cRuntimeError("No replacementMsg available, this method should not be called here");
        return hasReplacementMsg() && this->replacementMsg == replMsg;
    }

    bool hasReplacingNode() const
    {
        return (this->replacementData != nullptr && this->replacementData->replacingNode != nullptr);
    }

    GenericNode* getReplacingNode() const
    {
        if (not hasReplacingNode()) {
            std::string error_msg = std::string(node->getFullName()) + ": no replacingNode available, NodeShadow::getReplacingNode() should not be called.";
            throw cRuntimeError(error_msg.c_str());
        }
        return this->replacementData->replacingNode;
    }

    int getReplacingNodeIndex() const
    {
        if (not hasReplacingNode()) {
            std::string error_msg = std::string(node->getFullName()) + ": no replacingNode available, NodeShadow::getReplacingNodeIndex() should not be called.";
            throw cRuntimeError(error_msg.c_str());
        }
        return this->replacementData->replacingNode->getIndex();
    }

    simtime_t getReplacementTime() const
    {
        if (not hasReplacementData()) {
            std::string error_msg = std::string(node->getFullName()) + ": no replacementData available, NodeShadow::getReplacementTime() should not be called.";
            throw cRuntimeError(error_msg.c_str());
        }
        return this->replacementData->timeOfReplacement;
    }

    void clearReplacementMsg();
    void clearReplacementData();
    void clearReplacementNode();
    void setStatus(NodeStatus status);
    void setReplacementData(ReplacementData* replacementData);
    void setReplacementMsg(cMessage* replacementMsg);
    void setReplacingNode(GenericNode* replacingNode);

    Battery* getKnownBattery()
    {
        return knownBattery;
    }
    
    void setKnownBattery(Battery* knownBattery = nullptr)
    {
        if (this->knownBattery != nullptr) delete this->knownBattery;
        this->knownBattery = knownBattery;
    }
};

/**
 * A comprising map of all NodeShadow objects needed by the MissionControl for node management.
 */
class ManagedNodeShadows {
private:
    std::map<int, NodeShadow*> managedNodes;
public:
    ManagedNodeShadows();
    virtual ~ManagedNodeShadows();
    bool has(int index);
    void add(NodeShadow* nodeShadow);
    void remove(int index);
    void setStatus(int index, NodeStatus newStatus);
    void setStatus(GenericNode* node, NodeStatus newStatus);
    NodeShadow* get(int index);
    NodeShadow* get(GenericNode* node);
    NodeShadow* getFirst(NodeStatus currentStatus);
    NodeShadow* getClosest(NodeStatus currentStatus, float x, float y, float z);
    NodeShadow* getHighestCharged();
    NodeShadow* getNodeRequestingReplacement(cMessage *msg); //TODO: Replace!
    int size() const
    {
        return managedNodes.size();
    }
};

#endif /* MISSIONCONTROLDATAMAP_H_ */
