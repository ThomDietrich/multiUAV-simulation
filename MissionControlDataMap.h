/*
 * MissionControlDataMap.h
 *
 *  Created on: 01.06.2017
 *      Author: th
 */

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
typedef struct {
    GenericNode* node;
    NodeStatus status;
    ReplacementData* replacementData = nullptr;
    cMessage* replacementMsg = nullptr;
} NodeData;

/**
 * A map of all nodes managed by the MissionControl.
 * key: the nodeIndex of a GenericNode
 */
typedef std::map<int, NodeData*> NodeDataMap;

#endif /* MISSIONCONTROLDATAMAP_H_ */
