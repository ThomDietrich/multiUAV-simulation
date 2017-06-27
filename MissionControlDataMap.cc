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

#include "MissionControlDataMap.h"

NodeShadow::NodeShadow(GenericNode* node)
{
    this->node = node;
    this->index = node->getIndex();
}

NodeShadow::~NodeShadow()
{
    // TODO Auto-generated destructor stub
}

void NodeShadow::setReplacementData(ReplacementData* replacementData)
{
    this->replacementData = replacementData;
}

void NodeShadow::setReplacementMsg(cMessage* replacementMsg)
{
    this->replacementMsg = replacementMsg;
}

void NodeShadow::setStatus(NodeStatus status)
{
    this->status = status;
}

void NodeShadow::setReplacingNode(GenericNode* replacingNode)
{
    if (not hasReplacementData()) throw cRuntimeError("No replacementData available, this method should not be called here");
    this->replacementData->replacingNode = replacingNode;
}
