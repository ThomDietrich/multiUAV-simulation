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

#ifndef REPLACEMENTDATA_H_
#define REPLACEMENTDATA_H_

#include <omnetpp.h>

//#include "GenericNode.h"
class GenericNode;

using namespace omnetpp;

/**
 * Contains data needed for an exchange of a node.
 * In one scenario this data is sent from a node to the mission control to ask for a replacement at a given time and position
 */
class ReplacementData {
public:
    ReplacementData();
    virtual ~ReplacementData();

    /**
     * The node to be replaced by another
     */
    GenericNode* nodeToReplace;

    /**
     * The node replacing the other one. This field is optional.
     */
    GenericNode* replacingNode = nullptr;

    /**
     * When the replacement should take place.
     * Might be calculated by the node based on it's prediction for future maneuvers.
     */
    simtime_t timeOfReplacement;

    /**
     * Where the replacement should take place.
     * Might be calculated by the node based on it's prediction for future maneuvers.
     */
    double x, y, z;
//private:
};

#endif /* REPLACEMENTDATA_H_ */
