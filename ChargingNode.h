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

#ifndef CHARGINGNODE_H_
#define CHARGINGNODE_H_

#include <omnetpp.h>
#include "MobileNode.h"

using namespace omnetpp;

class ChargingNode : public GenericNode {
public:
    ChargingNode();
    virtual ~ChargingNode();
    virtual void updateState() override;
    virtual void updateState(double stepSize) override;
    virtual bool commandCompleted() override;
    virtual void loadNextCommand() override;
    virtual void initializeState() override;
    virtual double nextNeededUpdate() override;
};

#endif /* CHARGINGNODE_H_ */

