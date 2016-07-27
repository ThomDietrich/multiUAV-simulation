//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 2015 OpenSim Ltd.
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifndef __UAVNODE_H__
#define __UAVNODE_H__

#include <vector>
#include <string>
#include <omnetpp.h>
#include "MobileNode.h"

using namespace omnetpp;

/**
 * A mobile node that follows a predefined track.
 */
class UAVNode : public MobileNode
{
    friend class CommandExecEngine;
    friend class WaypointCEE;
    friend class HoldPositionCEE;
    friend class TakeoffCEE;
  protected:
    int targetPointIndex;

  private:
    static int normalizeAngle(int angle);

  public:
    UAVNode();
    virtual ~UAVNode();

  protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return 2; }
    void readWaypointsFromFile(const char *fileName);
    virtual void move() override;
    virtual bool commandCompleted() override;
    virtual void loadNextCommand() override;
    virtual void initializeState() override;
    virtual void updateState(double stepSize) override;
    virtual double getNextStepSize() override;
};

#endif
