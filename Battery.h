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

#ifndef BATTERY_H_
#define BATTERY_H_

#include <omnetpp.h>

using namespace omnetpp;

class Battery {
protected:
    /** the size of the battery in mAh (e.g. 2200) */
    float capacity;

    /** the remaining energy in the battery at creation (e.g. 1500) */
    float remaining;

    /** if 'true', the battery is always full and never empty */
    bool infinite;

    /** Amount of energy consumed after battery remaining runs out (lower than 0mAh remaining). */
    float overdraw;

public:
    Battery();
    Battery(float capacity);
    Battery(float capacity, float remaining);
    virtual ~Battery();
    bool charge(float amount);
    bool discharge(float amount);
    float getCapacity() const;
    float getMissing() const;
    float getRemaining() const;
    int getRemainingPercentage() const;
    bool isEmpty() const;
    bool isFull() const;
    float getAndResetOverdraw();
};

#endif /* BATTERY_H_ */
