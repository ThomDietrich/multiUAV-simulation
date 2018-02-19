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

#include "Battery.h"
#include <float.h>

/**
 * Create an infinite energy storage.
 * It will always be fully charged and never empty.
 * This battery should be used if energy consumption and recharging should be ignored.
 */
Battery::Battery() :
        capacity(0), remaining(0), infinite(true)
{
}

/**
 * Create a limited energy storage.
 * The battery will be fully charged in the beginning.
 * If capacity is initialized with 0, an infinite battery is created.
 *
 * @param capacity the size of the battery in mAh (e.g. 2200)
 */
Battery::Battery(float capacity) :
        capacity(capacity), remaining(capacity)
{
    this->infinite = (capacity == 0) ? true : false;
}

/**
 * Create a limited energy storage.
 *
 * @param capacity the size of the battery in mAh (e.g. 2200)
 * @param remaining the remaining energy in the battery at creation (e.g. 1500)
 */
Battery::Battery(float capacity, float remaining) :
        capacity(capacity), remaining(remaining)
{
    this->infinite = (capacity == 0) ? true : false;
    if (remaining > capacity) EV_WARN << "Battery initialized with remaining > capacity" << endl;
}

Battery::~Battery()
{
}

/**
 * Add stored energy.
 *
 * @param amount quantity of energy to add to the storage, in [mAh]
 * @return 'false' if maximum capacity was reached
 */
bool Battery::charge(float amount)
{
    if (infinite) return true;
    remaining += amount;
    if (remaining > capacity) {
        remaining = capacity;
        return false;
    }
    return true;
}

/**
 * Consume stored energy.
 *
 * @param amount quantity of energy consumed, in [mAh]
 * @return 'false' if more energy was consumed than available
 */
bool Battery::discharge(float amount)
{
    if (infinite) return true;
    remaining -= amount;
    if (remaining < 0) {
        remaining = 0;
        EV_WARN << "Battery storage exhausted over limit 0" << endl;
        throw cRuntimeError("Battery storage exhausted over limit 0");
        return false;
    }
    return true;
}

/**
 * @return the absolute capacity of the battery, in [mAh]
 */
float Battery::getCapacity()
{
    return capacity;
}

/**
 * @return difference between maximum capacity and remaining energy, in [mAh]
 */
float Battery::getMissing()
{
    if (infinite) return 0;
    return capacity - remaining;
}

/**
 * @return remaining energy, in [mAh]
 */
float Battery::getRemaining()
{
    if (infinite) return FLT_MAX;
    return remaining;
}

/**
 * @return remaining energy, in percentage
 */
int Battery::getRemainingPercentage()
{
    if (infinite) return 100;
    return 100 * remaining / capacity;
}

/**
 * @return 'true' if energy storage is empty
 */
bool Battery::isEmpty()
{
    if (infinite) return false;
    return (remaining <= 0);
}

/**
 * @return 'true' if energy storage is completely full
 */
bool Battery::isFull()
{
    if (infinite) return true;
    return (remaining >= capacity);
}
