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

#include <Battery.h>
#include <float.h>

Battery::Battery()
{
    this->capacity = 0;
    this->remaining = 0;
    this->infinit = true;
}

Battery::Battery(double capacity)
{
    this->capacity = capacity;
    this->remaining = capacity;
    this->infinit = false;
}

Battery::Battery(double capacity, double remaining)
{
    this->capacity = capacity;
    this->remaining = remaining;
    this->infinit = false;
}

Battery::~Battery()
{
}

bool Battery::charge(double amount)
{
    if (infinit) return true;
    remaining += amount;
    if (remaining > capacity) {
        remaining = capacity;
        return false;
    }
    return true;
}

bool Battery::discharge(double amount)
{
    if (infinit) return true;
    remaining -= amount;
    if (remaining < 0) {
        remaining = 0;
        return false;
    }
    return true;
}

double Battery::getRemaining()
{
    if (infinit) return DBL_MAX;
    return remaining;
}

int Battery::getRemainingPercentage()
{
    if (infinit) return 100;
    return 100 * remaining / capacity;
}

bool Battery::isEmpty()
{
    if (infinit) return false;
    return (remaining <= 0);
}

bool Battery::isFull()
{
    if (infinit) return true;
    return (remaining >= capacity);
}
