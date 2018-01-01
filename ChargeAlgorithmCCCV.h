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

#ifndef CHARGEALGORITHMCCCV_H_
#define CHARGEALGORITHMCCCV_H_

#include <iostream>
#include <cmath>
#include "IChargeAlgorithm.h"

class ChargeAlgorithmCCCV : public IChargeAlgorithm {
public:
    ChargeAlgorithmCCCV(double linearGradient, double current);
    double calculateChargeAmount(double remaining, double capacity, double seconds);
    double calculateChargeTime(double remaining, double capacity, double targetPercentage);
    double getFastChargePercentage(double maxCapacity);

    double getA() const
    {
        return a;
    }

    void setA(double a = 0.0013)
    {
        this->a = a;
    }

    double getLinearGradient() const
    {
        return linearGradient;
    }
    
    void setLinearGradient(double linearGradient)
    {
        this->linearGradient = linearGradient;
    }
    
    double getCurrent() const
    {
        return current;
    }
    
    void setCurrent(double current = 4)
    {
        this->current = current;
    }

protected:
    double eulerConstant;
    double a = 0.0013;
    double linearGradient = 0.2754;
    double current = 4;

    double calculateLinearChargeAmount(double remaining, double capacity, double seconds);
    double calculateNonLinearChargeAmount(double capacity, double seconds);
    double calculateLinearSeconds(double remaining, double capacity, double targetPercentage);
    double calculateNonLinearSeconds(double remaining, double capacity, double targetPercentage);
    double calculateNonLinearSeconds(double remaining, double capacity);
    double calculateNonLinearStart(double capacity);
    double calculateNonLinearGradient(double current, double capacity, double a);
};

#endif /* CHARGEALGORITHMCCCV_H_ */
