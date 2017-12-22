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

class ChargeAlgorithmCCCV : public IChargeAlgorithm{
public:
    ChargeAlgorithmCCCV(double linearGradient, double expGradient, double nonLinearPhaseStartPercentage, double nonLinearPhaseLimitPercentage);
    double calculateChargeAmount(double remaining, double capacity, double seconds);
    double calculateChargeTime(double remaining, double capacity, double targetPercentage);

    double getLinearGradient() const
    {
        return linearGradient;
    }
    
    void setLinearGradient(double linearGradient)
    {
        this->linearGradient = linearGradient;
    }
    
    double getExpGradient() const
    {
        return expGradient;
    }
    
    void setNonLinearExpGradient(double expGradient)
    {
        this->expGradient = expGradient;
    }
    
    double getNonLinearPhaseLimitPercentage() const
    {
        return nonLinearPhaseLimitPercentage;
    }
    
    void setNonLinearPhaseLimitPercentage(double nonLinearPhaseLimitPercentage)
    {
        this->nonLinearPhaseLimitPercentage = nonLinearPhaseLimitPercentage;
    }
    
    double getNonLinearPhaseStartPercentage() const
    {
        return nonLinearPhaseStartPercentage;
    }
    
    void setNonLinearPhaseStartPercentage(double nonLinearPhaseStartPercentage)
    {
        this->nonLinearPhaseStartPercentage = nonLinearPhaseStartPercentage;
    }
    
protected:
    /**
     * ToDo: Include the batteryCapacity into the generation of the used expGradient.
     * Intended to generate a charging speed independent from the battery capacity.
     *
     * Close to real values for a battery with 5200mAh capacity are:
     * linearGradient = 0.4;
     * expGradient = 0.0006;
     * nonLinearPhaseStartPercentage = 90;
     * nonLinearPhaseLimitPercentage = 101;
     */
    double linearGradient;
    double expGradient;
    double nonLinearPhaseStartPercentage;
    double nonLinearPhaseLimitPercentage;
    double eulerConstant;
    double calculateLinearChargeAmount(double remaining, double capacity, double seconds);
    double calculateNonLinearChargeAmount(double remaining, double capacity, double seconds);
    double calculateLinearSeconds(double remaining, double capacity, double targetPercentage);
    double calculateNonLinearSeconds(double remaining, double capacity, double targetPercentage);
    double calculateNonLinearSecondsStartToTarget(double capacity, double targetPercentage);
    double calculateStart(double capacity);
    double calculateLimit(double capacity);
};

#endif /* CHARGEALGORITHMCCCV_H_ */
