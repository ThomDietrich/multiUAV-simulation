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

#include "ChargeAlgorithmCCCV.h"

/**
 * Constructor, calculates the eulerConstant, which provides the system precision
 */
ChargeAlgorithmCCCV::ChargeAlgorithmCCCV(double linearGradient, double expGradient, double nonLinearPhaseStartPercentage, double nonLinearPhaseLimitPercentage)
{
    this->linearGradient = linearGradient;
    this->expGradient = expGradient;
    this->nonLinearPhaseStartPercentage = nonLinearPhaseStartPercentage;
    this->nonLinearPhaseLimitPercentage = nonLinearPhaseLimitPercentage;
    this->eulerConstant = std::exp(1.0);
}

/**
 * @return chargeAmount in mAh
 */
double ChargeAlgorithmCCCV::calculateChargeAmount(double remaining, double capacity, double seconds)
{
    double secondsLin = fmax(0.0, fmin(seconds, calculateLinearSeconds(remaining, capacity, 100.0)));
    double chargeAmountLin = calculateLinearChargeAmount(remaining, capacity, secondsLin);
    double secondsNonLin = seconds - secondsLin;
    double chargeAmountNonLin = calculateNonLinearChargeAmount(remaining + chargeAmountLin, capacity, secondsNonLin);
    return chargeAmountLin + chargeAmountNonLin;
}

/**
 * @return chargeTime in seconds
 */
double ChargeAlgorithmCCCV::calculateChargeTime(double remaining, double capacity, double targetPercentage)
{
    return calculateLinearSeconds(remaining, capacity, targetPercentage) + calculateNonLinearSeconds(remaining, capacity, (capacity * targetPercentage / 100));
}

/*
 * @return chargeAmount for linear charge process  (phase 1)
 */
double ChargeAlgorithmCCCV::calculateLinearChargeAmount(double remaining, double capacity, double seconds)
{
    double missing = fmax(0.0, calculateStart(capacity) - remaining);
    return fmin(missing, seconds * linearGradient);
}

/*
 * @return chargeTime for linear charge process (phase 1) in seconds
 */
double ChargeAlgorithmCCCV::calculateLinearSeconds(double remaining, double capacity, double targetPercentage)
{
    return (fmin(capacity * targetPercentage / 100, calculateStart(capacity)) - remaining) / linearGradient;
}

/*
 * Use function with limited growth to calculate the charge amount for phase 2.
 * higher remaining -> lower chargeAmount
 * @return chargeAmount for non-linear charge process (phase 2) in seconds
 */
double ChargeAlgorithmCCCV::calculateNonLinearChargeAmount(double remaining, double capacity, double seconds)
{
    double start = calculateStart(capacity);
    double limit = calculateLimit(capacity);
    return fmin(capacity - remaining,
            (limit - ((limit - start) * pow(eulerConstant, (-expGradient * (seconds + calculateNonLinearSecondsStartToTarget(capacity, remaining))))))
            - (limit - ((limit - start) * pow(eulerConstant, (-expGradient * (calculateNonLinearSecondsStartToTarget(capacity, remaining)))))));
}

/*
 * @return chargeTime needed for target in non-linear charge process (phase 2) in seconds
 */
double ChargeAlgorithmCCCV::calculateNonLinearSeconds(double remaining, double capacity, double targetAmount)
{
    return calculateNonLinearSecondsStartToTarget(capacity, targetAmount) - calculateNonLinearSecondsStartToTarget(capacity, remaining);
}

/*
 * @return chargeTime from phase 2 start to target in seconds
 */
double ChargeAlgorithmCCCV::calculateNonLinearSecondsStartToTarget(double capacity, double targetAmount)
{
    double start = calculateStart(capacity);
    double limit = calculateLimit(capacity);
    return fmax(0.0, std::log((limit - targetAmount) / (limit - start)) / (-expGradient));
}

/*
 * @return start of non-linear part (phase 2) in mAh
 */
double ChargeAlgorithmCCCV::calculateStart(double capacity)
{
    return capacity * nonLinearPhaseStartPercentage / 100;
}

/*
 * @return limit for non-linear part (phase 2) in mAh
 * When this value is equal to the capacity, the capacity will never be reached.
 */
double ChargeAlgorithmCCCV::calculateLimit(double capacity)
{
    return capacity * nonLinearPhaseLimitPercentage / 100;
}
