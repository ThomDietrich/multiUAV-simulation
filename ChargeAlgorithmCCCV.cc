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
ChargeAlgorithmCCCV::ChargeAlgorithmCCCV(double linearGradient, double current)
{
    this->eulerConstant = std::exp(1.0);
    this->linearGradient = linearGradient;
    this->current = current / linearGradient;
}

/**
 * @return chargeAmount in mAh
 */
double ChargeAlgorithmCCCV::calculateChargeAmount(double remaining, double capacity, double seconds)
{
    double secondsLin = fmax(0.0, calculateLinearSeconds(remaining, capacity, 100.0));
    double amountLin = fmax(0.0, calculateLinearChargeAmount(remaining, capacity, fmin(seconds, secondsLin)));
    if (secondsLin > seconds) {
        return amountLin;
    }
    double secondsNonLin = seconds - secondsLin + fmax(0.0, calculateNonLinearSeconds(remaining + amountLin, capacity));
    return calculateNonLinearChargeAmount(capacity, secondsNonLin) - remaining - amountLin;
}

double ChargeAlgorithmCCCV::getFastChargePercentage(double maxCapacity)
{
    return (-0.0291 * current + 1) * 100;
}

/**
 * @return chargeTime in seconds
 */
double ChargeAlgorithmCCCV::calculateChargeTime(double remaining, double capacity, double targetPercentage)
{
    double secondsLin = fmax(0.0, calculateLinearSeconds(remaining, capacity, targetPercentage));
    if (remaining + calculateLinearChargeAmount(remaining, capacity, secondsLin) >= capacity * targetPercentage / 100) {
        return secondsLin;
    }
    return secondsLin + calculateNonLinearSeconds(capacity * targetPercentage / 100, capacity, remaining);
}

/*
 * @return chargeAmount for linear charge process  (phase 1) in mAh
 */
double ChargeAlgorithmCCCV::calculateLinearChargeAmount(double remaining, double capacity, double seconds)
{
    double missing = fmax(0.0, calculateNonLinearStart(capacity) - remaining);
    return fmin(missing, seconds * current * linearGradient);
}

/*
 * @return chargeTime for linear charge process (phase 1) in seconds
 */
double ChargeAlgorithmCCCV::calculateLinearSeconds(double remaining, double capacity, double targetPercentage)
{
    return (fmin(capacity * targetPercentage / 100, calculateNonLinearStart(capacity)) - remaining) / (current * linearGradient);
}

/*
 * higher remaining -> lower chargeAmount
 * @return chargeAmount whole chargeprocess in phase 2. only applyable when at least a part of the process happens in phase 2!
 */
double ChargeAlgorithmCCCV::calculateNonLinearChargeAmount(double capacity, double seconds)
{
    return capacity * (a + 1) - (capacity * (a + 1) - calculateNonLinearStart(capacity)) * pow(eulerConstant, ((-calculateNonLinearGradient(current, capacity, a)) * seconds));
}

/*
 * @return chargeTime needed for target in non-linear charge process (phase 2) in seconds
 */
double ChargeAlgorithmCCCV::calculateNonLinearSeconds(double targetAmount, double capacity, double remaining)
{
    return calculateNonLinearSeconds(targetAmount, capacity) - calculateNonLinearSeconds(remaining, capacity);
}
/*
 * @return chargeTime needed for target in non-linear charge process (phase 2) in seconds
 */
double ChargeAlgorithmCCCV::calculateNonLinearSeconds(double targetAmount, double capacity)
{
    double zaehler = capacity * (a + 1) - targetAmount;
    double nenner = capacity * (a + 1) - calculateNonLinearStart(capacity);
    return fmax(0.0, log(zaehler / nenner) / (-calculateNonLinearGradient(current, capacity, a)));
}

/*
 * @return start of non-linear part (phase 2) in mAh
 */
double ChargeAlgorithmCCCV::calculateNonLinearStart(double capacity)
{
    return capacity * getFastChargePercentage(capacity) / 100;
}

/*
 * @return non-linear gradient (phase 2) (k)
 */
double ChargeAlgorithmCCCV::calculateNonLinearGradient(double current, double capacity, double a)
{
    return (linearGradient * current) / (capacity * (a + 0.0291 * current));
}

