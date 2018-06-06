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

#include "ChargeAlgorithmCCCVCurrent.h"

/**
 * Constructor, calculates the eulerConstant, which provides the system precision
 */
ChargeAlgorithmCCCVCurrent::ChargeAlgorithmCCCVCurrent(double current, int cccvShiftPercentage)
{
    this->current = current;
    this->cccvShiftPercentage = cccvShiftPercentage;
}

/**
 * @return chargeAmount in mAh
 */
double ChargeAlgorithmCCCVCurrent::calculateChargeAmount(double remaining, double capacity, double seconds)
{
    bool inCcStage = remaining < capacity / 100 * cccvShiftPercentage;
    double stageCurrent = 0;

    if (inCcStage) {
        stageCurrent = current;
    }
    else {
        double ccStageCharge = capacity / 100 * cccvShiftPercentage;
        double cvStageCharge = capacity - ccStageCharge;
        double cvStageProgressSeconds = getCVTime(capacity) / cvStageCharge * (remaining - ccStageCharge);

        stageCurrent = current - current / getCVTime(capacity) * (cvStageProgressSeconds + seconds / 2);
    }
    double amount = stageCurrent * 1000 * seconds / 3600;
    return amount;
}

/**
 * @return chargeTime in seconds
 */
double ChargeAlgorithmCCCVCurrent::calculateChargeTime(double remaining, double capacity, double targetPercentage)
{
    double remainingPercentage = 100 / capacity * remaining;
    double timeInCC = 0;
    double timeInCV = 0;

    if (targetPercentage <= remainingPercentage) return 0;

    if (remainingPercentage < cccvShiftPercentage) {
        if (targetPercentage <= cccvShiftPercentage) {
            timeInCC = getCCTime(capacity) / cccvShiftPercentage * (targetPercentage - remainingPercentage);
        }
        else {
            timeInCC = getCCTime(capacity) / cccvShiftPercentage * (cccvShiftPercentage - remainingPercentage);
            //TODO not correct
            timeInCV = getCVTime(capacity) / (100 - cccvShiftPercentage) * (targetPercentage - cccvShiftPercentage);
        }
    }
    else {
        //TODO not correct
        timeInCV = getCVTime(capacity) / (100 - cccvShiftPercentage) * (targetPercentage - remainingPercentage);
    }

    return timeInCC + timeInCV;
}

double ChargeAlgorithmCCCVCurrent::getFastChargePercentage(double maxCapacity)
{
    return cccvShiftPercentage;
}

double ChargeAlgorithmCCCVCurrent::getCCTime(double capacity)
{
    double ccStageCharge = capacity / 100 * cccvShiftPercentage;
    double seconds = (ccStageCharge / (current * 1000)) * 3600;
    return seconds;
}

double ChargeAlgorithmCCCVCurrent::getCVTime(double capacity)
{
    double cvStageCharge = capacity / 100 * (100 - cccvShiftPercentage);
    double seconds = (cvStageCharge / (current / 2 * 1000)) * 3600;
    return seconds;
}
