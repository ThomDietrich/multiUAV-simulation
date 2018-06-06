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

#ifndef CHARGEALGORITHMCCCVCURRENT_H_
#define CHARGEALGORITHMCCCVCURRENT_H_

#include <iostream>
#include <cmath>
#include "IChargeAlgorithm.h"

class ChargeAlgorithmCCCVCurrent : public IChargeAlgorithm {
public:
    ChargeAlgorithmCCCVCurrent(double current, int cccvShiftPercentage);
    double calculateChargeAmount(double remaining, double capacity, double seconds) override;
    double calculateChargeTime(double remaining, double capacity, double targetPercentage) override;
    double getFastChargePercentage(double maxCapacity) override;

protected:
    double current;
    int cccvShiftPercentage = 80;

    double getCCTime(double capacity);
    double getCVTime(double capacity);
};

#endif /* CHARGEALGORITHMCCCVCURRENT_H_ */
