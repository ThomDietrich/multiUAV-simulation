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

#ifndef ICHARGEALGORITHM_H_
#define ICHARGEALGORITHM_H_

class IChargeAlgorithm {
public:
    virtual ~IChargeAlgorithm() {};
    virtual double calculateChargeAmount(double remaining, double capacity, double seconds) = 0;
    virtual double calculateChargeTime(double remaining, double capacity, double targetPercentage) = 0;
    virtual double getFastChargePercentage(double capacity) = 0;
};

#endif /* ICHARGEALGORITHM_H_ */
