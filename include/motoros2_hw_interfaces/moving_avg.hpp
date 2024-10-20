// Copyright (c) 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOTOROS2_HW_INTERFACES__MOVING_AVG_HPP_
#define MOTOROS2_HW_INTERFACES__MOVING_AVG_HPP_

namespace motoros2_hw_interfaces
{

class MovingAvg
{
public:
  MovingAvg() {}
  explicit MovingAvg(double smoothing_factor) : smoothing_factor_(smoothing_factor) {}
  void update(double new_value)
  {
    if (is_first_update_)
    {
      moving_avg_ = new_value;
      is_first_update_ = false;
    }
    else
    {
      moving_avg_ = (1 - smoothing_factor_) * moving_avg_ + smoothing_factor_ * new_value;
    }
  }
  double get_moving_avg() const { return moving_avg_; }
  void set_smoothing_factor(double smoothing_factor) { smoothing_factor_ = smoothing_factor; }

private:
  double smoothing_factor_{0.1};  // Smoothing factor alpha (0 < alpha < 1) for EMA
  double moving_avg_;
  bool is_first_update_{true};
};

}  // namespace motoros2_hw_interfaces

#endif  // MOTOROS2_HW_INTERFACES__MOVING_AVG_HPP_
