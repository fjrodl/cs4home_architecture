// Copyright 2024 Intelligent Robotics Lab
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

#include "cs4home_core/Core.hpp"

namespace cs4home_core
{

/**
 * @brief Constructs a Core object associated with a given lifecycle node.
 * @param parent Shared pointer to the lifecycle node managing this Core instance.
 */
Core::Core(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
: parent_(parent)
{
}

/**
 * @brief Configures the Core component.
 * @return True if configuration is successful.
 */
bool
Core::configure()
{
  return true;
}

}  // namespace cs4home_core
