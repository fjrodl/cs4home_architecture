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


#include "ament_index_cpp/get_package_share_directory.hpp"

#include "cs4home_core/Flow.hpp"
#include "cs4home_core/CognitiveModule.hpp"
#include "gtest/gtest.h"


TEST(flow_test, flow_creation)
{
  Flow flow1({"A", "B", "C", "D"});
  Flow flow2({"A", "B", "C", "E"});

  ASSERT_EQ(flow1.get_nodes(), std::vector<std::string>({"A", "B", "C", "D"}));
  ASSERT_EQ(flow2.get_nodes(), std::vector<std::string>({"A", "B", "C", "E"}));
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
