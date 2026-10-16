cmake_minimum_required(VERSION 3.8)
project(gazebo_actors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Gazebo Sim System Plugin for Actor-Collider Tight Coupling (Primary Generic Target)
add_library(GazeboActorFollower SHARED
  src/GazeboActorFollower.cc
)
target_link_libraries(GazeboActorFollower
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# Backwards Compatibility Target
add_library(VineyardActorFollower SHARED
  src/VineyardActorFollower.cc
)
target_link_libraries(VineyardActorFollower
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS
  GazeboActorFollower
  VineyardActorFollower
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts / nodes
install(PROGRAMS
  scripts/actor_manager
  scripts/actor_teleop
  scripts/actor_tracker
  scripts/human_teleop
  scripts/human_worker_tracker
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories (launch, models, worlds, web, config)
install(DIRECTORY
  launch
  models
  worlds
  web
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/environment/gazebo_actors.sh.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/environment/gazebo_actors.dsv.in"
)

ament_package()
