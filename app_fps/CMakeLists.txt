add_compile_options(-O2 -g -W -Wall -Wextra -rdynamic)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common features filters io segmentation surface console)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ../pubsub_ext
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ../pubsub_ext
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_library(pcpp_native SHARED
 pcpp_native.cc
)

target_link_libraries(pcpp_native
  ${PCL_LIBRARIES}
  pubsub_ext
)

add_library(pcpp_native_direct SHARED
 pcpp_native_direct.cc
)

target_link_libraries(pcpp_native_direct
  ${PCL_LIBRARIES}
  pubsub_ext
)

add_executable(publisher publisher.cc)
target_link_libraries(publisher
  ${PCL_LIBRARIES}
  pubsub_ext
  ${PROJECT_NAME}
  gflags
)

add_executable(run_mapping_fps run_mapping_fps.cc)
target_link_libraries(run_mapping_fps
  ${PCL_LIBRARIES}
  pubsub_ext
  ${PROJECT_NAME}
  gflags
)
