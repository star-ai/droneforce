
# Include ROS dirs
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Compile ROS node
add_executable(SimROSClient SimROSClient.cpp)
target_link_libraries(SimROSClient ${OpenCV_LIBS} FlightGogglesClientLib pthread ${catkin_LIBRARIES})