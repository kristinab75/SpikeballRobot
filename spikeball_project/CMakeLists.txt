# - SAI2-URDF
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/spikeball_project)
ADD_EXECUTABLE (simviz_sb simviz_sb.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_posori simviz_posori.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_sb controller_sb.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_sb_trial controller_sb_trial.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (controller_sb_posori controller_sb_posori.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz_sb ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_posori ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_sb ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_sb_trial ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (controller_sb_posori ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY world_posori.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_spikeball.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY ball.urdf DESTINATION ${APP_RESOURCE_DIR})
