cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_recorder)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 rosneuro_data
			 rosneuro_msgs)

find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)
find_library(XDFIO_LIBRARIES xdffileio)
find_path(XDFIO_INCLUDE_DIRS xdfio.h)


catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_data
	rosneuro_msgs

  DEPENDS
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
					${XDFIO_INCLUDE_DIRS}
					include
)

add_library(${PROJECT_NAME}
	src/Writer.cpp
	src/XDFWriter.cpp
	src/DummyWriter.cpp
	src/FactoryWriter.cpp
	src/Recorder.cpp
)

target_link_libraries(${PROJECT_NAME} 
					  ${catkin_LIBRARIES} 
					  ${XDFIO_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
)

#################
## Executables ##
#################

add_executable(recorder src/recorder.cpp)
target_link_libraries(recorder rosneuro_recorder)


add_executable(test_writerfactory src/test_writerfactory.cpp)
target_link_libraries(test_writerfactory rosneuro_recorder)

#################
##   Install   ##
#################
install(TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )

install(TARGETS recorder
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


