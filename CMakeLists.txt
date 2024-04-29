cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_recorder)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 rosneuro_data
			 rosneuro_msgs)

find_package(PkgConfig)
find_package(GTest REQUIRED)
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
					include/${PROJECT_NAME})

catkin_add_gtest(TestDummyWriter test/TestDummyWriter.cpp)
target_link_libraries(TestDummyWriter ${GTEST_BOTH_LIBRARIES} pthread gmock ${Eigen3_LIBRARIES})
target_link_libraries(TestDummyWriter ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(TestFactoryWriter test/TestFactoryWriter.cpp)
target_link_libraries(TestFactoryWriter ${GTEST_BOTH_LIBRARIES} pthread gmock ${Eigen3_LIBRARIES})
target_link_libraries(TestFactoryWriter ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(TestXDFWriter test/TestXdfWriter.cpp)
target_link_libraries(TestXDFWriter ${GTEST_BOTH_LIBRARIES} pthread gmock ${Eigen3_LIBRARIES})
target_link_libraries(TestXDFWriter ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(TestRecorder test/TestRecorder.cpp)
target_link_libraries(TestRecorder ${GTEST_BOTH_LIBRARIES} pthread gmock ${Eigen3_LIBRARIES})
target_link_libraries(TestRecorder ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(TestWriter test/TestWriter.cpp)
target_link_libraries(TestWriter ${GTEST_BOTH_LIBRARIES} pthread gmock ${Eigen3_LIBRARIES})
target_link_libraries(TestWriter ${PROJECT_NAME} ${catkin_LIBRARIES})


include_directories(${GTEST_INCLUDE_DIRS} gmock pthread include include/${PROJECT_NAME}/)

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

add_executable(recorder example/RunRecorder.cpp)
target_link_libraries(recorder rosneuro_recorder)


add_executable(test_writerfactory example/RunWriterFactory.cpp)
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


