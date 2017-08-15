#############
### CMake ###
#############

cmake_minimum_required(VERSION 2.8.0)
project(dep_gui)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS qt_build roscpp roboy_dep)
catkin_package(CATKIN_DEPENDS roscpp roboy_dep)

###########
### Qt #### 
###########

include_directories(qcustomplot)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

set(QT_FORMS ui/mainwindow.ui)
set(QT_MOC include/dep_gui/mainwindow.h qcustomplot/qcustomplot.h)
set(QT_SOURCES src/mainwindow.cpp src/main.cpp qcustomplot/qcustomplot.cpp)

QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

###################
### Executables ###
###################

add_executable(dep_gui ${QT_SOURCES} ${QT_FORMS_HPP} ${QT_MOC_HPP} )
add_dependencies(dep_gui ${catkin_EXPORTED_TARGETS} roboy_dep_generate_messsages_cpp)
target_link_libraries(dep_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})

####################
### Installation ###
####################

install(TARGETS dep_gui
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

