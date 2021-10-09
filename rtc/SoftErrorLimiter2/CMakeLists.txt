add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  joint_limit_table
  )
find_package(choreonoid REQUIRED)
find_package(PkgConfig)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${hrpsys_INCLUDE_DIRS}
  )

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

rtmbuild_add_library(SoftErrorLimiter2 SHARED
  SoftErrorLimiter2.cpp
  SoftErrorLimiter2Service_impl.cpp
  beep.cpp
  )
target_link_libraries(SoftErrorLimiter2
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  HRPDataTypesSkel HRPDataTypesStub hrpsysUtil
  )
set_target_properties(SoftErrorLimiter2 PROPERTIES PREFIX "") # libSoftErrorLimiter2.so -> SoftErrorLimiter2.so
add_dependencies(SoftErrorLimiter2 RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(SoftErrorLimiter2Comp SoftErrorLimiter2Comp.cpp)
target_link_libraries(SoftErrorLimiter2Comp SoftErrorLimiter2)
