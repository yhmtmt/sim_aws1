find_package(PythonLibs 3.6 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS python-py36 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)
include_directories(/usr/include/eigen3)
add_library(sim_aws1 SHARED f_sim_aws1.cpp)
target_include_directories(sim_aws1 PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_library(pysim_aws1 SHARED c_model.cpp)
target_link_libraries(pysim_aws1 boost_python-py36 python3.6m)
install(TARGETS sim_aws1 DESTINATION lib)
install(TARGETS pysim_aws1 DESTINATION lib)

file(GLOB TESTS test/*)
install(FILES ${TESTS}
  PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE
  DESTINATION ftest)
