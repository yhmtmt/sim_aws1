find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
STRING(REGEX REPLACE "^([0-9]+)\\.[0-9]+\\.[0-9]+" "\\1" python_major_vers "${PYTHONLIBS_VERSION_STRING}")
STRING(REGEX REPLACE "^[0-9]+\\.([0-9])+\\.[0-9]+" "\\1" python_minor_vers "${PYTHONLIBS_VERSION_STRING}")
STRING(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+)" "\\1" python_patch_vers "${PYTHONLIBS_VERSION_STRING}")
find_package(Boost COMPONENTS python-py${python_major_vers}${python_minor_vers})

include_directories(EIGEN3_INCLUDE_DIR)
include_directories(/usr/include/eigen3)
add_library(sim_aws1 SHARED f_sim_aws1.cpp)
target_include_directories(sim_aws1 PUBLIC ${PROJECT_SOURCE_DIR}/include)

message("  Boost_INCLUDE_DIR=${Boost_INCLUDE_DIR}")
message("  Boost_LIBRARY_DIR=${Boost_LIBRARY_DIR}")
message("  Boost_LIBRARIES  =${Boost_LIBRARIES}")
message("  PYTHON_LIBRARY = ${PYTHON_LIBRARY}")
message("  PYTHONLIBS_VERSION_STRING=${PYTHONLIBS_VERSION_STRING}")

add_library(pysim_aws1 SHARED c_model.cpp)
target_link_libraries(pysim_aws1 ${Boost_LIBRARIES} ${PYTHON_LIBRARY})
install(TARGETS sim_aws1 DESTINATION lib)
install(TARGETS pysim_aws1 DESTINATION lib)
file(GLOB TESTS test/*)
install(FILES ${TESTS}
  PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE
  DESTINATION ftest)
