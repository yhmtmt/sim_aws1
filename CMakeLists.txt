find_package(PythonLibs 3 REQUIRED)
STRING(REGEX REPLACE "^([0-9]+)\\.[0-9]+\\.[0-9]+" "\\1" python_major_vers "${PYTHONLIBS_VERSION_STRING}")
STRING(REGEX REPLACE "^[0-9]+\\.([0-9])+\\.[0-9]+" "\\1" python_minor_vers "${PYTHONLIBS_VERSION_STRING}")
STRING(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+)" "\\1" python_patch_vers "${PYTHONLIBS_VERSION_STRING}")
find_package(Boost COMPONENTS python-py${python_major_vers}${python_minor_vers} REQUIRED)

add_library(sim_aws1 SHARED f_sim_aws1.cpp c_model.cpp)
target_include_directories(sim_aws1 PUBLIC ${PROJECT_SOURCE_DIR}/include)
install(TARGETS sim_aws1 DESTINATION lib)

set(CMAKE_SHARED_MODULE_PREFIX "")
add_library(model_aws1 SHARED c_model.cpp)
target_include_directories(model_aws1 PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(model_aws1 PUBLIC ${PYTHON_INCLUDE_DIRS})
target_compile_definitions(model_aws1 PUBLIC PY_EXPORT)
target_link_libraries(model_aws1 ${Boost_LIBRARIES})

install(TARGETS model_aws1 DESTINATION lib)
file(GLOB TESTS test/*)
install(FILES ${TESTS}
  PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE
  DESTINATION ftest)
