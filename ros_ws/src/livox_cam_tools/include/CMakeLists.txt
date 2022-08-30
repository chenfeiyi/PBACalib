# *****************************************
add_library(lib_bagreader SHARED BagReader.cc)
target_include_directories(lib_bagreader
PUBLIC
${GLOBAL_INCLUDE_DISR}
)
target_link_libraries(lib_bagreader
  ${catkin_LIBRARIES}
)
