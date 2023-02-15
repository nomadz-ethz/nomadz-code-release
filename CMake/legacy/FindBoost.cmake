set(Boost_LIBRARIES
  "${CMAKE_SOURCE_DIR}/Util/boost/linux/Install/libboost_filesystem.so"
  "${CMAKE_SOURCE_DIR}/Util/boost/linux/Install/libboost_system.so"
  CACHE INTERNAL "" FORCE)
include_directories("${CMAKE_SOURCE_DIR}/Util/boost/linux/" SYSTEM) # FIXME
