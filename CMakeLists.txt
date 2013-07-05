cmake_minimum_required (VERSION 2.8) 
project (osm_reader) 

option(WBOOST	"Enable Boost graph support"		OFF)
option(WGDAL	"Enable GDAL shapefile support"		OFF)

set(deps_dir c:/opt/misc)

find_package(ZLIB)
if (ZLIB_LIBRARY STREQUAL ZLIB_LIBRARY-NOTFOUND)
	FIND_LIBRARY(ZLIB_LIBRARY 
	  NAMES zdll
	  PATH_SUFFIXES lib64 lib
	  PATHS
		${deps_dir}/Win32/lib/zlib
	)	
	message(STATUS "ZLIB_LIBRARY: " ${ZLIB_LIBRARY})
endif()
if (ZLIB_INCLUDE_DIR STREQUAL ZLIB_INCLUDE_DIR-NOTFOUND)
		FIND_PATH(ZLIB_INCLUDE_DIR zlib.h
		PATH_SUFFIXES
			 include/gdal
			 include/GDAL
			 include
		  PATHS
			${deps_dir}/include/zlib
	)		
	message(STATUS "ZLIB_INCLUDE_DIR: " ${ZLIB_INCLUDE_DIR})
endif()
find_package(PROTOBUF)
if (PROTOBUF_LIBRARY STREQUAL PROTOBUF_LIBRARY-NOTFOUND)
	FIND_LIBRARY(PROTOBUF_LIBRARY 
	  NAMES protobuf libprotobuf
	  PATH_SUFFIXES lib64 lib32 lib
	  PATHS
		${deps_dir}/Win32/lib/protobuf
	)	
	message(STATUS "PROTOBUF_LIBRARY: " ${PROTOBUF_LIBRARY})
endif()



include_directories(${ZLIB_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/deps)
include_directories(${deps_dir}/include)
add_executable(osmExample src/example.cpp)
target_link_libraries(osmExample ${ZLIB_LIBRARY})
add_library(osmpbf osmpbf/osmformat.pb.cc osmpbf/fileformat.pb.cc)
target_link_libraries(osmpbf ${PROTOBUF_LIBRARY})
target_link_libraries(osmExample osmpbf)
if (MSVC)
	# find_library(Ws2_32 Ws2_32)
	target_link_libraries(osmExample Ws2_32)
endif()


if (WBOOST)
	add_definitions(-DWBOOST)
	find_package(Boost COMPONENTS graph REQUIRED)
	include_directories(${Boost_INCLUDE_DIR})
	link_directories(${Boost_LIBRARY_DIRS})
endif()

if (WGDAL)
	add_definitions(-DWGDAL)
	find_package(GDAL)
	if (${GDAL_INCLUDE_DIR} STREQUAL GDAL_INCLUDE_DIR-NOTFOUND AND MSVC)
		FIND_PATH(GDAL_INCLUDE_DIR gdal.h
		PATH_SUFFIXES
			 include/gdal
			 include/GDAL
			 include
		  PATHS
			"C:/Program Files (x86)/GDAl"
	)	
	endif()
	if (${GDAL_LIBRARY} STREQUAL GDAL_LIBRARY-NOTFOUND AND MSVC)
		FIND_LIBRARY(GDAL_LIBRARY 
		  NAMES gdal gdal_i gdal1.5.0 gdal1.4.0 gdal1.3.2 GDAL
		  PATH_SUFFIXES lib64 lib
		  PATHS
			${deps_dir}/Win32/lib/gdal)	  
	endif()
	message(STATUS "GDAL_INCLUDE_DIR: " ${GDAL_INCLUDE_DIR})
	message(STATUS "GDAL_LIBRARY: " ${GDAL_LIBRARY})	
	set(GDAL_LIBRARIES ${GDAL_LIBRARY})
	set(GDAL_INCLUDE_DIRS ${GDAL_INCLUDE_DIR})
	include_directories(${GDAL_INCLUDE_DIR})
	target_link_libraries(osmExample ${GDAL_LIBRARY})
endif()




