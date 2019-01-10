if (LIBDW_FOUND)
    add_definitions(-DBACKWARD_HAS_DW=1)
    set(backward_ros_forced_LIBRARIES "${backward_ros_LIBRARIES};${LIBDW_LIBRARIES}")
elseif(LIBBFD_FOUND)
    add_definitions(-DBACKWARD_HAS_BFD=1)
    set(backward_ros_forced_LIBRARIES "${backward_ros_LIBRARIES};${LIBBFD_LIBRARIES}")
else()
    set(backward_ros_forced_LIBRARIES "${backward_ros_LIBRARIES}")
endif()
if (ROSCPP_FOUND)
    add_definitions(-DBACKWARD_HAS_ROSCPP=1)
endif()
set(backward_ros_LIBRARIES "") #This is used by catkin, but we don't need it since we force it below
set(backward_ros_full_path_LIBRARIES "") #This is used by catkin, but we don't need it since we force it below

#Hack to find absolute path to libraries, won't work if library is not compiled yet
foreach(lib ${backward_ros_forced_LIBRARIES})
    if(NOT EXISTS ${lib})
        message("${lib} doesn't exist, trying to find it in ${backward_ros_PREFIX}")
        find_library(backward_ros_lib_path 
            NAMES ${lib} 
            PATHS ${backward_ros_PREFIX})
        if(NOT ${backward_ros_lib_path})
            message("${lib} not found")
        else()
            message("${backward_ros_lib_path} found")
            set(backward_ros_full_path_LIBRARIES "${backward_ros_full_path_LIBRARIES} ${backward_ros_lib_path}")
        endif()
    else()
        set(backward_ros_full_path_LIBRARIES "${backward_ros_full_path_LIBRARIES} ${lib}")
    endif()
endforeach()
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--no-as-needed ${backward_ros_full_path_LIBRARIES} -Wl,--as-needed ${CMAKE_EXE_LINKER_FLAGS}")

