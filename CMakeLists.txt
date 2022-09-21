cmake_minimum_required(VERSION 3.10)

project(SDCLExp VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories("./SDCL")
include_directories("./utils")

find_package(ompl)

if(NOT OMPL_FOUND)
    message(FATAL_ERROR "ompl library not found")
endif()

find_package(Boost 1.54 QUIET REQUIRED COMPONENTS serialization filesystem system program_options)
include_directories(${Boost_INCLUDE_DIR})
add_definitions(-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)

# pkg-config setup
find_package(PkgConfig)

# Search for the pkg-config packages
pkg_search_module(SDL2 REQUIRED sdl2)
pkg_search_module(GL REQUIRED gl)
pkg_search_module(AMINO REQUIRED amino)
pkg_search_module(AMINO_GL REQUIRED amino-gl)

# ------------------------------ environment --------------------------------------------------
add_library(
    env
    utils/environment.h
    utils/src/environment2d.cpp
    utils/src/HighDofEnvironment.cpp
    utils/src/HyperBallEnvironment.cpp
    utils/src/MultiRobotEnv.cpp
    utils/src/PosScaleEnv.cpp
    )

# opencv
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(env PRIVATE ${OpenCV_LIBS})

# amino
target_include_directories(env PUBLIC ${AMINO_INCLUDE_DIRS})
target_include_directories(env PUBLIC ${SDL2_INCLUDE_DIRS})

target_link_libraries(env PRIVATE "${AMINO_GL_LIBRARIES}")
# target_link_libraries(env PRIVATE "${AMINO_COLLISION_LIBRARIES}")
target_link_libraries(env PRIVATE "amino-collision")
target_link_libraries(env PRIVATE "${AMINO_LIBRARIES}")

target_include_directories(env PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/env")


# -------------------------------------- scenes -----------------------------------------------
add_library(
    scenes
    utils/scenes.h
    utils/src/scenes.cpp
    )
target_link_libraries(scenes PRIVATE env)
target_include_directories(scenes PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/scenes")



# -------------------------------------- parser -----------------------------------------------
add_library(
    parser
    utils/parser.h
    utils/src/parser.cpp
    )

target_include_directories(parser PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/parser")

# -------------------------------------- SDCL -------------------------------------------------

message(STATUS "Finding LZ4 libraries")
find_library(LZ4_LIBRARIES NAMES liblz4.so HINTS "lz4/lib")
if(LZ4_LIBRARIES)
message(STATUS "Found: ${LZ4_LIBRARIES}")
else()
message(STATUS "Not found: ${LZ4_LIBRARIES}")
message(FATAL_ERROR "Cannot find required LZ4 libraries")
endif()


add_library(
  SDCL
  SDCL/SDCL.h
  SDCL/SDCL.cpp
)

target_link_libraries(SDCL PRIVATE ${Boost_LIBRARIES})
target_link_libraries(SDCL PRIVATE ${OMPL_LIBRARIES})
target_link_libraries(SDCL PRIVATE ${LZ4_LIBRARIES})
target_link_libraries(SDCL PRIVATE "nlopt")
target_link_libraries(SDCL PRIVATE "thundersvm")

target_include_directories(SDCL PRIVATE "/usr/local/include")
target_include_directories(SDCL PRIVATE "${OMPL_INCLUDE_DIRS}")

target_include_directories(SDCL PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/SDCL")

# for thread
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)


add_compile_options(-g)
add_compile_options(-frounding-math)


# --------------------------------------------------------------------------------
add_executable(RunSDCL runSDCL.cpp)

target_link_libraries(RunSDCL PRIVATE env)
target_link_libraries(RunSDCL PRIVATE SDCL)
target_link_libraries(RunSDCL PRIVATE scenes)
target_link_libraries(RunSDCL PRIVATE parser)

target_link_libraries(RunSDCL PRIVATE "${OMPL_LIBRARIES}")

target_include_directories(RunSDCL PRIVATE "${OMPL_INCLUDE_DIRS}")