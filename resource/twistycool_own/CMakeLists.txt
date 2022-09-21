cmake_minimum_required (VERSION 2.6)
project (AminoTutorials)

# pkg-config setup
find_package(PkgConfig)

# Search for the pkg-config packages
pkg_search_module(SDL2 REQUIRED sdl2)
pkg_search_module(GL REQUIRED gl)
pkg_search_module(AMINO REQUIRED amino)
pkg_search_module(AMINO_GL REQUIRED amino-gl)

# Compile scene graph to C
add_custom_command(
  OUTPUT twistycool-scene.c twistycool-scene.c.h
  DEPENDS twistycool.robray
  COMMAND aarxc "${CMAKE_SOURCE_DIR}/twistycool.robray" -o twistycool-scene.c -n "twistycool"
  )

# Compile plugin (shared library)
add_library( scene MODULE twistycool-scene.c )
target_include_directories( scene PUBLIC
  ${AMINO_INCLUDE_DIRS}
  )
target_link_libraries( scene
  ${AMINO_GL_LIBRARIES}
  ${AMINO_LIBRARIES}
  )
