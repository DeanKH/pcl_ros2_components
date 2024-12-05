# find all directory in this directory

file(GLOB_RECURSE subdirs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */CMakeLists.txt)

foreach(subdir ${subdirs})
  get_filename_component(dir ${subdir} DIRECTORY)
  add_subdirectory(${dir})
endforeach()
