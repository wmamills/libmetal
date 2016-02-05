collect (PROJECT_LIB_TESTS atomic.c)
collect (PROJECT_LIB_TESTS mutex.c)
collect (PROJECT_LIB_TESTS shmem.c)

if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_MACHINE})
  add_subdirectory(${PROJECT_MACHINE})
endif (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_MACHINE})

# vim: expandtab:ts=2:sw=2:smartindent
