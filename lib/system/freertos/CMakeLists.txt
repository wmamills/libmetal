collect (PROJECT_LIB_HEADERS sys.h)
collect (PROJECT_LIB_HEADERS alloc.h)

collect (PROJECT_LIB_SOURCES init.c)
collect (PROJECT_LIB_SOURCES irq.c)

if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_MACHINE})
  add_subdirectory(${PROJECT_MACHINE})
endif (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_MACHINE})

# vim: expandtab:ts=2:sw=2:smartindent
