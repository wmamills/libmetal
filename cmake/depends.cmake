check_include_files (stdatomic.h HAVE_STDATOMIC_H)
check_include_files (linux/futex.h HAVE_FUTEX_H)

find_package (Doxygen)

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")

  find_package (HugeTLBFS REQUIRED)
  collect (PROJECT_INC_DIRS "${HUGETLBFS_INCLUDE_DIR}")
  collect (PROJECT_LIB_DEPS "${HUGETLBFS_LIBRARIES}")

  find_package (LibSysFS REQUIRED)
  collect (PROJECT_INC_DIRS "${LIBSYSFS_INCLUDE_DIR}")
  collect (PROJECT_LIB_DEPS "${LIBSYSFS_LIBRARIES}")

  find_package(Threads REQUIRED)
  collect (PROJECT_LIB_DEPS "${CMAKE_THREAD_LIBS_INIT}")

endif ("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")

# vim: expandtab:ts=2:sw=2:smartindent
