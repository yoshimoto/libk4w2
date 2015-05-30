# Default installation directory, based on operating system
IF (PROJECT_OS_WIN)
    SET (CMAKE_INSTALL_PREFIX "C:\\Program Files\\libk4w2" CACHE PATH "Installation directory")
ELSE (PROJECT_OS_WIN)
    SET (CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Installation directory")
ENDIF (PROJECT_OS_WIN)

MESSAGE (STATUS "${PROJECT_NAME} will be installed to ${CMAKE_INSTALL_PREFIX}")

# Installation prefix for include files
STRING (TOLOWER ${PROJECT_NAME} projectNameLower)
SET (PROJECT_INCLUDE_INSTALL_DIR "include/${projectNameLower}")

SET (PROJECT_LIBRARY_INSTALL_DIR "lib")

MESSAGE (STATUS "Headers will be installed to ${CMAKE_INSTALL_PREFIX}/${PROJECT_INCLUDE_INSTALL_DIR}")
MESSAGE (STATUS "Libraries will be installed to ${CMAKE_INSTALL_PREFIX}/${PROJECT_LIBRARY_INSTALL_DIR}")

