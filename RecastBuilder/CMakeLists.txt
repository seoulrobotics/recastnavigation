file(GLOB SOURCES Source/*.cpp Contrib/fastlz/fastlz.c)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(SYSTEM Contrib/fastlz)
include_directories(SYSTEM Contrib)
include_directories(../DebugUtils/Include)
include_directories(../Detour/Include)
include_directories(../DetourCrowd/Include)
include_directories(../DetourTileCache/Include)
include_directories(../Recast/Include)
include_directories(Include)

if (WIN32)
    add_executable(RecastBuilder WIN32 ${SOURCES})
    set_target_properties(RecastBuilder PROPERTIES LINK_FLAGS /SUBSYSTEM:CONSOLE)
elseif(APPLE)
    add_executable(RecastBuilder MACOSX_BUNDLE ${SOURCES})
else()
    add_executable(RecastBuilder ${SOURCES})
    set(cmake_exe_linker_flags "${cmake_exe_linker_flags} /subsystem:console")
endif()

add_dependencies(RecastBuilder DebugUtils Detour DetourCrowd DetourTileCache Recast)
target_link_libraries(RecastBuilder DebugUtils Detour DetourCrowd DetourTileCache Recast)


install(TARGETS RecastBuilder
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin)

install(TARGETS RecastBuilder DESTINATION ../../Util/DockerUtils/dist)
