function (setupDependencies)
    set(options TESTS)
    set(args TARGET)
    set(list_args LIBS)
    cmake_parse_arguments(
        PARSE_ARGV 0
        dep # prefix
        "${options}"
        "${args}"
        "${list_args}"
    )
    if(DEFINED dep_LIBS)
        foreach(arg IN LISTS dep_LIBS)
            if(${arg} STREQUAL "osqp") # OSQP
                if (NOT TARGET ${arg})
                    message(STATUS "${dep_TARGET} : `${arg}` targets not found. Attempting to fetch contents...")
                    FetchContent_Declare(
                        ${arg}
                        GIT_REPOSITORY https://github.com/oxfordcontrol/${arg}.git
                        GIT_TAG        origin/master
                    )
                    FetchContent_MakeAvailable(${arg})
                else()
                    message(STATUS "${dep_TARGET} : `${arg}` targets found.")
                endif()
            elseif((${arg} STREQUAL "gtest") AND ${dep_TESTS}) # GTEST
                enable_testing()
                if (NOT TARGET gtest OR NOT TARGET gmock OR NOT TARGET gtest_main)
                    message(STATUS "${dep_TARGET} : `${arg}` targets not found. Attempting to fetch contents...")
                    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
                    FetchContent_Declare(
                        googletest
                        GIT_REPOSITORY https://github.com/google/googletest.git
                        GIT_TAG        origin/master
                    )
                    FetchContent_MakeAvailable(googletest)
                    include(GoogleTest)
                endif()
            endif()
        endforeach()
    endif()
endfunction()

function(setupFlags _FLAGS_)
    set(DEFAULT_FLAGS "-pedantic -Wall -Wextra -Wcast-align -Wcast-qual \
        -Wdisabled-optimization -Wformat=2 \
        -Winit-self -Wlogical-op -Wmissing-include-dirs -Wnoexcept \
        -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow \
        -Wsign-conversion -Wsign-promo -Wstrict-null-sentinel \
        -Wunused")

    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "9.0")
        set(DEFAULT_FLAGS "${DEFAULT_FLAGS} -Wpessimizing-move -Wredundant-move")
    endif()

    set(${_FLAGS_} "${DEFAULT_FLAGS}" PARENT_SCOPE)
endfunction()

function(AppendTestFlags _FLAGS_)
    set(${_FLAGS_} "${${_FLAGS_}} -coverage -fsanitize=address -fsanitize=leak -fsanitize=undefined" PARENT_SCOPE)
endfunction()
