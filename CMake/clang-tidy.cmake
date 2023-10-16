find_program(CLANG_TIDY NAMES "clang-tidy" "clang-tidy-6.0")
# Enable clang tidy only if using a clang compiler
if(CLANG_TIDY)
    # If debug build enabled do automatic clang tidy
    if(CMAKE_BUILD_TYPE MATCHES Debug)
        set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY} "-header-filter='${CMAKE_SOURCE_DIR}'")
    endif()

    # Enable checking and formatting through run-clang-tidy if available
    # FIXME Make finding this program more portable
    get_filename_component(CLANG_TIDY ${CLANG_TIDY} REALPATH)
    get_filename_component(CLANG_DIR ${CLANG_TIDY} DIRECTORY)
    find_program(RUN_CLANG_TIDY NAMES "run-clang-tidy" "run-clang-tidy.py" "run-clang-tidy-4.0.py" HINTS /usr/share/clang/ ${CLANG_DIR}/../share/clang/ /usr/bin/)
    if(RUN_CLANG_TIDY)
        message(STATUS "Found clang-tidy and run-clang-tidy, adding linting targets")

        # Set export commands on
        set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

        # Get amount of processors to speed up linting
        include(ProcessorCount)
        ProcessorCount(NPROC)
        IF(NPROC EQUAL 0)
            set(NPROC 1)
        ENDIF()

        # Lint everything in Src except Utils
        set(LINT_CXX_REGEX "${SRC_DIR}/\\(\\(\\?\\!Utils\\).*\\)/\\(.*\\)")

        # Apply fixits to the given targets
        add_custom_target(
            lint COMMAND
            ${RUN_CLANG_TIDY} -fix -format -header-filter=${LINT_CXX_REGEX} -j${NPROC} ${LINT_CXX_REGEX}
            COMMENT "Auto fixing problems in all source files"
        )

        add_custom_target(
            check-lint COMMAND
            ${RUN_CLANG_TIDY} -header-filter=${LINT_CXX_REGEX} -j${NPROC} ${LINT_CXX_REGEX}
            | tee ${CMAKE_BINARY_DIR}/check_lint_file.txt
            # WARNING: fix to stop with error if there are problems
            COMMAND ! grep -c ": error: " ${CMAKE_BINARY_DIR}/check_lint_file.txt > /dev/null
            COMMENT "Checking for problems in source files"
        )
    else()
        message(STATUS "Could NOT find run-clang-tidy script")
    endif()
else()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        message(STATUS "Could NOT find clang-tidy")
    else()
        message(STATUS "Could NOT check for clang-tidy, wrong compiler: ${CMAKE_CXX_COMPILER_ID}")
    endif()
endif()
