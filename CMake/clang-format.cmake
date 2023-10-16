# Adding clang-format check and formatter if found
find_program(CLANG_FORMAT NAMES "clang-format-10" "clang-format-10.0" "clang-format")
if(CLANG_FORMAT)
    exec_program(${CLANG_FORMAT} ${CMAKE_CURRENT_SOURCE_DIR} ARGS --version OUTPUT_VARIABLE CLANG_FORMAT_VERSION_STRING)
    string(REGEX MATCH "[0-9]+(.[0-9]+)?(.[0-9]+)?(.[a-z0-9]+)?" CLANG_VERSION ${CLANG_FORMAT_VERSION_STRING})

    if((${CLANG_VERSION} VERSION_GREATER_EQUAL "10.0.0") AND (${CLANG_VERSION} VERSION_LESS "11.0.0"))

        message(STATUS "Found ${CLANG_FORMAT} version ${CLANG_VERSION}, adding formatting targets")

        add_custom_target(
            do-format
            COMMAND
            ${CLANG_FORMAT}
            -i
            -style=file
            ${FORMAT_CXX_SRC_FILES}
            COMMENT "Auto formatting of all source files"
        )

        add_custom_target(
            check-format
            COMMAND
            ${CLANG_FORMAT}
            -style=file
            -output-replacements-xml
            ${FORMAT_CXX_SRC_FILES}
            # print output
            | tee ${CMAKE_BINARY_DIR}/check_format_file.txt | grep -c "replacement " |
            tr -d "[:cntrl:]" && echo " replacements necessary"
            # WARNING: fix to stop with error if there are problems
            COMMAND ! grep -c "replacement "
            ${CMAKE_BINARY_DIR}/check_format_file.txt > /dev/null
            COMMENT "Checking format compliance"
        )
    else()
        message(STATUS "Could only find version ${CLANG_VERSION} of clang-format, but version 10 is required.")
    endif()
else()
    message(STATUS "Could NOT find clang-format")
endif()
