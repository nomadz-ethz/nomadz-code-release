#!/bin/bash

# check if clang-tidy is installed
if ! command -v clang-tidy-14 &> /dev/null
then
    echo "clang-tidy-14 is not installed. Please install it with your system package manager."
    exit 1
fi

# cd to root of the project
cd $(realpath $(dirname -- "$( readlink -f -- "$0"; )")/../..)

# check if compile_commands.json exists
if [ ! -f "build/compile_commands.json" ]; then
    echo "compile_commands.json not found - please execute the host build first."
    exit 1
fi

exclude_patterns=':!:src/nomadz_apps/* :!:src/nomadz_modules_examples/*'
all_files=$(git ls-files *.cpp $exclude_patterns)

run-clang-tidy-14 -quiet -j $(nproc --ignore=1) -p build $all_files
