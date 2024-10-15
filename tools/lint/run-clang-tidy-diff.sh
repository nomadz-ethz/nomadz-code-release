#!/bin/bash

help() {
    echo "Usage: $0 <base_branch> - where base_branch is the branch to compare against."
    exit 1
}

# check if clang-tidy-diff is installed
if ! command -v clang-tidy-diff &> /dev/null
then
    echo "clang-tidy-diff is not installed. Please install it with your system package manager."
    exit 1
fi

# check if clang tidy 14 is installed
if ! command -v clang-tidy-14 &> /dev/null
then
    echo "clang-tidy-14 is not installed. Please install it with your system package manager."
    exit 1
fi
readonly clang_tidy_binary=$(which clang-tidy-14)

# first optional argument is the base branch
base_branch=${1:-"origin/master"}

echo "Running clang-tidy-diff against $base_branch"

# cd to root of the project
cd $(realpath $(dirname -- "$( readlink -f -- "$0"; )")/../..)

# check if compile_commands.json exists
if [ ! -f "build/compile_commands.json" ]; then
    echo "compile_commands.json not found - please execute the host build first."
    exit 1
fi

# capture the output of clang-tidy-diff but also tee it to stderr
# so it's displayed in the console
output=$(\
    git diff -U0 --no-color "$base_branch" -- '*.hpp' '*.cpp' | \
    clang-tidy-diff -clang-tidy-binary $clang_tidy_binary -use-color -p1 -j $(nproc --ignore 1) -path build | \
    tee /dev/fd/2)

# hack because prior to llvm 18 clang-tidy-diff doesn't return a non-zero exit code
if $(echo "$output" | grep -q "error:")
then
  exit 1
fi
