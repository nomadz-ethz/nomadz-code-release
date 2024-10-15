#!/bin/bash

set -e

build_type=""

usage() {
    echo "Usage: $0 <Debug|Release>"
    echo "Sets up host toolchains."
    exit 1
}

parse_args() {
    if [ "$#" -ne 1 ]; then
        usage
        return 1
    fi

    build_type="$1"
    for X in Debug Release
    do
        if [ "$X" == "$build_type" ]; then
            return 0
        fi
    done

    usage
    return 1
}

generate_toolchain() {
    local script_dir=$(realpath "$(dirname $0)")
    local nomadz_ng_dir=$(realpath "$script_dir/../../")
    local host_toolchains_dir=$nomadz_ng_dir/build/toolchains
    local conan_out_dir=$host_toolchains_dir/$build_type

    rm -rf $conan_out_dir
    mkdir -p $conan_out_dir

    echo "Generating $build_type host toolchain."

    conan install $nomadz_ng_dir \
        -vquiet \
        --output-folder=$conan_out_dir \
        --build=missing \
        -s build_type=$build_type \
        -pr:h $script_dir/conan/profile_host \
        -pr:b $script_dir/conan/profile_host

    if [ $? -ne 0 ]; then
        echo "An error occurred."
        return 1
    fi

    return 0
}


main() {

    if ! parse_args $@; then
        return 1
    fi

    generate_toolchain
}

main $@
