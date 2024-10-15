#!/bin/bash

set -e

readonly script_dir=$(realpath "$(dirname $0)")
readonly cyan='\033[0;36m'
readonly yellow='\033[1;33m'
readonly bold='\033[1m'
readonly nc='\033[0m'

echo -e "${bold}${yellow}Setting up nomadz-ng workspace${nc}"

echo -e "${bold}${cyan}Source and system dependencies${nc}"
bash $script_dir/install-deps.sh

echo -e "${bold}${cyan}Conan toolchains${nc}"
bash $script_dir/setup-toolchain-host.sh Debug
bash $script_dir/setup-toolchain-host.sh Release

echo -e "${bold}${cyan}Colcon mixins${nc}"
bash $script_dir/setup-colcon-mixins.sh

echo -e "${bold}${yellow}nomadz-ng workspace was setup successfully!${nc}"
