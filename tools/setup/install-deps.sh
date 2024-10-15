#!/bin/bash

readonly SCRIPT_DIR=$(realpath "$(dirname $0)")
readonly NOMADZ_NG_DIR=$(realpath "$SCRIPT_DIR/../../")

import_source_dependencies() {
    echo "Importing source dependencies"
    vcs import --input $NOMADZ_NG_DIR/nomadz-ng.repos --repos $NOMADZ_NG_DIR/src/deps --force &> /dev/null
    if [ $? -ne 0 ]; then
        echo "Failed to clone source dependencies"
        return 1
    fi
}

install_system_dependencies() {
  echo "Installing system dependencies"
  rosdep install --from-paths $NOMADZ_NG_DIR/src --ignore-src -r -y
  if [ $? -ne 0 ]; then
      echo "Failed to install system dependencies"
      return 1
  fi
  return 0
}

main() {
    if ! import_source_dependencies
    then
        return 1
    fi

    if ! install_system_dependencies
    then
        return 1
    fi

    return 0
}

main
