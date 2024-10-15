#!/bin/bash

set -e

readonly script_dir=$(realpath "$(dirname $0)")
readonly nomadz_ng_dir=$(realpath "$script_dir/../../")
readonly nomadz_colcon_home_dir=$nomadz_ng_dir/.colcon

generate_nomadz_ng_mixins() {
  echo "Generating nomadz-ng mixins"

  local generated_mixins_dir=$nomadz_colcon_home_dir/mixin
  mkdir -p $generated_mixins_dir

  local index="mixin:\n"
  for template in $script_dir/templates/mixin/*.mixin.in
  do
    local mixin=$(basename ${template%.in})

    sed "s|@NOMADZ_NG_DIR@|$nomadz_ng_dir|g" $template > $generated_mixins_dir/$mixin

    if [ $? -ne 0 ]; then
        echo "Failed to generate $mixin mixin"
        return 1
    fi
    index="$index  - $mixin\n"
  done

  echo -e $index > $generated_mixins_dir/index.yaml

  return 0
}

main() {

  if ! generate_nomadz_ng_mixins; then
    return 1
  fi

  return 0
}

main
