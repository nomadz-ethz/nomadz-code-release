#!/bin/bash

set -e
set -o pipefail

readonly robot_nomadz_ng_install_dir="/home/nao/nomadz-ng_install"
readonly script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
readonly nomadz_ng_dir=$(readlink -f "$script_dir/..")
readonly ssh_config="$nomadz_ng_dir/config/ssh/config"
readonly ssh_id="$nomadz_ng_dir/config/ssh/id_rsa_nao"

usage() {
  echo "usage:"
  echo "${0} robot-ssh-alias"
  echo "${0} -h"
  echo ""
  echo " -h               : Display this help"
  echo " robot-ssh-alias  : SSH alias of the robot"
  echo ""
  exit 1
}

parse_args() {
  if [ "$1" == "-h" ]; then
    usage
  fi

  if [ "$#" -ne 1 ]; then
    echo "Illegal number of arguments"
    usage
  fi

  robot_ssh_alias="$1"

  return 0
}

validate_ssh_id_permissions() {
  local permissions=$(stat -c "%a" "$ssh_id")
  if [ "$permissions" != "400" ]
  then
    chmod 400 "$ssh_id"
  fi

  return 0
}

robot_is_reachable() {
  ssh -F "$ssh_config" -i "$ssh_id" "$robot_ssh_alias" -q "exit"
  local ret_val=$?
  if [ "$ret_val" != 0 ]
  then
    echo "Cannot establish SSH connection to $robot_ssh_alias!"
    exit 1
  fi

  return $ret_val
}

upload_nomadz_ng() {
  # check if install-robot directory exists
  if [ ! -d "${nomadz_ng_dir}/install-robot" ]; then
    echo "install-robot directory not found. You must cross-compile nomadz-ng first."
    exit 1
  fi

  echo "Deploying nomadz-ng to $robot_ssh_alias"
  rsync -avL -e "ssh -F $ssh_config -i $ssh_id" "${nomadz_ng_dir}/install-robot/" "$robot_ssh_alias:$robot_nomadz_ng_install_dir"

  return 0
}

restart_nomadz_ng() {
  echo "Restarting nomadz-ng service"
  ssh -F "$ssh_config" -i "$ssh_id" "$robot_ssh_alias" "systemctl restart nomadz-ng"

  return 0
}

main() {
  parse_args "$@"

  validate_ssh_id_permissions
  robot_is_reachable
  upload_nomadz_ng
  restart_nomadz_ng

  return 0
}

main "$@"
