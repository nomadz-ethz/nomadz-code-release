import os
import re
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from typing import Mapping

from returns.result import Failure, Result, Success

script_dir = os.path.dirname(os.path.realpath(__file__))
deploy_script_path = os.path.join(script_dir, "..", "deploy.sh")
setup_bash_path = os.path.join(script_dir, "../../install/setup.bash")


@dataclass(frozen=True)
class Robot:
    name: str
    ip: str
    domainId: int


@dataclass
class RobotStatus:
    battery_status: float = 0.0
    field: str = "NomadZ"
    role: str = "Player"
    connection_lan: bool = False
    connection_wlan: bool = False
    player_id: int = 0


BlameType = Mapping[Robot, Exception]
RobotEntities = Mapping[Robot, RobotStatus]


class Backend:
    """Interface for executing actions in the GUI."""

    @staticmethod
    def deploy(robots: RobotEntities) -> Result[None, BlameType]:
        """
        Deploy the code to the robots using the deploy.sh script of nomadz tools

        @param robots: A dictionary of robots to deploy to.
        @type robots: RobotEntities
        @return: A Result indicating success or failure.
        @rtype: Result[None, BlameType]
        """
        failures = []
        for robot in robots:
            connection = "eth-" if robots[robot].connection_lan else ""
            hostname = f"robot-{connection}{robot.name.lower()}"
            command = f"{deploy_script_path} {hostname}"
            try:
                result = subprocess.run(
                    command,
                    shell=True,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=15,
                )
                result.check_returncode()
            except subprocess.CalledProcessError as e:
                print(f"Failed to deploy to {hostname}: {e.stderr}")
                failures.append((robot, e))
            except TimeoutError:
                print(f"Timeout error deploying to {hostname}")
            except Exception as e:
                print(f"Command failed: {e}")
        if not failures:
            return Success(None)
        else:
            return Failure(dict(failures))

    @staticmethod
    def load_wifi_config(robots: RobotEntities) -> Result[None, BlameType]:
        """
        update the wifi config on the robots.
        by updateting the teams.cfg file and running the "legacy" wireless.sh script
        @param robots: A dictionary of robots to deploy to.
        @type robots: RobotEntities
        @return: A Result indicating success or failure.
        @rtype: Result[None, BlameType]
        """
        failures = []
        for robot in robots:
            connection = "eth-" if robots[robot].connection_lan else ""
            hostname = f"robot-{connection}{robot.name.lower()}"
            print(f"Loading wifi config to {hostname}")
            modify_teams_cfg_cmd = f"ssh -t {hostname} \"sed -i '/^[ \t]*wlanConfig/c\\\\  wlanConfig = {robots[robot].field}.wpa;'  /home/nao/Config/teams.cfg\""
            try:
                result = subprocess.run(
                    modify_teams_cfg_cmd,
                    shell=True,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=10,
                )
                result.check_returncode()
            except subprocess.CalledProcessError as e:
                print(f"Failed to modify teams.cfg on {hostname}: {e.stderr}")
                failures.append((robot, e))
            except TimeoutError:
                print(f"Timeout error modifying teams.cfg on {hostname}")
            except Exception as e:
                print(f"Command failed: {e}")
            command = f"ssh -t {hostname} 'systemctl restart network-wireless'"
            try:
                result = subprocess.run(
                    command,
                    shell=True,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=5,
                )
                result.check_returncode()
            except subprocess.CalledProcessError as e:
                print(f"Failed to load wifi config to {hostname}: {e.stderr}")
                failures.append((robot, e))
            except TimeoutError:
                print(f"Timeout error loading wifi config to {hostname}")
            except Exception as e:
                print(f"Command failed: {e}")
        if not failures:
            return Success(None)
        else:
            return Failure(dict(failures))

    @staticmethod
    def restart(robots: RobotEntities) -> Result[None, BlameType]:
        """
        Restart the nomadz-ng service on the robots using "systemctl restart nomadz-ng"
        @param robots: A dictionary of robots to deploy to.
        @type robots: RobotEntities
        @return: A Result indicating success or failure.
        @rtype: Result[None, BlameType]

        """
        failures = []
        for robot in robots:
            connection = "eth-" if robots[robot].connection_lan else ""
            hostname = f"robot-{connection}{robot.name.lower()}"
            command = f"ssh -t {hostname} 'systemctl restart nomadz-ng'"
            try:
                result = subprocess.run(
                    command,
                    shell=True,
                    text=True,
                    timeout=10,
                )
                result.check_returncode()
            except subprocess.CalledProcessError as e:
                print(f"Failed to restart {hostname}: {e.stderr}")
                failures.append((robot, e))
            except Exception as e:
                print(f"Command failed: {e}")
        if not failures:
            return Success(None)
        else:
            return Failure(dict(failures))

    @staticmethod
    def reboot(robots: RobotEntities) -> Result[None, BlameType]:
        """
        Reboot the robots by sshing into the robots and running "sudo reboot now"
        @param robots: A dictionary of robots to deploy to.
        @type robots: RobotEntities
        @return: A Result indicating success or failure.
        @rtype: Result[None, BlameType]
        """

        def reboot_robot(hostname, command):
            try:
                subprocess.run(
                    command,
                    shell=True,
                    text=True,
                    timeout=10,
                )
            except TimeoutError:
                print(f"Timeout error rebooting {hostname}")
            except Exception as e:
                print(f"Command failed: {e}")

        for robot in robots:
            connection = "eth-" if robots[robot].connection_lan else ""
            hostname = f"robot-{connection}{robot.name.lower()}"
            command = f"ssh -t {hostname} 'sudo reboot now'"
            thread = threading.Thread(target=reboot_robot, args=(hostname, command))
            thread.start()
        return Success(None)

    @staticmethod
    def shutdown(robots: RobotEntities) -> Result[None, BlameType]:
        """
        Shutdown the robots by sshing into the robots and running "sudo shutdown now"
        @param robots: A dictionary of robots to deploy to.
        @type robots: RobotEntities
        @return: A Result indicating success or failure.
        @rtype: Result[None, BlameType]
        """
        if robots == {}:
            print("No robots selected.")
            return Failure({})
        for robot in robots:
            connection = "eth-" if robots[robot].connection_lan else ""
            hostname = f"robot-{connection}{robot.name.lower()}"
            command = ["ssh", "-t", hostname, "sudo", "shutdown", "now"]
            try:
                subprocess.run(
                    command,
                    text=True,
                    timeout=10,
                )
            except TimeoutError:
                print(f"Timeout error shutting down {hostname}")
            except Exception as e:
                print(f"Command failed: {e}")
        return Success(None)

    @staticmethod
    def download_logs(robots: RobotEntities) -> Result[None, BlameType]:
        """
        Download logs from the robots using rsync.
        @param robots: A dictionary of robots to deploy to.
        @type robots: RobotEntities
        @return: A Result indicating success or failure.
        @rtype: Result[None, BlameType]

        """
        for robot in robots:
            connection = "eth-" if robots[robot].connection_lan else ""
            hostname = f"robot-{connection}{robot.name.lower()}"
            source = f"{hostname}:/home/nao/logs"
            destination = "downloads/"
            try:
                subprocess.run(
                    ["rsync", "-avL", "-e", "ssh", source, destination], check=True
                )
                print(f"Successfully downloaded logs for {robot.name}")
            except subprocess.CalledProcessError as e:
                print(f"Failed to download logs for {robot.name}: {e}")
                return Failure("DownloadFailed")

        return Success(None)

    @staticmethod
    def set_player_id(robots: RobotEntities) -> Result[None, BlameType]:
        """
        Set the player id on the robots using sed changing the player_id in the game_settings.yaml file
        @param robots: A dictionary of robots to deploy to.
        @type robots: RobotEntities
        @return: A Result indicating success or failure.
        @rtype: Result[None, BlameType]
        """
        for robot in robots:
            connection = "eth-" if robots[robot].connection_lan else ""
            hostname = f"robot-{connection}{robot.name.lower()}"
            print(f"setting player id of {hostname}")
            modify_player_id_cmd = f"ssh -t {hostname} \"sed -i '/^[ \t]*player_id/c\\\\player_id: {robots[robot].player_id}\n'  /home/nao/nomadz-ng_install/share/nomadz_configuration/config/game_settings.yaml\""
            try:
                subprocess.run(
                    modify_player_id_cmd,
                    shell=True,
                    text=True,
                    timeout=10,
                )
            except TimeoutError:
                print(f"Timeout error modifying on {hostname}")
            except Exception as e:
                print(f"Command failed: {e}")

    @staticmethod
    def set_role(robots: RobotEntities) -> Result[None, BlameType]:
        """
        Set the role on the robots using sed changing the player_role in the game_settings.yaml file
        @param robots: A dictionary of robots to deploy to.
        @type robots: RobotEntities
        @return: A Result indicating success or failure.
        @rtype: Result[None, BlameType]
        """
        for robot in robots:
            connection = "eth-" if robots[robot].connection_lan else ""
            hostname = f"robot-{connection}{robot.name.lower()}"
            print(f"setting role of {hostname}")
            modify_role_cfg_cmd = f"ssh -t {hostname} \"sed -i '/^[ \t]*player_role/c\\\\player_role: {robots[robot].role}\n'  /home/nao/nomadz-ng_install/share/nomadz_configuration/config/game_settings.yaml\""
            try:
                subprocess.run(
                    modify_role_cfg_cmd,
                    shell=True,
                    text=True,
                    timeout=10,
                )
            except TimeoutError:
                print(f"Timeout error modifying teams.cfg on {hostname}")
            except Exception as e:
                print(f"Command failed: {e}")

    @staticmethod
    def get_battery_status(robots: RobotEntities) -> Result[RobotEntities, BlameType]:
        """
        Get the battery status of the robots by setting the ROS_DOMAIN_ID and running "ros2 topic echo --once sensors/battery"
        @param robots: A dictionary of robots to deploy to.
        @type robots: RobotEntities
        @return: A Result indicating success or failure.
        @rtype: Result[RobotEntities, BlameType]
        """

        def query_robot(robot):
            combined_command = f"export ROS_DOMAIN_ID={robot.domainId} ; source {setup_bash_path} ; ros2 topic echo --once sensors/battery"
            try:
                result = subprocess.run(
                    ["bash", "-c", combined_command],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=9,
                )
                pattern = re.compile(r"charge:\s+(\d+\.\d+)")
                match = pattern.search(result.stdout)
                if match:
                    battery_status = float(match.group(1))
                    robots[robot].battery_status = round(battery_status / 100, 2)
            except TimeoutError:
                print(f"Timeout error getting battery status for {robot.name}")
            except Exception as e:
                print(f"Failed to get battery status for {robot.name}: {e}")

        with ThreadPoolExecutor() as executor:
            future_to_robot = {
                executor.submit(query_robot, robot): robot
                for robot in robots
                if robots[robot].connection_lan or robots[robot].connection_wlan
            }
            for future in as_completed(future_to_robot):
                future.result()
        return Success(robots)

    @staticmethod
    def get_connection(robots: RobotEntities) -> Result[RobotEntities, BlameType]:
        """
        Get the connection status of the robots by pinging the robots on both the LAN and WLAN interfaces
        @param robots: A dictionary of robots to deploy to.
        @type robots: RobotEntities
        @return: A Result indicating success or failure.
        @rtype: Result[RobotEntities, BlameType]
        """

        def ping_robot(robot):
            lan_or_wlan = [True, False]
            results = {}
            for lan in lan_or_wlan:
                robot_ip = robot.ip if not lan else robot.ip.replace(".0.", ".1.")
                command = ["ping", "-c", "1", robot_ip]
                try:
                    result = subprocess.run(
                        command,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        timeout=5,
                    )
                    results[lan] = result.returncode == 0
                except TimeoutError:
                    results[lan] = False
                except Exception as e:
                    results[lan] = False
            return robot, results

        with ThreadPoolExecutor() as executor:
            future_to_robot = {
                executor.submit(ping_robot, robot): robot for robot in robots
            }
            for future in as_completed(future_to_robot):
                robot, ping_results = future.result()
                robots[robot].connection_lan = ping_results[True]
                robots[robot].connection_wlan = ping_results[False]
                if not ping_results[True] and not ping_results[False]:
                    robots[robot].battery_status = 0.0
        return Success(robots)
