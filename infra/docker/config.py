import getpass
import hashlib
import os
import pathlib
import subprocess
from typing import Any, Callable, Dict

HOME_PATH = pathlib.Path(os.environ.get("HOME"))
PATH_TO_CUSTOM_CONFIG = HOME_PATH / ".sda_config.json"
SDA_CONTEXT_DIR = HOME_PATH / "sda_context"
REP_IN_DOCKER = "/rep"
SDA_CONTEXT_DIR_IN_DOCKER = "/sda_context"


def get_default_container_name():
    username = getpass.getuser()
    file_hash = hashlib.sha1(__file__.encode("utf8")).hexdigest()[:7]
    # we need file hash here to distinguish two containers from two physical copies of the repo
    return f"rviz-{username}-{file_hash}"


def get_default_image_name():
    username = getpass.getuser()
    file_hash = hashlib.sha1(__file__.encode("utf8")).hexdigest()[:7]
    # we need file hash here to distinguish two images from two physical copies of the repo
    return f"rviz-ubuntu-20.04-{username}-{file_hash}"


def get_default_user_home():
    return pathlib.Path.home()


def is_nvidia_docker_installed():
    cmd = ["bash", "-c", "dpkg -l | grep nvidia-container-toolkit"]
    try:
        out = subprocess.check_output(cmd)
    except subprocess.CalledProcessError:
        return False
    if out and b"nvidia-container-toolkit" in out:
        return True
    return False


def get_default_host_platform() -> str:
    """Available platforms: linux_amd64,
    linux_amd64_with_cuda, linux_arm64 (jetson)
    """
    host_platform = os.uname()
    if host_platform.sysname == "Linux":
        if host_platform.machine == "aarch64":
            return "linux_arm64"
        elif is_nvidia_docker_installed():
            return "linux_amd64_with_cuda"
        else:
            return "linux_amd64"


def get_default_config():
    config = {
        "image_name": get_default_image_name(),
        "sda_context_dir": SDA_CONTEXT_DIR,
        "rep_in_docker": REP_IN_DOCKER,
        "container_name": get_default_container_name(),
        "user_home": get_default_user_home(),
        "create_user_home": True,
        # https://sbtatlas.sigma.sbrf.ru/wiki/pages/viewpage.action?pageId=3100876277
        "ros_domain_id": None,  # value in range 0-232, or None which means the value is not set
        # value is name of rover (carnelion, vortex), or None which means the value is not set
        "rover_name": "",
        "host_platform": get_default_host_platform(),
    }
    return config


def read_env_vars_config():
    config = {}
    ROS_DOMAIN_ID_ENV_VAR = "ROS_DOMAIN_ID"
    ROVER_NAME_ENV_VAR = "ROVER_NAME"
    LOCATION_NAME_ENV_VAR = "LOCATION_NAME"
    HOST_PLATFORM_ENV_VAR = "HOST_PLATFORM"
    if ROS_DOMAIN_ID_ENV_VAR in os.environ:
        config["ros_domain_id"] = os.environ[ROS_DOMAIN_ID_ENV_VAR]
    if ROVER_NAME_ENV_VAR in os.environ:
        config["rover_name"] = os.environ[ROVER_NAME_ENV_VAR]
    if LOCATION_NAME_ENV_VAR in os.environ:
        config["location_name"] = os.environ[LOCATION_NAME_ENV_VAR]
    if HOST_PLATFORM_ENV_VAR in os.environ:
        config["host_platform"] = os.environ[HOST_PLATFORM_ENV_VAR]
    config["service_node"] = os.environ.get("SERVICE_NODE") is not None
    return config


def safe_load_config(
    config_path: pathlib.Path, loader: Callable[[pathlib.Path], Dict[str, Any]]
) -> Dict[str, Any]:
    if not config_path.exists():
        return {}
    with open(config_path) as f:
        return loader(f)
