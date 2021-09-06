#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

import argparse
import getpass
import grp
import logging
import os
import pathlib
import pwd
from typing import Tuple
import psutil
import json
import argcomplete

import infra.docker.docker_impl as docker_impl
import infra.docker.config as cfg

logging.basicConfig(level=logging.INFO, format="%(message)s")

REP_DIR = os.path.abspath(os.path.dirname(__file__))
BASH_HISTORY = cfg.REP_IN_DOCKER + "/.bash_history"
PROFILE_DIR_NAME = "profile"

X_SOCKET_DIR = "/tmp/.X11-unix"
X_AUTHORITY_FILENAME = ".Xauthority"


def get_xauth() -> Tuple:
    """Return tuple of (Xserver socket dir path, Xauthority file path) for current user."""
    return X_SOCKET_DIR, os.path.join(pathlib.Path.home(), X_AUTHORITY_FILENAME)


def has_default_primary_ip() -> bool:
    """Returns True is machine has default (192.168.1.111) primary ip"""
    for dev, addrs in psutil.net_if_addrs().items():
        if any(addr.address == "192.168.1.111" for addr in addrs):
            return True
    return False


def build_image(args, argv, config) -> None:
    logging.info(f"Build image!")
    docker = docker_impl.DockerImpl()
    docker.build(
        argv,
        tag=args.image_name,
        path=args.path_to_context,
        target="debug",
        buildargs={"host_docker_group_id": str(grp.getgrnam("docker").gr_gid)},
        network_mode="host"
    )


def start_container(args, argv, config) -> docker_impl.ContainerImpl:
    docker = docker_impl.DockerImpl()
    if docker.is_container_running(args.container):
        logging.info(f"Container {args.container} already running")
        return docker.get_container(args.container)

    assert 0 <= args.ros_domain_id <= 255, \
        f"ros_domain_id has invalid value: {args.ros_domain_id}. Should be in range [0,255]"

    # for opening rviz inside docker http://wiki.ros.org/docker/Tutorials/GUI
    # Get X server socket dir and authentication for current user.
    xsocket, xauthority = get_xauth()

    docker.run(
        argv,
        name=args.container,
        auto_remove=True,
        detach=True,
        image=args.image,
        ipc_mode="host",
        network_mode="host",
        pid_mode="host",
        privileged=True,
        security_opt=["apparmor:unconfined"],
        stdin_open=True,
        tty=True,
        gpus="all" if cfg.is_nvidia_docker_installed() else False,
        environment={
            "HISTFILE": BASH_HISTORY,
            "ROS_DOMAIN_ID": args.ros_domain_id,
            "REP_ROOT": cfg.REP_IN_DOCKER,
            "USER": getpass.getuser(),
            "DISPLAY": args.display_id,
            "QT_STYLE_OVERRIDE": "kvantum",
            "XAUTHORITY": f"/tmp/{X_AUTHORITY_FILENAME}",
            "GST_DEBUG": 2,
            **({"FASTRTPS_DEFAULT_PROFILES_FILE": os.path.join(cfg.REP_IN_DOCKER, "default_profile.xml")}
                if has_default_primary_ip() else {}),
            **({"NVIDIA_DOCKER_INSTALLED": 1} if cfg.is_nvidia_docker_installed() else {}),
        },
        volumes=[
            "/dev:/dev",
            f"{args.context_dir}:{cfg.SDA_CONTEXT_DIR_IN_DOCKER}",
            f"{REP_DIR}:{cfg.REP_IN_DOCKER}",
            # There will be permission problem if mount like "-v", f"{xauthority}:{xauthority}",
            # Cuz by default it will create /home/USERNAME/.Xauthority file and
            # useradd command will fail altogether with `ln -s` cmds below
            f"{xsocket}:{xsocket}",
            f"{xauthority}:/tmp/{X_AUTHORITY_FILENAME}",
            "/var/run/docker.sock:/var/run/docker.sock"
        ]
    )

    container = docker.get_container(args.container)
    try:
        user_id = str(pwd.getpwnam(getpass.getuser())[2])
        username = getpass.getuser()
        container.exec_run(["useradd", *(["-m"] if config["create_user_home"] else []),
                            "-d", config["user_home"],
                            "-G", "sudo,dialout,docker",
                            "-s", "/bin/bash", username])
        container.exec_run(["usermod", "--uid", os.getuid(), username])
        container.exec_run(["groupmod", "--gid", os.getgid(), username])
        container.exec_run(
            ["bash", "-c", r'echo "{0} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/{0}'.format(username)])
        container.exec_run(["chmod", "0440", f"/etc/sudoers.d/{username}"])
        container.exec_run(
            ["/bin/bash", "-c", r"echo Set disable_coredump false >> /etc/sudo.conf"])

        def exec_as_user(cmd):
            container.exec_run(cmd, user=user_id,
                               workdir=str(config["user_home"]))
        # set symlinks on context dir and rep
        exec_as_user(["mkdir", config["user_home"] / ".ssh"])
        container.copy(str(pathlib.Path.home() / ".ssh"),
                       str(config["user_home"] / ".ssh"))
        exec_as_user(["ln", "-s", cfg.REP_IN_DOCKER,
                     os.path.basename(cfg.REP_IN_DOCKER)])

        if config["create_user_home"]:
            path_to_docker_profile_file = os.path.join(
                cfg.REP_IN_DOCKER, "docker/profile.bash")
            exec_as_user(
                ["bash", "-c", f"echo source {path_to_docker_profile_file} >> .profile"])
    except docker_impl.DockerImplException:
        container.stop(timeout=0)
        raise

    return container


def stop_container(args, argv, config) -> None:
    docker = docker_impl.DockerImpl()
    try:
        container = docker.get_container(args.container)
        container.stop(args.time)
    except docker_impl.DockerImplException as err:
        logging.warning("Problem with stopping container: %s", err)


def attach_to_container(args, argv, config) -> None:
    user_id = str(pwd.getpwnam(getpass.getuser())[2])
    docker = docker_impl.DockerImpl()
    try:
        container = docker.get_container(args.container)
        container.exec_run(["bash", "--login"], user=user_id,
                           workdir=cfg.REP_IN_DOCKER, stdin=True, tty=True)
    except docker_impl.DockerImplException as err:
        logging.warning("Attach to container failed: %s", err)


def build_all_command(args, argv, config):
    # Stop container already running
    docker_client = docker_impl.DockerImpl()
    if docker_client.is_container_running(args.container):
        stop_container(args, argv, config)
    build_image(args, argv, config)
    container = start_container(args, argv, config)
    try:
        user_id = str(pwd.getpwnam(getpass.getuser())[2])
        build_cmd = [
            "/bin/bash",
            "--login",
            "-c",
            f"colcon build --symlink-install && source {cfg.REP_IN_DOCKER}/install/setup.sh"
        ]
        container.exec_run(build_cmd, user=user_id,
                           workdir=cfg.REP_IN_DOCKER)
    finally:
        stop_container(args, argv, config)


def run_app(args, argv, config):
    docker_client = docker_impl.DockerImpl()
    if docker_client.is_container_running(args.container):
        container = docker_client.get_container(args.container)
    else:
        container = start_container(args, argv, config)
    try:
        user_id = str(pwd.getpwnam(getpass.getuser())[2])

        # run app
        display_config = args.rviz_display_config

        # cp display_config
        config_path = pathlib.Path(display_config)
        config_destination = f"/tmp/{config_path.name}"
        container.copy(str(config_path.parent), "/tmp")

        cmd = f"ros2 launch launch/rviz.launch.py display-config:={config_destination}"
        window_mode = args.rviz_window_mode
        if window_mode:
            cmd += f" window-mode:={window_mode}"

        window_style = args.rviz_window_style
        if window_style:
            cmd += f" window-style:={window_style}"

        stream_uri = args.stream_uri
        if stream_uri:
            stream_host = args.stream_host
            stream_port = args.stream_port
            cmd += f" stream-host:={stream_host} stream-port:={stream_port} stream-uri:={stream_uri}"
            stream_gst_pipeline = args.stream_gst_pipeline
            if stream_gst_pipeline:
                cmd += f" stream-gst-pipeline:={stream_gst_pipeline}"

        container.exec_run(
            ["/bin/bash", "--login", "-c", cmd],
            user=user_id,
            workdir=cfg.REP_IN_DOCKER
        )
    finally:
        stop_container(args, argv, config)


def add_build_image_subcommand_args(parser, config):
    parser.add_argument(
        '--docker-context',
        dest='path_to_context',
        default='./docker',
        help='Path to folder with dockerfile and build context'
    )
    parser.add_argument(
        '-n',
        '--name',
        dest='image_name',
        default=config['image_name'],
        help=f'Name of image (default: {config["image_name"]})'
    )
    return parser


def add_build_all_subcommand_args(parser, config):
    parser = add_build_image_subcommand_args(parser, config)
    parser = add_start_subcommand_args(parser, config)
    parser.add_argument(
        "-t",
        "--time",
        dest="time",
        default=0,
        type=int,
        help="Seconds to wait for stop before killing it (default 0)"
    )
    return parser


def add_start_subcommand_args(parser, config):
    parser.add_argument(
        "-c",
        "--container",
        dest="container",
        default=config["container_name"],
        help="Container name"
    )
    parser.add_argument(
        "-i",
        "--image",
        dest="image",
        default=config["image_name"],
        help="Image name"
    )
    parser.add_argument(
        '--context',
        dest='context_dir',
        default=config['sda_context_dir'],
        help='Path to context dir in host system'
    )
    parser.add_argument(
        "--ros-domain-id",
        dest="ros_domain_id",
        default=100,
        type=int,
        help="ROS_DOMAIN_ID env var to set in docker"
             "See https://sbtatlas.sigma.sbrf.ru/wiki/pages/viewpage.action?pageId=3100876277"
    )
    parser.add_argument(
        "--display-id",
        dest="display_id",
        default=os.getenv("DISPLAY", ":0"),
        type=str,
        help="X11 display ID"
    )

    return parser


def add_stop_subcommand_args(parser, config):
    parser.add_argument(
        "-c",
        "--container",
        dest="container",
        default=config["container_name"],
        help="Container to stop"
    )
    parser.add_argument(
        "-t",
        "--time",
        dest="time",
        default=0,
        type=int,
        help="Seconds to wait for stop before killing it (default 0)"
    )
    return parser


def add_attach_subcommand_args(parser, config):
    parser.add_argument(
        "-c",
        "--container",
        dest="container",
        default=config["container_name"],
        help="Container to stop"
    )
    return parser


def add_run_app_subcommand_args(parser, config):
    parser = add_start_subcommand_args(parser, config)
    parser.add_argument(
        "-t",
        "--time",
        dest="time",
        default=0,
        type=int,
        help="Seconds to wait for stop before killing it (default 0)"
    )

    # rviz related args
    parser.add_argument(
        "--rviz-display-config",
        dest="rviz_display_config",
        required=True,
        help="Path to a rviz config",
        type=str
    )
    parser.add_argument(
        "--rviz-fixed-frame",
        dest="rviz_fixed_frame",
        help="Set the fixed frame for rviz",
        type=int
    )
    parser.add_argument(
        "--rviz-window-mode",
        dest="rviz_window_mode",
        default="regular",
        help="Application window mode",
        choices=["regular", "fullscreen", "headless"],
        type=str
    )
    parser.add_argument(
        "--rviz-window-style",
        dest="rviz_window_style",
        default="kvantum",
        help="QT supported styles",
        type=str
    )
    parser.add_argument(
        "--stream-host",
        dest="stream_host",
        default="127.0.0.1",
        help="Screen stream host",
        type=str
    )
    parser.add_argument(
        "--stream-port",
        dest="stream_port",
        default=554,
        help="Screen stream port",
        type=int
    )
    parser.add_argument(
        "--stream-uri",
        dest="stream_uri",
        default="",
        help="Screen stream uri",
        type=str
    )
    parser.add_argument(
        "--stream-gst-pipeline",
        dest="stream_gst_pipeline",
        default="",
        help="Screen stream gst pipelines config",
        type=str
    )
    return parser


def main():
    parser = argparse.ArgumentParser(
        description="Tool for rviz docker environment manipulation")
    parser.set_defaults(func=lambda *args: parser.print_usage())

    config = cfg.get_default_config()
    config.update(cfg.read_env_vars_config())
    config.update(cfg.safe_load_config(cfg.PATH_TO_CUSTOM_CONFIG, json.load))

    subparsers = parser.add_subparsers(dest="subparser_name")

    # build image
    parser_build_image = subparsers.add_parser(
        "build-image",
        help="Build docker image"
    )
    parser_build_image = add_build_image_subcommand_args(
        parser_build_image, config)
    parser_build_image.set_defaults(func=build_image)

    # start container
    parser_start = subparsers.add_parser(
        "start",
        aliases=["u"],
        help="Start a container"
    )
    parser_start = add_start_subcommand_args(parser_start, config)
    parser_start.set_defaults(func=start_container)

    # stop container
    parser_stop = subparsers.add_parser(
        "stop",
        aliases=["s"],
        help="Stop the container"
    )
    parser_stop = add_stop_subcommand_args(parser_stop, config)
    parser_stop.set_defaults(func=stop_container)

    # attach to container
    parser_attach = subparsers.add_parser(
        "attach",
        help="Run an interactive session",
        aliases=["a"]
    )
    parser_attach = add_attach_subcommand_args(parser_attach, config)
    parser_attach.set_defaults(func=attach_to_container)

    # build all
    parser_build_all = subparsers.add_parser(
        "build-all",
        help="Build docker image, start container and build code inside"
    )
    parser_build_all = add_build_all_subcommand_args(parser_build_all, config)
    parser_build_all.set_defaults(func=build_all_command)

    # run app
    parser_run_app = subparsers.add_parser(
        "run",
        help="Run rviz app"
    )
    parser_run_app = add_run_app_subcommand_args(parser_run_app, config)
    parser_run_app.set_defaults(func=run_app)

    argcomplete.autocomplete(parser)
    args, argv = parser.parse_known_args()
    args.func(args, argv, config)


if __name__ == "__main__":
    main()
