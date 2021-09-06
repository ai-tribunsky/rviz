import itertools
import logging
import re
import socket
import subprocess
import sys
import types
import typing as tp

try:
    import docker
    import dockerpty
    import dockerpty.io as dockerpty_io
    from docker.errors import DockerException
    from docker.models import containers
    from docker.utils import create_archive, parse_env_file
except ImportError as err:
    print(
        f"{err}, please install sat-host-utils package:\n"
        "    https://wiki.sberautotech.ru/pages/viewpage.action?pageId=36473905\n"
        "    or use configure script\n"
        "    ./configure",
        file=sys.stderr,
    )
    exit()


class DockerImplException(Exception):
    @staticmethod
    def wrap(func):
        def decorator(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except (DockerException, subprocess.SubprocessError) as err:
                raise DockerImplException(err) from err

        return decorator


def docker_stream_pp(stream: bytes) -> str:
    return stream.decode("utf-8").rstrip("\n")


class ContainerImpl:
    def __init__(self, containter: containers.Container):
        self.containter: containers.Container = containter
        self.logger = logging.getLogger(f"docker.{self.containter.name}")

    @DockerImplException.wrap
    def exec_run(self, cmd, detach: bool = False, **kwargs) -> None:
        cmd = list(map(str, cmd))
        command_pp = " ".join(cmd)
        logger = self.logger.getChild(f"[{command_pp}]")
        tty = kwargs.get("tty", False)
        exec_id = self.containter.client.api.exec_create(
            self.containter.name, cmd, **kwargs)["Id"]
        stream = self.containter.client.api.exec_start(
            exec_id, detach=detach, tty=tty, stream=not tty, socket=tty
        )
        # stream
        if isinstance(stream, types.GeneratorType):
            for line in stream:
                logger.info(docker_stream_pp(line))
        # socket
        elif isinstance(stream, socket.SocketIO):
            operation = dockerpty.ExecOperation(
                self.containter.client.api, exec_id)
            dockerpty.PseudoTerminal(self.containter.client.api, operation).start(
                dockerpty_io.Stream(stream)
            )
        # detach
        else:
            return

        exit_code = self.containter.client.api.exec_inspect(exec_id)[
            "ExitCode"]
        if exit_code:
            raise DockerImplException(f"{command_pp} returns {exit_code}")

    @DockerImplException.wrap
    def copy(self, source: str, destination: str) -> None:
        with create_archive(source) as tar:
            succeded = self.containter.put_archive(destination, tar)
            if not succeded:
                raise DockerImplException(
                    f"Error on copying {source} to {destination}")

    @DockerImplException.wrap
    def stop(self, timeout: int) -> None:
        self.containter.stop(timeout=timeout)
        self.logger.info(f"Container '{self.containter.name}' stopped")


class DockerImpl:
    def __init__(self):
        self.logger = logging.getLogger("docker")
        self.client = docker.from_env()

    @DockerImplException.wrap
    def build(self, additional_args=list(), **kwargs) -> None:
        if additional_args:
            self.__build_using_subprocess(additional_args, **kwargs)
        else:
            self.__build_using_docker_api(**kwargs)

    def __build_using_docker_api(self, **kwargs) -> None:
        logger = self.logger.getChild(f'build.{kwargs.get("tag", "image")}')
        kwargs["decode"] = True
        for line in self.client.api.build(**kwargs):
            for key, val in line.items():
                val = re.sub(r"\s+", " ", str(val).rstrip("\n"))
                if key in ("stream"):
                    logger.info("%s", val)
                elif key in ("error"):
                    logger.error("%s", val)

    def __build_using_subprocess(self, additional_args, **kwargs) -> None:
        path = kwargs.pop("path")
        cmd = (
            ["docker", "build"] +
            ApiToShellArgsConvertor.convert(kwargs) + additional_args + [path]
        )
        self.logger.info("Running cmd: %s", " ".join(cmd))
        subprocess.check_call(cmd)

    @DockerImplException.wrap
    def run(self, additional_args=list(), **kwargs) -> None:
        if additional_args:
            self.__run_using_subprocess(additional_args, **kwargs)
        else:
            self.__run_using_docker_api(**kwargs)

    def __run_using_docker_api(self, **kwargs) -> None:
        # There is no direct support for --gpus flag in python-docker API
        gpus = kwargs.pop("gpus", False)
        if gpus:
            kwargs.setdefault("device_requests", list()).append(
                {
                    "Driver": "",
                    "Capabilities": [["gpu"]],
                    "Count": -1 if gpus == "all" else int(gpus),
                }
            )
        # There is no direct support for env-files in python-docker API
        environment_files = kwargs.pop("environment_files", list())
        for env_file_path in environment_files:
            kwargs.setdefault("environment", dict()).update(
                parse_env_file(env_file_path))
        self.client.containers.run(**kwargs)

    def __run_using_subprocess(self, additional_args, **kwargs) -> None:
        image_name = kwargs.pop("image")
        cmd = (
            ["docker", "run"]
            + ApiToShellArgsConvertor.convert(kwargs)
            + additional_args
            + [image_name]
        )
        self.logger.info("Running cmd: %s", " ".join(cmd))
        subprocess.check_call(cmd)

    @DockerImplException.wrap
    def get_container(self, container_id: str) -> ContainerImpl:
        return ContainerImpl(self.client.containers.get(container_id))

    @DockerImplException.wrap
    def is_container_running(self, name: str) -> bool:
        return bool(self.client.containers.list(filters={"name": name, "status": "running"}))


class ApiToShellArgsConvertor:
    DOCKER_API_ARGS_TO_BASH_MAP: tp.Dict[str, tp.Callable[[tp.Any], tp.Iterable[str]]] = {
        "auto_remove": lambda val: ["--rm"] if val else [],
        "buildargs": lambda vals: itertools.chain.from_iterable(
            [["--build-arg", f"{key}={val}"] for key, val in vals.items()]
        ),
        "dockerfile": lambda val: ["-f", val] if val else [],
        "detach": lambda val: ["-d"] if val else [],
        "environment": lambda vals: itertools.chain.from_iterable(
            [["-e", f"{key}={val}"] for key, val in vals.items()]
        ),
        "environment_files": lambda vals: itertools.chain.from_iterable(
            [["--env-file", val] for val in vals]
        ),
        "gpus": lambda val: ["--gpus", val] if val is not False else [],
        "ipc_mode": lambda val: ["--ipc", val],
        "name": lambda val: ["--name", val],
        "network_mode": lambda val: ["--network", val],
        "pid_mode": lambda val: ["--pid", val],
        "privileged": lambda val: ["--privileged"] if val else [],
        "security_opt": lambda vals: itertools.chain.from_iterable(
            [["--security-opt", val] for val in vals]
        ),
        "stdin_open": lambda val: ["-i"] if val else [],
        "tag": lambda val: ["--tag", val],
        "target": lambda val: ["--target", val],
        "tty": lambda val: ["-t"] if val else [],
        "volumes": lambda vals: itertools.chain.from_iterable([["-v", val] for val in vals]),
        "user": lambda val: ["--user", val],
    }

    @staticmethod
    def convert(kwargs: tp.Dict[str, tp.Any]) -> tp.List[str]:
        cmd: tp.List[str] = list()
        for key, value in kwargs.items():
            if key in ApiToShellArgsConvertor.DOCKER_API_ARGS_TO_BASH_MAP:
                cmd.extend(
                    ApiToShellArgsConvertor.DOCKER_API_ARGS_TO_BASH_MAP[key](value))
            else:
                raise DockerImplException(f"There is no rule for key {key}")
        return cmd
