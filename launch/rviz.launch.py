import os
import yaml
import subprocess as sp
import re

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, DeclareLaunchArgument, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration

SDA_REP_ROOT = os.environ["REP_ROOT"]
DISPLAY = os.environ["DISPLAY"]


def get_rviz_config(config_path):
    with open(config_path) as file:
        return yaml.load(file, Loader=yaml.SafeLoader)


def run_streamer(context, *args, **kwargs):
    stream_uri = LaunchConfiguration("stream-uri").perform(context)
    stream_host = LaunchConfiguration("stream-host").perform(context)
    stream_port = LaunchConfiguration("stream-port").perform(context)
    pipeline = LaunchConfiguration("stream-gst-pipeline").perform(context)
    if not pipeline:
        display_config = LaunchConfiguration("display-config").perform(context)
        window_mode = LaunchConfiguration("window-mode").perform(context)
        if window_mode == "fullscreen":
            pipeline = f'"( ximagesrc startx=0 starty=0 ! video/x-raw,format=BGRx,framerate=30/1 ! videoconvert ! video/x-raw,format=I420 ! queue ! x265enc tune=zerolatency speed-preset=2 bitrate=4500 ! video/x-h265, stream-format=byte-stream, alignment=au, profile=main ! rtph265pay name=pay0 pt=96 )"'
        else:
            window_name = f"{display_config} - RViz"
            pipeline = f'"( ximagesrc xname=\"{window_name}\" ! video/x-raw,format=BGRx,framerate=30/1 ! videoconvert ! video/x-raw,format=I420 ! queue ! x265enc tune=zerolatency speed-preset=2 bitrate=4500 ! video/x-h265, stream-format=byte-stream, alignment=au, profile=main ! rtph265pay name=pay0 pt=96 )"'

    # gst-rtsp "( ximagesrc xname='/tmp/flip_interior_screen.rviz - RViz' startx=1 starty=27 endx=1920 endy=387 ! videocrop top=27 bottom=747 ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=I420 ! x265enc tune=zerolatency speed-preset=2 bitrate=4500 ! video/x-h265, stream-format=byte-stream, alignment=au, profile=main ! rtph265pay name=pay0 pt=96 )" -a 127.0.0.1 -p 8888 -m /inner

    return [ExecuteProcess(
        name="streamer",
        cmd=[
            "gst-rtsp", pipeline,
            "-a", stream_host,
            "-p", stream_port,
            "-m", stream_uri,
        ],
        sigterm_timeout="60",
        sigkill_timeout="60",
    )]


def launch_setup(context, *args, **kwargs):
    # setup and launch RViz
    display_config = LaunchConfiguration("display-config").perform(context)
    window_mode = LaunchConfiguration("window-mode").perform(context)
    window_style = LaunchConfiguration("window-style").perform(context)
    run_rviz_action = ExecuteProcess(
        name="rviz",
        cmd=[
            f"{SDA_REP_ROOT}/install/rviz2/bin/rviz2",
            "--display-config", display_config,
            "--window-mode", window_mode,
            "-style", window_style,
        ],
        sigterm_timeout="60",
        sigkill_timeout="60",
        respawn=True,
        respawn_delay=1,
        on_exit=Shutdown(reason="Rviz shutdown"),
    )
    ld = [run_rviz_action]

    # setup and launch Rviz video stream
    stream_uri = LaunchConfiguration("stream-uri").perform(context)
    if stream_uri:
        ld.append(
            TimerAction(period=2.0, actions=[
                OpaqueFunction(function=run_streamer)])
        )

    return ld


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "display-config",
                description="Path to a rviz config",
                default_value="None"
            ),
            DeclareLaunchArgument(
                "window-mode",
                description="Rviz window mode",
                default_value="headless",
                choices=["regular", "fullscreen", "headless"]
            ),
            DeclareLaunchArgument(
                "window-style",
                description="Rviz window style",
                default_value="kvantum",
            ),
            DeclareLaunchArgument(
                "stream-host",
                description="Screen stream host",
                default_value="127.0.0.1",
            ),
            DeclareLaunchArgument(
                "stream-port",
                description="Screen stream port",
                default_value="554",
            ),
            DeclareLaunchArgument(
                "stream-uri",
                description="Screen stream uri",
                default_value="",
            ),
            DeclareLaunchArgument(
                "stream-gst-pipeline",
                description="Screen stream gst pipelines config",
                default_value="",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
