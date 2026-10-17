"""Run the configured Gazebo navigation stack under one supervisor."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml

from qwen_robotnav.runtime_profile import (
    configure_navigation,
    load_profile,
    profile_hash,
    ros_parameters,
    validate_profile,
)


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--navigation-dir", type=Path, required=True)
    parser.add_argument("--expected-domain", type=int, required=True)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument(
        "--candidate-selector",
        choices=("deterministic", "deepseek"),
        default=None,
        help="Choose how G2 ranks its map-derived safe frontier candidates.",
    )
    parser.add_argument("--mission-instruction")
    parser.add_argument(
        "--brain-base-url",
        default=os.environ.get(
            "MISSION_BRAIN_BASE_URL",
            os.environ.get("DEEPSEEK_VLM_BASE_URL"),
        ),
    )
    parser.add_argument(
        "--brain-model",
        default=os.environ.get(
            "MISSION_BRAIN_MODEL",
            os.environ.get("DEEPSEEK_VLM_MODEL"),
        ),
    )
    parser.add_argument("--brain-timeout", type=float)
    parser.add_argument("--brain-max-candidates", type=int)
    return parser


def _validate_environment(parser, args, profile, root):
    if (
        args.expected_domain in (0, 42, 43)
        or not 1 <= args.expected_domain <= 232
        or os.environ.get("ROS_DOMAIN_ID") != str(args.expected_domain)
        or not os.environ.get("IGN_PARTITION")
    ):
        parser.error("Require matching isolated simulator domain and Gazebo partition")
    if not (root / "slam_navigation.launch.py").is_file():
        parser.error("Prepared navigation directory is missing slam_navigation.launch.py")
    if (root / "runtime_manifest.json").exists():
        parser.error("Require a fresh prepared navigation directory")
    if args.candidate_selector != "deepseek":
        return
    if not (
        os.environ.get("MISSION_BRAIN_API_KEY")
        or os.environ.get("DEEPSEEK_API_KEY")
        or os.environ.get("VLLM_API_KEY")
    ):
        parser.error("DeepSeek candidate selection requires an API key")


def _check_ros_graph(parser, actuator_topic):
    import rclpy

    rclpy.init(args=[])
    probe = rclpy.create_node("runtime_startup_probe")
    try:
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=0.05)
        reserved = {
            "velocity_guard",
            "g3_state_machine",
            "vision_explorer",
            "cmd_vel_scaler",
            "controller_server",
            "g3_sim_battery",
            "mission_brain",
            "qwen_robotnav_nav_node",
            "deepseek_nav2_waypoint_bridge",
            "qwen_robotnav_go2_controller",
            "go2_ros_bridge",
        }
        occupied = {
            name for name, _namespace in probe.get_node_names_and_namespaces()
        } & reserved
        publishers = probe.get_publishers_info_by_topic(actuator_topic)
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    if occupied:
        parser.error("Runtime nodes already exist: " + ", ".join(sorted(occupied)))
    if publishers:
        owners = sorted(
            {f"{info.node_namespace.rstrip('/')}/{info.node_name}" for info in publishers}
        )
        parser.error(
            f"Final actuator topic {actuator_topic} already has publishers: "
            + ", ".join(owners)
        )


def _resolve_decision(args, profile):
    # Resolve all overrides into the snapshot before hashing or generating params.
    for name in profile['decision']:
        supplied=getattr(args,name)
        if supplied is not None:profile['decision'][name]=supplied
        else:setattr(args,name,profile['decision'][name])
    validate_profile(profile)


def _stop_child(child, first_signal, first_timeout, second_timeout):
    if child.poll() is not None:
        return
    try:
        os.killpg(child.pid, first_signal)
    except ProcessLookupError:
        return
    try:
        child.wait(timeout=first_timeout)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(child.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        child.wait(timeout=second_timeout)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGKILL)
        child.wait()


def main():
    parser = _parser()
    args = parser.parse_args()
    profile = load_profile(args.profile)
    _resolve_decision(args, profile)
    root = args.navigation_dir.resolve()
    _validate_environment(parser, args, profile, root)
    _check_ros_graph(parser, profile["topics"]["actuator_command"])

    configure_navigation(root, profile)
    snapshot = root / "runtime_profile.yaml"
    snapshot.write_text(yaml.safe_dump(profile, sort_keys=False))
    runtime_parameters = ros_parameters(profile, root)
    params = root / "runtime_ros_params.yaml"
    params.write_text(yaml.safe_dump(runtime_parameters, sort_keys=False))

    project = Path(__file__).resolve().parents[4]
    nav = Path(__file__).resolve().parents[1]
    child_environment = dict(os.environ)
    if args.candidate_selector == "deepseek":
        child_environment.update(
            MISSION_BRAIN_API_KEY=(
                os.environ.get("MISSION_BRAIN_API_KEY")
                or os.environ.get("DEEPSEEK_API_KEY")
                or os.environ.get("VLLM_API_KEY")
            ),
            MISSION_BRAIN_BASE_URL=args.brain_base_url.rstrip("/"),
            MISSION_BRAIN_MODEL=args.brain_model,
            MISSION_BRAIN_TIMEOUT_S=str(args.brain_timeout),
        )
    env = dict(
        child_environment,
        PYTHONPATH=":".join(
            [
                str(nav),
                str(nav / "ros2_ws/src/qwen_robotnav_ros"),
                str(project / "baseline/g3_integration_eval"),
                str(project / "baseline/g2_vision_explore"),
                str(project / "baseline/g2_vision_explore/vision_explorer"),
                os.environ.get("PYTHONPATH", ""),
            ]
        ),
    )
    remaps = [
        "-r",
        "/tf:=" + profile["topics"]["tf"],
        "-r",
        "/tf_static:=" + profile["topics"]["tf_static"],
    ]
    ros = ["--ros-args", "--params-file", str(params), *remaps]
    commands = [
        (
            "guard",
            [
                "python3",
                "-m",
                "qwen_robotnav_ros.velocity_guard",
                "--profile",
                str(snapshot),
                *(["--enable-motion"] if args.enable_motion else []),
                "--ros-args",
                "-p",
                "use_sim_time:=true",
                *remaps,
            ],
        ),
        ("battery", ["python3", "-m", "g3_integration_eval.sim_battery_node", *ros]),
        (
            "scaler",
            [
                "python3",
                str(project / "baseline/g1_mapping_nav/tools/cmd_vel_scaler.py"),
                *ros,
            ],
        ),
        ("g3", ["python3", "-m", "g3_integration_eval.state_machine_node", *ros]),
        ("navigation", ["ros2", "launch", str(root / "slam_navigation.launch.py")]),
    ]
    if args.candidate_selector == "deepseek":
        commands.append(
            ("mission_brain", ["python3", "-m", "g3_integration_eval.mission_brain_node", *ros])
        )
    commands.append(
        ("frontier", ["python3", "-m", "vision_explorer.explorer_node", *ros])
    )

    children = []
    streams = []
    code = 0

    def interrupted(_signal, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    manifest = {
        "profile_sha256": profile_hash(profile),
        "backend": profile["backend"],
        "motion_enabled": args.enable_motion,
        "goal_source": "g2_frontier",
        "candidate_selector": args.candidate_selector,
        "mission_instruction": args.mission_instruction,
        "domain": args.expected_domain,
        "truth_used_by_runtime": False,
        "velocity_chain": [
            profile["topics"]["nav_command"],
            profile["topics"]["guarded_input"],
            profile["topics"]["actuator_command"],
        ],
        "children": [],
    }
    if args.candidate_selector == "deepseek":
        manifest["mission_brain"] = {
            "base_url": args.brain_base_url,
            "model": args.brain_model,
            "api_key_recorded": False,
        }
    try:
        for name, command in commands:
            stream = (root / ("runtime_" + name + ".log")).open("w")
            streams.append(stream)
            child = subprocess.Popen(
                command,
                env=env,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            children.append((name, child))
            manifest["children"].append(
                {"name": name, "pid": child.pid, "command": command}
            )
        (root / "runtime_manifest.json").write_text(json.dumps(manifest, indent=2))
        while True:
            for name, child in children:
                if child.poll() is not None:
                    raise RuntimeError(
                        name + " exited unexpectedly: " + str(child.returncode)
                    )
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        code = 1
        manifest["error"] = str(exc)
        print(str(exc), flush=True)
    finally:
        # Goal owners and Nav2 release their work before the sole actuator
        # publisher sends its final zero and exits.
        for name, child in reversed(children):
            if name != "guard":
                _stop_child(child, signal.SIGINT, 10, 3)
        for name, child in children:
            if name == "guard":
                _stop_child(child, signal.SIGINT, 5, 3)
        if any(child.returncode != 0 for name, child in children):
            code = 1
        manifest["exit_code"] = code
        manifest["child_exit_codes"] = {
            name: child.returncode for name, child in children
        }
        (root / "runtime_exit.json").write_text(json.dumps(manifest, indent=2))
        for stream in streams:
            stream.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
