import json
from pathlib import Path
import subprocess


PACKAGE = Path(__file__).resolve().parents[1]


def normalized(text: str) -> str:
    return "\n".join(line.rstrip() for line in text.splitlines()).rstrip() + "\n"


def test_installed_interfaces_match_frozen_ros2_show_snapshots():
    manifest = json.loads(
        (PACKAGE / "contract/interface_manifest.json").read_text()
    )
    expected_types = {
        f"robots_dog_msgs/{item['logical_name']}" for item in manifest["interfaces"]
    }
    listed = subprocess.run(
        ["ros2", "interface", "list"], check=True, capture_output=True, text=True
    ).stdout.splitlines()
    actual_types = {name.strip() for name in listed if "robots_dog_msgs/" in name}
    assert actual_types == expected_types

    snapshot_root = PACKAGE / "contract/ros2_interface_show"
    for type_name in sorted(expected_types):
        logical_name = type_name.removeprefix("robots_dog_msgs/")
        kind, name = logical_name.split("/", 1)
        expected = (snapshot_root / f"{kind}__{name}.txt").read_text()
        actual = subprocess.run(
            ["ros2", "interface", "show", type_name],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        assert normalized(actual) == normalized(expected), type_name
