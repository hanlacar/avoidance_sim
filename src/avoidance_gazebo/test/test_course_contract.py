from pathlib import Path
import re


ROOT = Path(__file__).parents[1]


def test_launch_defaults_to_obstacle_free_mode():
    launch = (ROOT / 'launch' / 'straight_avoidance.launch.py').read_text()
    assert "'spawn_obstacles', default_value='false'" in launch
    assert 'REFERENCE RECORDING MODE: OBSTACLES DISABLED' in launch
    assert "for name in ('obstacle_1', 'obstacle_2')" in launch


def test_world_has_expected_obstacle_templates():
    world = (ROOT / 'worlds' / 'straight_avoidance.sdf').read_text()
    assert len(re.findall(r'<model name="obstacle_[12]">', world)) == 2
    assert '<size>0.65 0.39 0.15</size>' in world


def test_launch_preserves_vehicle_and_sensor_contracts():
    launch = (ROOT / 'launch' / 'straight_avoidance.launch.py').read_text()
    for expected in ("'-x', '3.25'", "'/scan_front@", "'/scan_rear@",
                     "'/odom@", "'/cmd_vel@"):
        assert expected in launch
