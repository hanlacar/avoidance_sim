"""Obstacle layout generation with reproducible tests and repeat protection."""

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
import random

import yaml


@dataclass(frozen=True)
class ObstacleLayout:
    left_x: float
    right_x: float
    first_side: str
    generated_at: str
    seed: int

    @property
    def ordered(self):
        if self.left_x < self.right_x:
            return ((self.left_x, 0.780), (self.right_x, -0.780))
        return ((self.right_x, -0.780), (self.left_x, 0.780))


def default_state_path():
    return Path.home() / '.local/state/avoidance_sim_ws/last_obstacle_layout.yaml'


def _draw(rng, seed):
    spacing_um = rng.randint(10_000_000, 11_350_000)
    first_x_um = rng.randint(9_325_000, 20_675_000-spacing_um)
    second_x_um = first_x_um+spacing_um
    # SystemRandom produces uniformly distributed seeds; using their low bit
    # keeps the 50:50 side decision independent from the longitudinal draws.
    first_side = 'LEFT' if seed & 1 else 'RIGHT'
    left_x, right_x = ((first_x_um, second_x_um) if first_side == 'LEFT'
                       else (second_x_um, first_x_um))
    return ObstacleLayout(
        left_x/1_000_000.0, right_x/1_000_000.0, first_side,
        datetime.now(timezone.utc).isoformat(), seed)


def load_layout(path):
    try:
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        return ObstacleLayout(
            float(data['left_x']), float(data['right_x']),
            str(data['first_side']), str(data['generated_at']),
            int(data['seed']))
    except (OSError, TypeError, KeyError, ValueError, yaml.YAMLError):
        return None


def same_positions(first, second, tolerance=1.0e-4):
    return (first is not None and
            math.isclose(first.left_x, second.left_x, abs_tol=tolerance) and
            math.isclose(first.right_x, second.right_x, abs_tol=tolerance) and
            first.first_side == second.first_side)


def generate_layout(seed=-1, state_path=None, tolerance=1.0e-4):
    """Generate a layout; only system-random runs persist/avoid the last one."""
    seed = int(seed)
    if seed >= 0:
        return _draw(random.Random(seed), seed)
    path = Path(state_path) if state_path else default_state_path()
    previous = load_layout(path)
    system_rng = random.SystemRandom()
    for _ in range(100):
        actual_seed = system_rng.randrange(0, 2**63)
        layout = _draw(random.Random(actual_seed), actual_seed)
        if not same_positions(previous, layout, tolerance):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(yaml.safe_dump(asdict(layout), sort_keys=True),
                            encoding='utf-8')
            return layout
    raise RuntimeError('could not generate a layout distinct from previous run')
