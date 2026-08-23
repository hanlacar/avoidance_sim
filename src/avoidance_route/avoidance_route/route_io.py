"""CSV helpers compatible with mission_manager's version-1 route loader."""

import csv
from datetime import datetime
import math
from pathlib import Path


CSV_HEADER = ('index', 'timestamp', 'latitude', 'longitude', 'x_m', 'y_m',
              'yaw', 'direction', 'mode', 'drive_level')


def non_overwriting_path(requested):
    path = Path(requested).expanduser().resolve()
    if not path.exists() and not path.with_suffix('.yaml').exists():
        return path
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    candidate = path.with_name(f'{path.stem}_{stamp}{path.suffix}')
    sequence = 1
    while candidate.exists() or candidate.with_suffix('.yaml').exists():
        candidate = path.with_name(f'{path.stem}_{stamp}_{sequence}{path.suffix}')
        sequence += 1
    return candidate


def load_path_points(path):
    points = []
    csv_path = Path(path).expanduser()
    if not csv_path.is_file():
        return points
    with csv_path.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not {'x_m', 'y_m'}.issubset(reader.fieldnames):
            raise ValueError('CSV must contain x_m and y_m')
        for row in reader:
            x, y = float(row['x_m']), float(row['y_m'])
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError('CSV contains non-finite coordinates')
            points.append((x, y))
    return points
