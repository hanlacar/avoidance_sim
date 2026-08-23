import csv
from pathlib import Path

from avoidance_route.route_io import CSV_HEADER, load_path_points, non_overwriting_path


def test_header_preserves_mission_manager_required_columns():
    required = {'index', 'latitude', 'longitude', 'x_m', 'y_m',
                'direction', 'mode', 'drive_level'}
    assert required.issubset(CSV_HEADER)
    assert {'timestamp', 'yaw'}.issubset(CSV_HEADER)


def test_existing_route_is_not_overwritten(tmp_path):
    requested = tmp_path / 'route.csv'
    requested.write_text('existing', encoding='utf-8')
    assert non_overwriting_path(requested) != requested


def test_load_path_points(tmp_path):
    route = Path(tmp_path) / 'route.csv'
    with route.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_HEADER)
        writer.writeheader()
        writer.writerow({'x_m': '1.0', 'y_m': '2.0'})
    assert load_path_points(route) == [(1.0, 2.0)]
