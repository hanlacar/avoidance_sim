import math

from avoidance_lidar.processing import cluster_points, valid_roi_points


def test_filter_rejects_invalid_outside_roi_and_self_points():
    ranges = [math.inf, math.nan, 0.05, 0.5, 2.0]
    points = valid_roi_points(
        ranges, -0.2, 0.1, 0.12, 12.0, 0.12, 12.0,
        0.12, 5.0, 1.0, -1.0, 0.1, 0.45)
    assert [point.index for point in points] == [3, 4]


def test_clusters_do_not_bridge_missing_scan_indices():
    points = valid_roi_points(
        [1.0, 1.0, math.inf, 1.0, 1.0], -0.02, 0.01,
        0.12, 12.0, 0.12, 12.0, 0.12, 5.0, 1.0,
        -1.0, 0.1, 0.45)
    assert len(cluster_points(points, 0.18, 2)) == 2


def test_minimum_cluster_size_removes_single_point_noise():
    points = valid_roi_points(
        [1.0, math.inf, 1.0, 1.0, 1.0], -0.02, 0.01,
        0.12, 12.0, 0.12, 12.0, 0.12, 5.0, 1.0,
        -1.0, 0.1, 0.45)
    clusters = cluster_points(points, 0.18, 3)
    assert len(clusters) == 1
    assert len(clusters[0].points) == 3
