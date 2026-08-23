"""Single-simulator guard for a ROS domain.

The advisory lock prevents two launches that share a domain from publishing
different Gazebo epochs on /clock.  The PID text is diagnostic only; flock is
the authority, so a stale file from a dead process is recovered safely.
"""

import fcntl
import os
from pathlib import Path


_HELD_LOCKS = []


def _runtime_directory():
    configured = os.environ.get('XDG_RUNTIME_DIR', '')
    if configured:
        path = Path(configured)
        if path.is_dir() and os.access(path, os.W_OK):
            return path
    return Path('/tmp')


def _ancestor_pids():
    ancestors = {os.getpid()}
    pid = os.getppid()
    while pid > 1 and pid not in ancestors:
        ancestors.add(pid)
        try:
            fields = (Path('/proc') / str(pid) / 'stat').read_text().split()
            pid = int(fields[3])
        except (OSError, ValueError, IndexError):
            break
    return ancestors


def _legacy_domain_conflicts(domain_id):
    """Find pre-lock simulations, including launches made by older code."""
    conflicts = []
    ignored = _ancestor_pids()
    markers = (
        'ros2 launch avoidance_gazebo',
        'parameter_bridge /cmd_vel',
        '/avoidance_route/route_follower',
        '/avoidance_planner/avoidance_coordinator',
        'gz sim ',
    )
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) in ignored:
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(
                errors='replace')
            if not any(marker in command for marker in markers):
                continue
            environment = (entry / 'environ').read_bytes().split(b'\0')
            values = dict(item.split(b'=', 1) for item in environment if b'=' in item)
            process_domain = values.get(b'ROS_DOMAIN_ID', b'0').decode(errors='replace')
            if process_domain == str(domain_id):
                conflicts.append((int(entry.name), command.strip()))
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            continue
    return sorted(conflicts)


def acquire_simulation_lock(domain_id):
    path = _runtime_directory() / f'avoidance_sim_ros_domain_{domain_id}.lock'
    stream = path.open('a+', encoding='utf-8')
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        stream.seek(0)
        owner = stream.read().strip() or 'unknown live launch process'
        stream.close()
        raise RuntimeError(
            f'ROS domain {domain_id} already has an avoidance simulation: {owner}. '
            'Stop the existing launch before starting another one.') from exc
    stream.seek(0)
    stream.truncate()
    stream.write(f'pid={os.getpid()} workspace={Path.cwd()}')
    stream.flush()
    conflicts = _legacy_domain_conflicts(domain_id)
    if conflicts:
        details = '; '.join(f'pid={pid} {command}' for pid, command in conflicts)
        fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        stream.close()
        raise RuntimeError(
            f'ROS domain {domain_id} already contains simulator/control processes: '
            f'{details}. Stop the existing launch before starting another one.')
    _HELD_LOCKS.append(stream)
    return path
