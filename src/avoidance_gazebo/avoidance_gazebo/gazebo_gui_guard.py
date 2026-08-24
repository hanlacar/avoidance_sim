"""Start the Gazebo GUI after the server scene is complete and retry failures."""

import argparse
import signal
import subprocess
import sys
import time


def _run_listing(command, timeout=4.0):
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout,
            check=False)
    except (OSError, subprocess.SubprocessError):
        return ()
    if result.returncode != 0:
        return ()
    return tuple(line.strip() for line in result.stdout.splitlines() if line.strip())


def _gui_transport_ready(world_name):
    services = _run_listing(['gz', 'service', '-l'])
    topics = _run_listing(['gz', 'topic', '-l'])
    scene_ready = f'/world/{world_name}/scene/info' in services
    # These endpoints are created by the GUI process, not by a server-only
    # `gz sim -s` process. Requiring one prevents a live-but-windowless child
    # from being accepted merely because the world server is healthy.
    gui_ready = any(
        endpoint == '/gui/camera/pose' or endpoint.startswith('/gui/')
        for endpoint in (*services, *topics))
    return scene_ready and gui_ready


def _stop_child(child):
    if child.poll() is not None:
        return
    child.send_signal(signal.SIGINT)
    try:
        child.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        child.terminate()
        try:
            child.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=2.0)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True)
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--startup-timeout', type=float, default=20.0)
    # launch_ros appends ``--ros-args`` even though this guard is deliberately
    # transport-only. Ignore those launch plumbing arguments.
    args, _unknown = parser.parse_known_args(argv)
    if args.attempts < 1 or args.startup_timeout <= 0.0:
        parser.error('attempts and startup-timeout must be positive')

    child = None
    shutting_down = False

    def _forward_signal(_signum, _frame):
        nonlocal shutting_down
        shutting_down = True
        # Signal handlers must not wait: launch may deliver SIGTERM while an
        # earlier SIGINT handler is still blocked, which used to deadlock the
        # guard and require SIGKILL.  The normal loop performs bounded cleanup.
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, _forward_signal)
    signal.signal(signal.SIGTERM, _forward_signal)

    for attempt in range(1, args.attempts + 1):
        print(
            f'[facility_gui] START attempt={attempt}/{args.attempts} '
            f'world={args.world}', flush=True)
        try:
            child = subprocess.Popen(
                ['gz', 'sim', '-g', '--force-version', '8'])
        except OSError as error:
            print(f'[facility_gui] launch failed: {error}', file=sys.stderr,
                  flush=True)
            continue

        deadline = time.monotonic() + args.startup_timeout
        while not shutting_down and time.monotonic() < deadline:
            returncode = child.poll()
            if returncode is not None:
                print(
                    f'[facility_gui] early exit code={returncode} '
                    f'attempt={attempt}', file=sys.stderr, flush=True)
                break
            if _gui_transport_ready(args.world):
                print(
                    f'[facility_gui] VERIFIED transport_ready=true '
                    f'pid={child.pid} attempt={attempt}', flush=True)
                while child.poll() is None and not shutting_down:
                    time.sleep(0.25)
                if shutting_down:
                    _stop_child(child)
                    return 0
                returncode = child.returncode
                if returncode == 0:
                    return 0
                print(
                    f'[facility_gui] unexpected exit code={returncode}; retrying',
                    file=sys.stderr, flush=True)
                break
            time.sleep(0.25)
        else:
            if shutting_down:
                return 0
            print(
                f'[facility_gui] readiness timeout after '
                f'{args.startup_timeout:.1f}s; retrying',
                file=sys.stderr, flush=True)

        _stop_child(child)
        child = None

    print(
        f'[facility_gui] FAILED after {args.attempts} attempts; '
        'no verified Gazebo GUI transport endpoint',
        file=sys.stderr, flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
