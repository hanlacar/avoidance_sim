"""Exclusive ownership of the simulation's /cmd_vel command authority."""

import fcntl
import os
from pathlib import Path


LOCK_PATH = Path('/tmp/avoidance_sim_cmd_vel.lock')


class CommandAuthorityError(RuntimeError):
    pass


class CommandAuthority:
    def __init__(self, owner):
        self.owner = owner
        self.stream = LOCK_PATH.open('a+', encoding='utf-8')
        try:
            fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.stream.seek(0)
            active = self.stream.read().strip() or 'unknown command node'
            self.stream.close()
            raise CommandAuthorityError(
                f'/cmd_vel authority is already owned by {active}') from exc
        self.stream.seek(0)
        self.stream.truncate()
        self.stream.write(f'{owner} pid={os.getpid()}')
        self.stream.flush()

    def close(self):
        if not self.stream.closed:
            fcntl.flock(self.stream.fileno(), fcntl.LOCK_UN)
            self.stream.close()
