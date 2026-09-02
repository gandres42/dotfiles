from rclone_python import rclone
import socket
import schedule
import time
from threading import Lock
import hashlib
from dirhash import dirhash
import os

BACKUP_FOLDERS = ["Desktop", "Documents", "Music", "Obsidian", "Pictures", "Videos"]

HOSTNAME = socket.gethostname()
SYNC_LOCK = Lock()
LAST_HASH = None
FILTERS = []
for folder in BACKUP_FOLDERS:
    FILTERS.append('--filter')
    FILTERS.append(f'"+ /{folder}/**"')
FILTERS.append('--filter')
FILTERS.append('"- *"')

def home_hash(algorithm="sha1", jobs=None):
    combined = hashlib.new(algorithm)
    for folder in BACKUP_FOLDERS:
        try:
            folder_hash = dirhash(
                os.path.join('/home/gavin', folder),
                algorithm,
                linked_dirs=False,
                allow_cyclic_links=True,
                jobs=jobs or os.cpu_count(),
            )
        except (FileNotFoundError, ValueError):
            folder_hash = ""
        combined.update(f"{folder}:{folder_hash}\n".encode())
    return combined.hexdigest()

def changes_made():
    global LAST_HASH
    new_hash = home_hash()
    if LAST_HASH is None:
        # nothing to compare against yet, so ask the remote directly
        LAST_HASH = new_hash
        return rclone.check('/home/gavin', f'discovision:/home/gavin/snapshot/{HOSTNAME}', args=list(FILTERS))[0]
    if new_hash != LAST_HASH:
        LAST_HASH = new_hash
        return True
    return False

def sync():
    if not SYNC_LOCK.acquire(blocking=False):
        return

    try:
        # verify remote has been added
        if not rclone.check_remote_existing('discovision') or not changes_made():
            print('already synced')
            return

        # # sync files
        # rclone.sync('/home/gavin', f'discovision:/home/gavin/snapshot/{HOSTNAME}', args=FILTERS)
        print('start sync')
    finally:
        SYNC_LOCK.release()


# dirhash's multiprocessing pool re-imports this module in the workers, so the
# scheduler loop has to stay behind a __main__ guard.
if __name__ == "__main__":
    schedule.every(2).seconds.do(sync)
    while True:
        schedule.run_pending()
        time.sleep(1)