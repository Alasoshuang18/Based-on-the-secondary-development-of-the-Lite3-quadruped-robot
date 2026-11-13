#!/usr/bin/python3
"""Save an actual named X11 application window, never a synthetic screenshot."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--title', required=True, help='Exact title from xwininfo -root -tree')
    parser.add_argument('--section', required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', args.section):
        parser.error('Section name must be a filename-safe label')
    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    windows = []
    for line in tree.splitlines():
        match = re.match(r'\s*(0x[0-9a-fA-F]+) "([^"]*)"', line)
        title = match[2].replace('.rviz* - RViz', '.rviz - RViz') if match else ''
        if match and (title == args.title or title.endswith('/'+args.title)):
            windows.append((int(match[1], 16), match[2]))
    if len(windows) != 1:
        raise RuntimeError(f'Expected exactly one {args.title!r} window; got {len(windows)}')
    from PyQt5.QtGui import QGuiApplication, QWindow
    app = QGuiApplication([])
    foreign = QWindow.fromWinId(windows[0][0])
    foreign.raise_(); foreign.requestActivate(); app.processEvents()
    time.sleep(.5); app.processEvents()
    image = app.primaryScreen().grabWindow(windows[0][0])
    if image.isNull():
        raise RuntimeError('Window capture returned no pixels')
    root = Path(__file__).resolve().parent
    folder = root/'images/slam3d'
    folder.mkdir(parents=True, exist_ok=True)
    path = folder/(args.section+'.png')
    if not image.save(str(path), 'PNG'):
        raise RuntimeError('Screenshot could not be saved')
    record = dict(section=args.section, file=path.name, title=windows[0][1],
                  window_id=hex(windows[0][0]), display=os.environ.get('DISPLAY'),
                  utc=datetime.now(timezone.utc).isoformat(),
                  width=image.width(), height=image.height(),
                  sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    with (folder/'capture_manifest.jsonl').open('a') as output:
        output.write(json.dumps(record)+'\n')
    print(json.dumps(record), flush=True)


if __name__ == '__main__':
    main()
