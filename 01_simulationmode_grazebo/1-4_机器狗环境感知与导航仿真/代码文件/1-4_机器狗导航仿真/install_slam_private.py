#!/usr/bin/python3
"""Fetch Humble binaries into this chapter only; never call apt install or sudo."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.request

import apt
import apt_pkg

ROOT = Path(__file__).resolve().parent
MIRROR = 'https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/'


def main():
    cache_dir = ROOT/'.runtime/slam_debs'
    install = ROOT/'.runtime/slam_root'
    cache_dir.mkdir(parents=True, exist_ok=True)
    install.mkdir(parents=True, exist_ok=True)
    index = gzip.decompress(urllib.request.urlopen(
        MIRROR+'dists/jammy/main/binary-arm64/Packages.gz', timeout=60).read()).decode()
    records = {}
    for block in index.split('\n\n'):
        fields = dict(line.split(': ', 1) for line in block.splitlines()
                      if ': ' in line and not line.startswith(' '))
        name = fields.get('Package')
        if name and (name not in records or apt_pkg.version_compare(
                fields['Version'], records[name]['Version']) > 0):
            records[name] = fields
    apt_pkg.config.set('APT::Install-Recommends', 'false')
    cache = apt.Cache()
    for name in ('ros-humble-slam-toolbox', 'ros-humble-pointcloud-to-laserscan'):
        cache[name].mark_install()
    report = {'system_install_modified': False, 'packages': []}
    for package in sorted(cache.get_changes(), key=lambda p: p.name):
        candidate = package.candidate
        if package.name.startswith('ros-humble-'):
            info = records[package.name]
            path = cache_dir/Path(info['Filename']).name
            expected = info['SHA256']
            if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                with urllib.request.urlopen(MIRROR+info['Filename'], timeout=60) as response:
                    data = response.read()
                if hashlib.sha256(data).hexdigest() != expected:
                    raise RuntimeError('Checksum mismatch: '+package.name)
                path.write_bytes(data)
            size = int(info['Size'])
            disk = int(info['Installed-Size'])*1024
            version = info['Version']
        else:
            path = Path(candidate.fetch_binary(str(cache_dir)))
            size, disk, version = candidate.size, candidate.installed_size, candidate.version
        subprocess.run(['dpkg-deb', '-x', str(path), str(install)], check=True)
        report['packages'].append(dict(name=package.name, version=version,
                                       download_bytes=size, installed_bytes=disk))
        print('READY', package.name, flush=True)
    report['download_bytes'] = sum(p['download_bytes'] for p in report['packages'])
    report['installed_bytes'] = sum(p['installed_bytes'] for p in report['packages'])
    (ROOT/'.runtime/slam_packages.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k != 'packages'}), flush=True)


if __name__ == '__main__':
    main()
