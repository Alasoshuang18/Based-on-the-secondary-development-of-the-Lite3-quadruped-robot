#!/usr/bin/env python3
"""Run this board's RViz with private, explicit map sampler bindings.

The observed OpenGL 4.5 compatibility renderer supports GLSL 4.20. Keep the
original indexed-map algorithm, textures and material bindings; initialise the
two sampler units in GLSL before OGRE's first program validation. No /opt/ros
file is changed. This entry requires OpenGL >= 4.2 compatibility support.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys


ROOT = Path(__file__).resolve().parent


def prepare():
    source = Path('/opt/ros/humble/share/rviz_rendering')
    relative = Path('ogre_media/materials/glsl120')
    names = ('indexed_8bit_image.vert', 'indexed_8bit_image.frag')
    originals = {name: (source/relative/name).read_text() for name in names}
    changed = {}
    for name, text in originals.items():
        if text.count('#version 120') != 1:
            raise ValueError('Unexpected installed shader; left unchanged: '+name)
        changed[name] = text.replace('#version 120', '#version 420 compatibility')
    fragment = changed[names[1]]
    for declaration, unit in (('uniform sampler2D eight_bit_image;', 0),
                              ('uniform sampler1D palette;', 1)):
        if fragment.count(declaration) != 1:
            raise ValueError('Unexpected map samplers; no overlay prepared')
        fragment = fragment.replace(declaration, f'layout(binding = {unit}) '+declaration)
    changed[names[1]] = fragment
    signature = hashlib.sha256(json.dumps(changed, sort_keys=True).encode()).hexdigest()
    overlay = ROOT/'.runtime/rviz_overlay'
    target = overlay/'share/rviz_rendering'
    stamp = overlay/'shader_signature'
    if not stamp.exists() or stamp.read_text() != signature:
        # A private resource copy is sufficient: RViz resolves its resource
        # directory through the ament package index. Libraries stay in /opt/ros.
        shutil.copytree(source, target, dirs_exist_ok=True)
        for name, text in changed.items():
            (target/relative/name).write_text(text)
        index = overlay/'share/ament_index/resource_index/packages'
        index.mkdir(parents=True, exist_ok=True)
        (index/'rviz_rendering').write_text('')
        stamp.write_text(signature)
    return overlay


def main():
    overlay = prepare()
    if sys.argv[1:] == ['--prepare-only']:
        print(overlay)
        return
    env = dict(os.environ)
    env['AMENT_PREFIX_PATH'] = str(overlay)+os.pathsep+env.get('AMENT_PREFIX_PATH', '')
    executable = '/opt/ros/humble/lib/rviz2/rviz2'
    args = sys.argv[1:] or ['-d', str(ROOT/'rviz/lite3_slam.rviz'),
                           '--ros-args', '-p', 'use_sim_time:=true']
    print('[rviz-compat] private map bindings: image=0, palette=1; requires GL4.2+', flush=True)
    os.execvpe(executable, [executable]+args, env)


if __name__ == '__main__':
    main()
