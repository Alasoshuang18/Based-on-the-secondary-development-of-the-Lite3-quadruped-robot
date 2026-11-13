import json, struct
from pathlib import Path
root=Path(__file__).resolve().parent
meta=json.loads((root/'cad/box_export.json').read_text(encoding='utf-8-sig'))
raw=next((root/'meshes').glob('box_raw.*')).read_bytes()
n=struct.unpack_from('<I',raw,80)[0]
assert len(raw)==84+n*50
vertices=[]
for i in range(n):
 row=struct.unpack_from('<12fH',raw,84+i*50)
 vertices.extend([row[3:6],row[6:9],row[9:12]])
lo=[min(p[k] for p in vertices) for k in range(3)]
hi=[max(p[k] for p in vertices) for k in range(3)]
cad=meta['bounds_m']; expected=[cad[k+3]-cad[k] for k in range(3)]
scales=[expected[k]/(hi[k]-lo[k]) for k in range(3)]
assert max(scales)-min(scales)<max(scales)*1e-5
scale=sum(scales)/3
center=[(lo[k]+hi[k])/2 for k in range(3)]
out=bytearray(b'box.SLDPRT exported by SolidWorks; centered mesh in metres'.ljust(80,b' ')+struct.pack('<I',n))
for i in range(n):
 row=list(struct.unpack_from('<12fH',raw,84+i*50))
 for j in (3,6,9):
  for k in range(3): row[j+k]=(row[j+k]-center[k])*scale
 out.extend(struct.pack('<12fH',*row))
(root/'meshes/box.stl').write_bytes(out)
meta.update(mesh_units='metres', mesh_center='bounding-box center', stl_export_unit_scale=scale, triangles=n, dimensions_m=expected, mass_note='CAD-assigned density; verify actual payload mass before gait qualification')
(root/'cad/box_export.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
print(meta)
