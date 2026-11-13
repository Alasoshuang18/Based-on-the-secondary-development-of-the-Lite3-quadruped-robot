import json
import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from lite3_lab_5_2.checkpoint import digest, finite_pose, load_checkpoint
from lite3_lab_5_2.checkpoint import current_audit, register_audit


def bundle(tmp_path):
    files={}
    for name in ('map.yaml','map.pgm','map.pcd','slam.posegraph','slam.data'):
        (tmp_path/name).write_bytes(b'nonempty fixture '+name.encode())
        files[name]=digest(tmp_path/name)
    data=dict(schema=1,complete=True,stage=2,configuration={'length':.779},
              saved_pose_map=[1.,2.,.3],files=files)
    path=tmp_path/'checkpoint.json'
    path.write_text(json.dumps(data))
    return path,data


def test_resume_accepts_previous_stage_and_unchanged_configuration(tmp_path):
    path,data=bundle(tmp_path)
    assert load_checkpoint(path,3,data['configuration'])['stage']==2


def test_missing_serialized_state_rejected_even_with_grid_map(tmp_path):
    path,_=bundle(tmp_path)
    (tmp_path/'slam.data').unlink()
    with pytest.raises(ValueError):
        load_checkpoint(path,3)


def test_changed_file_rejected(tmp_path):
    path,_=bundle(tmp_path)
    (tmp_path/'map.pgm').write_bytes(b'changed')
    with pytest.raises(ValueError):
        load_checkpoint(path,3)


def test_wrong_parent_stage_and_changed_geometry_rejected(tmp_path):
    path,_=bundle(tmp_path)
    with pytest.raises(ValueError):
        load_checkpoint(path,4)
    with pytest.raises(ValueError):
        load_checkpoint(path,3,{'length':1.2})


@pytest.mark.parametrize('value',[[0.,0.,float('nan')],[1.,2.],None,[0.,0.,float('inf')]])
def test_invalid_restore_pose_rejected(value):
    with pytest.raises(ValueError):
        finite_pose(value)


def test_restart_f_switches_evidence_pointer_and_preserves_first_audit(tmp_path):
    (tmp_path/'session.json').write_text(json.dumps({'stage':2}))
    first=tmp_path/'exploration_first'
    second=tmp_path/'exploration_second'
    first.mkdir(); second.mkdir()
    (first/'events.jsonl').write_text('old evidence')
    register_audit(first,2)
    assert current_audit(tmp_path)==first
    register_audit(second,2)
    assert current_audit(tmp_path)==second
    assert (first/'events.jsonl').read_text()=='old evidence'


def test_audit_rejects_wrong_stage_or_pointer_outside_session(tmp_path):
    run=tmp_path/'run'
    run.mkdir()
    (run/'session.json').write_text(json.dumps({'stage':2}))
    audit=run/'exploration_new'
    audit.mkdir()
    with pytest.raises(ValueError):
        register_audit(audit,3)
    outside=tmp_path/'exploration_elsewhere'
    outside.mkdir()
    (run/'exploration_current.txt').write_text(str(outside))
    with pytest.raises(ValueError):
        current_audit(run)


def test_legacy_fixed_audit_directory_remains_readable(tmp_path):
    assert current_audit(tmp_path)==tmp_path/'exploration'
