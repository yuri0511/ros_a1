import copy,json,sys
from pathlib import Path
import numpy as np
import pytest
from moveit_msgs.msg import AllowedCollisionMatrix,AllowedCollisionEntry
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from support_contact import (scoped_matrix,matrix_key,SupportMatrix,OBJECT,SUPPORT,
                             contact_landing_trajectory,sample_trajectory,cube_bounds,ROUND)
from geometry import ROBOT,ROOT
from diagnose_held import remove_char_operations
from moveit_msgs.srv import GetMotionPlan
from rosidl_runtime_py.set_message import set_message_fields
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'voice_four'))
from support_evidence import SupportEvidence,box_bottom


def test_scope_changes_only_the_specific_pair_and_preserves_baseline():
    base=AllowedCollisionMatrix(entry_names=['arm','wall'],entry_values=[AllowedCollisionEntry(enabled=[True,False]),AllowedCollisionEntry(enabled=[False,True])])
    before=copy.deepcopy(base);scoped=scoped_matrix(base)
    def allowed(m,a,b):
        if a not in m.entry_names or b not in m.entry_names:return False
        return m.entry_values[m.entry_names.index(a)].enabled[m.entry_names.index(b)]
    for a in scoped.entry_names:
        for b in scoped.entry_names:
            assert allowed(scoped,a,b)==(True if {a,b}=={OBJECT,SUPPORT} else allowed(base,a,b))
    assert matrix_key(base)==matrix_key(before)


def test_matrix_comparison_is_order_independent_not_permission_independent():
    a=AllowedCollisionMatrix(entry_names=['b','a'],entry_values=[AllowedCollisionEntry(enabled=[False,True]),AllowedCollisionEntry(enabled=[True,False])])
    b=AllowedCollisionMatrix(entry_names=['a','b'],entry_values=[AllowedCollisionEntry(enabled=[False,True]),AllowedCollisionEntry(enabled=[True,False])])
    assert matrix_key(a)==matrix_key(b)
    b.entry_values[0].enabled[1]=False
    assert matrix_key(a)!=matrix_key(b)


def test_missing_preconditions_never_apply_ACM():
    obj=object.__new__(SupportMatrix)
    for args in ((False,True,'depart'),(True,False,'depart'),(True,True,'free')):
        with pytest.raises(RuntimeError):obj.enable(grasp_verified=args[0],support_verified=args[1],audited_direction=args[2])


def test_defaults_are_not_silently_reinterpreted():
    base=AllowedCollisionMatrix(default_entry_names=['wall'],default_entry_values=[True])
    with pytest.raises(ValueError):scoped_matrix(base)


def test_native_support_is_bounded_by_observed_baseline_without_numeric_depth_allowance():
    s=SupportEvidence();p={'phase':'departing','max_support_depth':1e-6,'native_floor_min':.719999,'floor_top':.72}
    pair=('calibration_red4::link::collision','tabletop::link::floor_red')
    assert s.contact(*pair,[1e-6],[(0,0,1)],1,1,p) is None
    assert s.contact(*pair,[1.1e-6],[(0,0,1)],1.001,1.001,p)=='native_support_penetration_grows'


def test_other_obstacles_and_recontact_remain_blocked():
    s=SupportEvidence();p={'phase':'free'}
    assert s.contact('calibration_red4::link::collision','bin_red::wall::collision',[1e-9],[(0,0,1)],1,1,p)=='unexpected_fixture_obstacle_contact'
    assert s.contact('calibration_red4::link::collision','tabletop::link::floor_red',[1e-9],[(0,0,1)],1,1,p)=='support_recontact_after_separation'


def test_pose_check_cannot_hide_growth_or_nonfinite_state():
    s=SupportEvidence();p={'phase':'departing','floor_top':.72,'native_floor_min':.719999}
    assert s.pose([.295,-.235,.746],[0,0,0,1],1,1,p) is None
    assert s.separated
    assert s.pose([.295,-.235,.74499],[0,0,0,1],1.001,1.001,p)=='native_cube_bottom_penetration_grows'
    with pytest.raises(ValueError):box_bottom([0,0,float('nan')],[0,0,0,1])


def test_contact_landing_preserves_intended_goal_and_scaled_limits():
    tr=JointTrajectory(joint_names=['arm2_wrist_3_joint'],points=[JointTrajectoryPoint(positions=[0.],velocities=[0.],accelerations=[0.],time_from_start=Duration()),JointTrajectoryPoint(positions=[.1000001],velocities=[0.],accelerations=[-1.],time_from_start=Duration(nanosec=100000000))])
    limit={'arm2_wrist_3_joint':{'has_acceleration_limits':True,'max_acceleration':5.}}
    out=contact_landing_trajectory(tr,ROBOT,limit,exact_goal=[.1]);h=out.points[-1].time_from_start.sec+out.points[-1].time_from_start.nanosec/1e9
    assert list(out.points[-1].positions)==[.1] and list(tr.points[-1].positions)==[.1000001]
    assert 1.875*.1/h<=float(next(j for j in ROBOT.findall('joint') if j.get('name')=='arm2_wrist_3_joint').find('limit').get('velocity'))*.2
    assert (10/np.sqrt(3))*.1/h**2<=1.+1e-14
    assert all(list(p.velocities)==[0.] and list(p.accelerations)==[0.] for p in out.points)


def test_recorded_51nm_contact_is_not_rounded_away():
    row=json.loads((ROOT/'calibration_runs/C-20261017T102733-3d3e3a/held-cycle.json').read_text())[0]
    data=copy.deepcopy(row['request']);remove_char_operations(data);request=GetMotionPlan.Request();set_message_fields(request,data)
    state=request.motion_plan_request.start_state;values=dict(zip(state.joint_state.name,state.joint_state.position))
    bottom=cube_bounds(values,state.attached_collision_objects[0])[0][2]
    assert 5.15e-8<.72-bottom<5.16e-8 and ROUND<1e-12


def test_failed_ACM_readback_restores_baseline_before_propagating_error():
    obj=object.__new__(SupportMatrix);obj.base=AllowedCollisionMatrix();obj.scoped=scoped_matrix(obj.base);obj.active=False
    from types import SimpleNamespace as N
    obj.read=lambda:N(allowed_collision_matrix=obj.base)
    changes=[]
    def setting(matrix):
        changes.append(matrix_key(matrix))
        if len(changes)==1:raise RuntimeError('readback failed after mutation')
    obj._set=setting;obj.emit=lambda **kw:None
    with pytest.raises(RuntimeError):obj.enable(grasp_verified=True,support_verified=True,audited_direction='depart')
    assert changes==[matrix_key(obj.scoped),matrix_key(obj.base)] and not obj.active


def test_fixture_sensor_instrumentation_keeps_geometry_and_mass():
    import xml.etree.ElementTree as ET
    import importlib.util
    from calibrate_idle import instrument_fixture
    path=Path(__file__).resolve().parents[2]/'src/a1_cell_sim/scripts/spawn_parts.py'
    spec=importlib.util.spec_from_file_location('fixture_source',path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    before=ET.fromstring(mod.part_sdf(0,'red'));after=ET.fromstring(instrument_fixture(mod.part_sdf(0,'red')))
    assert after.find('model/link/sensor/contact/collision').text=='collision'
    after.find('model/link').remove(after.find('model/link/sensor'))
    # Whitespace is formatting; compare every actual element, attribute and value.
    canonical=lambda x:(x.tag,tuple(x.attrib.items()),(x.text or '').strip(),tuple(canonical(c) for c in x))
    assert canonical(before)==canonical(after)
