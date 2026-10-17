"""Static release-envelope sensitivity, NOT a Gazebo contact measurement."""
import json,xml.etree.ElementTree as E
from pathlib import Path
import numpy as np
from geometry import ROOT,ROBOT,SUFFIXES,fk,origin
W=Path(__file__).resolve().parents[2]
cfg=json.loads((W/'src/a1_sorting_core/config/four_slot_candidate.json').read_text())
world=E.parse(W/'src/a1_cell_sim/worlds/four_slot_1ms.sdf')
def gap(A,sa,B,sb):
    ra,rb=A[:3,:3],B[:3,:3];d=A[:3,3]-B[:3,3]
    axes=np.vstack([ra.T,rb.T,np.cross(ra.T[:,None,:],rb.T[None,:,:]).reshape(-1,3)])
    n=np.linalg.norm(axes,axis=1);axes=axes[n>1e-10]/n[n>1e-10,None]
    return float(np.max(np.abs(axes@d)-np.abs(axes@ra)@(sa/2)-np.abs(axes@rb)@(sb/2)))
rows=[]
for bn,b in cfg['bins'].items():
    m=next(m for m in world.findall('.//model') if m.get('name')==bn)
    walls=[(c.get('name'),origin(m.find('pose'),True)@origin(c.find('pose'),True),np.array(list(map(float,c.find('geometry/box/size').text.split())))) for c in m.findall('link/collision')]
    for slot in b['slots']:
        prefix='' if slot['arm']=='arm1' else 'arm2_'
        for finger_q in (-.003,.008,.010,.014,.04):
            vals=dict(zip([prefix+n for n in SUFFIXES],slot['waypoints']['settle_bottom']))
            obstacles=list(walls)
            for s in b['slots']:
                M=np.eye(4);M[:3,3]=s['xyz'];obstacles.append((s['id'],M,np.array([.066,.066,.05])))
            hits=[];minimum=1e9
            for side in ('left','right'):
                n=prefix+'gripper_finger_'+side+'_link';vals[prefix+'gripper_finger_'+side+'_joint']=finger_q
                c=next(l for l in ROBOT.findall('link') if l.get('name')==n).find('collision');A=fk(n,vals)@origin(c.find('origin'));size=np.array(list(map(float,c.find('geometry/box').get('size').split())))
                for name,B,sb in obstacles:
                    g=gap(A,size,B,sb);minimum=min(minimum,g)
                    if g<=0:hits.append({'finger':side,'obstacle':name,'overlap_m':-g})
            rows.append({'slot':slot['id'],'q_each_finger_m':finger_q,'minimum_gap_m':minimum,'intersections':hits})
(ROOT/'audit/release-envelope-sensitivity.json').write_text(json.dumps({'method':'static OBB SAT; deposited cubes +/-8mm XY assumption, not measured','rows':rows},indent=2))
print(json.dumps({str(q):sum(bool(r['intersections']) for r in rows if r['q_each_finger_m']==q) for q in (-.003,.008,.010,.014,.04)},indent=2))
