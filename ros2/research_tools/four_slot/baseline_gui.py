"""Display-only client for the unchanged gain1+READY eight-part baseline."""
import os,json,sys,subprocess
from pathlib import Path

def command(run):
    d=json.loads((Path(run)/'metadata.json').read_text());cmd=d['command'];env=d['environment']
    if 'feed_policy:=rule' not in cmd or 'demo_parts:=8' not in cmd or 'controllers_file:=ur5e_cell_controllers_stability_gain1.yaml' not in cmd or 'world_file:=fast_sim_1ms.sdf' not in cmd:raise ValueError('not the frozen Rule8 gain1 baseline')
    return ['gz','sim','-g'],{k:env[k] for k in ('GZ_PARTITION','ROS_DOMAIN_ID','LP_NUM_THREADS','DISPLAY','LIBGL_ALWAYS_SOFTWARE','GALLIUM_DRIVER')}
def main():
    if len(sys.argv)!=2:raise SystemExit('usage: baseline_gui.py EXACT_RUN_DIRECTORY')
    cmd,profile=command(sys.argv[1]);env=os.environ.copy();env.update(profile,QT_QPA_PLATFORM='xcb')
    for key in ('__NV_PRIME_RENDER_OFFLOAD','__NV_PRIME_RENDER_OFFLOAD_PROVIDER','__GLX_VENDOR_LIBRARY_NAME','DRI_PRIME','WAYLAND_DISPLAY'):env.pop(key,None)
    print('Display-only, same GZ_PARTITION:',profile['GZ_PARTITION'],flush=True)
    raise SystemExit(subprocess.call(cmd,env=env))
if __name__=='__main__':main()
