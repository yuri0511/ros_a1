"""Partition-local bounded stop requests. Pause before any potentially slow ROS RPC."""
import json,subprocess,time

def stop_requests(run,env,runner=subprocess.run):
    steps=[('pause_world',['gz','service','-s','/world/sorting_cell/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],4,'pause.log'),
           ('halt_new_feed',['ros2','service','call','/voice_batch/stop','std_srvs/srv/Trigger','{}'],3,'stop-service.log')]
    records=[]
    attempt=time.monotonic_ns()
    for name,command,bound,log in steps:
        started=time.monotonic()
        evidence=run/f'{attempt}-{log}'
        record={'attempt':attempt,'evidence':evidence.name,'step':name,'start_wall':time.time(),'command':command,'timeout_s':bound}
        try:
            response=runner(command,env=env,capture_output=True,text=True,timeout=bound)
            evidence.write_text(response.stdout+response.stderr)
            record.update(returncode=response.returncode,acknowledged=(response.returncode==0 and ('data: true' in response.stdout if name=='pause_world' else 'success=True' in response.stdout)))
        except (OSError,subprocess.TimeoutExpired) as exc:
            evidence.write_text(str(exc));record.update(error=str(exc),acknowledged=False)
        # Keep the original attempt as well as every subsequent stop request.
        if not (run/log).exists():(run/log).write_text(evidence.read_text())
        record['end_wall']=time.time();record['duration_monotonic_s']=time.monotonic()-started;records.append(record)
        with (run/'stop-timeline.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
    return records
