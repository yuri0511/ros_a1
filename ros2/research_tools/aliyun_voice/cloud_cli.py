#!/usr/bin/env python3
"""Human-facing cloud/audio audit CLI. Never submits ROS plans or spawns parts."""
import argparse,array,hashlib,json,math,os,subprocess,sys,wave
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/a1_sorting_core'))
from a1_sorting_core.aliyun_voice_cloud import BailianClient,CloudRejected
from a1_sorting_core.voice_batch_plan import validate_candidate,audit_utterance,PlanRejected

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['status','text','record','asr','parse-asr']);p.add_argument('--output',type=Path);p.add_argument('--text');p.add_argument('--wav',type=Path);p.add_argument('--seconds',type=int,default=8)
    a=p.parse_args();client=BailianClient()
    if a.action=='status':print(json.dumps(client.status(),ensure_ascii=False));return
    if a.output is None:raise SystemExit('--output is required, use a new local run directory')
    if a.output.resolve().is_relative_to(Path('/mnt/hgfs')):raise SystemExit('Raw voice data stays local; output under hgfs refused')
    a.output.mkdir(parents=True,exist_ok=True)
    if a.action=='record':
        if not 5<=a.seconds<=10:raise SystemExit('Recording must be 5..10 seconds')
        path=a.output/'microphone.wav'
        if path.exists():raise SystemExit('Existing recording preserved; choose another directory')
        print('即将录音，请说原始指令。此步骤不调用云、不启动ROS。',flush=True)
        subprocess.run(['arecord','-D','default','-t','wav','-f','S16_LE','-r','16000','-c','1','-d',str(a.seconds),str(path)],check=True,timeout=a.seconds+5)
        with wave.open(str(path),'rb') as w:data=array.array('h',w.readframes(w.getnframes()))
        peak=max((abs(x) for x in data),default=0);rms=math.sqrt(sum(x*x for x in data)/max(1,len(data)))
        report={'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'peak':peak,'rms':rms,'samples':len(data),'normalization_applied':False}
        (a.output/'audio-quality.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,ensure_ascii=False))
        return
    name='asr' if a.action=='asr' else 'llm-'+a.action
    target=a.output/(name+'.json')
    if target.exists():raise SystemExit('Existing evidence preserved; use a fresh output directory before any API call')
    if a.action=='asr':
        if not a.wav:raise SystemExit('--wav required')
        result,usage=client.transcribe_wav(a.wav);name='asr';payload={'transcript':result,'api':usage}
    else:
        text=a.text
        if a.action=='parse-asr':text=json.loads((a.output/'asr.json').read_text())['transcript']
        if not text:raise SystemExit('--text or a saved ASR result required')
        result,usage=client.parse_text(text);name='llm-'+a.action
        payload={'transcript':text,'raw_candidate':result,'api':usage,'validated':False}
        try:
            plan=validate_candidate(result);audit_utterance(text,plan)
            payload.update(validated=True,plan=plan.as_dict(),plan_sha256=plan.sha256)
        except PlanRejected as e:payload['rejection']=str(e)
    target=a.output/(name+'.json')
    if target.exists():raise SystemExit('Existing evidence preserved (API already used); choose a fresh output before calling')
    target.write_text(json.dumps(payload,ensure_ascii=False,indent=2));print(json.dumps(payload,ensure_ascii=False,indent=2))
    print('未发送ROS计划，未开始送料；实际执行另需用户确认。')

if __name__=='__main__':
    try:main()
    except CloudRejected as e:raise SystemExit('云调用被阻断：'+str(e)) from None
