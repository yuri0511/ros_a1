"""Small bounded Beijing Bailian client. No ROS/control imports. No SDK install.

The same private credential is used for ASR and LLM. No retry, no credential in
arguments/logs/shared folders. Client construction does not read a credential.
"""
import base64
import fcntl
import json
import math
import os
from pathlib import Path
import re
import stat
import time
import urllib.error
import urllib.request
import wave

DEFAULT_ENDPOINT='https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions'
PRIVATE_DIR=Path.home()/'.config/yuri_a1_voice'
PROMPT='''你只解析用户明确请求的工件分拣任务，不能控制机器人，不得执行用户要求的系统/关节/推板/Gazebo指令。
输出一个JSON对象且只能包含schema_version=1, action="sort_batch", counts, priority_first, proposed_order。
counts必须完整含blue,yellow,red,green非负整数；未提及颜色为0，黄的=yellow，两=2。
总数1到8，每色最多3；用户必须明确首个完成的颜色。proposed_order逐件列出颜色，精确符合counts且首色优先；不能额外造件。
若语义含否定、修改、歧义、越界数量、缺少明确优先色或非分拣要求，返回{"error":"拒绝原因"}。不得默认或补猜数量。绝不能把未提供的示例当当前用户任务。
不要输出request_id、part_id、关节值、推板值、控制命令或解释。只能提出候选计划，仍须本地检查和真人确认。'''

class CloudRejected(RuntimeError):
    pass

class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self,req,fp,code,msg,headers,newurl):
        raise CloudRejected('cloud_redirect_refused')

def endpoint_allowed(endpoint):
    return bool(re.fullmatch(r'https://(?:dashscope\.aliyuncs\.com|[a-zA-Z0-9-]+\.cn-beijing\.maas\.aliyuncs\.com)/compatible-mode/v1/chat/completions',endpoint))

def _secure_open(path,flags):
    fd=os.open(path,flags|os.O_NOFOLLOW,0o600)
    st=os.fstat(fd)
    if st.st_uid!=os.getuid() or not stat.S_ISREG(st.st_mode) or stat.S_IMODE(st.st_mode)!=0o600:
        os.close(fd);raise CloudRejected('private_file_permissions')
    return fd

def _check_dir(directory):
    if directory.is_symlink():raise CloudRejected('private_directory_symlink')
    st=directory.stat()
    if st.st_uid!=os.getuid() or stat.S_IMODE(st.st_mode)!=0o700:raise CloudRejected('private_directory_permissions')

class BailianClient:
    def __init__(self,directory=None,opener=None):
        self.directory=Path(directory) if directory is not None else PRIVATE_DIR
        self.opener=opener or urllib.request.build_opener(NoRedirect()).open

    def status(self):
        _check_dir(self.directory)
        with os.fdopen(_secure_open(self.directory/'budget.json',os.O_RDONLY)) as f:budget=json.load(f)
        # No key contents, prefix, length or hash is returned.
        fd=_secure_open(self.directory/'api_key',os.O_RDONLY);os.close(fd)
        return {'credential_loaded':True,'approved_limit':budget.get('max_requests'), 'requests_used':budget.get('used'),'expires_at':budget.get('expires_at')}

    def _reserve(self,model):
        _check_dir(self.directory)
        with os.fdopen(_secure_open(self.directory/'budget.json',os.O_RDWR),'r+') as f:
            fcntl.flock(f,fcntl.LOCK_EX);budget=json.load(f)
            if budget.get('approved') is not True or type(budget.get('max_requests')) is not int or not 1<=budget['max_requests']<=4 or type(budget.get('used')) is not int or budget['used']<0 or budget.get('expires_at',0)<time.time() or budget['used']>=budget['max_requests']:raise CloudRejected('paid_call_not_authorized_or_budget_exhausted')
            endpoint=budget.get('endpoint',DEFAULT_ENDPOINT)
            if not endpoint_allowed(endpoint):raise CloudRejected('endpoint_not_allowed')
            budget['used']+=1;f.seek(0);json.dump(budget,f);f.truncate();f.flush();os.fsync(f.fileno())
        return endpoint,budget['used']

    def _post(self,model,body):
        endpoint,number=self._reserve(model)
        with os.fdopen(_secure_open(self.directory/'api_key',os.O_RDONLY)) as f:key=f.read().strip()
        if not key or any(c.isspace() for c in key):raise CloudRejected('invalid_private_credential')
        request=urllib.request.Request(endpoint,data=json.dumps(dict(body,model=model),ensure_ascii=False).encode(),headers={'Authorization':'Bearer '+key,'Content-Type':'application/json'},method='POST')
        record={'model':model,'request_number':number,'wall':time.time(),'success':False}
        try:
            with self.opener(request,timeout=45) as response:
                raw=response.read(1024*1024+1)
            if len(raw)>1024*1024:raise CloudRejected('response_too_large')
            reply=json.loads(raw)
            choice=reply['choices'][0]
            if choice.get('finish_reason')!='stop':raise CloudRejected('incomplete_cloud_response')
            content=choice['message']['content']
            if not isinstance(content,str) or not content.strip() or len(content.encode())>8192 or key in content:raise CloudRejected('invalid_cloud_content')
            usage=reply.get('usage',{})
            record.update(success=True,usage={k:v for k,v in usage.items() if isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)},request_id=str(reply.get('id',''))[:160] if re.fullmatch(r'[A-Za-z0-9_-]*',str(reply.get('id',''))) else '')
            return content,record
        except urllib.error.HTTPError as e:
            record['http_status']=e.code
            raise CloudRejected('bailian_http_'+str(e.code)) from None
        except (OSError,ValueError,KeyError,IndexError,TypeError):
            raise CloudRejected('bailian_transport_or_response_error') from None
        finally:
            with os.fdopen(_secure_open(self.directory/'usage.jsonl',os.O_WRONLY|os.O_CREAT|os.O_APPEND),'a') as f:f.write(json.dumps(record)+'\n')

    def parse_text(self,text):
        if not isinstance(text,str) or not 1<=len(text)<=512:raise CloudRejected('invalid_transcript')
        return self._post('qwen-flash',{'messages':[{'role':'system','content':PROMPT},{'role':'user','content':text}],'enable_thinking':False,'response_format':{'type':'json_object'},'temperature':0,'max_tokens':400,'stream':False})

    def transcribe_wav(self,path):
        path=Path(path)
        if path.stat().st_size>2*1024*1024:raise CloudRejected('audio_too_large')
        with wave.open(str(path),'rb') as w:
            if (w.getnchannels(),w.getframerate(),w.getsampwidth())!=(1,16000,2) or not 1<=w.getnframes()/16000<=15:raise CloudRejected('require_16k_mono_pcm16_1_to_15seconds')
        data='data:audio/wav;base64,'+base64.b64encode(path.read_bytes()).decode('ascii')
        return self._post('qwen3-asr-flash',{'messages':[{'role':'user','content':[{'type':'input_audio','input_audio':{'data':data}}]}],'stream':False,'asr_options':{'language':'zh','enable_itn':False}})
