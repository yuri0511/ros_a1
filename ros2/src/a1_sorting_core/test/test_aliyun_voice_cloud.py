import io,json,os,time,urllib.error,wave
from pathlib import Path
import pytest
from a1_sorting_core.aliyun_voice_cloud import BailianClient,CloudRejected,endpoint_allowed,NoRedirect,DEFAULT_ENDPOINT

@pytest.fixture
def private(tmp_path):
    d=tmp_path/'private';d.mkdir(mode=0o700)
    for name,obj in [('api_key','offline-test-placeholder\n'),('budget.json',json.dumps(dict(approved=True,max_requests=4,used=0,expires_at=time.time()+3600,endpoint=DEFAULT_ENDPOINT)))]:
        p=d/name;p.write_text(obj);p.chmod(0o600)
    return d

def response(text):return io.BytesIO(json.dumps(dict(id='mock-id',choices=[dict(finish_reason='stop',message=dict(content=text))],usage=dict(total_tokens=12))).encode())

def test_shared_credential_two_models_and_no_ros(private,tmp_path):
    seen=[]
    def opener(req,timeout):
        body=json.loads(req.data);seen.append((body,req.get_header('Authorization')));return response('测试')
    c=BailianClient(private,opener)
    c.parse_text('测试')
    p=tmp_path/'audio.wav'
    with wave.open(str(p),'wb') as w:w.setnchannels(1);w.setsampwidth(2);w.setframerate(16000);w.writeframes(bytes(32000))
    c.transcribe_wav(p)
    assert [x[0]['model'] for x in seen]==['qwen-flash','qwen3-asr-flash']
    assert seen[0][1]==seen[1][1]
    assert seen[0][0]['enable_thinking'] is False and seen[0][0]['response_format']=={'type':'json_object'}
    assert seen[1][0]['messages'][0]['content'][0]['input_audio']['data'].startswith('data:audio/wav;base64,')
    assert 'offline-test-placeholder' not in (private/'usage.jsonl').read_text()
    assert 'offline-test-placeholder' not in json.dumps(c.status())

def test_requests_are_bounded_and_not_retried(private):
    calls=[]
    def fail(req,timeout):calls.append(1);raise urllib.error.HTTPError(req.full_url,401,'private server body',{},io.BytesIO(b'do not log this'))
    c=BailianClient(private,fail)
    for _ in range(4):
        with pytest.raises(CloudRejected,match='401'):c.parse_text('测试')
    with pytest.raises(CloudRejected,match='budget'):c.parse_text('测试')
    assert len(calls)==4 and 'private server body' not in (private/'usage.jsonl').read_text()

@pytest.mark.parametrize('field,value',[('approved',False),('expires_at',0),('endpoint','https://example.com/compatible-mode/v1/chat/completions')])
def test_no_authorization_no_call(private,field,value):
    p=private/'budget.json';b=json.loads(p.read_text());b[field]=value;p.write_text(json.dumps(b))
    def forbidden(*a,**kw):pytest.fail('Unauthorized network call')
    with pytest.raises(CloudRejected):BailianClient(private,forbidden).parse_text('测试')

def test_unsafe_key_permissions_fail_closed(private):
    (private/'api_key').chmod(0o644)
    with pytest.raises(CloudRejected,match='permissions'):BailianClient(private).status()

def test_symlink_secret_refused(private,tmp_path):
    (private/'api_key').unlink();(private/'api_key').symlink_to(tmp_path/'elsewhere')
    with pytest.raises(OSError):BailianClient(private).status()

def test_redirect_never_forwards_authorization():
    with pytest.raises(CloudRejected,match='redirect'):NoRedirect().redirect_request(None,None,302,'',{},'https://example.com')

@pytest.mark.parametrize('url',['http://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions','https://dashscope.aliyuncs.com.example.com/compatible-mode/v1/chat/completions','https://dashscope-intl.aliyuncs.com/compatible-mode/v1/chat/completions','https://deepseek.com'])
def test_endpoint_allowlist(url):assert not endpoint_allowed(url)
