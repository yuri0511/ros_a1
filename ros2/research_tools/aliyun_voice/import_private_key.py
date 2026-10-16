#!/usr/bin/env python3
"""Run by the human in their Ubuntu terminal. Never prints a credential."""
import getpass,json,os,stat,time,warnings
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/a1_sorting_core'))
from a1_sorting_core.aliyun_voice_cloud import DEFAULT_ENDPOINT,endpoint_allowed

def main():
    if not sys.stdin.isatty() or not sys.stderr.isatty():raise SystemExit('必须由用户在Ubuntu真实交互终端执行；不接受重定向/聊天粘贴密钥。')
    private=Path.home()/'.config/yuri_a1_voice'
    if private.is_symlink():raise SystemExit('拒绝私有目录符号链接')
    private.mkdir(mode=0o700,exist_ok=True);os.chmod(private,0o700)
    print('密钥仅保存于本Ubuntu用户私有目录，不写入源码、共享目录或日志。')
    warnings.simplefilter('error',getpass.GetPassWarning)
    try:key=getpass.getpass('请粘贴同一个北京地域百炼API Key（不回显）：').strip()
    except getpass.GetPassWarning:raise SystemExit('无法关闭终端回显，拒绝导入；请使用Ubuntu本机终端。')
    if not key or any(c.isspace() for c in key):raise SystemExit('输入为空或含空白，未保存。')
    endpoint=input('北京兼容API完整地址（回车使用官方仍支持的dashscope北京地址）：').strip() or DEFAULT_ENDPOINT
    if not endpoint_allowed(endpoint):raise SystemExit('地址不在北京官方域名白名单，未保存。')
    for name,data in [('api_key',key)]:
        fd=os.open(private/name,os.O_WRONLY|os.O_CREAT|os.O_TRUNC|os.O_NOFOLLOW,0o600)
        os.fchmod(fd,0o600)
        with os.fdopen(fd,'w') as f:f.write(data+'\n')
    key=''
    print('已保存密钥。接下来是付费调用授权，不是Gazebo任务确认。')
    print('预计最多4次：最多2次纯文本解析＋1次≤10秒ASR＋1次识别文本解析；输出每次≤400 tokens。可能收费，实际金额以百炼账单为准。无自动重试。')
    approval=input('授权请输入“同意最多4次”，否则不授权：').strip()
    budget={'approved':approval=='同意最多4次','max_requests':4 if approval=='同意最多4次' else 0,'used':0,'expires_at':time.time()+24*3600,'endpoint':endpoint}
    fd=os.open(private/'budget.json',os.O_WRONLY|os.O_CREAT|os.O_TRUNC|os.O_NOFOLLOW,0o600);os.fchmod(fd,0o600)
    with os.fdopen(fd,'w') as f:json.dump(budget,f)
    print('密钥已加载；'+('已授权最多4次调用（24小时有效）。' if budget['approved'] else '尚未授权API调用。'))

if __name__=='__main__':main()
