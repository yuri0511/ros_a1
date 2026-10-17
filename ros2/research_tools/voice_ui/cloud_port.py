"""Future explicitly authorized cloud boundary. Default port cannot access network/key.
An approved integration can inject the existing BailianClient adapter. No ROS here.
"""
from typing import Protocol
class CloudPort(Protocol):
    def transcribe_wav(self,path): ...
    def parse_text(self,text): ...
class DisabledCloud:
    def transcribe_wav(self,path):raise PermissionError('本轮云调用关闭：需要新付费授权')
    def parse_text(self,text):raise PermissionError('本轮云调用关闭：需要新付费授权')
class PreviewPipeline:
    def __init__(self,cloud=None):self.cloud=cloud or DisabledCloud()
    def preview(self,path):
        # Caller must run in a worker; never auto-confirm/submit any returned plan.
        from a1_sorting_core.voice_batch_plan import validate_candidate,audit_utterance
        text,asr_meta=self.cloud.transcribe_wav(path)
        candidate,llm_meta=self.cloud.parse_text(text)
        plan=validate_candidate(candidate);audit_utterance(text,plan)
        return {'source':'new_recording_cloud_preview','transcript':text,'plan':plan.as_dict(),'plan_sha256':plan.sha256,'asr':asr_meta,'llm':llm_meta,'human_confirmed':False}
