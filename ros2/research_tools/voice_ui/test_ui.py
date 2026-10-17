import json, sys, wave, tempfile
from pathlib import Path
from unittest.mock import patch
from PyQt5 import QtWidgets, QtCore, QtTest
from app import Window, ROOT
app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
def test_plans_and_confirmation():
    w=Window(); w.load_four(); assert w.plan['counts']['green']==2; assert not w.submitted
    with patch.object(QtWidgets.QMessageBox,'question',return_value=QtWidgets.QMessageBox.No):w.confirm_plan()
    assert not w.submitted
    with patch.object(QtWidgets.QMessageBox,'question',return_value=QtWidgets.QMessageBox.Yes):w.confirm_plan()
    assert w.submitted and not w.confirm.isEnabled()
    w.load_history(); assert w.plan['counts']['green']==3; assert not w.confirm.isEnabled(); w.close()
def test_real_microphone_mouse_buttons():
    w=Window(); w.show(); w.load_four(); QtTest.QTest.mouseClick(w.start,QtCore.Qt.LeftButton); QtTest.QTest.qWait(2200); QtTest.QTest.mouseClick(w.end,QtCore.Qt.LeftButton); QtTest.QTest.qWait(1000)
    assert w.wav is not None
    with wave.open(str(w.wav)) as r:assert r.getnchannels()==1 and r.getframerate()==16000 and r.getnframes()>16000
    out=Path(tempfile.mkdtemp(prefix='ui-test-',dir=ROOT/'audit')); w.grab().save(str(out/'voice-ui.png')); (out/'recording-test.json').write_text(json.dumps({'path':str(w.wav),'test':'QtTest mouse press on real record/stop buttons, actual default ALSA capture','cloud_calls':0},indent=2)); w.close()
def test_cloud_disabled_and_history_cannot_confirm():
    import pytest
    from cloud_port import DisabledCloud
    with pytest.raises(PermissionError):DisabledCloud().transcribe_wav('unused.wav')
    with pytest.raises(PermissionError):DisabledCloud().parse_text('unused')
    w=Window();w.load_history()
    with patch.object(QtWidgets.QMessageBox,'question',side_effect=AssertionError('must not prompt')):w.confirm_plan()
    assert not w.submitted;w.close()
def test_pipeline_fake_cloud_never_confirms():
    from cloud_port import PreviewPipeline
    class FakeCloud:
        def transcribe_wav(self,path):return '开始分拣，一个黄色、一个红色、两个绿色，先完成黄色。',{'mock':True}
        def parse_text(self,text):return json.dumps(json.loads((ROOT/'tasks/four-part-text.json').read_text())['plan']),{'mock':True}
    d=PreviewPipeline(FakeCloud()).preview('mock.wav')
    assert d['plan']['counts']['green']==2 and d['human_confirmed'] is False

def test_main_window_has_no_fixture_or_history_buttons():
    w=Window()
    labels={button.text() for button in w.findChildren(QtWidgets.QPushButton)}
    assert labels=={'开始录音','结束录音','播放本地录音','确认模拟任务…','查看状态'}
    assert not hasattr(w,'history') and not hasattr(w,'fixture')
    assert w.asr.isReadOnly() and w.plan is None and not w.confirm.isEnabled()
    assert '未知' in w.robot.text()
    w.close()
