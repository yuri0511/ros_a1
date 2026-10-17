"""Local desktop recorder and provenance-aware plan preview. NO ROS or cloud client."""
import array, hashlib, json, math, os, sys, time, uuid, wave
from pathlib import Path
from PyQt5 import QtCore, QtWidgets
ROOT=Path('/home/yu/yuri_gazebo_performance/voice_batch_four')
HISTORY=ROOT.parent/'voice_batch_aliyun/audio/voice-primary'
class Window(QtWidgets.QWidget):
    def __init__(self):
        super().__init__(); self.setWindowTitle('Yuri A1 · 语音任务工作台'); self.resize(620,760)
        self.proc=None; self.pcm=bytearray(); self.wav=None; self.plan=None; self.submitted=False
        self.setStyleSheet('QWidget {font: 11pt "Noto Sans CJK SC"; background:#f5f7fb; color:#172b4d;} QGroupBox {font-weight:bold; border:1px solid #ccd6e5; border-radius:8px; margin-top:14px; padding:12px;} QGroupBox::title {subcontrol-origin:margin; left:12px;} QPushButton {background:#e1e9f7; padding:8px; border-radius:5px;} QPushButton:disabled {color:#8894a7;} QTextEdit {background:white;}')
        outer=QtWidgets.QVBoxLayout(self); scroll=QtWidgets.QScrollArea(); scroll.setWidgetResizable(True); outer.addWidget(scroll); content=QtWidgets.QWidget(); scroll.setWidget(content); layout=QtWidgets.QVBoxLayout(content)
        title=QtWidgets.QLabel('Yuri A1 智能双机械臂分拣系统'); title.setStyleSheet('font-size:19pt;font-weight:bold'); layout.addWidget(title)
        self.banner=QtWidgets.QLabel('模拟数据 / DRY-RUN · 云调用关闭 · ROS任务提交关闭'); self.banner.setStyleSheet('background:#fff0cb;padding:10px;font-weight:bold'); layout.addWidget(self.banner)
        def group(name):
            box=QtWidgets.QGroupBox(name); g=QtWidgets.QVBoxLayout(box); layout.addWidget(box); return g
        g=group('1  语音输入 · 真实本地麦克风'); row=QtWidgets.QHBoxLayout(); g.addLayout(row)
        self.start=QtWidgets.QPushButton('开始录音'); self.end=QtWidgets.QPushButton('结束录音'); self.play=QtWidgets.QPushButton('播放本地录音')
        for b in (self.start,self.end,self.play):row.addWidget(b)
        self.end.setEnabled(False); self.play.setEnabled(False); self.audio=QtWidgets.QLabel('麦克风未打开 · 0.0 秒'); g.addWidget(self.audio); self.level=QtWidgets.QProgressBar(); self.level.setRange(0,100); self.level.setFormat('输入音量 %p%'); g.addWidget(self.level)
        self.start.clicked.connect(self.record); self.end.clicked.connect(self.finish); self.play.clicked.connect(self.playback)
        g=group('2  语音识别 · 仅显示已保存记录'); self.asr=QtWidgets.QTextEdit(); self.asr.setReadOnly(True); self.asr.setMaximumHeight(70); g.addWidget(self.asr)
        self.connection=QtWidgets.QLabel('ASR未连接 · 本轮无付费调用 · 耗时：未测量'); g.addWidget(self.connection)
        row=QtWidgets.QHBoxLayout(); g.addLayout(row); self.history=QtWidgets.QPushButton('查看历史真实三绿五件'); self.fixture=QtWidgets.QPushButton('加载四件文字测试数据'); row.addWidget(self.history); row.addWidget(self.fixture)
        self.history.clicked.connect(self.load_history); self.fixture.clicked.connect(self.load_four)
        g=group('3  AI任务规划'); self.source=QtWidgets.QLabel('尚未加载计划'); self.source.setWordWrap(True); g.addWidget(self.source)
        self.table=QtWidgets.QLabel('黄色 —    红色 —    绿色 —    蓝色 —\n总任务 —    优先级 —'); self.table.setStyleSheet('font-size:15pt;'); g.addWidget(self.table)
        self.state=QtWidgets.QLabel('计划未确认'); g.addWidget(self.state)
        g=group('4  执行控制 · 只操作模拟任务'); row=QtWidgets.QHBoxLayout(); g.addLayout(row); self.confirm=QtWidgets.QPushButton('确认模拟任务…'); self.status=QtWidgets.QPushButton('查看状态'); row.addWidget(self.confirm); row.addWidget(self.status)
        self.confirm.setEnabled(False); self.confirm.clicked.connect(self.confirm_plan); self.status.clicked.connect(self.show_status)
        g=group('5  机器人执行状态'); self.robot=QtWidgets.QLabel('状态来源：无ROS连接\n工件编号 / 颜色 / 机械臂：未知\n送料 / 视觉 / 抓取 / 料箱：未知\n完成 / 失败件数：未观测\n安全状态：未知，不能据此启动机器人'); g.addWidget(self.robot)
        self.error=QtWidgets.QLabel(''); self.error.setWordWrap(True); self.error.setStyleSheet('color:#a12c27'); layout.addWidget(self.error)
        self.timer=QtCore.QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(100)
    def record(self):
        if self.proc is not None:return
        self.pcm=bytearray(); self.started=time.monotonic(); self.error.clear(); self.proc=QtCore.QProcess(self)
        self.proc.readyReadStandardOutput.connect(self.capture); self.proc.finished.connect(self.saved); self.proc.errorOccurred.connect(lambda _:self.error.setText('录音设备或进程错误；请检查Ubuntu声音输入设置'))
        self.proc.start('/usr/bin/arecord',['-q','-D','default','-t','raw','-f','S16_LE','-r','16000','-c','1'])
        self.start.setEnabled(False); self.end.setEnabled(True); self.play.setEnabled(False)
    def capture(self):
        if self.proc:self.pcm.extend(bytes(self.proc.readAllStandardOutput()))
    def tick(self):
        if self.proc:
            elapsed=time.monotonic()-self.started; self.audio.setText(f'麦克风录音中 · {elapsed:.1f} 秒（最长30秒）')
            samples=array.array('h',bytes(self.pcm[-3200:len(self.pcm)-len(self.pcm)%2]))
            rms=math.sqrt(sum(x*x for x in samples)/max(1,len(samples))); self.level.setValue(min(100,int(rms/32768*500)))
            if elapsed>=30:self.finish()
    def finish(self):
        if self.proc:
            old=self.proc; old.terminate(); QtCore.QTimer.singleShot(2000,lambda: self.kill_recording() if self.proc is old else None)
    def kill_recording(self):
        if self.proc:self.proc.kill()
    def saved(self,code,status):
        self.capture(); proc=self.proc; self.proc=None; self.start.setEnabled(True); self.end.setEnabled(False)
        data=bytes(self.pcm[:len(self.pcm)-len(self.pcm)%2])
        if not data:self.error.setText('没有收到音频，未保存有效录音'); proc.deleteLater(); return
        directory=ROOT/'ui_audio'/uuid.uuid4().hex; directory.mkdir(parents=True,mode=0o700); self.wav=directory/'microphone.wav'
        with wave.open(str(self.wav),'wb') as w:w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000); w.writeframes(data)
        os.chmod(self.wav,0o600); samples=array.array('h',data); rms=math.sqrt(sum(x*x for x in samples)/len(samples))
        (directory/'metadata.json').write_text(json.dumps({'source':'local_microphone_no_cloud','seconds':len(data)/32000,'rms':rms,'sha256':hashlib.sha256(self.wav.read_bytes()).hexdigest(),'api_calls':0},indent=2))
        self.audio.setText(f'已保存 {len(data)/32000:.2f} 秒 · RMS {rms:.1f}'); self.play.setEnabled(True); proc.deleteLater()
        if rms<1:self.error.setText('录音近乎静音，请检查麦克风输入；未进行语音识别。')
    def playback(self):
        if self.wav:QtCore.QProcess.startDetached('/usr/bin/aplay',[str(self.wav)])
    def display(self,plan,source,text,confirmable):
        plan=json.loads(plan) if isinstance(plan,str) else plan
        self.plan=plan; self.confirmable=confirmable; self.submitted=False; self.asr.setPlainText(text); self.source.setText(source); c=plan['counts']
        self.table.setText(f"黄色 {c['yellow']}    红色 {c['red']}    绿色 {c['green']}    蓝色 {c['blue']}\n总任务 {sum(c.values())}    优先级：黄色先完成放置 + ARM_READY")
        self.state.setText('等待人工确认（模拟）' if confirmable else '被安全拒绝：历史三绿五件仅供查阅，不能提交'); self.confirm.setEnabled(confirmable)
    def load_four(self):
        d=json.loads((ROOT/'tasks/four-part-text.json').read_text()); self.display(d['plan'],'模拟数据：新四件文字测试计划；不是本轮ASR或Qwen输出',d['transcript_unmodified'],True)
    def load_history(self):
        d=json.loads((HISTORY/'llm-parse-asr.json').read_text()); a=json.loads((HISTORY/'asr.json').read_text()); self.display(d.get('plan',d['raw_candidate']),'历史真实ASR + Qwen原始五件记录；与四件文字测试不同源',a['transcript'],False)
    def confirm_plan(self):
        if self.submitted or self.plan is None or not getattr(self,'confirmable',False):return
        if QtWidgets.QMessageBox.question(self,'二次确认（模拟数据）','确认只模拟提交黄1、红1、绿2？\n不会调用ROS，不会送料，不会运动。',QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No,QtWidgets.QMessageBox.No)!=QtWidgets.QMessageBox.Yes:return
        self.submitted=True; self.confirm.setEnabled(False); self.state.setText('已提交（仅模拟）；无真实执行结果')
    def show_status(self):self.error.setText('无ROS状态来源；真实执行中 / 已完成 / 执行失败均未观测。模拟确认不能代表物理READY。')
    def closeEvent(self,event):
        if self.proc:self.proc.terminate(); self.proc.waitForFinished(1500); self.kill_recording()
        event.accept()
if __name__=='__main__':
    app=QtWidgets.QApplication(sys.argv); w=Window(); w.show(); sys.exit(app.exec_())
