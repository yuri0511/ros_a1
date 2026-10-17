"""Offline top-view schematic; explicitly not a Gazebo screenshot."""
import os,json
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from pathlib import Path
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QImage,QPainter,QPen,QColor,QFont
from PyQt5.QtCore import QRectF
root=Path(__file__).resolve().parents[4];cfg=json.loads((root/'worktree/ros2/src/a1_sorting_core/config/four_slot_candidate.json').read_text())
app=QApplication([]);im=QImage(1200,820,QImage.Format_RGB32);im.fill(QColor('#f5f7fa'));p=QPainter(im);p.setRenderHint(QPainter.Antialiasing);p.setFont(QFont('Sans',15));p.setPen(QColor('#18233a'));p.drawText(30,40,'OFFLINE GEOMETRY CANDIDATE — NOT A GAZEBO EXECUTION SCREENSHOT');p.setFont(QFont('Sans',11));p.drawText(30,68,'240 mm outer / 220 mm inner / 95 mm height / 90 mm slot pitch — dynamic gate BLOCKED')
def rect(x,y,w,h):return QRectF(80+(x-.05)*850,115+(.4-y-h)*700,w*850,h*700)
p.setBrush(QColor('#dfe5ec'));p.drawRect(rect(.05,-.4,1.2,.8))
colors={'blue':'#448fe5','yellow':'#e6c543','red':'#da6363','green':'#5aaa83'}
for bn,b in cfg['bins'].items():
 x,y=b['center'];color=colors[bn.removeprefix('bin_')];p.setBrush(QColor(color));p.drawRect(rect(x-.12,y-.12,.24,.24));p.setBrush(QColor('#ffffff'));p.drawRect(rect(x-.11,y-.11,.22,.22))
 for s in b['slots']:
  sx,sy,_=s['xyz'];p.setBrush(QColor(color));p.drawRect(rect(sx-.025,sy-.025,.05,.05));p.setPen(QColor('#172033'));r=rect(sx-.025,sy-.025,.05,.05);p.drawText(r,0x84,str(s['index']))
 p.drawText(rect(x-.12,y+.12,.24,.03),0x84,bn)
p.setBrush(QColor('#b8c2cf'));p.drawRect(rect(.15,-.05,.35,.1));p.drawText(480,405,'belt (schematic)');p.setPen(QColor('#9e2938'));p.drawText(35,740,'No physical four-part capacity or five-part speech task has been validated in this candidate.');p.setPen(QColor('#18233a'));p.drawText(35,775,'Squares represent nominal 50 mm cubes. Full gripper and trajectory review is in the audit evidence.');p.end();im.save(str(root/'audit/OFFLINE_LAYOUT.png'))
