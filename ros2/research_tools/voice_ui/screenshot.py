"""Capture actual X11 Gazebo window; Xwayland root capture is not a desktop image."""
import json,re,subprocess,sys
from pathlib import Path
from PyQt5.QtWidgets import QApplication
app=QApplication(sys.argv)
tree=subprocess.check_output(['/usr/bin/xwininfo','-root','-tree'],text=True)
Path(sys.argv[1]+'.windows.txt').write_text(tree)
ids=[int(m.group(1),16) for line in tree.splitlines() if 'Gazebo' in line for m in [re.search(r'(0x[0-9a-f]+)',line)] if m]
if not ids:raise SystemExit('No visible Gazebo X11 window found')
image=app.primaryScreen().grabWindow(ids[0])
if image.isNull() or not image.save(sys.argv[1]):raise SystemExit('Gazebo window capture failed')
