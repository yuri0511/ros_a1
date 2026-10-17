"""Read-only same-partition GUI/camera prerequisite, before any task submit."""
import os,json,re,subprocess,time
from pathlib import Path

def _confirm(run,partition):
    run=Path(run)
    if (run/'GUI_CONFIRMED.json').exists():return True
    state=json.loads((run/'observed-status.json').read_text()) if (run/'observed-status.json').exists() else {}
    if state.get('rgb_frames',0)<3 or state.get('logical_frames',0)<3 or state.get('stats_messages',0)<3:return False
    rgb=state.get('rgb_summary',{})
    if not rgb and (run/'rgb-health.json').exists():rgb=json.loads((run/'rgb-health.json').read_text())
    if rgb.get('max',0)-rgb.get('min',0)<20 or rgb.get('std',0)<5:return False
    tree=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True,timeout=2).stdout
    for line in tree.splitlines():
        if not re.search('gazebo',line,re.I):continue
        m=re.search(r'(0x[0-9a-f]+)',line)
        if not m:continue
        wid=m.group(1);prop=subprocess.run(['xprop','-id',wid,'_NET_WM_PID'],capture_output=True,text=True,timeout=2).stdout
        match=re.search(r'= (\d+)',prop)
        if not match:continue
        pid=match.group(1)
        try:env=Path('/proc',pid,'environ').read_bytes().split(b'\0')
        except OSError:continue
        if ('GZ_PARTITION='+partition).encode() not in env:continue
        details=subprocess.run(['xwininfo','-id',wid],capture_output=True,text=True,timeout=2).stdout
        if 'IsViewable' not in details:continue
        import ctypes
        x=ctypes.CDLL('libX11.so.6');x.XOpenDisplay.restype=ctypes.c_void_p;x.XRaiseWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong];x.XFlush.argtypes=[ctypes.c_void_p];x.XCloseDisplay.argtypes=[ctypes.c_void_p]
        d=x.XOpenDisplay(None)
        if not d:return False
        x.XRaiseWindow(d,int(wid,16));x.XFlush(d);x.XCloseDisplay(d);time.sleep(.2)
        def number(label):return int(re.search(re.escape(label)+r':\s+(-?\d+)',details).group(1))
        a,b=number('Absolute upper-left X'),number('Absolute upper-left Y');w,h=number('Width'),number('Height')
        from PIL import ImageGrab
        try:
            ImageGrab.grab(bbox=(a,b,a+w,b+h),xdisplay=os.environ.get('DISPLAY')).save(run/'gazebo-visible.png')
        except OSError:
            # XWayland denies root drawable capture; read only the owned GUI window.
            class XImage(ctypes.Structure):
                _fields_=[('width',ctypes.c_int),('height',ctypes.c_int),('xoffset',ctypes.c_int),('format',ctypes.c_int),('data',ctypes.c_void_p),('byte_order',ctypes.c_int),('bitmap_unit',ctypes.c_int),('bitmap_bit_order',ctypes.c_int),('bitmap_pad',ctypes.c_int),('depth',ctypes.c_int),('bytes_per_line',ctypes.c_int),('bits_per_pixel',ctypes.c_int),('red_mask',ctypes.c_ulong),('green_mask',ctypes.c_ulong),('blue_mask',ctypes.c_ulong)]
            x.XGetImage.restype=ctypes.POINTER(XImage);x.XGetImage.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint,ctypes.c_ulong,ctypes.c_int];x.XDestroyImage.argtypes=[ctypes.POINTER(XImage)]
            d=x.XOpenDisplay(None);img=x.XGetImage(d,int(wid,16),0,0,w,h,ctypes.c_ulong(-1).value,2)
            if not img:x.XCloseDisplay(d);return False
            try:
                m=img.contents
                if m.bits_per_pixel!=32 or (m.red_mask,m.green_mask,m.blue_mask)!=(0xff0000,0xff00,0xff):raise RuntimeError('unsupported_GUI_pixel_format')
                from PIL import Image
                raw=ctypes.string_at(m.data,m.bytes_per_line*m.height);Image.frombytes('RGB',(m.width,m.height),raw,'raw','BGRX',m.bytes_per_line).save(run/'gazebo-visible.png')
            finally:x.XDestroyImage(img);x.XCloseDisplay(d)
        record=dict(window=wid,pid=int(pid),partition=partition,window_details=details,observed_clock=state.get('latest_clock'),rgb_frames=state['rgb_frames'],logical_frames=state['logical_frames'],wall=time.time(),same_partition=True)
        (run/'GUI_CONFIRMED.json').write_text(json.dumps(record,indent=2));return True
    return False

def confirm(run,partition):
    # Xlib failures must never terminate the owning simulator supervisor.
    run=Path(run)
    if (run/'GUI_CONFIRMED.json').exists():return True
    import sys
    try:
        child=subprocess.run([sys.executable,str(Path(__file__).resolve()),str(run.resolve()),partition],capture_output=True,text=True,timeout=5)
        if child.returncode:
            (run/'gui-capture-error.log').write_text(child.stdout+child.stderr)
            return False
        return (run/'GUI_CONFIRMED.json').exists()
    except subprocess.TimeoutExpired:return False

if __name__=='__main__':
    import sys
    raise SystemExit(0 if _confirm(sys.argv[1],sys.argv[2]) else 1)
