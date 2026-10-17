"""Request orderly stop of the owning four-part supervisor, without ROS/pkill."""
import json
from pathlib import Path
root=Path(__file__).resolve().parents[4]
latest=root/'latest-run.json'
if not latest.exists():raise SystemExit('No four-part simulation run exists.')
run=Path(json.loads(latest.read_text())['run']).resolve()
if not run.is_relative_to((root/'runs').resolve()):raise SystemExit('Unexpected run path; inspect metadata manually.')
(run/'STOP').write_text('operator requested controlled stop\n')
print('Stop requested: '+str(run))
