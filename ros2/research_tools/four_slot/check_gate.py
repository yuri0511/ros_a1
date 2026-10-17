"""Read-only gate summary. Deliberately has no simulator or actuator launcher."""
from pathlib import Path
import argparse,json
parser=argparse.ArgumentParser();parser.add_argument('--request-demo',action='store_true');args=parser.parse_args()
root=Path(__file__).resolve().parents[4]
gate=json.loads((root/'audit/gate-status.json').read_text())
print(json.dumps(gate,ensure_ascii=False,indent=2))
if args.request_demo:
    raise SystemExit('BLOCKED: stage 2 incomplete. No Gazebo or motion process was started.')
