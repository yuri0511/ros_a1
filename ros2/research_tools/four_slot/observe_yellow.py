"""Original real ROS/native observer with a strictly one-yellow task loader."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from yellow_task import load_task
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'voice_four'))
import observe_four
observe_four.load_task=load_task
observe_four.SUBMISSION_GATE=lambda run:(run/'GUI_CONFIRMED.json').exists()
if __name__=='__main__':observe_four.main()
