"""Read-only final CSV checks against the confirmed dynamic plan."""
import csv
from collections import Counter
from .voice_batch_plan import PlanRejected,COLORS
from .voice_batch_state import ARM_FOR

def validate_batch_results(path,plan):
    with open(path,newline='',encoding='utf-8') as f:rows=list(csv.DictReader(f))
    if len(rows)!=plan.total or len({r['part_id'] for r in rows})!=plan.total or any(not r['part_id'] for r in rows):raise PlanRejected('csv_count_or_duplicate_part')
    if [r['category'] for r in rows]!=list(plan.order):raise PlanRejected('csv_order_or_color_mismatch')
    for row in rows:
        color=row['category']
        if row['result']!='PLACED' or row['target_bin']!='bin_'+color or row['final_bin']!=row['target_bin'] or row['arm']!=ARM_FOR[color]:raise PlanRejected('csv_placement_or_arm_mismatch')
    return {'rows':len(rows),'unique_parts':len({r['part_id'] for r in rows}),'bin_counts':dict(Counter(r['final_bin'] for r in rows)),'first_color':rows[0]['category'],'all_placed':True}
