"""Execution gate distinct from cloud-plan syntax validation.

No operator/launch bypass. Green third-slot mesh/SAT risk is unresolved in this
revision. Clearing it requires reviewed geometry/path evidence and a new commit.
"""
from .voice_batch_plan import PlanRejected

def ensure_geometry_review(plan):
    if plan.counts[3]>=3:raise PlanRejected('green_third_slot_safety_not_cleared')
