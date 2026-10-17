"""Experimental independent-finger effort control; not a joint-state setter.

Uses the existing effort controller. 150 N/m was first exercised by the empty
finger preconditioning calibration. +/-1.5 N preserves the candidate release
force cap. Only gate-2 calibration uses this helper until loaded exit is verified.
"""
import math

def opening_efforts(positions, target=.012):
    if len(positions)!=2 or not all(math.isfinite(q) for q in positions):
        raise ValueError('invalid_two_finger_feedback')
    if not .010<=target<=.012:raise ValueError('target_outside_calibration_range')
    return [max(-1.5,min(1.5,150*(target-q))) for q in positions]
