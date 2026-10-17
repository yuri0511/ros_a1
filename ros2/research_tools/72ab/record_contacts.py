#!/usr/bin/env python3
"""Record Gazebo contact points from /a1_contact topics.

Sensors only observe collisions that already exist. This script does not
publish anything back into the simulation.
"""
import csv
import json
import resource
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node

CLOCK_TOPIC = '/world/sorting_cell/clock'
TOPICS = [
    '/a1_contact/pedestal',
    '/a1_contact/shoulder_link',
    '/a1_contact/upper_arm_link',
    '/a1_contact/forearm_link',
    '/a1_contact/wrist_1_link',
    '/a1_contact/wrist_2_link',
    '/a1_contact/wrist_3_link',
    '/a1_contact/gripper_finger_left_link',
    '/a1_contact/gripper_finger_right_link',
    '/a1_contact/pedestal2',
    '/a1_contact/arm2_shoulder_link',
    '/a1_contact/arm2_upper_arm_link',
    '/a1_contact/arm2_forearm_link',
    '/a1_contact/arm2_wrist_1_link',
    '/a1_contact/arm2_wrist_2_link',
    '/a1_contact/arm2_wrist_3_link',
    '/a1_contact/arm2_gripper_finger_left_link',
    '/a1_contact/arm2_gripper_finger_right_link',
]


def main() -> None:
    if len(sys.argv) != 2:
        print('usage: record_contacts.py OUT.csv', file=sys.stderr)
        sys.exit(2)
    out = Path(sys.argv[1])
    beat = out.with_suffix('.heartbeat')
    handle = out.open('w', newline='')
    writer = csv.writer(handle)
    writer.writerow([
        'wall_iso', 'sim_sec', 'topic', 'collision1', 'collision2',
        'x', 'y', 'z', 'depth',
    ])
    handle.flush()
    lock = threading.Lock()
    rows = 0
    nonempty = 0
    first_sim = None
    latest_sim = None

    def on_clock(msg: Clock) -> None:
        nonlocal first_sim, latest_sim
        sim = msg.sim.sec + msg.sim.nsec * 1e-9
        with lock:
            latest_sim = sim
            if first_sim is not None:
                return
            first_sim = sim
            text = f'first_sim={sim:.6f}\n'
        beat.write_text(text)
        print(f'FIRST_SIM {sim:.6f}', flush=True)

    def on_contacts(topic):
        def callback(msg: Contacts) -> None:
            nonlocal rows, nonempty
            stamp = msg.header.stamp
            sim = stamp.sec + stamp.nsec * 1e-9
            wall = datetime.now(timezone.utc).isoformat()
            batch = []
            for contact in msg.contact:
                if not contact.collision1 and not contact.collision2:
                    continue
                points = list(contact.position) or [None]
                for index, point in enumerate(points):
                    depth = ''
                    if index < len(contact.depth):
                        depth = f'{contact.depth[index]:.6f}'
                    if point is None:
                        x = y = z = ''
                    else:
                        x = f'{point.x:.5f}'
                        y = f'{point.y:.5f}'
                        z = f'{point.z:.5f}'
                    batch.append([
                        wall, f'{sim:.3f}', topic,
                        contact.collision1.name, contact.collision2.name,
                        x, y, z, depth,
                    ])
            if not batch:
                return
            with lock:
                writer.writerows(batch)
                handle.flush()
                rows += len(batch)
                nonempty += 1
        return callback

    poses = {}
    def on_poses(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec*1e-9
        with lock:
            for pose in msg.pose:
                if pose.name.startswith('part_') and pose.name[5:].isdigit():
                    poses[pose.name] = {'xyz':[pose.position.x,pose.position.y,pose.position.z], 'last_pose_sim':stamp}
    node = Node()
    for topic in ('/world/sorting_cell/pose/info','/world/sorting_cell/dynamic_pose/info'):
        node.subscribe(Pose_V,topic,on_poses)
    pending = list(TOPICS)
    clock_ok = False
    deadline = time.monotonic() + 300.0
    while pending or not clock_ok:
        if time.monotonic() > deadline:
            print('contact topics not ready within 300s', file=sys.stderr)
            sys.exit(1)
        still = []
        for topic in pending:
            ok = bool(node.subscribe(Contacts, topic, on_contacts(topic)))
            print(f'subscribe {topic} ok={ok}', flush=True)
            if not ok:
                still.append(topic)
        if not clock_ok:
            clock_ok = bool(node.subscribe(Clock, CLOCK_TOPIC, on_clock))
            print(f'subscribe {CLOCK_TOPIC} ok={clock_ok}', flush=True)
        pending = still
        if pending or not clock_ok:
            time.sleep(1.0)

    (out.parent/'CONTACT_READY').touch()
    while not (out.parent/'CONTACT_STOP').exists():
        time.sleep(1)
        with lock:
            count = rows
            messages = nonempty
            first = first_sim
            latest = latest_sim
        with lock:
            snapshot=dict(poses)
        (out.parent/'tracked-positions.json').write_text(json.dumps({'source':'latest native world model pose before configured 12-sim-second recycle','parts':snapshot}))
        with (out.parent/'contact-health.jsonl').open('a') as health:
            health.write(json.dumps({'wall':time.time(),'monotonic':time.monotonic(),'process_cpu_s':time.process_time(),'max_rss_kib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,'rows':count,'messages':messages,'latest_sim':latest})+'\n')
        beat.write_text(
            f'first_sim={first} latest_sim={latest} '
            f'rows={count} messages={messages} '
            f'wall={datetime.now(timezone.utc).isoformat()}\n'
        )


if __name__ == '__main__':
    main()
