#!/usr/bin/env python3

import json
import os

verified_file = 'verified_coordinates.json'

# Load existing verified data or create empty
if os.path.exists(verified_file):
    with open(verified_file, 'r') as f:
        verified_data = json.load(f)
else:
    verified_data = {}

# IDs from 223 down to 0
all_ids = list(map(str, range(223, -1, -1)))

for square_id in all_ids:
    if square_id in verified_data:
        continue  # Skip already verified

    print(f"\n➡️ Verifying ID {square_id}")

    action = input("↪️ Press Enter to continue, or type 'h' for home: ").strip().lower()
    if action == 'h':
        os.system('ros2 action send_goal /follow_ik_trajectory control_msgs/action/FollowJointTrajectory '
                  '"{trajectory: { points: [{positions: [0.01, 0.1, 0.3, 0.0]}]}}"')
        continue

    try:
        x = float(input("🔢 Enter X: "))
        y = float(input("🔢 Enter Y: "))
        z_input = input("🔢 Enter Z (default 0.07): ")
        z = float(z_input) if z_input else 0.07

        angle_input = input("🌀 Enter angle (default -1.57): ")
        angle = float(angle_input) if angle_input else -1.57

        # Send command to robot
        cmd = f'ros2 action send_goal /follow_ik_trajectory control_msgs/action/FollowJointTrajectory ' \
              f'"{{trajectory: {{ points: [{{positions: [{x}, {y}, {z}, {angle}]}}]}}}}"'
        os.system(cmd)

        feedback = input(f"❓ Is this correct for ID {square_id}? [y / n / overwrite_id]: ").strip().lower()

        if feedback == 'y':
            verified_data[square_id] = [x, y, z, angle]
            print(f"✔️ Saved under ID {square_id}")
        elif feedback.isdigit():
            overwrite_id = feedback
            if overwrite_id in verified_data:
                confirm = input(f"⚠️ ID {overwrite_id} already exists. Overwrite? [y/n]: ").strip().lower()
                if confirm != 'y':
                    print("❌ Skipped.")
                    continue
            verified_data[overwrite_id] = [x, y, z, angle]
            print(f"✔️ Saved under ID {overwrite_id}")
        else:
            print("❌ Discarded. Try again.")
            continue

        # Save updated verified data
        with open(verified_file, 'w') as f:
            json.dump(verified_data, f, indent=2)

    except Exception as e:
        print(f"⚠️ Error: {e}. Skipping ID {square_id}")
