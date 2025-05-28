import json
import os
import subprocess

# File where verified coordinates will be stored
json_file = "verified_coordinates.json"

# Load existing data or create new structure
if os.path.exists(json_file):
    with open(json_file, "r") as f:
        verified_coords = json.load(f)
else:
    verified_coords = {}

# Send command to the robot using ROS 2 action CLI
def send_to_robot(x, y, z, angle):
    command = f"""ros2 action send_goal /follow_ik_trajectory control_msgs/action/FollowJointTrajectory "{{trajectory: {{ points: [{{positions: [{x}, {y}, {z}, {angle}]}}]}}}}" """
    print(f"📤 Sending command to robot: x={x}, y={y}, z={z}, angle={angle}")
    subprocess.run(command, shell=True)

def save_to_json(id_str, x, y, z, angle):
    verified_coords[id_str] = [x, y, z, angle]
    with open(json_file, "w") as f:
        json.dump(verified_coords, f, indent=4)
    print(f"✅ Saved under ID {id_str}")

def prompt_for_coords():
    x = float(input("➡️ Enter x: "))
    y = float(input("➡️ Enter y: "))
    z = input("➡️ Enter z [default = 0.07]: ") or "0.07"
    angle = input("➡️ Enter angle [default = -1.57]: ") or "-1.57"
    return float(x), float(y), float(z), float(angle)

def main():
    print("🔧 Interactive Position Verification Program")

    while True:
        id_input = input("\n➡️ Enter ID (0-223) or 'h' for home: ").strip().lower()

        if id_input == 'h':
            send_to_robot(0.01, 0.1, 0.3, 0.0)
            continue

        if not id_input.isdigit() or not (0 <= int(id_input) <= 223):
            print("❌ Invalid ID. Please enter a number between 0 and 223.")
            continue

        id_str = str(id_input)

        if id_str in verified_coords:
            choice = input("ℹ️ ID exists. [v = verify / enter = skip]: ").strip().lower()
            if choice == "v":
                x, y, z, angle = verified_coords[id_str]
                send_to_robot(x, y, z, angle)
                confirm = input("❓ Is this still correct? [y / n]: ").strip().lower()
                if confirm == 'y':
                    print("✅ Position confirmed.")
                    continue
                elif confirm == 'n':
                    edit = input("✏️ Do you want to edit the coordinates? [y / n]: ").strip().lower()
                    if edit == 'y':
                        x, y, z, angle = prompt_for_coords()
                        send_to_robot(x, y, z, angle)
                        final_check = input("❓ Is this correct? [y / n / id]: ").strip().lower()
                        if final_check == 'y':
                            save_to_json(id_str, x, y, z, angle)
                        elif final_check.isdigit():
                            if final_check in verified_coords:
                                print("⚠️ That ID already exists. Going back to main menu.")
                            else:
                                save_to_json(final_check, x, y, z, angle)
                        else:
                            print("🔁 Discarded. Returning to main menu.")
                    continue
            else:
                continue  # skip verification, return to main menu

        # New ID path
        x, y, z, angle = prompt_for_coords()
        send_to_robot(x, y, z, angle)
        correct = input("❓ Is this correct? [y / n / id]: ").strip().lower()

        if correct == "y":
            save_to_json(id_str, x, y, z, angle)
        elif correct.isdigit():
            if correct in verified_coords:
                print("⚠️ That ID already exists. Discarded.")
            else:
                save_to_json(correct, x, y, z, angle)
        else:
            print("🗑️ Discarded. Returning to main menu.")

if __name__ == "__main__":
    main()
