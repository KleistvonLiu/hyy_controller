#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt

def main():
    csv_path = "/home/kleist/Documents/Code/temp/joint_target_state_log.csv"

    times = []
    target = []
    state = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

        # 自动忽略最后一行（无论它是否有效）
        for row in rows[:-1]:
            try:
                t = float(row["time"])
                tgt = float(row["target_joint"])
                st = float(row["state_joint"])
            except (KeyError, ValueError, TypeError):
                continue

            times.append(t)
            target.append(tgt)
            state.append(st)

    if not times:
        print("No data loaded from", csv_path)
        return

    plt.figure()
    plt.plot(times, target, label="target_joint")
    plt.plot(times, state, label="state_joint")
    plt.xlabel("time [s]")
    plt.ylabel("joint position [rad]")
    plt.title("Joint target vs. state")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()