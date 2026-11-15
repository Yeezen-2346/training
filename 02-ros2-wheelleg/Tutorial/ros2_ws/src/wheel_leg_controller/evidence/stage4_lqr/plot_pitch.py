import math
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

base = Path(__file__).resolve().parent
bag_path = base / "stage4_lqr_balance"

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id="sqlite3"
    ),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
)

topics = {
    item.name: item.type
    for item in reader.get_all_topics_and_types()
}

imu_type = get_message(topics["/imu"])

samples = []

while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic != "/imu":
        continue

    msg = deserialize_message(data, imu_type)
    q = msg.orientation

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.degrees(
        math.asin(max(-1.0, min(1.0, sinp)))
    )

    samples.append((timestamp, pitch))

if not samples:
    raise RuntimeError("No IMU samples found")

samples.sort(key=lambda x: x[0])

t0 = samples[0][0]
times = [(t - t0) / 1e9 for t, _ in samples]
pitches = [p for _, p in samples]

csv_path = base / "pitch_data.csv"

with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time_s", "pitch_deg"])
    writer.writerows(zip(times, pitches))

plt.figure(figsize=(11, 5))
plt.plot(times, pitches, linewidth=0.8, label="Measured pitch")
plt.axhline(10, color="red", linestyle="--", label="+10 deg")
plt.axhline(-10, color="red", linestyle="--", label="-10 deg")
plt.axhline(0, color="gray", linewidth=0.6)

plt.xlabel("Recording time (s)")
plt.ylabel("Pitch (deg)")
plt.title("Stage 4 - LQR Balance Pitch")
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()

png_path = base / "pitch_curve.png"
plt.savefig(png_path, dpi=200)

max_pitch = max(abs(p) for p in pitches)

print("===== STAGE 4 BAG ANALYSIS =====")
print(f"IMU samples: {len(samples)}")
print(f"Recording span: {times[-1]:.3f} s")
print(f"Max absolute pitch: {max_pitch:.4f} deg")
print(f"All sampled pitch within 10 deg: {max_pitch < 10}")
print(f"CSV: {csv_path}")
print(f"PNG: {png_path}")
print("================================")
