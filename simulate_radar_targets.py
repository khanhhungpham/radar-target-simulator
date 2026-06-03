import csv
import json
import math
import os
import random
import sys
from datetime import datetime, timedelta

from geopy.distance import distance

# --- CÁC HẰNG SỐ CẤU HÌNH NHIỄU SENSOR (QUY TẮC 3-SIGMA) ---
SIGMA_DIST_M = 15.0 / 3.0  # Đơn vị: Mét (m)
SIGMA_BEARING_DEG = 0.7 / 3.0  # Đơn vị: Độ (deg)
SIGMA_SPEED_KNOTS = 3.0 / 3.0  # Đơn vị: Hải lý / giờ (knots)
SIGMA_COURSE_DEG = 10.0 / 3.0  # Đơn vị: Độ (deg)


def load_input_config(file_path="input.json"):
    """Đọc cấu hình đầu vào từ file JSON."""
    if not os.path.exists(file_path):
        print(
            f"LỖI: Không tìm thấy file cấu hình '{file_path}'. Vui lòng tạo file trước khi chạy."
        )
        sys.exit(1)

    with open(file_path, "r", encoding="utf-8") as f:
        return json.load(f)


def generate_initial_targets(num_targets, radars):
    """Khởi tạo mục tiêu nằm trong vùng quét của ít nhất một radar."""
    targets = []
    for i in range(num_targets):
        ref_radar = random.choice(radars)
        ref_lat = ref_radar["lat"]
        ref_lon = ref_radar["lon"]
        max_dist_km = ref_radar["range_km"]

        dist_km = random.uniform(0, max_dist_km)
        bearing = random.uniform(0, 360)
        origin = (ref_lat, ref_lon)
        target_pos = distance(kilometers=dist_km).destination(origin, bearing)

        speed_knots = random.uniform(5.0, 28.0)
        speed_mps = speed_knots * 0.514444
        course = random.uniform(0.0, 360.0)

        targets.append(
            {
                "true_id": i + 1,
                "lat": target_pos.latitude,
                "lon": target_pos.longitude,
                "speed_mps": speed_mps,
                "speed_knots": speed_knots,
                "course": course,
            }
        )
    return targets


def update_target_positions(targets, dt):
    """Cập nhật vị trí, vận tốc và hướng mục tiêu di chuyển (Smooth Random Walk)."""
    for t in targets:
        course_change = random.uniform(-2.0, 2.0)
        t["course"] = (t["course"] + course_change) % 360.0

        speed_change = random.uniform(-0.2, 0.2)
        new_speed_knots = t["speed_knots"] + speed_change

        t["speed_knots"] = max(5.0, min(30.0, new_speed_knots))
        t["speed_mps"] = t["speed_knots"] * 0.514444

        dist_moved_km = (t["speed_mps"] * dt) / 1000.0
        current_pos = (t["lat"], t["lon"])
        next_pos = distance(kilometers=dist_moved_km).destination(
            current_pos, t["course"]
        )

        t["lat"] = next_pos.latitude
        t["lon"] = next_pos.longitude


def calculate_radar_detection(radar, target):
    """Tính toán thông số nếu mục tiêu nằm trong tầm quét bằng Geopy (Độ chính xác cao)."""
    radar_pos = (radar["lat"], radar["lon"])
    target_pos = (target["lat"], target["lon"])
    dist_km = distance(radar_pos, target_pos).kilometers

    if dist_km <= radar["range_km"]:
        lat1, lon1 = math.radians(radar["lat"]), math.radians(radar["lon"])
        lat2, lon2 = math.radians(target["lat"]), math.radians(target["lon"])

        d_lon = lon2 - lon1
        y = math.sin(d_lon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(
            lat2
        ) * math.cos(d_lon)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

        return {"distance_km": dist_km, "bearing_deg": bearing}
    return None


def main():
    config = load_input_config()
    radars = config["radars"]
    num_targets = config["num_targets"]
    duration = config.get("simulation_duration_s", 60)

    start_time = datetime.now()

    dir_name = start_time.strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join("outputs", dir_name)
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "radar_target_data.csv")

    targets = generate_initial_targets(num_targets, radars)

    csv_headers = [
        "timestamp",
        "radar_id",
        "local_target_id",
        "true_target_id",
        "distance_km",
        "bearing_deg",
        "speed_knots",
        "course_deg",
        "lat",
        "lon",
    ]

    # Mở file ghi luồng trực tiếp theo từng giây
    with open(output_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(csv_headers)

        for step in range(duration + 1):
            current_sim_time = start_time + timedelta(seconds=step)
            timestamp_str = current_sim_time.strftime("%Y-%m-%d %H:%M:%S")

            step_records = []

            for radar in radars:
                r_id = int(radar["id"])
                scan_period = radar["scan_period_s"]

                current_position_in_cycle = step % scan_period
                min_angle = (current_position_in_cycle / scan_period) * 360.0
                max_angle = ((current_position_in_cycle + 1) / scan_period) * 360.0
                normalized_max = max_angle % 360.0

                for t in targets:
                    detection = calculate_radar_detection(radar, t)
                    if detection:
                        true_bearing = detection["bearing_deg"]

                        if min_angle < max_angle and max_angle <= 360.0:
                            is_in_scan_sector = min_angle <= true_bearing < max_angle
                        else:
                            is_in_scan_sector = (
                                true_bearing >= min_angle
                                or true_bearing < normalized_max
                            )

                        if is_in_scan_sector:
                            true_id = t["true_id"]
                            local_id = true_id + r_id * 1000

                            # SỬ DỤNG HẰNG SỐ CHỮ HOA TOÀN CỤC Ở ĐÂY
                            noise_dist_km = random.gauss(0, SIGMA_DIST_M) / 1000.0
                            noisy_dist = max(
                                0.0, detection["distance_km"] + noise_dist_km
                            )

                            noisy_bearing = (
                                true_bearing + random.gauss(0, SIGMA_BEARING_DEG)
                            ) % 360.0
                            noisy_speed = max(
                                0.0,
                                t["speed_knots"] + random.gauss(0, SIGMA_SPEED_KNOTS),
                            )
                            noisy_course = (
                                t["course"] + random.gauss(0, SIGMA_COURSE_DEG)
                            ) % 360.0

                            radar_pos = (radar["lat"], radar["lon"])
                            noisy_pos = distance(kilometers=noisy_dist).destination(
                                radar_pos, noisy_bearing
                            )

                            step_records.append(
                                [
                                    timestamp_str,
                                    r_id,
                                    local_id,
                                    true_id,
                                    round(noisy_dist, 3),
                                    round(noisy_bearing, 2),
                                    round(noisy_speed, 2),
                                    round(noisy_course, 2),
                                    round(noisy_pos.latitude, 6),
                                    round(noisy_pos.longitude, 6),
                                ]
                            )

            if step_records:
                writer.writerows(step_records)

            if step < duration:
                update_target_positions(targets, dt=1)

    print(f"Mô phỏng hoàn tất! Dữ liệu đã được lưu tại: {output_file}")


if __name__ == "__main__":
    main()
