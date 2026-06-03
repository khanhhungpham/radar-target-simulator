import csv
import json
import math
import os
import random
import sys
from datetime import datetime, timedelta

from geopy.distance import distance

# --- CÁC HẰNG SỐ CẤU HÌNH NHIỄU SENSOR (QUY TẮC 3-SIGMA) ---
# Đây chính là sai số đo tối đa của thiết bị Radar (Sai số hệ thống)
MAX_ERROR_DIST_M = 15.0
MAX_ERROR_BEARING_DEG = 0.7
MAX_ERROR_SPEED_KNOTS = 3.0  # Sai số đo tốc độ tối đa của Radar: 3 knots
MAX_ERROR_COURSE_DEG = 10.0  # Sai số đo hướng đi tối đa của Radar: 10 độ

# Tính toán tự động giá trị Sigma đầu vào phục vụ hàm random.gauss
SIGMA_DIST_M = MAX_ERROR_DIST_M / 3.0
SIGMA_BEARING_DEG = MAX_ERROR_BEARING_DEG / 3.0
SIGMA_SPEED_KNOTS = MAX_ERROR_SPEED_KNOTS / 3.0
SIGMA_COURSE_DEG = MAX_ERROR_COURSE_DEG / 3.0

# --- CẤU HÌNH ĐỘ PHÂN GIẢI CỦA RADAR (RADAR RESOLUTION) ---
MIN_RESOLUTION_DIST_M = 100.0  # Ngưỡng vật lý phần cứng (độ rộng xung)
MIN_RESOLUTION_BEARING_DEG = 3.0  # Ngưỡng vật lý phần cứng (độ rộng búp sóng)

# Tự động đồng bộ ngưỡng phân biệt vận tốc/hướng đi dựa trên chính sai số đo của Radar
MIN_RESOLUTION_SPEED_KNOTS = MAX_ERROR_SPEED_KNOTS  # Ngưỡng trùng tốc độ: 3.0 knots
MIN_RESOLUTION_COURSE_DEG = MAX_ERROR_COURSE_DEG  # Ngưỡng trùng hướng đi: 10.0 độ


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
    """
    Khởi tạo mục tiêu di chuyển.
    Luôn cài cắm sẵn 3 cặp mục tiêu đặc biệt để kiểm tra tính năng phân giải (Resolution):
      - Cặp 1 (ID 1 & 2): KHÔNG PHÂN BIỆT ĐƯỢC (Trùng vị trí, trùng vận tốc & hướng)
      - Cặp 2 (ID 3 & 4): PHÂN BIỆT ĐƯỢC do KHÁC ĐỘ LỚN vận tốc
      - Cặp 3 (ID 5 & 6): PHÂN BIỆT ĐƯỢC do KHÁC HƯỚNG di chuyển
    """
    targets = []

    # Lấy radar đầu tiên làm trạm mốc để đặt các cặp tàu test vào tầm quét
    ref_radar = radars[0]
    ref_lat, ref_lon = ref_radar["lat"], ref_radar["lon"]
    origin = (ref_lat, ref_lon)

    # =========================================================================
    # CẶP 1 (ID 1 & 2): KHÔNG PHÂN BIỆT ĐƯỢC (Radar sẽ gộp làm 1)
    # Vị trí lệch 50m (<100m), Bearing lệch 0.1 độ (<3°), Vận tốc & Hướng giống hệt
    # =========================================================================
    pos_t1 = distance(kilometers=10.0).destination(origin, 30.0)
    targets.append(
        {
            "true_id": 1,
            "lat": pos_t1.latitude,
            "lon": pos_t1.longitude,
            "speed_mps": 15.0 * 0.514444,
            "speed_knots": 15.0,
            "course": 45.0,
        }
    )

    pos_t2 = distance(kilometers=10.05).destination(origin, 30.1)
    targets.append(
        {
            "true_id": 2,
            "lat": pos_t2.latitude,
            "lon": pos_t2.longitude,
            "speed_mps": 15.0 * 0.514444,
            "speed_knots": 15.0,
            "course": 45.0,  # Trùng vector vận tốc
        }
    )

    # =========================================================================
    # CẶP 2 (ID 3 & 4): PHÂN BIỆT ĐƯỢC DO KHÁC ĐỘ LỚN VẬN TỐC (Speed)
    # Vị trí lệch 50m (<100m), Bearing lệch 0.1 độ (<3°), Hướng giống hệt nhưng Tốc độ lệch 5 knots (>3.0)
    # =========================================================================
    pos_t3 = distance(kilometers=15.0).destination(origin, 60.0)
    targets.append(
        {
            "true_id": 3,
            "lat": pos_t3.latitude,
            "lon": pos_t3.longitude,
            "speed_mps": 12.0 * 0.514444,
            "speed_knots": 12.0,
            "course": 90.0,
        }
    )

    pos_t4 = distance(kilometers=15.05).destination(origin, 60.1)
    targets.append(
        {
            "true_id": 4,
            "lat": pos_t4.latitude,
            "lon": pos_t4.longitude,
            "speed_mps": 17.2 * 0.514444,
            "speed_knots": 17.2,
            "course": 90.0,  # Khác tốc độ (lệch 5.2 knots)
        }
    )

    # =========================================================================
    # CẶP 3 (ID 5 & 6): PHÂN BIỆT ĐƯỢC DO KHÁC HƯỚNG DI CHUYỂN (Course)
    # Vị trí lệch 50m (<100m), Bearing lệch 0.1 độ (<3°), Tốc độ giống hệt nhưng Hướng lệch 20 độ (>10°)
    # =========================================================================
    pos_t5 = distance(kilometers=20.0).destination(origin, 90.0)
    targets.append(
        {
            "true_id": 5,
            "lat": pos_t5.latitude,
            "lon": pos_t5.longitude,
            "speed_mps": 18.0 * 0.514444,
            "speed_knots": 18.0,
            "course": 135.0,
        }
    )

    pos_t6 = distance(kilometers=20.05).destination(origin, 90.1)
    targets.append(
        {
            "true_id": 6,
            "lat": pos_t6.latitude,
            "lon": pos_t6.longitude,
            "speed_mps": 18.0 * 0.514444,
            "speed_knots": 18.0,
            "course": 155.0,  # Khác hướng đi (lệch 20 độ)
        }
    )

    # =========================================================================
    # CÁC MỤC TIÊU CÒN LẠI: Sinh ngẫu nhiên hoàn toàn (nếu num_targets > 6)
    # =========================================================================
    for i in range(6, num_targets):
        r_radar = random.choice(radars)
        max_dist_km = r_radar["range_km"]

        dist_km = math.sqrt(random.uniform(0, 1)) * max_dist_km
        bearing = random.uniform(0, 360)
        target_pos = distance(kilometers=dist_km).destination(
            (r_radar["lat"], r_radar["lon"]), bearing
        )

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


def process_radar_detection(
    radar, target, detection, min_angle, max_angle, normalized_max, timestamp_str
):
    """Kiểm tra góc quét sector và tính toán dữ liệu nhiễu cho mục tiêu."""
    true_bearing = detection["bearing_deg"]

    # Kiểm tra mục tiêu có nằm trong sector quét hiện tại hay không
    if min_angle < max_angle and max_angle <= 360.0:
        is_in_scan_sector = min_angle <= true_bearing < max_angle
    else:
        is_in_scan_sector = true_bearing >= min_angle or true_bearing < normalized_max

    if not is_in_scan_sector:
        return None

    r_id = int(radar["id"])
    true_id = target["true_id"]
    local_id = true_id + r_id * 1000

    # Thêm nhiễu Gaussian (Quy tắc 3-Sigma)
    noise_dist_km = random.gauss(0, SIGMA_DIST_M) / 1000.0
    noisy_dist = max(0.0, detection["distance_km"] + noise_dist_km)

    noisy_bearing = (true_bearing + random.gauss(0, SIGMA_BEARING_DEG)) % 360.0
    noisy_speed = max(0.0, target["speed_knots"] + random.gauss(0, SIGMA_SPEED_KNOTS))
    noisy_course = (target["course"] + random.gauss(0, SIGMA_COURSE_DEG)) % 360.0

    radar_pos = (radar["lat"], radar["lon"])
    noisy_pos = distance(kilometers=noisy_dist).destination(radar_pos, noisy_bearing)

    return [
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


def filter_radar_resolution(detected_targets):
    """
    Lọc gộp các mục tiêu không thể phân biệt được dựa trên độ phân giải phần cứng
    và khả năng đo đạc sai số vận tốc/hướng đi của hệ thống Radar.
    """
    if not detected_targets:
        return []

    filtered_records = []

    for current in detected_targets:
        current_record, current_det, current_target_obj = current
        should_keep = True

        current_dist = current_det["distance_km"]
        current_bearing = current_det["bearing_deg"]
        current_speed = current_target_obj["speed_knots"]
        current_course = current_target_obj["course"]

        for approved in filtered_records:
            _, app_det, app_target_obj = approved
            app_dist = app_det["distance_km"]
            app_bearing = app_det["bearing_deg"]
            app_speed = app_target_obj["speed_knots"]
            app_course = app_target_obj["course"]

            # 1. Độ lệch cự ly (mét)
            delta_dist_m = abs(current_dist - app_dist) * 1000.0

            # 2. Độ lệch góc bearing từ radar (ngắn nhất)
            delta_bearing = abs(current_bearing - app_bearing)
            if delta_bearing > 180.0:
                delta_bearing = 360.0 - delta_bearing

            # 3. Độ lệch tốc độ của mục tiêu (knots)
            delta_speed = abs(current_speed - app_speed)

            # 4. Độ lệch hướng di chuyển của mục tiêu (ngắn nhất)
            delta_course = abs(current_course - app_course)
            if delta_course > 180.0:
                delta_course = 360.0 - delta_course

            # ĐIỀU KIỆN GỘP: Phải đồng thời vi phạm vị trí vật lý VÀ nằm trong ngưỡng nhiễu vận tốc của thiết bị
            if (
                delta_dist_m < MIN_RESOLUTION_DIST_M
                and delta_bearing < MIN_RESOLUTION_BEARING_DEG
                and delta_speed < MIN_RESOLUTION_SPEED_KNOTS
                and delta_course < MIN_RESOLUTION_COURSE_DEG
            ):
                should_keep = False
                break

        if should_keep:
            filtered_records.append(current)

    # Trả về mảng chứa chuỗi dữ liệu gốc để ghi vào file CSV
    return [item[0] for item in filtered_records]


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

    with open(output_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(csv_headers)

        for step in range(duration + 1):
            current_sim_time = start_time + timedelta(seconds=step)
            timestamp_str = current_sim_time.strftime("%Y-%m-%d %H:%M:%S")

            step_records = []

            for radar in radars:
                scan_period = radar["scan_period_s"]

                current_position_in_cycle = step % scan_period
                min_angle = (current_position_in_cycle / scan_period) * 360.0
                max_angle = ((current_position_in_cycle + 1) / scan_period) * 360.0
                normalized_max = max_angle % 360.0

                # Pool tạm lưu trữ các mục tiêu quét được của riêng trạm radar hiện tại ở giây này
                radar_detected_pool = []

                for target in targets:
                    detection = calculate_radar_detection(radar, target)
                    if detection:
                        record = process_radar_detection(
                            radar,
                            target,
                            detection,
                            min_angle,
                            max_angle,
                            normalized_max,
                            timestamp_str,
                        )
                        if record:
                            # Đẩy đầy đủ bộ 3 (record_csv, true_detection, target_object) vào bộ lọc phân giải
                            radar_detected_pool.append((record, detection, target))

                # Thực hiện lọc độ phân giải Radar (Resolution Filter) cho trạm hiện tại
                final_radar_records = filter_radar_resolution(radar_detected_pool)
                step_records.extend(final_radar_records)

            if step_records:
                writer.writerows(step_records)

            if step < duration:
                update_target_positions(targets, dt=1)

    print(f"Mô phỏng hoàn tất! Dữ liệu đã được lưu tại: {output_file}")


if __name__ == "__main__":
    main()
