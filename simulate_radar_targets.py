import os
import json
import math
import random
import csv
from datetime import datetime, timedelta
from geopy.distance import distance

def load_input_config(file_path="input.json"):
    """Đọc cấu hình đầu vào từ file JSON."""
    if not os.path.exists(file_path):
        sample_config = {
            "radars": [
                {"id": 1, "lat": 20.849, "lon": 106.711, "range_km": 50.0, "scan_period_s": 3.0},
                {"id": 2, "lat": 20.705, "lon": 106.785, "range_km": 60.0, "scan_period_s": 4.0}
            ],
            "num_targets": 5,
            "simulation_duration_s": 60
        }
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(sample_config, f, indent=4)
        print(f"Đã tạo file cấu hình mẫu tại {file_path}. Hãy chỉnh sửa và chạy lại.")
        return sample_config

    with open(file_path, "r", encoding="utf-8") as f:
        return json.load(f)

def generate_initial_targets(num_targets, radars):
    """Khởi tạo mục tiêu nằm trong vùng quét của ít nhất một radar."""
    targets = []
    for i in range(num_targets):
        ref_radar = random.choice(radars)
        ref_lat = ref_radar["lat"]
        ref_lon = ref_radar["lon"]
        max_dist = ref_radar["range_km"]

        dist_km = random.uniform(0, max_dist)
        bearing = random.uniform(0, 360)
        origin = (ref_lat, ref_lon)
        target_pos = distance(kilometers=dist_km).destination(origin, bearing)

        # Khởi tạo vận tốc ban đầu từ 5 đến 28 hải lý/giờ (chừa biên để tăng tốc)
        speed_knots = random.uniform(5.0, 28.0)
        speed_mps = speed_knots * 0.514444
        course = random.uniform(0.0, 360.0)

        targets.append({
            "true_id": i + 1,
            "lat": target_pos.latitude,
            "lon": target_pos.longitude,
            "speed_mps": speed_mps,
            "speed_knots": speed_knots,
            "course": course
        })
    return targets

def update_target_positions(targets, dt):
    """Cập nhật vị trí, vận tốc và hướng mục tiêu di chuyển thực tế (mô hình Smooth Random Walk)."""
    for t in list(targets):
        # 1. Thay đổi hướng đi mượt mà (Quán tính lớn, chỉ lệch tối đa 2 độ mỗi giây)
        course_change = random.uniform(-2.0, 2.0)
        t["course"] = (t["course"] + course_change) % 360.0
        
        # 2. Thay đổi vận tốc mượt mà (Gia tốc nhỏ, tối đa 0.2 hải lý/giờ mỗi giây)
        speed_change = random.uniform(-0.2, 0.2)
        new_speed_knots = t["speed_knots"] + speed_change
        
        # Giới hạn vận tốc nghiêm ngặt trong dải [5.0, 30.0] hải lý/giờ theo yêu cầu đề bài
        t["speed_knots"] = max(5.0, min(30.0, new_speed_knots))
        t["speed_mps"] = t["speed_knots"] * 0.514444
        
        # 3. Tính quãng đường và cập nhật vị trí kinh vĩ độ mới
        dist_moved_km = (t["speed_mps"] * dt) / 1000.0
        current_pos = (t["lat"], t["lon"])
        next_pos = distance(kilometers=dist_moved_km).destination(current_pos, t["course"])
        
        t["lat"] = next_pos.latitude
        t["lon"] = next_pos.longitude

def calculate_radar_detection(radar, target):
    """Tính toán thông số nếu mục tiêu nằm trong tầm quét."""
    radar_pos = (radar["lat"], radar["lon"])
    target_pos = (target["lat"], target["lon"])
    dist_km = distance(radar_pos, target_pos).kilometers
    
    if dist_km <= radar["range_km"]:
        lat1, lon1 = math.radians(radar["lat"]), math.radians(radar["lon"])
        lat2, lon2 = math.radians(target["lat"]), math.radians(target["lon"])
        
        d_lon = lon2 - lon1
        y = math.sin(d_lon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.sin(d_lon)
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
        "timestamp", "radar_id", "local_target_id", "true_target_id", 
        "distance_km", "bearing_deg", "speed_knots", "course_deg", "lat", "lon"
    ]
    
    detected_records = []
    
    for step in range(duration + 1):
        current_sim_time = start_time + timedelta(seconds=step)
        timestamp_str = current_sim_time.strftime("%Y-%m-%d %H:%M:%S")
        
        for radar in radars:
            rem = step % radar["scan_period_s"]
            if math.isclose(rem, 0, abs_tol=1e-5) or math.isclose(rem, radar["scan_period_s"], abs_tol=1e-5):
                r_id = int(radar["id"])
                
                for t in targets:
                    detection = calculate_radar_detection(radar, t)
                    if detection:
                        true_id = t["true_id"]
                        
                        # ĐÃ CẬP NHẬT: Tính local_id theo công thức mới của bạn
                        local_id = true_id + r_id * 1000
                        
                        detected_records.append([
                            timestamp_str,
                            r_id,
                            local_id,
                            true_id,
                            round(detection["distance_km"], 3),
                            round(detection["bearing_deg"], 2),
                            round(t["speed_knots"], 2),
                            round(t["course"], 2),
                            round(t["lat"], 6),
                            round(t["lon"], 6)
                        ])
                        
        if step < duration:
            update_target_positions(targets, dt=1)

    with open(output_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(csv_headers)
        writer.writerows(detected_records)
        
    print(f"Mô phỏng hoàn tất! Dữ liệu di chuyển thực tế đã được lưu tại: {output_file}")

if __name__ == "__main__":
    main()
