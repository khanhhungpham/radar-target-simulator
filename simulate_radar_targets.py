import csv
import json
import math
import os
import random
import sys
from datetime import datetime, timedelta
from typing import Dict, Any, List, Tuple

from geopy.distance import distance

MIN_TARGETS = 6
MAX_TARGETS = 999

MAX_ERROR_DIST_M = 15.0
MAX_ERROR_BEARING_DEG = 0.7
MAX_ERROR_SPEED_KNOTS = 3.0
MAX_ERROR_COURSE_DEG = 10.0

SIGMA_DIST_M = MAX_ERROR_DIST_M / 3.0
SIGMA_BEARING_DEG = MAX_ERROR_BEARING_DEG / 3.0
SIGMA_SPEED_KNOTS = MAX_ERROR_SPEED_KNOTS / 3.0
SIGMA_COURSE_DEG = MAX_ERROR_COURSE_DEG / 3.0

MIN_RESOLUTION_DIST_M = 100.0
MIN_RESOLUTION_DIST_KM = MIN_RESOLUTION_DIST_M / 1000.0
MIN_RESOLUTION_BEARING_DEG = 3.0
MIN_RESOLUTION_SPEED_KNOTS = MAX_ERROR_SPEED_KNOTS
MIN_RESOLUTION_COURSE_DEG = MAX_ERROR_COURSE_DEG

KNOTS_TO_MPS = 0.514444


def load_input_config(file_path: str = "input.json") -> Dict[str, Any]:
    if not os.path.exists(file_path):
        print(f"ERROR: Configuration file '{file_path}' not found.")
        sys.exit(1)

    with open(file_path, "r", encoding="utf-8") as f:
        return json.load(f)


def generate_initial_targets(num_targets: int, radars: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    targets = []
    current_id = 1
    MIN_PAIR_RATIO = 0.5

    if not (MIN_TARGETS <= num_targets <= MAX_TARGETS):
        raise ValueError(
            f"[VALIDATION ERROR] num_targets is {num_targets}. "
            f"Must be within range [{MIN_TARGETS} -> {MAX_TARGETS}]."
        )

    # Ensure special targets leave room for at least some standard targets if possible
    max_allowed_pairs = (num_targets - 1) // 2 if num_targets % 2 != 0 else (num_targets // 2) - 1
    max_allowed_pairs = max(3, max_allowed_pairs)

    min_special_targets = math.ceil(num_targets * MIN_PAIR_RATIO)
    if min_special_targets % 2 != 0:
        min_special_targets += 1

    total_needed_pairs = min(max_allowed_pairs, max(3, min_special_targets // 2))

    # Base allocation (guarantee at least 1 pair per edge-case type)
    num_pairs_type1 = 1
    num_pairs_type2 = 1
    num_pairs_type3 = 1

    # Dynamically distribute the remaining special pairs
    for _ in range(total_needed_pairs - 3):
        chosen_type = random.choice([1, 2, 3])
        if chosen_type == 1:
            num_pairs_type1 += 1
        elif chosen_type == 2:
            num_pairs_type2 += 1
        else:
            num_pairs_type3 += 1

    # Print initialization summary
    print("--- Target Generation Summary ---")
    print(f"Total Targets Requested: {num_targets}")
    print(f"Type 1 Pairs (Overlapping): {num_pairs_type1}")
    print(f"Type 2 Pairs (Diff Speed) : {num_pairs_type2}")
    print(f"Type 3 Pairs (Diff Course): {num_pairs_type3}")
    print(
        f"Total Edge-Case Targets  : {(num_pairs_type1 + num_pairs_type2 + num_pairs_type3) * 2}"
    )
    print("---------------------------------")

    def get_random_position_in_radar_range() -> Tuple[float, float]:
        selected_radar = random.choice(radars)
        radar_origin = (selected_radar["lat"], selected_radar["lon"])
        rand_dist_km = math.sqrt(random.uniform(0.05, 0.8)) * selected_radar["range_km"]
        rand_bearing = random.uniform(0.0, 360.0)
        pos = distance(kilometers=rand_dist_km).destination(radar_origin, rand_bearing)
        return pos.latitude, pos.longitude

    # Type 1: Overlapping pairs (merged by radar resolution)
    for _ in range(num_pairs_type1):
        base_lat, base_lon = get_random_position_in_radar_range()
        base_speed = random.uniform(10.0, 25.0)
        base_course = random.uniform(0.0, 360.0)

        targets.append(
            {
                "true_id": current_id,
                "lat": base_lat,
                "lon": base_lon,
                "speed_knots": base_speed,
                "speed_mps": base_speed * KNOTS_TO_MPS,
                "course": base_course,
            }
        )
        current_id += 1

        pos_b = distance(meters=random.uniform(10, 50)).destination(
            (base_lat, base_lon), random.uniform(0, 360)
        )
        targets.append(
            {
                "true_id": current_id,
                "lat": pos_b.latitude,
                "lon": pos_b.longitude,
                "speed_knots": base_speed,
                "speed_mps": base_speed * KNOTS_TO_MPS,
                "course": base_course,
            }
        )
        current_id += 1

    # Type 2: Adjacent pairs distinguishable by speed
    for _ in range(num_pairs_type2):
        base_lat, base_lon = get_random_position_in_radar_range()
        base_course = random.uniform(0.0, 360.0)
        speed_a = random.uniform(5.0, 15.0)
        speed_b = speed_a + random.uniform(4.0, 8.0)

        targets.append(
            {
                "true_id": current_id,
                "lat": base_lat,
                "lon": base_lon,
                "speed_knots": speed_a,
                "speed_mps": speed_a * KNOTS_TO_MPS,
                "course": base_course,
            }
        )
        current_id += 1

        pos_b = distance(meters=random.uniform(10, 50)).destination(
            (base_lat, base_lon), random.uniform(0, 360)
        )
        targets.append(
            {
                "true_id": current_id,
                "lat": pos_b.latitude,
                "lon": pos_b.longitude,
                "speed_knots": speed_b,
                "speed_mps": speed_b * KNOTS_TO_MPS,
                "course": base_course,
            }
        )
        current_id += 1

    # Type 3: Adjacent pairs distinguishable by course
    for _ in range(num_pairs_type3):
        base_lat, base_lon = get_random_position_in_radar_range()
        base_speed = random.uniform(10.0, 25.0)
        course_a = random.uniform(0.0, 360.0)
        course_b = (course_a + random.uniform(15.0, 30.0)) % 360.0

        targets.append(
            {
                "true_id": current_id,
                "lat": base_lat,
                "lon": base_lon,
                "speed_knots": base_speed,
                "speed_mps": base_speed * KNOTS_TO_MPS,
                "course": course_a,
            }
        )
        current_id += 1

        pos_b = distance(meters=random.uniform(10, 50)).destination(
            (base_lat, base_lon), random.uniform(0, 360)
        )
        targets.append(
            {
                "true_id": current_id,
                "lat": pos_b.latitude,
                "lon": pos_b.longitude,
                "speed_knots": base_speed,
                "speed_mps": base_speed * KNOTS_TO_MPS,
                "course": course_b,
            }
        )
        current_id += 1

    # Standard targets generation
    for i in range(current_id, num_targets + 1):
        rand_lat, rand_lon = get_random_position_in_radar_range()
        speed_knots = random.uniform(5.0, 28.0)

        targets.append(
            {
                "true_id": i,
                "lat": rand_lat,
                "lon": rand_lon,
                "speed_knots": speed_knots,
                "speed_mps": speed_knots * KNOTS_TO_MPS,
                "course": random.uniform(0.0, 360.0),
            }
        )

    return targets


def update_target_positions(targets: List[Dict[str, Any]], dt: float):
    """Updates target positions using smooth random walk."""
    for t in targets:
        t["course"] = (t["course"] + random.uniform(-2.0, 2.0)) % 360.0
        new_speed_knots = max(
            5.0, min(30.0, t["speed_knots"] + random.uniform(-0.2, 0.2))
        )
        t["speed_knots"] = new_speed_knots
        t["speed_mps"] = new_speed_knots * KNOTS_TO_MPS

        dist_moved_km = (t["speed_mps"] * dt) / 1000.0
        next_pos = distance(kilometers=dist_moved_km).destination(
            (t["lat"], t["lon"]), t["course"]
        )
        t["lat"], t["lon"] = next_pos.latitude, next_pos.longitude


def get_radar_detection(
    radar: Dict[str, Any],
    target: Dict[str, Any],
    radar_lat_rad: float,
    radar_lon_rad: float,
) -> Any:
    radar_pos = (radar["lat"], radar["lon"])
    target_pos = (target["lat"], target["lon"])
    dist_km = distance(radar_pos, target_pos).kilometers

    if dist_km <= radar["range_km"]:
        lat2, lon2 = math.radians(target["lat"]), math.radians(target["lon"])

        y = math.sin(lon2 - radar_lon_rad) * math.cos(lat2)
        x = math.cos(radar_lat_rad) * math.sin(lat2) - math.sin(
            radar_lat_rad
        ) * math.cos(lat2) * math.cos(lon2 - radar_lon_rad)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

        return {"distance_km": dist_km, "bearing_deg": bearing}
    return None


def calculate_radar_record(
    radar: Dict[str, Any],
    target: Dict[str, Any],
    detection: Dict[str, Any],
    min_angle: float,
    max_angle: float,
    normalized_max: float,
    timestamp_str: str,
) -> Any:
    true_bearing = detection["bearing_deg"]

    if min_angle < max_angle and max_angle <= 360.0:
        is_in_scan_sector = min_angle <= true_bearing < max_angle
    else:
        is_in_scan_sector = true_bearing >= min_angle or true_bearing < normalized_max

    if not is_in_scan_sector:
        return None

    r_id = int(radar["id"])
    true_id = target["true_id"]
    local_id = true_id + r_id * (MAX_TARGETS + 1)

    # Inject Gaussian noise
    noisy_dist = max(
        0.0, detection["distance_km"] + (random.gauss(0, SIGMA_DIST_M) / 1000.0)
    )
    noisy_bearing = (true_bearing + random.gauss(0, SIGMA_BEARING_DEG)) % 360.0
    noisy_speed = max(0.0, target["speed_knots"] + random.gauss(0, SIGMA_SPEED_KNOTS))
    noisy_course = (target["course"] + random.gauss(0, SIGMA_COURSE_DEG)) % 360.0

    noisy_pos = distance(kilometers=noisy_dist).destination(
        (radar["lat"], radar["lon"]), noisy_bearing
    )

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


def filter_radar_resolution(
    detected_targets: List[Tuple[List, Dict, Dict]],
) -> List[List]:
    """Filters and merges detections based on radar resolution limits (Early Exit optimization)."""
    if not detected_targets:
        return []

    detected_targets.sort(key=lambda x: x[1]["distance_km"])
    filtered_records: List[Tuple[List, Dict, Dict]] = []

    for current in detected_targets:
        # Replaced unused 'current_record' with '_' to silence Pylance warnings
        _, current_det, current_target_obj = current
        should_keep = True

        current_dist = current_det["distance_km"]
        current_bearing = current_det["bearing_deg"]
        current_speed = current_target_obj["speed_knots"]
        current_course = current_target_obj["course"]

        for approved in reversed(filtered_records):
            _, app_det, app_target_obj = approved
            app_dist = app_det["distance_km"]

            if (current_dist - app_dist) > MIN_RESOLUTION_DIST_KM:
                break

            delta_bearing = abs(current_bearing - app_det["bearing_deg"])
            if delta_bearing > 180.0:
                delta_bearing = 360.0 - delta_bearing
            if delta_bearing >= MIN_RESOLUTION_BEARING_DEG:
                continue

            if (
                abs(current_speed - app_target_obj["speed_knots"])
                >= MIN_RESOLUTION_SPEED_KNOTS
            ):
                continue

            delta_course = abs(current_course - app_target_obj["course"])
            if delta_course > 180.0:
                delta_course = 360.0 - delta_course
            if delta_course >= MIN_RESOLUTION_COURSE_DEG:
                continue

            # Target overlaps completely; merge required
            should_keep = False
            break

        if should_keep:
            filtered_records.append(current)

    return [item[0] for item in filtered_records]


def main():
    config = load_input_config()
    radars = config["radars"]
    num_targets = config["num_targets"]
    duration = config.get("simulation_duration_s", 60)

    start_time = datetime.now()
    output_dir = os.path.join("outputs", start_time.strftime("%Y%m%d_%H%M%S"))
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

                radar_lat_rad = math.radians(radar["lat"])
                radar_lon_rad = math.radians(radar["lon"])

                radar_detected_pool = []

                for target in targets:
                    detection = get_radar_detection(
                        radar, target, radar_lat_rad, radar_lon_rad
                    )
                    if detection:
                        record = calculate_radar_record(
                            radar,
                            target,
                            detection,
                            min_angle,
                            max_angle,
                            normalized_max,
                            timestamp_str,
                        )
                        if record:
                            radar_detected_pool.append((record, detection, target))

                final_radar_records = filter_radar_resolution(radar_detected_pool)
                step_records.extend(final_radar_records)

            if step_records:
                writer.writerows(step_records)

            if step < duration:
                update_target_positions(targets, dt=1.0)

    print(f"Simulation completed successfully! Data saved to: {output_file}")


if __name__ == "__main__":
    main()
