import csv
import math
import os
import random
from dataclasses import dataclass
from datetime import datetime, timedelta
from enum import Enum

from geographiclib.geodesic import Geodesic
from pydantic import BaseModel, Field, field_validator


KNOTS_TO_MPS = 0.514444
KM_TO_M = 1000.0
MAX_ANGLE_DEG = 360.0

# Radar parameters
MAX_RADARS = 10

ROUND_DIST_KM_DIGITS = 3
ROUND_BEARING_DEG_DIGITS = 2
ROUND_SPEED_KNOTS_DIGITS = 2
ROUND_COURSE_DEG_DIGITS = 2
ROUND_LAT_LON_DIGITS = 6

MAX_RADAR_DIST_M = 15.0
MAX_RADAR_BEARING_DEG = 0.7
MAX_RADAR_SPEED_KNOTS = 3.0
MAX_RADAR_COURSE_DEG = 10.0

SIGMA_RADAR_DIST_M = MAX_RADAR_DIST_M / 3.0
SIGMA_RADAR_DIST_KM = SIGMA_RADAR_DIST_M / KM_TO_M
SIGMA_RADAR_BEARING_DEG = MAX_RADAR_BEARING_DEG / 3.0
SIGMA_RADAR_SPEED_KNOTS = MAX_RADAR_SPEED_KNOTS / 3.0
SIGMA_RADAR_COURSE_DEG = MAX_RADAR_COURSE_DEG / 3.0

MIN_RADAR_RESOLUTION_DIST_M = 100.0
MIN_RADAR_RESOLUTION_DIST_KM = MIN_RADAR_RESOLUTION_DIST_M / 1000.0
MIN_RADAR_RESOLUTION_BEARING_DEG = 3.0
MIN_RADAR_RESOLUTION_SPEED_KNOTS = MAX_RADAR_SPEED_KNOTS
MIN_RADAR_RESOLUTION_COURSE_DEG = MAX_RADAR_COURSE_DEG

# Target parameters
MAX_TARGETS = 999
MAX_TARGET_SPEED_KNOTS = 30.0


class CloseTargetPairType(Enum):
    SAME_SPEED_SAME_COURSE = 1
    DIFF_SPEED_SAME_COURSE = 2
    SAME_SPEED_DIFF_COURSE = 3


# Close target pair config
MIN_CLOSE_TARGET_PAIRS_PER_TYPE = 1
MIN_CLOSE_TARGET_PAIRS = len(CloseTargetPairType) * MIN_CLOSE_TARGET_PAIRS_PER_TYPE
MIN_TARGETS = 2 * MIN_CLOSE_TARGET_PAIRS
MIN_CLOSE_PAIR_RATIO = 0.5

# Smooth random walk config
MAX_WALK_COURSE_DELTA_DEG = 2.0
MAX_WALK_SPEED_DELTA_KNOTS = 0.2
DEFAULT_MOVEMENT_DT_S = 1.0

# Generator config
MIN_GENERATED_TARGET_TO_RADAR_DISTANCE_KM = 1.0
MIN_GENERATED_TARGET_TO_TARGET_DISTANCE_M = 1.0
MIN_GENERATED_TARGET_SPEED_KNOTS = 5.0


# Input config
class RadarConfig(BaseModel):
    id: int = Field(gt=0)
    lat: float = Field(ge=-90, le=90)
    lon: float = Field(ge=-180, le=180)
    range_km: float = Field(gt=MIN_GENERATED_TARGET_TO_RADAR_DISTANCE_KM)
    scan_period_s: int = Field(gt=0)


class Config(BaseModel):
    num_targets: int = Field(ge=MIN_TARGETS, le=MAX_TARGETS)
    simulation_duration_s: int = Field(gt=0)
    radars: list[RadarConfig] = Field(min_length=1, max_length=MAX_RADARS)

    @field_validator("radars")
    @classmethod
    def validate_radars(cls, radars: list[RadarConfig]) -> list[RadarConfig]:
        radar_ids = set()

        for radar in radars:
            if radar.id in radar_ids:
                raise ValueError(f"[ERROR] Duplicate radar id: {radar.id}")

            radar_ids.add(radar.id)

        return radars


@dataclass
class Radar:
    id: int
    lat: float
    lon: float
    range_km: float
    scan_period_s: int


@dataclass
class Target:
    id: int
    lat: float
    lon: float
    speed_knots: float
    course_deg: float


@dataclass
class NumCloseTargetPairs:
    same_speed_same_course: int = MIN_CLOSE_TARGET_PAIRS_PER_TYPE
    diff_speed_same_course: int = MIN_CLOSE_TARGET_PAIRS_PER_TYPE
    same_speed_diff_course: int = MIN_CLOSE_TARGET_PAIRS_PER_TYPE


@dataclass
class RadarDetection:
    distance_km: float
    bearing_deg: float


@dataclass
class DetectedTarget:
    target: Target
    detection: RadarDetection


@dataclass
class RadarRecord:
    timestamp: str
    radar_id: int
    local_target_id: int
    true_target_id: int
    dist_km: float
    bearing_deg: float
    speed_knots: float
    course_deg: float
    lat: float
    lon: float


# Geodesic helpers
def move_position(
    lat: float,
    lon: float,
    bearing_deg: float,
    distance_m: float,
) -> tuple[float, float]:
    result = Geodesic.WGS84.Direct(lat, lon, bearing_deg, distance_m)
    return result["lat2"], result["lon2"]


def calculate_distance_and_bearing(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
) -> tuple[float, float]:
    result = Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2)

    distance_km = result["s12"] / KM_TO_M
    bearing_deg = result["azi1"] % MAX_ANGLE_DEG

    return distance_km, bearing_deg


def calculate_num_close_target_pairs(
    num_targets: int,
) -> NumCloseTargetPairs:
    max_close_target_pairs = num_targets // 2

    min_targets_in_close_pairs = math.ceil(num_targets * MIN_CLOSE_PAIR_RATIO)
    if min_targets_in_close_pairs % 2 != 0:
        min_targets_in_close_pairs += 1

    required_close_pair_count = min(
        max_close_target_pairs,
        max(
            MIN_CLOSE_TARGET_PAIRS,
            min_targets_in_close_pairs // 2,
        ),
    )

    num_close_target_pairs = NumCloseTargetPairs()
    close_pair_types = list(CloseTargetPairType)
    for _ in range(required_close_pair_count - MIN_CLOSE_TARGET_PAIRS):
        selected_pair_type = random.choice(close_pair_types)
        match selected_pair_type:
            case CloseTargetPairType.SAME_SPEED_SAME_COURSE:
                num_close_target_pairs.same_speed_same_course += 1
            case CloseTargetPairType.DIFF_SPEED_SAME_COURSE:
                num_close_target_pairs.diff_speed_same_course += 1
            case CloseTargetPairType.SAME_SPEED_DIFF_COURSE:
                num_close_target_pairs.same_speed_diff_course += 1

    return num_close_target_pairs


def get_random_position_in_radar_range(
    radars: list[Radar],
) -> tuple[float, float]:
    while True:
        selected_radar = random.choice(radars)

        rand_dist_km = random.uniform(
            MIN_GENERATED_TARGET_TO_RADAR_DISTANCE_KM, selected_radar.range_km
        )

        rand_bearing = random.random() * MAX_ANGLE_DEG

        lat, lon = move_position(
            selected_radar.lat,
            selected_radar.lon,
            rand_bearing,
            rand_dist_km * KM_TO_M,
        )

        is_valid = True

        for radar in radars:
            for radar in radars:
                distance_km, _ = calculate_distance_and_bearing(
                    lat,
                    lon,
                    radar.lat,
                    radar.lon,
                )

                if distance_km < MIN_GENERATED_TARGET_TO_RADAR_DISTANCE_KM:
                    is_valid = False
                    break

            if is_valid:
                return lat, lon


def append_close_target_pair(
    targets: list[Target],
    current_target_id: int,
    base_lat: float,
    base_lon: float,
    first_target_speed_knots: float,
    second_target_speed_knots: float,
    first_target_course_deg: float,
    second_target_course_deg: float,
):
    targets.append(
        Target(
            current_target_id,
            base_lat,
            base_lon,
            first_target_speed_knots,
            first_target_course_deg,
        )
    )

    current_target_id += 1
    second_target_lat, second_target_lon = move_position(
        base_lat,
        base_lon,
        random.random() * MAX_ANGLE_DEG,
        random.uniform(
            MIN_GENERATED_TARGET_TO_TARGET_DISTANCE_M, MIN_RADAR_RESOLUTION_DIST_M
        ),
    )
    targets.append(
        Target(
            current_target_id,
            second_target_lat,
            second_target_lon,
            second_target_speed_knots,
            second_target_course_deg,
        )
    )


def generate_targets(num_targets: int, radars: list[Radar]) -> list[Target]:
    num_close_target_pairs = calculate_num_close_target_pairs(num_targets)

    print(
        f"[INFO] Number of close same speed same course pairs: {num_close_target_pairs.same_speed_same_course}"
    )
    print(
        f"[INFO] Number of close diff speed same course pairs: {num_close_target_pairs.diff_speed_same_course}"
    )
    print(
        f"[INFO] Number of close same speed diff course pairs: {num_close_target_pairs.same_speed_diff_course}"
    )

    targets: list[Target] = []
    current_target_id = 1

    for _ in range(num_close_target_pairs.same_speed_same_course):
        base_lat, base_lon = get_random_position_in_radar_range(radars)
        base_speed_knots = random.uniform(
            MIN_GENERATED_TARGET_SPEED_KNOTS, MAX_TARGET_SPEED_KNOTS
        )
        base_course_deg = random.random() * MAX_ANGLE_DEG
        append_close_target_pair(
            targets,
            current_target_id,
            base_lat,
            base_lon,
            base_speed_knots,
            base_speed_knots,
            base_course_deg,
            base_course_deg,
        )
        current_target_id += 2

    for _ in range(num_close_target_pairs.diff_speed_same_course):
        base_lat, base_lon = get_random_position_in_radar_range(radars)
        base_course_deg = random.random() * MAX_ANGLE_DEG
        first_target_speed_knots = random.uniform(
            MIN_GENERATED_TARGET_SPEED_KNOTS,
            MAX_TARGET_SPEED_KNOTS - MIN_RADAR_RESOLUTION_SPEED_KNOTS,
        )
        second_target_speed_knots = random.uniform(
            first_target_speed_knots + MIN_RADAR_RESOLUTION_SPEED_KNOTS,
            MAX_TARGET_SPEED_KNOTS,
        )
        append_close_target_pair(
            targets,
            current_target_id,
            base_lat,
            base_lon,
            first_target_speed_knots,
            second_target_speed_knots,
            base_course_deg,
            base_course_deg,
        )
        current_target_id += 2

    for _ in range(num_close_target_pairs.same_speed_diff_course):
        base_lat, base_lon = get_random_position_in_radar_range(radars)
        base_speed_knots = random.uniform(
            MIN_GENERATED_TARGET_SPEED_KNOTS, MAX_TARGET_SPEED_KNOTS
        )
        first_target_course_deg = random.random() * MAX_ANGLE_DEG
        second_target_course_deg = (
            first_target_course_deg
            + random.uniform(MIN_RADAR_RESOLUTION_COURSE_DEG, MAX_ANGLE_DEG / 2.0)
        ) % MAX_ANGLE_DEG
        append_close_target_pair(
            targets,
            current_target_id,
            base_lat,
            base_lon,
            base_speed_knots,
            base_speed_knots,
            first_target_course_deg,
            second_target_course_deg,
        )
        current_target_id += 2

    for i in range(current_target_id, num_targets + 1):
        rand_lat, rand_lon = get_random_position_in_radar_range(radars)
        speed_knots = random.uniform(
            MIN_GENERATED_TARGET_SPEED_KNOTS, MAX_TARGET_SPEED_KNOTS
        )
        course_deg = random.random() * MAX_ANGLE_DEG
        targets.append(
            Target(
                i,
                rand_lat,
                rand_lon,
                speed_knots,
                course_deg,
            )
        )

    return targets


def update_target_positions(targets: list[Target], dt_s: float):
    for target in targets:
        target.course_deg = (
            target.course_deg
            + random.uniform(-MAX_WALK_COURSE_DELTA_DEG, MAX_WALK_COURSE_DELTA_DEG)
        ) % MAX_ANGLE_DEG

        target.speed_knots = max(
            MIN_GENERATED_TARGET_SPEED_KNOTS,
            min(
                MAX_TARGET_SPEED_KNOTS,
                target.speed_knots
                + random.uniform(
                    -MAX_WALK_SPEED_DELTA_KNOTS, MAX_WALK_SPEED_DELTA_KNOTS
                ),
            ),
        )

        dist_moved_m = target.speed_knots * KNOTS_TO_MPS * dt_s

        target.lat, target.lon = move_position(
            target.lat,
            target.lon,
            target.course_deg,
            dist_moved_m,
        )


def get_radar_detection(radar: Radar, target: Target) -> RadarDetection | None:
    dist_km, bearing_deg = calculate_distance_and_bearing(
        radar.lat, radar.lon, target.lat, target.lon
    )

    if dist_km > radar.range_km:
        return None

    return RadarDetection(distance_km=dist_km, bearing_deg=bearing_deg)


def apply_radar_resolution_filter(
    detected_targets: list[DetectedTarget],
) -> list[DetectedTarget]:
    if not detected_targets:
        return []

    sorted_detected_targets = sorted(
        detected_targets,
        key=lambda detected_target: detected_target.detection.distance_km,
    )
    filtered_detected_targets: list[DetectedTarget] = []

    for current_detected_target in sorted_detected_targets:
        current_detection = current_detected_target.detection
        current_target = current_detected_target.target

        should_keep = True

        for approved_detected_target in reversed(filtered_detected_targets):
            approved_detection = approved_detected_target.detection
            approved_target = approved_detected_target.target

            if (
                current_detection.distance_km - approved_detection.distance_km
            ) > MIN_RADAR_RESOLUTION_DIST_KM:
                break

            delta_bearing = abs(
                current_detection.bearing_deg - approved_detection.bearing_deg
            )

            if delta_bearing > MAX_ANGLE_DEG / 2.0:
                delta_bearing = MAX_ANGLE_DEG - delta_bearing

            if delta_bearing >= MIN_RADAR_RESOLUTION_BEARING_DEG:
                continue

            if (
                abs(current_target.speed_knots - approved_target.speed_knots)
                >= MIN_RADAR_RESOLUTION_SPEED_KNOTS
            ):
                continue

            delta_course = abs(current_target.course_deg - approved_target.course_deg)

            if delta_course > MAX_ANGLE_DEG / 2.0:
                delta_course = MAX_ANGLE_DEG - delta_course

            if delta_course >= MIN_RADAR_RESOLUTION_COURSE_DEG:
                continue

            should_keep = False
            break

        if should_keep:
            filtered_detected_targets.append(current_detected_target)

    return filtered_detected_targets


def calculate_radar_record(
    radar: Radar,
    target: Target,
    detection: RadarDetection,
    min_angle: float,
    max_angle: float,
    timestamp: str,
) -> RadarRecord | None:
    true_bearing_deg = detection.bearing_deg

    if min_angle < max_angle:
        is_in_scan_sector = min_angle <= true_bearing_deg < max_angle
    else:
        is_in_scan_sector = (
            true_bearing_deg >= min_angle or true_bearing_deg < max_angle
        )

    if not is_in_scan_sector:
        return None

    radar_id = radar.id
    true_target_id = target.id
    local_target_id = true_target_id + radar_id * (MAX_TARGETS + 1)

    noisy_dist_km = max(
        0.0, detection.distance_km + random.gauss(0, SIGMA_RADAR_DIST_KM)
    )
    noisy_bearing_deg = (
        true_bearing_deg + random.gauss(0, SIGMA_RADAR_BEARING_DEG)
    ) % MAX_ANGLE_DEG
    noisy_speed_knots = max(
        0.0, target.speed_knots + random.gauss(0, SIGMA_RADAR_SPEED_KNOTS)
    )
    noisy_course_deg = (
        target.course_deg + random.gauss(0, SIGMA_RADAR_COURSE_DEG)
    ) % MAX_ANGLE_DEG

    noisy_lat, noisy_lon = move_position(
        radar.lat,
        radar.lon,
        noisy_bearing_deg,
        noisy_dist_km * KM_TO_M,
    )

    return RadarRecord(
        timestamp=timestamp,
        radar_id=radar_id,
        local_target_id=local_target_id,
        true_target_id=true_target_id,
        dist_km=round(noisy_dist_km, ROUND_DIST_KM_DIGITS),
        bearing_deg=round(noisy_bearing_deg, ROUND_BEARING_DEG_DIGITS),
        speed_knots=round(noisy_speed_knots, ROUND_SPEED_KNOTS_DIGITS),
        course_deg=round(noisy_course_deg, ROUND_COURSE_DEG_DIGITS),
        lat=round(noisy_lat, ROUND_LAT_LON_DIGITS),
        lon=round(noisy_lon, ROUND_LAT_LON_DIGITS),
    )


def main():
    with open("input.json", encoding="utf-8") as f:
        config = Config.model_validate_json(f.read())

    radars = [
        Radar(
            id=radar.id,
            lat=radar.lat,
            lon=radar.lon,
            range_km=radar.range_km,
            scan_period_s=radar.scan_period_s,
        )
        for radar in config.radars
    ]
    print(f"[INFO] Number of radars: {len(radars)}")

    num_targets = config.num_targets
    print(f"[INFO] Number of targets: {num_targets}")

    duration_s = config.simulation_duration_s
    print(f"[INFO] Simulation duration : {duration_s}s")

    start_time = datetime.now()
    start_time_s = start_time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[INFO] Start simulation time: {start_time_s}")

    output_dir = os.path.join("outputs", start_time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "radar_target_data.csv")

    targets = generate_targets(num_targets, radars)
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

        for step in range(1, duration_s + 1):
            current_time = start_time + timedelta(seconds=step)
            timestamp = current_time.strftime("%Y-%m-%d %H:%M:%S")
            step_records = []

            for radar in radars:
                scan_period_s = radar.scan_period_s
                previous_position_in_cycle = (step - 1) % scan_period_s
                current_position_in_cycle = step % scan_period_s

                min_angle = (previous_position_in_cycle / scan_period_s) * MAX_ANGLE_DEG
                max_angle = (current_position_in_cycle / scan_period_s) * MAX_ANGLE_DEG

                detected_targets: list[DetectedTarget] = []

                for target in targets:
                    radar_detection = get_radar_detection(radar, target)
                    if radar_detection:
                        detected_targets.append(
                            DetectedTarget(
                                detection=radar_detection,
                                target=target,
                            )
                        )

                filtered_detected_targets = apply_radar_resolution_filter(
                    detected_targets
                )
                for detected_target in filtered_detected_targets:
                    radar_record = calculate_radar_record(
                        radar,
                        detected_target.target,
                        detected_target.detection,
                        min_angle,
                        max_angle,
                        timestamp,
                    )

                    if radar_record:
                        step_records.append(
                            [
                                radar_record.timestamp,
                                radar_record.radar_id,
                                radar_record.local_target_id,
                                radar_record.true_target_id,
                                radar_record.dist_km,
                                radar_record.bearing_deg,
                                radar_record.speed_knots,
                                radar_record.course_deg,
                                radar_record.lat,
                                radar_record.lon,
                            ]
                        )

            if step_records:
                writer.writerows(step_records)

            if step < duration_s:
                update_target_positions(targets, DEFAULT_MOVEMENT_DT_S)

    print(f"[INFO] Simulation output saved to {output_file}")


if __name__ == "__main__":
    main()
