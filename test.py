from geographiclib.geodesic import Geodesic

KM_TO_M = 1000.0
MAX_ANGLE_DEG = 360.0


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


dist_km, bearing_deg = calculate_distance_and_bearing(
    20.705, 106.785, 22.836354, 104.966355
)
print(dist_km)
print(bearing_deg)
