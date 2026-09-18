"""GPS interpretation for the explicitly defined synthetic evaluation charts."""

import math


def gps_route_to_map(raw, context):
    origin = context.get("map_origin_wgs84", {})
    if origin.get("projection") != "local_equirectangular_east_north":
        raise ValueError("GPS review requires an explicit map projection")
    latitude, longitude, map_x, map_y = (
        float(origin[key]) for key in ("latitude", "longitude", "map_x", "map_y")
    )
    if not all(map(math.isfinite, (latitude, longitude, map_x, map_y))):
        raise ValueError("Map origin must be finite")
    if not -89 < latitude < 89 or not -180 <= longitude <= 180:
        raise ValueError("Map origin is outside the supported geographic range")
    if not isinstance(raw, str) or not raw.strip():
        raise ValueError("gps_waypoints must contain a geographic route")
    # ponytail: local chart approximation; use a surveyed projection for physical trials.
    metres_per_degree = math.pi * 6378137.0 / 180.0
    points = []
    for token in raw.split(";"):
        fields = [float(value) for value in token.split(",")]
        if len(fields) not in (2, 3, 4) or not all(map(math.isfinite, fields)):
            raise ValueError("GPS waypoint must contain finite lat,lon[,yaw] or lat,lon,alt,yaw")
        lat, lon = fields[:2]
        if not -90 <= lat <= 90 or not -180 <= lon <= 180:
            raise ValueError("GPS waypoint latitude or longitude is out of range")
        x = map_x + (lon - longitude) * metres_per_degree * math.cos(math.radians(latitude))
        y = map_y + (lat - latitude) * metres_per_degree
        points.append((x, y, fields[-1] if len(fields) > 2 else 0.0))
    return points
