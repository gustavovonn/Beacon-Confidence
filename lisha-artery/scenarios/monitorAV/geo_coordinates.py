from dataclasses import dataclass
from math import isclose
from typing import Optional

import pyproj
import utm


@dataclass
class UTMInfo:
    # coordinates
    easting: float
    northing: float

    # metadata
    zone_number: int
    zone_letter: str
    epsg_code: str

    def coordinates_are_close_to(self, other: "UTMInfo", abs_tol: float = 1.0) -> bool:
        # Absolute tolerance in meters
        easting_is_close: bool = isclose(self.easting, other.easting, abs_tol=abs_tol)
        northing_is_close: bool = isclose(self.northing, other.northing, abs_tol=abs_tol)
        return easting_is_close and northing_is_close

    def metadata_are_equal_to(self, other: "UTMInfo") -> bool:
        return (
            self.zone_number == other.zone_number
            and self.zone_letter == other.zone_letter
            and self.epsg_code == other.epsg_code
        )

    def __eq__(self, other):
        return self.coordinates_are_close_to(other, abs_tol=1.0) and self.metadata_are_equal_to(other)


def get_utm_epsg_code(utm_zone_number: int, utm_zone_letter: str) -> str:
    if utm_zone_number <= 0 or utm_zone_number >= 60:
        raise ValueError("Zone number must be between 1 and 60.")

    if utm_zone_letter not in "CDEFGHJKLMNPQRSTUVWX":
        raise ValueError("Zone letter must be between C and X.")

    northern: bool = utm_zone_letter >= "N"  # northern or southern hemisphere
    if northern:
        return f"EPSG:326{utm_zone_number:02d}"
    else:
        return f"EPSG:327{utm_zone_number:02d}"


def find_utm_info(latitude: float, longitude: float) -> UTMInfo:
    # expects lat/lon in WGS84
    utm_easting, utm_northing, zone_number, zone_letter = utm.from_latlon(latitude, longitude)
    epsg_code: str = get_utm_epsg_code(utm_zone_number=zone_number, utm_zone_letter=zone_letter)
    return UTMInfo(
        easting=utm_easting,
        northing=utm_northing,
        zone_number=zone_number,
        zone_letter=zone_letter,
        epsg_code=epsg_code,
    )


def get_transformer_wgs84_to_utm(origin_lat: float, origin_lon: float) -> pyproj.Transformer:
    wgs84_crs = pyproj.CRS("EPSG:4326")  # WGS84 coordinate reference system
    utm_info: UTMInfo = find_utm_info(latitude=origin_lat, longitude=origin_lon)
    utm_crs = pyproj.CRS(utm_info.epsg_code)
    return pyproj.Transformer.from_crs(wgs84_crs, utm_crs, always_xy=True)


class WGS84Transformer:
    def __init__(self, origin_lat: Optional[float], origin_lon: Optional[float]):
        # Origin in the atlantic ocean if not specified
        origin_lat = origin_lat or 0.0
        origin_lon = origin_lon or 0.0

        # Make sure all transformations use the same UTM zone.
        # This is relevant if the data is at the border of different UTM zones.
        self.transformer = get_transformer_wgs84_to_utm(origin_lat=origin_lat, origin_lon=origin_lon)

        # Remember UTM info of origin
        self.origin_utm_info: UTMInfo = find_utm_info(latitude=origin_lat, longitude=origin_lon)
        self.origin_easting: float = self.origin_utm_info.easting
        self.origin_northing: float = self.origin_utm_info.northing

    def transform_wgs84_to_utm(self, longitude: float, latitude: float):
        # returns x_utm, y_utm (corresponding to easting, northing)
        return self.transformer.transform(longitude, latitude)

    def transform_wgs84_to_local_metric(self, longitude: float, latitude: float):
        easting, northing = self.transform_wgs84_to_utm(longitude, latitude)
        return easting - self.origin_easting, northing - self.origin_northing
