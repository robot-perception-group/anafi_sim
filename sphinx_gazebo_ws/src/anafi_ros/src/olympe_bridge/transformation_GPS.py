# -*- coding: UTF-8 -*-

"""

Author: ZEHUI LU
Reference: https://onlinelibrary.wiley.com/doi/pdf/10.1002/9780470099728.app3

"""

from math import sin, cos, radians, sqrt
import numpy as np


#def GPS2ECEF(latitude, longitude, altitude, flag_R_ECEF2NED, ellipsoid):
def GPS2ECEF(*argv):
    # latitude in decimal degree, [-90, +90]
    # longitude in decimal degree, [-180, +180]
    # altitude in meter, obtained by GPS

    # flag_R_ECEF2NED = 0 or 1, default is 0
    # if flag = 1, this point is the origin of NED, calculate the rotation matrix from the ECEF frame to the local NED frame
    # if flag = 0, output a 3 by 3 Identity matrix, won't use it

    # ellipsoid = (a, rf), a is semi-major axis [meter], rf is reciprocal flattening (1/f)
    # default: WGS84 = 6378137, 298.257223563
    
    latitude_radian = radians(argv[0])   # degree to radian
    longitude_radian = radians(argv[1])  # degree to radian
    altitude = argv[2]                   # meter

    # calculate some values in advanced
    sin_lat = sin(latitude_radian)
    cos_lat = cos(latitude_radian)
    sin_lon = sin(longitude_radian)
    cos_lon = cos(longitude_radian)

    if len(argv) > 3:
        flag_R_ECEF2NED = argv[3]
    else:
        flag_R_ECEF2NED = 0

    if flag_R_ECEF2NED == 1:

        R_ECEF2NED = np.array([\
            [-sin_lat*cos_lon, -sin_lat*sin_lon, cos_lat], \
            [-sin_lon, cos_lon, 0.0], \
            [-cos_lat*cos_lon, -cos_lat*sin_lon, -sin_lat]])

    else:
        R_ECEF2NED = np.eye(3)

    if len(argv) > 4:
        a, rf = argv[4]
    else:
        a, rf = 6378137, 298.257223563
    
    e2 = 1 - (1 - 1 / rf) ** 2           # squared eccentricity
    n = a / sqrt(1 - e2 * sin_lat ** 2)  # prime vertical radius
    r = (n + altitude) * cos_lat         # perpendicular distance in z axis
    x = r * cos_lon
    y = r * sin_lon
    z = (n * (1 - e2) + altitude) * sin_lat
    # ECEF coordinates for GPS
    # 3 by 1
    point_ECEF = np.array([[x], [y], [z]])
    return point_ECEF, R_ECEF2NED


def ECEF2NED(origin_ECEF, point_ECEF, R_ECEF2NED):
    # NED coordinates, the origin is defined by yourself
    # 3 by 1, North, East, Down
    point_NED = np.dot(R_ECEF2NED, (point_ECEF - origin_ECEF))
    return point_NED


def GPS2NED(latitude, longitude, altitude, origin_ECEF, R_ECEF2NED):
    point_ECEF, R_eye3 = GPS2ECEF(latitude, longitude, altitude)
    point_NED = ECEF2NED(origin_ECEF, point_ECEF, R_ECEF2NED)
    # 3 by 1, North, East, Down
    return point_NED

