#pragma once
#include <Eigen/Eigen>
#include <bits/stdc++.h>

constexpr double kFirstEccentricitySquared = 6.69437999014 * 0.001;
constexpr double kSemimajorAxis = 6378137;
constexpr double kSemiminorAxis = 6356752.3142;
constexpr double kSecondEccentricitySquared = 6.73949674228 * 0.001;

struct ECEF
{
    ECEF(){}
    ECEF(double x, double y, double z):x(x),y(y),z(z){}
    double x;
    double y;
    double z;
};

struct WGS84
{
    WGS84(){}
    WGS84(double lat, double lon, double alt):lat(lat),lon(lon),alt(alt){}
    double lat;
    double lon;
    double alt;
};

struct ENU
{
    ENU(){}
    ENU(double east, double north, double up):x(east),y(north),z(up){}
    double x;
    double y;
    double z;
};

struct NED
{
    NED(){}
    NED(double north, double east, double down):x(north),y(east),z(down){}
    double x;
    double y;
    double z;
};

template<typename T>
inline double deg2Rad(T&& deg)
{
    return deg / 180 * M_PI;
}
template<typename T>
inline double rad2Deg(T&& rad)
{
    return rad / M_PI * 180.;
}

class GpsConverter{

public:

    void ecef2ned(ECEF& ecef_xyz, NED& ned_xyz);

    void ecef2enu(ECEF& ecef_xyz, ENU& enu);

    void ned2ecef(NED& ned, ECEF& ecef_xyz);

    void enu2ecef(ENU& enu, ECEF& ecef_xyz);

    void enu2ned(ENU& enu_xyz, NED& ned_xyz);

    void ned2enu(NED& ned_xyz,ENU& enu_xyz);

    void ecef2lla(ECEF& ecef, WGS84& lla);

    void lla2ecef(WGS84& lla, ECEF& ecef);

    void setEcefRef(WGS84& lla);

    ECEF& get_ecef_ref()
    {
        return ecef_ref_;
    }

private:
    inline Eigen::Matrix3d nRe(double lat_radians, double lon_radians)
    {
        const double sLat = std::sin(lat_radians);
        const double sLon = std::sin(lon_radians);
        const double cLat = std::cos(lat_radians);
        const double cLon = std::cos(lon_radians);

        Eigen::Matrix3d ret;
        ret(0, 0) = -sLat * cLon;
        ret(0, 1) = -sLat * sLon;
        ret(0, 2) = cLat;
        ret(1, 0) = -sLon;
        ret(1, 1) = cLon;
        ret(1, 2) = 0.0;
        ret(2, 0) = cLat * cLon;
        ret(2, 1) = cLat * sLon;
        ret(2, 2) = sLat;
        return ret;
    }
    ECEF ecef_ref_;
    Eigen::Matrix3d ecef2ned_mat_;
    Eigen::Matrix3d ned2ecef_mat_;
    bool have_ecef_ref_{false};
};