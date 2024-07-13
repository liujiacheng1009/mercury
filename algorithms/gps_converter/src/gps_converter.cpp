#include "gps_converter.hpp"

void GpsConverter::ecef2ned(ECEF &ecef, NED &ned)
{
    Eigen::Vector3d vect, ret;
    vect(0) = ecef.x - ecef_ref_.x;
    vect(1) = ecef.y - ecef_ref_.y;
    vect(2) = ecef.z - ecef_ref_.z;
    ret = ecef2ned_mat_ * vect;
    ned.x = ret(0);
    ned.y = ret(1);
    ned.z = -ret(2);
}

void GpsConverter::ecef2enu(ECEF &ecef, ENU &enu)
{
    NED ned;
    ecef2ned(ecef, ned);
    ned2enu(ned, enu);
}

void GpsConverter::enu2ecef(ENU &enu, ECEF &ecef)
{
    NED ned;
    enu2ned(enu, ned);
    ned2ecef(ned, ecef);
}

void GpsConverter::ned2ecef(NED &ned, ECEF &ecef)
{
    Eigen::Vector3d ned_vec(ned.x, ned.y, -ned.z);
    Eigen::Vector3d ret = ned2ecef_mat_ * ned_vec;
    ecef.x = ret(0) + ecef_ref_.x;
    ecef.y = ret(1) + ecef_ref_.y;
    ecef.z = ret(2) + ecef_ref_.z;
}

void GpsConverter::enu2ned(ENU &enu, NED &ned)
{
    ned.x = enu.y;
    ned.y = enu.x;
    ned.z = -enu.z;
    return;
}

void GpsConverter::ned2enu(NED &ned, ENU &enu)
{
    enu.x = ned.y;
    enu.y = ned.x;
    enu.z = -ned.z;
    return;
}

void GpsConverter::setEcefRef(WGS84 &lla)
{
    lla2ecef(lla, ecef_ref_);
    double phi = std::atan2(ecef_ref_.z, std::sqrt(std::pow(ecef_ref_.x, 2) + pow(ecef_ref_.y, 2)));
    ecef2ned_mat_ = nRe(phi, deg2Rad(lla.lon));
    ned2ecef_mat_ = nRe(deg2Rad(lla.lat), deg2Rad(lla.lon)).transpose();
    have_ecef_ref_ = true;
}

void GpsConverter::lla2ecef(WGS84 &lla, ECEF &ecef)
{
    double lat_rad = deg2Rad(lla.lat);
    double lon_rad = deg2Rad(lla.lon);
    double xi = std::sqrt(1 - kFirstEccentricitySquared * std::sin(lat_rad) * std::sin(lat_rad));
    ecef.x = (kSemimajorAxis / xi + lla.alt) * cos(lat_rad) * cos(lon_rad);
    ecef.y = (kSemimajorAxis / xi + lla.alt) * cos(lat_rad) * sin(lon_rad);
    ecef.z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + lla.alt) * sin(lat_rad);
    return;
}

void GpsConverter::ecef2lla(ECEF &ecef, WGS84 &lla)
{
    double r = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
    constexpr double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
    double F = 54 * kSemiminorAxis * kSemiminorAxis * ecef.z * ecef.z;
    double G =
        r * r + (1 - kFirstEccentricitySquared) * ecef.z * ecef.z - kFirstEccentricitySquared * Esq;
    double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / std::pow(G, 3);
    double S = std::cbrt(1 + C + std::sqrt(C * C + 2 * C));
    double P = F / (3 * std::pow((S + 1 / S + 1), 2) * G * G);
    double Q = std::sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
    double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q) +
                 std::sqrt(0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q) -
                           P * (1 - kFirstEccentricitySquared) * ecef.z * ecef.z / (Q * (1 + Q)) -
                           0.5 * P * r * r);
    double U = std::sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + ecef.z * ecef.z);
    double V = std::sqrt(std::pow((r - kFirstEccentricitySquared * r_0), 2) +
                         (1 - kFirstEccentricitySquared) * ecef.z * ecef.z);
    double Z_0 = kSemiminorAxis * kSemiminorAxis * ecef.z / (kSemimajorAxis * V);
    lla.alt = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
    lla.lat = rad2Deg(std::atan((ecef.z + kSecondEccentricitySquared * Z_0) / r));
    lla.lon = rad2Deg(std::atan2(ecef.y, ecef.x));
    return;
}