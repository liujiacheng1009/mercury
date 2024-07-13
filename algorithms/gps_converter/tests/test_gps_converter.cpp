#include <gtest/gtest.h>
#include "gps_converter.hpp"

class MyTest : public ::testing::Test
{
protected:
    MyTest() {}
    ~MyTest() {}

    virtual void SetUp()
    {
        gps_conv_ptr = std::make_shared<GpsConverter>();
    }
    virtual void TearDown() {}

public:
    std::shared_ptr<GpsConverter> gps_conv_ptr;
};

TEST_F(MyTest, LLAAndECEF)
{
    WGS84 lla1(12.,13.,14.), lla2;
    ECEF ecef,ecef2;
    gps_conv_ptr->lla2ecef(lla1, ecef);
    gps_conv_ptr->ecef2lla(ecef, lla2);
    NED ned(1.,1.,1.), ned2;
    gps_conv_ptr->setEcefRef(lla1);
    gps_conv_ptr->ned2ecef(ned, ecef2);
    gps_conv_ptr->ecef2ned(ecef2, ned2);


    EXPECT_NEAR(lla1.lon, lla2.lon, 1e-6);
    EXPECT_NEAR(lla1.lat, lla2.lat, 1e-6);
    EXPECT_NEAR(lla1.alt, lla2.alt, 1e-3);
    EXPECT_NEAR(ned.x, ned2.x, 5e-3);
    EXPECT_NEAR(ned.y, ned2.y, 5e-3);
    EXPECT_NEAR(ned.z, ned2.z, 5e-3);
}