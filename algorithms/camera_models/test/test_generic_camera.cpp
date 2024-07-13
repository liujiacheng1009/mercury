#include "generic_camera.hpp"

#include <gtest/gtest.h>

template <class Scalar>
struct TestConstants;

template <>
struct TestConstants<double> {
  static constexpr double epsilon = 1e-8;
  static constexpr double max_norm = 1e-3;
};

template <>
struct TestConstants<float> {
  static constexpr double epsilon = 1e-2;
  static constexpr double max_norm = 1e-2;
};

template <typename Derived1, typename Derived2, typename F>
void test_jacobian(
    const std::string &name, const Eigen::MatrixBase<Derived1> &Ja, F func,
    const Eigen::MatrixBase<Derived2> &x0,
    double eps = TestConstants<typename Derived1::Scalar>::epsilon,
    double max_norm = TestConstants<typename Derived1::Scalar>::max_norm) {
  typedef typename Derived1::Scalar Scalar;

  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Jn = Ja;
  Jn.setZero();

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> inc = x0;
  for (int i = 0; i < Jn.cols(); i++) {
    inc.setZero();
    inc[i] += eps;

    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> fpe = func(x0 + inc);
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> fme = func(x0 - inc);

    Jn.col(i) = (fpe - fme);
  }

  Jn /= (2 * eps);

  EXPECT_TRUE(Ja.allFinite()) << name << ": Ja not finite\n " << Ja;
  EXPECT_TRUE(Jn.allFinite()) << name << ": Jn not finite\n " << Jn;

  if (Jn.isZero(max_norm) && Ja.isZero(max_norm)) {
    EXPECT_TRUE((Jn - Ja).isZero(max_norm))
        << name << ": Ja not equal to Jn(diff norm:" << (Jn - Ja).norm()
        << ")\nJa: (norm: " << Ja.norm() << ")\n"
        << Ja << "\nJn: (norm: " << Jn.norm() << ")\n"
        << Jn;
    //<< "\ndiff:\n" << Jn - Ja;
  } else {
    EXPECT_TRUE(Jn.isApprox(Ja, max_norm))
        << name << ": Ja not equal to Jn (diff norm:" << (Jn - Ja).norm()
        << ")\nJa: (norm: " << Ja.norm() << ")\n"
        << Ja << "\nJn: (norm: " << Jn.norm() << ")\n"
        << Jn;
    //<< "\ndiff:\n" << Jn - Ja;
  }
}

template <typename CamT>
void testProjectUnproject() {
  AlignedVector<CamT> test_cams = CamT::getTestProjections();

  using Scalar = typename CamT::Vec2::Scalar;
  using Vec2 = typename CamT::Vec2;
  using Vec4 = typename CamT::Vec4;

  for (const CamT &cam : test_cams) {
    for (int x = -10; x <= 10; x++) {
      for (int y = -10; y <= 10; y++) {
        for (int z = 0; z <= 5; z++) {
          Vec4 p(x, y, z, 0.23424);

          Vec4 p_normalized = Vec4::Zero();
          p_normalized.template head<3>() = p.template head<3>().normalized();
          Vec2 res;
          bool success = cam.project(p, res);

          if (success) {
            Vec4 p_uproj;
            cam.unproject(res, p_uproj);

            EXPECT_TRUE(p_normalized.isApprox(
                p_uproj, 1e-5))
                << "p_normalized " << p_normalized.transpose() << " p_uproj "
                << p_uproj.transpose();
          }
        }
      }
    }
  }
}

template <typename CamT>
void testProjectJacobian() {
  AlignedVector<CamT> test_cams = CamT::getTestProjections();

  using VecN = typename CamT::VecN;
  using Vec2 = typename CamT::Vec2;
  using Vec4 = typename CamT::Vec4;

  using Mat24 = typename CamT::Mat24;
  using Mat2N = typename CamT::Mat2N;

  for (const CamT &cam : test_cams) {
    for (int x = -10; x <= 10; x++) {
      for (int y = -10; y <= 10; y++) {
        for (int z = -1; z <= 5; z++) {
          Vec4 p(x, y, z, 1);

          Mat24 J_p;
          Mat2N J_param;

          Vec2 res1;

          bool success = cam.project(p, res1, &J_p, &J_param);

          if (success) {
            test_jacobian(
                "d_r_d_p", J_p,
                [&](const Vec4 &x) {
                  Vec2 res;
                  cam.project(p + x, res);
                  return res;
                },
                Vec4::Zero());

            test_jacobian(
                "d_r_d_param", J_param,
                [&](const VecN &x) {
                  Vec2 res;

                  CamT cam1 = cam;
                  cam1 += x;

                  cam1.project(p, res);
                  return res;
                },
                VecN::Zero());
          }
        }
      }
    }
  }
}


TEST(CameraTestCase, PinholeProjectUnproject) {
  testProjectUnproject<PinholeCamera<double>>();
}
TEST(CameraTestCase, PinholeProjectUnprojectFloat) {
  testProjectUnproject<PinholeCamera<float>>();
}

TEST(CameraTestCase, UnifiedProjectUnproject) {
  testProjectUnproject<UnifiedCamera<double>>();
}
TEST(CameraTestCase, UnifiedProjectUnprojectFloat) {
  testProjectUnproject<UnifiedCamera<float>>();
}

TEST(CameraTestCase, ExtendedUnifiedProjectUnproject) {
  testProjectUnproject<ExtendedUnifiedCamera<double>>();
}
TEST(CameraTestCase, ExtendedUnifiedProjectUnprojectFloat) {
  testProjectUnproject<ExtendedUnifiedCamera<float>>();
}

TEST(CameraTestCase, KannalaBrandtProjectUnproject) {
  testProjectUnproject<KannalaBrandtCamera4<double>>();
}
TEST(CameraTestCase, KannalaBrandtProjectUnprojectFloat) {
  testProjectUnproject<KannalaBrandtCamera4<float>>();
}

TEST(CameraTestCase, DoubleSphereProjectUnproject) {
  testProjectUnproject<DoubleSphereCamera<double>>();
}
TEST(CameraTestCase, DoubleSphereProjectUnprojectFloat) {
  testProjectUnproject<DoubleSphereCamera<float>>();
}

TEST(CameraTestCase, FovProjectUnproject) {
  testProjectUnproject<FovCamera<double>>();
}

TEST(CameraTestCase, FovProjectUnprojectFloat) {
  testProjectUnproject<FovCamera<float>>();
}

TEST(CameraTestCase, PinholeProjectJacobians) {
  testProjectJacobian<PinholeCamera<double>>();
}
TEST(CameraTestCase, PinholeProjectJacobiansFloat) {
  testProjectJacobian<PinholeCamera<float>>();
}

TEST(CameraTestCase, UnifiedProjectJacobians) {
  testProjectJacobian<UnifiedCamera<double>>();
}
TEST(CameraTestCase, UnifiedProjectJacobiansFloat) {
  testProjectJacobian<UnifiedCamera<float>>();
}

TEST(CameraTestCase, ExtendedUnifiedProjectJacobians) {
  testProjectJacobian<ExtendedUnifiedCamera<double>>();
}
TEST(CameraTestCase, ExtendedUnifiedProjectJacobiansFloat) {
  testProjectJacobian<ExtendedUnifiedCamera<float>>();
}

TEST(CameraTestCase, KannalaBrandtProjectJacobians) {
  testProjectJacobian<KannalaBrandtCamera4<double>>();
}
TEST(CameraTestCase, KannalaBrandtProjectJacobiansFloat) {
  testProjectJacobian<KannalaBrandtCamera4<float>>();
}

TEST(CameraTestCase, DoubleSphereJacobians) {
  testProjectJacobian<DoubleSphereCamera<double>>();
}
TEST(CameraTestCase, FovCameraJacobians) {
  testProjectJacobian<FovCamera<double>>();
}
