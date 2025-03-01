/*
@file
@brief Implementation of field-of-view camera model
*/

#pragma once

#include "common.hpp"
/// @brief Field-of-View camera model
///
/// \image html fov.png
/// This model has N=5 parameters \f$ \mathbf{i} = \left[f_x, f_y, c_x, c_y,
/// w \right]^T \f$. See \ref project and \ref unproject
/// functions for more details.
template <typename Scalar = double>
class FovCamera {
 public:
  static constexpr int N = 5;  ///< Number of intrinsic parameters.

  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

  using VecN = Eigen::Matrix<Scalar, N, 1>;

  using Mat24 = Eigen::Matrix<Scalar, 2, 4>;
  using Mat2N = Eigen::Matrix<Scalar, 2, N>;

  using Mat42 = Eigen::Matrix<Scalar, 4, 2>;
  using Mat4N = Eigen::Matrix<Scalar, 4, N>;

  using Mat44 = Eigen::Matrix<Scalar, 4, 4>;

  /// @brief Default constructor with zero intrinsics
  FovCamera() { param.setZero(); }

  /// @brief Construct camera model with given vector of intrinsics
  ///
  /// @param[in] p vector of intrinsic parameters [fx, fy, cx, cy, w]
  explicit FovCamera(const VecN& p) { param = p; }

  /// @brief Cast to different scalar type
  template <class Scalar2>
  FovCamera<Scalar2> cast() const {
    return FovCamera<Scalar2>(param.template cast<Scalar2>());
  }

  /// @brief Camera model name
  ///
  /// @return "fov"
  static const std::string getName() { return "fov"; }

  /// @brief Project the point and optionally compute Jacobians
  ///
  /// Projection function is defined as follows:
  /// \f{align}{
  ///  \DeclareMathOperator{\atantwo}{atan2}
  ///  \pi(\mathbf{x}, \mathbf{i}) &=
  ///  \begin{bmatrix} f_x r_d  \frac{x} { r_u }
  ///  \\ f_y r_d \frac{y} { r_u }
  ///  \\ \end{bmatrix}
  ///  +
  ///  \begin{bmatrix}
  ///  c_x
  ///  \\ c_y
  ///  \\ \end{bmatrix},
  ///  \\ r_u &= \sqrt{x^2 + y^2},
  ///  \\ r_d &= \frac{\atantwo(2 r_u \tan{\frac{w}{2}}, z)}{w}.
  /// \f}
  ///
  /// @param[in] p3d point to project
  /// @param[out] proj result of projection
  /// @param[out] d_proj_d_p3d if not nullptr computed Jacobian of projection
  /// with respect to p3d
  /// @param[out] d_proj_d_param point if not nullptr computed Jacobian of
  /// projection with respect to intrinsic parameters
  /// @return if projection is valid
  inline bool project(const Vec4& p, Vec2& res, Mat24* d_r_d_p = nullptr,
                      Mat2N* d_r_d_param = nullptr) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];
    const Scalar& w = param[4];

    const Scalar& x = p[0];
    const Scalar& y = p[1];
    const Scalar& z = p[2];

    Scalar r2 = x * x + y * y;
    Scalar r = sqrt(r2);

    Scalar z2 = z * z;

    const Scalar tanwhalf = std::tan(w / 2);
    const Scalar atan_wrd = std::atan2(2 * tanwhalf * r, z);

    Scalar rd = 1.0;
    Scalar d_rd_d_w = 0;
    Scalar d_rd_d_x = 0;
    Scalar d_rd_d_y = 0;
    Scalar d_rd_d_z = 0;

    Scalar tmp1 = 1.0 / std::cos(w / 2);
    Scalar d_tanwhalf_d_w = 0.5 * tmp1 * tmp1;
    Scalar tmp = (z2 + 4 * tanwhalf * tanwhalf * r2);
    Scalar d_atan_wrd_d_w = 2 * r * d_tanwhalf_d_w * z / tmp;

    if (w > 1e-5) {
      if (r2 < 1e-5) {
        if (z < 1e-5) return false;

        rd = 2 * tanwhalf / w;
        d_rd_d_w = 2 * (d_tanwhalf_d_w * w - tanwhalf) / (w * w);
      } else {
        rd = atan_wrd / (r * w);
        d_rd_d_w = (d_atan_wrd_d_w * w - atan_wrd) / (r * w * w);

        const Scalar d_r_d_x = x / r;
        const Scalar d_r_d_y = y / r;

        const Scalar d_atan_wrd_d_x = 2 * tanwhalf * d_r_d_x * z / tmp;
        const Scalar d_atan_wrd_d_y = 2 * tanwhalf * d_r_d_y * z / tmp;
        const Scalar d_atan_wrd_d_z = -2 * tanwhalf * r / tmp;

        d_rd_d_x = (d_atan_wrd_d_x * r - d_r_d_x * atan_wrd) / (r * r * w);
        d_rd_d_y = (d_atan_wrd_d_y * r - d_r_d_y * atan_wrd) / (r * r * w);
        d_rd_d_z = d_atan_wrd_d_z / (r * w);
      }
    }

    const Scalar mx = x * rd;
    const Scalar my = y * rd;

    res[0] = fx * mx + cx;
    res[1] = fy * my + cy;

    if (d_r_d_p) {
      d_r_d_p->setZero();
      (*d_r_d_p)(0, 0) = fx * (d_rd_d_x * x + rd);
      (*d_r_d_p)(0, 1) = fx * d_rd_d_y * x;
      (*d_r_d_p)(0, 2) = fx * d_rd_d_z * x;

      (*d_r_d_p)(1, 0) = fy * d_rd_d_x * y;
      (*d_r_d_p)(1, 1) = fy * (d_rd_d_y * y + rd);
      (*d_r_d_p)(1, 2) = fy * d_rd_d_z * y;
    }

    if (d_r_d_param) {
      d_r_d_param->setZero();
      (*d_r_d_param)(0, 0) = mx;
      (*d_r_d_param)(0, 2) = 1;
      (*d_r_d_param)(1, 1) = my;
      (*d_r_d_param)(1, 3) = 1;

      (*d_r_d_param)(0, 4) = fx * x * d_rd_d_w;
      (*d_r_d_param)(1, 4) = fy * y * d_rd_d_w;
    }

    return true;
  }

  /// @brief Unproject the point and optionally compute Jacobians
  ///
  /// The unprojection function is computed as follows: \f{align}{
  ///     \pi^{-1}(\mathbf{u}, \mathbf{i}) &=
  ///  \begin{bmatrix}
  ///  m_x \frac{\sin(r_d w)}{ 2 r_d \tan{\frac{w}{2}}}
  ///  \\ m_y \frac{\sin(r_d w)}{ 2 r_d \tan{\frac{w}{2}}}
  ///  \\ \cos(r_d w)
  ///  \\ \end{bmatrix},
  ///  \\ m_x &= \frac{u - c_x}{f_x},
  ///  \\ m_y &= \frac{v - c_y}{f_y},
  ///  \\ r_d &= \sqrt{m_x^2 + m_y^2}.
  /// \f}
  ///
  ///
  /// @param[in] proj point to unproject
  /// @param[out] p3d result of unprojection
  /// @param[out] d_p3d_d_proj if not nullptr computed Jacobian of unprojection
  /// with respect to proj
  /// @param[out] d_p3d_d_param point if not nullptr computed Jacobian of
  /// unprojection with respect to intrinsic parameters
  /// @return if unprojection is valid
  inline bool unproject(const Vec2& p, Vec4& res, Mat42* d_r_d_p = nullptr,
                        Mat4N* d_r_d_param = nullptr) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];
    const Scalar& w = param[4];

    const Scalar tan_w_2 = std::tan(w / 2.0);
    const Scalar mul2tanwby2 = tan_w_2 * 2.0;

    const Scalar mx = (p[0] - cx) / fx;
    const Scalar my = (p[1] - cy) / fy;

    const Scalar rd = sqrt(mx * mx + my * my);

    Scalar ru = 1.0;
    Scalar sin_rd_w = 0.0;
    Scalar cos_rd_w = 1.0;

    Scalar d_ru_d_rd = 0.0;

    Scalar rd_inv = 1.0;

    if (mul2tanwby2 > 1e-5 &&
        rd > 1e-5) {
      sin_rd_w = std::sin(rd * w);
      cos_rd_w = std::cos(rd * w);
      ru = sin_rd_w / (rd * mul2tanwby2);

      rd_inv = 1.0 / rd;

      d_ru_d_rd =
          (w * cos_rd_w * rd - sin_rd_w) * rd_inv * rd_inv / mul2tanwby2;
    }

    res[0] = mx * ru;
    res[1] = my * ru;
    res[2] = cos_rd_w;
    res[3] = 0;

    if (d_r_d_p || d_r_d_param) {
      Vec4 c0, c1;

      c0(0) = (ru + mx * d_ru_d_rd * mx * rd_inv) / fx;
      c0(1) = my * d_ru_d_rd * mx * rd_inv / fx;
      c0(2) = -sin_rd_w * w * mx * rd_inv / fx;
      c0(3) = 0;

      c1(0) = my * d_ru_d_rd * mx * rd_inv / fy;
      c1(1) = (ru + my * d_ru_d_rd * my * rd_inv) / fy;
      c1(2) = -sin_rd_w * w * my * rd_inv / fy;
      c1(3) = 0;

      if (d_r_d_p) {
        d_r_d_p->col(0) = c0;
        d_r_d_p->col(1) = c1;
      }

      if (d_r_d_param) {
        d_r_d_param->setZero();

        d_r_d_param->col(2) = -c0;
        d_r_d_param->col(3) = -c1;

        d_r_d_param->col(0) = -c0 * mx;
        d_r_d_param->col(1) = -c1 * my;

        Scalar tmp = (cos_rd_w - (tan_w_2 * tan_w_2 + 1) * sin_rd_w * rd_inv /
                                     (2 * tan_w_2)) /
                     mul2tanwby2;

        (*d_r_d_param)(0, 4) = mx * tmp;
        (*d_r_d_param)(1, 4) = my * tmp;
        (*d_r_d_param)(2, 4) = -sin_rd_w * rd;
      }

      Scalar norm = res.norm();
      Scalar norm_inv = Scalar(1.0) / norm;
      Scalar norm_inv2 = norm_inv * norm_inv;
      Scalar norm_inv3 = norm_inv2 * norm_inv;

      Mat44 d_p_norm_d_p;
      d_p_norm_d_p.setZero();

      d_p_norm_d_p(0, 0) = norm_inv * (1 - res[0] * res[0] * norm_inv2);
      d_p_norm_d_p(1, 0) = -res[1] * res[0] * norm_inv3;
      d_p_norm_d_p(2, 0) = -res[2] * res[0] * norm_inv3;

      d_p_norm_d_p(0, 1) = -res[1] * res[0] * norm_inv3;
      d_p_norm_d_p(1, 1) = norm_inv * (1 - res[1] * res[1] * norm_inv2);
      d_p_norm_d_p(2, 1) = -res[1] * res[2] * norm_inv3;

      d_p_norm_d_p(0, 2) = -res[2] * res[0] * norm_inv3;
      d_p_norm_d_p(1, 2) = -res[2] * res[1] * norm_inv3;
      d_p_norm_d_p(2, 2) = norm_inv * (1 - res[2] * res[2] * norm_inv2);

      if (d_r_d_p) (*d_r_d_p) = d_p_norm_d_p * (*d_r_d_p);
      if (d_r_d_param) (*d_r_d_param) = d_p_norm_d_p * (*d_r_d_param);
    }

    res /= res.norm();

    return true;
  }

  /// @brief Set parameters from initialization
  ///
  /// Initializes the camera model to  \f$ \left[f_x, f_y, c_x, c_y, 1
  /// \right]^T \f$
  ///
  /// @param[in] init vector [fx, fy, cx, cy]
  inline void setFromInit(const Vec4& init) {
    param[0] = init[0];
    param[1] = init[1];
    param[2] = init[2];
    param[3] = init[3];
    param[4] = 1;
  }

  /// @brief Increment intrinsic parameters by inc
  ///
  /// @param[in] inc increment vector
  void operator+=(const VecN& inc) { param += inc; }

  /// @brief Returns a const reference to the intrinsic parameters vector
  ///
  /// The order is following: \f$ \left[f_x, f_y, c_x, c_y, k1, k2, k3, k4
  /// \right]^T \f$
  /// @return const reference to the intrinsic parameters vector
  const VecN& getParam() const { return param; }

  /// @brief Projections used for unit-tests
  static AlignedVector<FovCamera> getTestProjections() {
    AlignedVector<FovCamera> res;

    VecN vec1;

    // Euroc
    vec1 << 379.045, 379.008, 505.512, 509.969, 0.9259487501905697;
    res.emplace_back(vec1);

    return res;
  }

  /// @brief Resolutions used for unit-tests
  static AlignedVector<Eigen::Vector2i> getTestResolutions() {
    AlignedVector<Eigen::Vector2i> res;

    res.emplace_back(752, 480);

    return res;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  VecN param;
};