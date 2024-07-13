/*
@file
@brief Implementation of double sphere camera model
*/

#pragma once

#include "common.hpp"
#include <Eigen/Dense>

template <typename Scalar = double>
class DoubleSphereCamera {
 public:
  static constexpr int N = 6;  ///< Number of intrinsic parameters.

  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;

  using VecN = Eigen::Matrix<Scalar, N, 1>;

  using Mat24 = Eigen::Matrix<Scalar, 2, 4>;
  using Mat2N = Eigen::Matrix<Scalar, 2, N>;

  using Mat42 = Eigen::Matrix<Scalar, 4, 2>;
  using Mat4N = Eigen::Matrix<Scalar, 4, N>;

  /// @brief Default constructor with zero intrinsics
  DoubleSphereCamera() { param.setZero(); }

  /// @brief Construct camera model with given vector of intrinsics
  ///
  /// @param[in] p vector of intrinsic parameters [fx, fy, cx, cy, xi, alpha]
  explicit DoubleSphereCamera(const VecN& p) { param = p; }

  /// @brief Cast to different scalar type
  template <class Scalar2>
  DoubleSphereCamera<Scalar2> cast() const {
    return DoubleSphereCamera<Scalar2>(param.template cast<Scalar2>());
  }

  /// @brief Camera model name
  ///
  /// @return "ds"
  static std::string getName() { return "ds"; }

  /// @brief Project the point and optionally compute Jacobians
  ///
  /// Projection function is defined as follows:
  /// \f{align}{
  ///    \pi(\mathbf{x}, \mathbf{i}) &=
  ///    \begin{bmatrix}
  ///    f_x{\frac{x}{\alpha d_2 + (1-\alpha) (\xi d_1 + z)}}
  ///    \\ f_y{\frac{y}{\alpha d_2 + (1-\alpha) (\xi d_1 + z)}}
  ///    \\ \end{bmatrix}
  ///    +
  ///    \begin{bmatrix}
  ///    c_x
  ///    \\ c_y
  ///    \\ \end{bmatrix},
  ///    \\ d_1 &= \sqrt{x^2 + y^2 + z^2},
  ///    \\ d_2 &= \sqrt{x^2 + y^2 + (\xi  d_1 + z)^2}.
  /// \f}
  /// A set of 3D points that results in valid projection is expressed as
  /// follows: \f{align}{
  ///    \Omega &= \{\mathbf{x} \in \mathbb{R}^3 ~|~ z > -w_2 d_1 \}
  ///    \\ w_2 &= \frac{w_1+\xi}{\sqrt{2w_1\xi + \xi^2 + 1}}
  ///    \\ w_1 &= \begin{cases} \frac{\alpha}{1-\alpha}, & \mbox{if } \alpha
  ///    \le 0.5 \\ \frac{1-\alpha}{\alpha} & \mbox{if } \alpha > 0.5
  ///    \end{cases}
  /// \f}
  ///
  /// @param[in] p3d point to project
  /// @param[out] proj result of projection
  /// @param[out] d_proj_d_p3d if not nullptr computed Jacobian of projection
  /// with respect to p3d
  /// @param[out] d_proj_d_param point if not nullptr computed Jacobian of
  /// projection with respect to intrinsic parameters
  /// @return if projection is valid
  inline bool project(const Vec4& p3d, Vec2& proj,
                      Mat24* d_proj_d_p3d = nullptr,
                      Mat2N* d_proj_d_param = nullptr) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];

    const Scalar& xi = param[4];
    const Scalar& alpha = param[5];

    const Scalar& x = p3d[0];
    const Scalar& y = p3d[1];
    const Scalar& z = p3d[2];

    const Scalar xx = x * x;
    const Scalar yy = y * y;
    const Scalar zz = z * z;

    const Scalar r2 = xx + yy;

    const Scalar d1_2 = r2 + zz;
    const Scalar d1 = sqrt(d1_2);

    const Scalar w1 = alpha > Scalar(0.5) ? (Scalar(1) - alpha) / alpha
                                          : alpha / (Scalar(1) - alpha);
    const Scalar w2 =
        (w1 + xi) / sqrt(Scalar(2) * w1 * xi + xi * xi + Scalar(1));
    if (z <= -w2 * d1) return false;

    const Scalar k = xi * d1 + z;
    const Scalar kk = k * k;

    const Scalar d2_2 = r2 + kk;
    const Scalar d2 = sqrt(d2_2);

    const Scalar norm = alpha * d2 + (1.0 - alpha) * k;
    const Scalar norm_inv = Scalar(1) / norm;

    const Scalar mx = x * norm_inv;
    const Scalar my = y * norm_inv;

    proj[0] = fx * mx + cx;
    proj[1] = fy * my + cy;

    if (d_proj_d_p3d || d_proj_d_param) {
      const Scalar d2_inv = Scalar(1) / d2;
      const Scalar norm_inv2 = norm_inv * norm_inv;

      if (d_proj_d_p3d) {
        const Scalar d1_inv = Scalar(1) / d1;
        const Scalar xy = x * y;
        const Scalar tt2 = xi * z * d1_inv + 1.0;

        const Scalar d_norm_d_r2 =
            (xi * (Scalar(1) - alpha) * d1_inv +
             alpha * (xi * k * d1_inv + Scalar(1)) * d2_inv) *
            norm_inv2;

        const Scalar tmp2 =
            ((Scalar(1) - alpha) * tt2 + alpha * k * tt2 * d2_inv) * norm_inv2;

        (*d_proj_d_p3d)(0, 0) = fx * (norm_inv - xx * d_norm_d_r2);
        (*d_proj_d_p3d)(1, 0) = -fy * xy * d_norm_d_r2;

        (*d_proj_d_p3d)(0, 1) = -fx * xy * d_norm_d_r2;
        (*d_proj_d_p3d)(1, 1) = fy * (norm_inv - yy * d_norm_d_r2);

        (*d_proj_d_p3d)(0, 2) = -fx * x * tmp2;
        (*d_proj_d_p3d)(1, 2) = -fy * y * tmp2;

        (*d_proj_d_p3d)(0, 3) = Scalar(0);
        (*d_proj_d_p3d)(1, 3) = Scalar(0);
      }

      if (d_proj_d_param) {
        (*d_proj_d_param).setZero();
        (*d_proj_d_param)(0, 0) = mx;
        (*d_proj_d_param)(0, 2) = Scalar(1);
        (*d_proj_d_param)(1, 1) = my;
        (*d_proj_d_param)(1, 3) = Scalar(1);

        const Scalar tmp4 =
            (alpha - Scalar(1) - alpha * k * d2_inv) * d1 * norm_inv2;
        const Scalar tmp5 = (k - d2) * norm_inv2;

        (*d_proj_d_param)(0, 4) = fx * x * tmp4;
        (*d_proj_d_param)(1, 4) = fy * y * tmp4;

        (*d_proj_d_param)(0, 5) = fx * x * tmp5;
        (*d_proj_d_param)(1, 5) = fy * y * tmp5;
      }
    }

    return true;
  }

  /// @brief Unproject the point and optionally compute Jacobians
  ///
  /// The unprojection function is computed as follows: \f{align}{
  ///    \pi^{-1}(\mathbf{u}, \mathbf{i}) &=
  ///    \frac{m_z \xi + \sqrt{m_z^2 + (1 - \xi^2) r^2}}{m_z^2 + r^2}
  ///    \begin{bmatrix}
  ///    m_x \\ m_y \\m_z
  ///    \\ \end{bmatrix}-\begin{bmatrix}
  ///    0 \\ 0 \\ \xi
  ///    \\ \end{bmatrix},
  ///    \\ m_x &= \frac{u - c_x}{f_x},
  ///    \\ m_y &= \frac{v - c_y}{f_y},
  ///    \\ r^2 &= m_x^2 + m_y^2,
  ///    \\ m_z &= \frac{1 - \alpha^2  r^2}{\alpha  \sqrt{1 - (2 \alpha - 1)
  ///    r^2}
  ///    + 1 - \alpha},
  /// \f}
  ///
  /// The valid range of unprojections is \f{align}{
  ///    \Theta &= \begin{cases}
  ///    \mathbb{R}^2 & \mbox{if } \alpha \le 0.5
  ///    \\ \{ \mathbf{u} \in \mathbb{R}^2 ~|~ r^2 \le \frac{1}{2\alpha-1} \}  &
  ///    \mbox{if} \alpha > 0.5 \end{cases}
  /// \f}
  ///
  /// @param[in] proj point to unproject
  /// @param[out] p3d result of unprojection
  /// @param[out] d_p3d_d_proj if not nullptr computed Jacobian of unprojection
  /// with respect to proj
  /// @param[out] d_p3d_d_param point if not nullptr computed Jacobian of
  /// unprojection with respect to intrinsic parameters
  /// @return if unprojection is valid
  inline bool unproject(const Vec2& proj, Vec4& p3d,
                        Mat42* d_p3d_d_proj = nullptr,
                        Mat4N* d_p3d_d_param = nullptr) const {
    const Scalar& fx = param[0];
    const Scalar& fy = param[1];
    const Scalar& cx = param[2];
    const Scalar& cy = param[3];

    const Scalar& xi = param[4];
    const Scalar& alpha = param[5];

    const Scalar mx = (proj[0] - cx) / fx;
    const Scalar my = (proj[1] - cy) / fy;

    const Scalar r2 = mx * mx + my * my;

    if (alpha > 0.5) {
      if (r2 >= Scalar(1.0) / (2 * alpha - 1)) return false;
    }

    const Scalar xi2_2 = alpha * alpha;
    const Scalar xi1_2 = xi * xi;

    const Scalar sqrt2 = sqrt(1 - (2 * alpha - 1) * r2);

    const Scalar norm2 = alpha * sqrt2 + 1 - alpha;

    const Scalar mz = (1 - xi2_2 * r2) / norm2;
    const Scalar mz2 = mz * mz;

    const Scalar norm1 = mz2 + r2;
    const Scalar sqrt1 = sqrt(mz2 + (1 - xi1_2) * r2);
    const Scalar k = (mz * xi + sqrt1) / norm1;

    p3d[0] = k * mx;
    p3d[1] = k * my;
    p3d[2] = k * mz - xi;
    p3d[3] = 0;

    if (d_p3d_d_proj || d_p3d_d_param) {
      const Scalar norm2_2 = norm2 * norm2;
      const Scalar norm1_2 = norm1 * norm1;

      const Scalar d_mz_d_r2 =
          (0.5 * alpha - xi2_2) * (r2 * xi2_2 - 1) / (sqrt2 * norm2_2) -
          xi2_2 / norm2;

      const Scalar d_mz_d_mx = 2 * mx * d_mz_d_r2;
      const Scalar d_mz_d_my = 2 * my * d_mz_d_r2;

      const Scalar d_k_d_mz =
          (norm1 * (xi * sqrt1 + mz) - 2 * mz * (mz * xi + sqrt1) * sqrt1) /
          (norm1_2 * sqrt1);

      const Scalar d_k_d_r2 =
          (xi * d_mz_d_r2 + 0.5 / sqrt1 * (2 * mz * d_mz_d_r2 + 1 - xi1_2)) /
              norm1 -
          (mz * xi + sqrt1) * (2 * mz * d_mz_d_r2 + 1) / norm1_2;

      const Scalar d_k_d_mx = d_k_d_r2 * 2 * mx;
      const Scalar d_k_d_my = d_k_d_r2 * 2 * my;

      Vec4 c0, c1;

      c0[0] = (mx * d_k_d_mx + k);
      c0[1] = my * d_k_d_mx;
      c0[2] = (mz * d_k_d_mx + k * d_mz_d_mx);
      c0[3] = 0;

      c0 /= fx;

      c1[0] = mx * d_k_d_my;
      c1[1] = (my * d_k_d_my + k);
      c1[2] = (mz * d_k_d_my + k * d_mz_d_my);
      c1[3] = 0;

      c1 /= fy;

      if (d_p3d_d_proj) {
        d_p3d_d_proj->col(0) = c0;
        d_p3d_d_proj->col(1) = c1;
      }

      if (d_p3d_d_param) {
        const Scalar d_k_d_xi1 = (mz * sqrt1 - xi * r2) / (sqrt1 * norm1);

        const Scalar d_mz_d_xi2 =
            (1 - r2 * xi2_2) * (r2 * alpha / sqrt2 - sqrt2 + 1) / norm2_2 -
            2 * r2 * alpha / norm2;

        const Scalar d_k_d_xi2 = d_k_d_mz * d_mz_d_xi2;

        (*d_p3d_d_param).col(0) = -c0 * mx;
        (*d_p3d_d_param).col(1) = -c1 * my;

        (*d_p3d_d_param).col(2) = -c0;
        (*d_p3d_d_param).col(3) = -c1;

        (*d_p3d_d_param)(0, 4) = mx * d_k_d_xi1;
        (*d_p3d_d_param)(1, 4) = my * d_k_d_xi1;
        (*d_p3d_d_param)(2, 4) = mz * d_k_d_xi1 - 1;
        (*d_p3d_d_param)(3, 4) = 0;

        (*d_p3d_d_param)(0, 5) = mx * d_k_d_xi2;
        (*d_p3d_d_param)(1, 5) = my * d_k_d_xi2;
        (*d_p3d_d_param)(2, 5) = mz * d_k_d_xi2 + k * d_mz_d_xi2;
        (*d_p3d_d_param)(3, 5) = 0;
      }
    }

    return true;
  }

  /// @brief Set parameters from initialization
  ///
  /// Initializes the camera model to  \f$ \left[f_x, f_y, c_x, c_y, 0, 0.5
  /// \right]^T \f$
  ///
  /// @param[in] init vector [fx, fy, cx, cy]
  inline void setFromInit(const Vec4& init) {
    param[0] = init[0];
    param[1] = init[1];
    param[2] = init[2];
    param[3] = init[3];
    param[4] = 0;
    param[5] = 0.5;
  }

  /// @brief Increment intrinsic parameters by inc and clamp the values to the
  /// valid range
  ///
  /// @param[in] inc increment vector
  void operator+=(const VecN& inc) {
    param += inc;
    param[4] = std::clamp(param[4], Scalar(-1), Scalar(1));
    param[5] = std::clamp(param[5], Scalar(0), Scalar(1));
  }

  /// @brief Returns a const reference to the intrinsic parameters vector
  ///
  /// The order is following: \f$ \left[f_x, f_y, c_x, c_y, \xi, \alpha
  /// \right]^T \f$
  /// @return const reference to the intrinsic parameters vector
  const VecN& getParam() const { return param; }

  /// @brief Projections used for unit-tests
  static AlignedVector<DoubleSphereCamera> getTestProjections() {
    AlignedVector<DoubleSphereCamera> res;

    VecN vec1;
    vec1 << 0.5 * 805, 0.5 * 800, 505, 509, 0.5 * -0.150694, 0.5 * 1.48785;
    res.emplace_back(vec1);

    return res;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  VecN param;
};