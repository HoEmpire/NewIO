//
// Created by ram on 18-3-19.
//

#pragma once

#include <Eigen/Dense>

// namespace Motion {

/// A column vector of size 1 (that is, a scalar), templated on scalar type.
    template <typename Scalar>
    using Vector1 = Eigen::Matrix<Scalar, 1, 1>;

/// A column vector of size 1 of doubles.
    using Vector1d = Eigen::Matrix<double, 1, 1>;

/// A column vector of size 2, templated on scalar type.
    template <typename Scalar>
    using Vector2 = Eigen::Matrix<Scalar, 2, 1>;

/// A column vector of size 3, templated on scalar type.
    template <typename Scalar>
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

/// A column vector of size 4, templated on scalar type.
    template <typename Scalar>
    using Vector4 = Eigen::Matrix<Scalar, 4, 1>;

/// A column vector of size 6.
    template <typename Scalar>
    using Vector6 = Eigen::Matrix<Scalar, 6, 1>;

/// A column vector of any size, templated on scalar type.
    template <typename Scalar>
    using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

/// A vector of dynamic size templated on scalar type, up to a maximum of 6
/// elements.
    template <typename Scalar>
    using VectorUpTo6 = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, 6, 1>;

/// A matrix of 2 rows and 2 columns, templated on scalar type.
    template <typename Scalar>
    using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;

/// A matrix of 3 rows and 3 columns, templated on scalar type.
    template <typename Scalar>
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

/// A matrix of 4 rows and 4 columns, templated on scalar type.
    template <typename Scalar>
    using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

/// A matrix of 6 rows and 6 columns, templated on scalar type.
    template <typename Scalar>
    using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;

/// A matrix of 2 rows, dynamic columns, templated on scalar type.
    template <typename Scalar>
    using Matrix2X = Eigen::Matrix<Scalar, 2, Eigen::Dynamic>;

/// A matrix of 3 rows, dynamic columns, templated on scalar type.
    template <typename Scalar>
    using Matrix3X = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;

/// A matrix of 4 rows, dynamic columns, templated on scalar type.
    template <typename Scalar>
    using Matrix4X = Eigen::Matrix<Scalar, 4, Eigen::Dynamic>;

/// A matrix of 6 rows, dynamic columns, templated on scalar type.
    template <typename Scalar>
    using Matrix6X = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;

/// A matrix of dynamic size, templated on scalar type.
    template <typename Scalar>
    using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

/// A matrix of dynamic size templated on scalar type, up to a maximum of 6 rows
/// and 6 columns. Rectangular matrices, with different number of rows and
/// columns, are allowed.
    template <typename Scalar>
    using MatrixUpTo6 =
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 6>;

/// A quaternion templated on scalar type.
    template <typename Scalar>
    using Quaternion = Eigen::Quaternion<Scalar>;

/// An AngleAxis templated on scalar type.
    template <typename Scalar>
    using AngleAxis = Eigen::AngleAxis<Scalar>;

/// An Isometry templated on scalar type.
    template <typename Scalar>
    using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

/// A translation in 3D templated on scalar type.
    template <typename Scalar>
    using Translation3 = Eigen::Translation<Scalar, 3>;

    template <typename Scalar>
    using Pose3d = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

    template <typename T>
    using vector_aligned = std::vector<T, Eigen::aligned_allocator<T>>;

    template <typename Scalar>
    inline const AngleAxis<Scalar> AngleAxisZ(const Scalar q) { return AngleAxis<Scalar>(q, Vector3<Scalar>::UnitZ());}

    template <typename Scalar>
    inline const AngleAxis<Scalar> AngleAxisY(const Scalar q) { return AngleAxis<Scalar>(q, Vector3<Scalar>::UnitY());}

    template <typename Scalar>
    inline const AngleAxis<Scalar> AngleAxisX(const Scalar q) { return AngleAxis<Scalar>(q, Vector3<Scalar>::UnitX());}
// }
