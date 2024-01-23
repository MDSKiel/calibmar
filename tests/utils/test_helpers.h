#pragma once

// dynamic message macro, because boost does not have one
#define ASSERT_WITH_MSG(cond, msg)    \
  do {                                \
    if (!(cond)) {                    \
      std::ostringstream str;         \
      str << msg;                     \
      std::cerr << str.str() << "\n"; \
      std::abort();                   \
    }                                 \
  } while (0)

/// @brief Compare two Eigen matrices for elementwise similarity
/// @param a First matrix
/// @param b Second matrix
/// @param atol Absolute tolerance (defaults to epsilon())
/// @param rtol Relative tolerance (defaults to 0)
/// @return
template <typename DerivedA, typename DerivedB>
bool ElementWiseClose(const Eigen::DenseBase<DerivedA>& a, const Eigen::DenseBase<DerivedB>& b,
                      const typename DerivedA::RealScalar& atol = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon(),
                      const typename DerivedA::RealScalar& rtol = 0) {
  return ((a.derived() - b.derived()).array().abs() <= (atol + rtol * b.derived().array().abs())).all();
}

template <typename T>
bool ElementWiseClose(const std::vector<T>& a, const std::vector<T>& b, T atol) {
  if (a.size() != b.size()) {
    return false;
  }

  for (size_t i = 0; i < a.size(); i++) {
    if (abs(a[i] - b[i]) > atol) {
      return false;
    }
  }

  return true;
}

inline std::vector<Eigen::Vector3d> CreatePoints3D(int rows, int columns, double square_size) {
  std::vector<Eigen::Vector3d> points3D;
  for (int row = 0; row < rows - 1; row++) {
    for (int column = 0; column < columns - 1; column++) {
      double x = (double)column * square_size;
      double y = (double)row * square_size;
      double z = 0.0;

      points3D.push_back({x, y, z});
    }
  }

  return points3D;
}