#pragma once

/// @brief Compare two Eigen matrices for elementwise similarity
/// @param a First matrix
/// @param b Second matrix
/// @param atol Absolute tolerance (defaults to epsilon())
/// @param rtol Relative tolerance (defaults to 0)
/// @return 
template<typename DerivedA, typename DerivedB>
bool ElementWiseClose(const Eigen::DenseBase<DerivedA>& a,
              const Eigen::DenseBase<DerivedB>& b,
              const typename DerivedA::RealScalar& atol
                  = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon(),
              const typename DerivedA::RealScalar& rtol
                  = 0)
{
  return ((a.derived() - b.derived()).array().abs()
          <= (atol + rtol * b.derived().array().abs())).all();
}