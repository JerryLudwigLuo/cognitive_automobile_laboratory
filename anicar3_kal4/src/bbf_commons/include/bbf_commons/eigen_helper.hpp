#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <set>
#define GCC_VERSION (__GNUC__ * 10000                 \
                     + __GNUC_MINOR__ * 100           \
                     + __GNUC_PATCHLEVEL__)

#if GCC_VERSION >= 70000
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#include <Eigen/Core>
#pragma GCC diagnostic pop
#else
#include <Eigen/Core>
#endif
#include <iomanip>
#include <algorithm>
#include <fstream>

#include <util_eigen/alignment.hpp>

template<typename EIGEN_MATRIX_BASE>
inline void cholSVD(const EIGEN_MATRIX_BASE &mat, EIGEN_MATRIX_BASE &chol) {

    assert(mat.rows() == mat.cols() && "Input matrix is not quadratic\n");
    Eigen::JacobiSVD< EIGEN_MATRIX_BASE > svd(mat, Eigen::ComputeFullU);
    EIGEN_MATRIX_BASE D;
    D.setZero();
    const auto &sValues = svd.singularValues();
    for(int i = 0; i < mat.rows(); i++) {
        D(i, i) = sqrt(sValues[i]);
    }
    chol = svd.matrixU() * D;
}

template<typename EIGEN_MATRIX_BASE>
inline bool isMatrixPositiveSemiDefinit(const EIGEN_MATRIX_BASE &mat) {
    assert(mat.rows() == mat.cols() && "Input matrix is not quadratic\n");
    Eigen::VectorXcd evalues = mat.eigenvalues();
    for(int i = 0; i < evalues.size(); i++) {
        if(evalues[i].real() < 0.) return false;
    }
    return true;
}


template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
   return ( (x - x).array() == (x - x).array()).all();
}


template<typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
   return ((x.array() == x.array())).all();
}

template<typename EIGEN_MATRIX_BASE>
inline bool checkMatrixData(const EIGEN_MATRIX_BASE &mat) {
    return is_finite(mat);
}

template<typename EIGEN_VECTOR, typename AR>
inline void vec_2_array(const EIGEN_VECTOR &vec, AR &ar) {
    int vSize = vec.rows() * vec.cols();
    assert(vSize == ar.size());
    std::copy(vec.data(), vec.data() + vSize, ar.begin());
}

template<typename AR, typename EIGEN_VECTOR>
inline void array_2_vec(const AR &ar, EIGEN_VECTOR &vec) {
    int vSize = vec.rows() * vec.cols();
    assert(vSize == ar.size());
    for(int i = 0; i < vSize; i++) {
        vec[i] = ar[i];
    }
}

template<typename EIGEN_MATRIX_BASE, typename AR>
inline void mat_2_array(const EIGEN_MATRIX_BASE &mat, AR &ar) {
    int mSize = mat.rows() * mat.cols();
    assert(mSize == ar.size());
    std::copy(mat.data(), mat.data() + mSize, ar.begin());
}

template<typename AR, typename EIGEN_MATRIX_BASE>
inline void array_2_mat(const AR &ar, EIGEN_MATRIX_BASE &mat) {
    int mSize = mat.rows() * mat.cols();
    assert(mSize == ar.size());
    for(int i = 0; i < mat.rows(); i++) {
        for(int j = 0; j < mat.cols(); j++) {
            mat(i, j) = ar[mat.cols() * i + j];
        }
    }
}

template<typename EigenMatrix>
void outstreamEigenMatrixRowMajor(const EigenMatrix& mat, std::ostream &o) {
    for(int i = 0u; i < mat.rows(); i++) {
        for(int j = 0u; j < mat.cols(); j++) {
            o << std::setprecision(12) << mat(i, j) << " ";
        }
    }
}

template<typename EigenMatrix>
void instreamEigenMatrixRowMajor(std::ifstream &is, EigenMatrix &mat) {
    for(int i = 0; i < mat.rows(); i++) {
        for(int j = 0; j < mat.cols(); j++) {
            if( !(is >> mat(i, j)) ) break;
        }
    }
}

