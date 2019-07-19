/*
 * Copyright 2014. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */

#ifndef HELPERS_HPP
#define HELPERS_HPP

//standard includes
#include <string>
#include <vector>
#include <tuple>
#include <memory>
#include <cassert>
#include <iostream>
#include <math.h>
#include <set>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp> //attention, eigen must be icluded before that!
//#define CPP97

namespace Helper{
/**
 * @brief predicate for removing items from a vector by indices, grabbed from http://stackoverflow.com/questions/7571937/how-to-delete-items-from-a-stdvector-given-a-list-of-indices
 *
 * usage example:
    vector<string> items;
    items.push_back("zeroth");
    items.push_back("first");
    items.push_back("second");
    items.push_back("third");
    items.push_back("fourth");
    items.push_back("fifth");

    vector<size_t> indicesToDelete;
    indicesToDelete.push_back(3);
    indicesToDelete.push_back(0);
    indicesToDelete.push_back(1);

    auto pos = remove_if(items.begin(), items.end(), predicate<std::string>(items.begin(), indicesToDelete));
    items.erase(pos, items.end());
 */
template<class T>
struct Predicate {
    public:
        typedef typename std::vector<T>::iterator itr;
    public:
        Predicate(const itr & begin, const std::vector<size_t> & indices) {
            m_begin = begin;
            m_indices.insert(indices.begin(), indices.end());
        }

        bool operator()(T & value) {
            const int index = std::distance(&m_begin[0], &value);
            std::set<size_t>::iterator target = m_indices.find(index);
            return target != m_indices.end();
        }

    private:
        itr m_begin;
        std::set<size_t> m_indices;
};

template<class Typ>
inline Eigen::Matrix<Typ,3,3> cross_prod_matrix(Eigen::Matrix<Typ,3,1> T)
{
    Eigen::Matrix<Typ,3,3> Out;
    Out<<       0.,     -T(2,0),    T(1,0),
            T(2,0),     0.,         -T(0,0),
            -T(1,0),    T(0,0),     0.;

    return Out;
}

/**
 * @brief is similar
 * @return  void
 * @par void
 */
template<typename T>
inline bool is_similar(Eigen::Matrix<T,3,1> N1,Eigen::Matrix<T,3,1> N2, T MaxDiffAngleDeg=30.)
{
#ifndef CPP97
    bool c=std::is_convertible<T,double>::value;
    assert(c && "Error template not convertible");
#endif
    N1=N1.template cast<double>();
    N2=N2.template cast<double>();
    if(N1.norm()==0. || N2.norm()==0.) return false;

    N1=N1/N1.norm();
    N2=N2/N2.norm();
    if (acos(std::abs(N1.dot(N2)))>MaxDiffAngleDeg*M_PI/180.)
    {
        return false;
    }
    return true;
}
#ifndef CPP97
inline double calc_std_dev(std::vector<double> v)
{
    double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
    double m =  sum / v.size();

    double accum = 0.0;
    std::for_each (std::begin(v), std::end(v), [&](const double d) {
        accum += (d - m) * (d - m);
    });

    double stdev = sqrt(accum / (v.size()-1));
    return stdev;
}
#endif
inline double get_angle_around_z(Eigen::Vector3d Vec,Eigen::Vector3d x,Eigen::Vector3d y,Eigen::Vector3d z)
{
    Vec=1./Vec.norm()*Vec;

    //        Eigen::Vector3d (0.,1.,0.);
    double Dist=Vec.dot(z);
    Eigen::Vector3d PointOnPlane=Vec-Dist*z;
    assert(std::abs(PointOnPlane.dot(z))<1e-06 && "plane projection failed");

    return std::atan2(PointOnPlane.dot(y),PointOnPlane.dot(x));
}

inline void get_angles_around_y_z(double& AngleY, double & AngleZ,Eigen::Vector3d Vec)
{
    AngleZ=get_angle_around_z(Vec,Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitY(),Eigen::Vector3d::UnitZ());

    Eigen::AngleAxisd BackRot(-AngleZ,Eigen::Vector3d::UnitZ());

    Eigen::Vector3d VecInPlane=BackRot*Vec;
    assert(std::abs(VecInPlane.dot(Eigen::Vector3d::UnitY()))<1e-06 && "back transformation failed");

    AngleY=-std::atan2(VecInPlane.dot(Eigen::Vector3d::UnitZ()),VecInPlane.dot(Eigen::Vector3d::UnitX()));

    // ------------------------ DEBUG -------------------------
    // @todo: remove debug section
#if 0
    Eigen::AngleAxisd BackRotY(-AngleY,Eigen::Vector3d::UnitY());
    auto ShouldBeX=BackRotY*VecInPlane;
    assert((ShouldBeX-Eigen::Vector3d::UnitX()).norm()<0.001 && "rotation failed");
    //Test
    Eigen::Vector3d TestVec,TestVec0;
    TestVec0=Eigen::AngleAxisd(AngleY,Eigen::Vector3d::UnitY())*Eigen::Vector3d::UnitX();
    TestVec=Eigen::AngleAxisd(AngleZ,Eigen::Vector3d::UnitZ())*TestVec0;

    assert((VecInPlane-TestVec0).norm()<0.001 && "AngleY wrong");
    assert((TestVec-Vec).norm()<0.01 && "rotation test failed");
#endif
}

template <typename _Tp>
inline cv::Mat_<_Tp> transform_laser_point(std::tuple<double,double,double,double> PointIn,cv::Mat_<_Tp> Pose)
{
    typedef typename cv::DataType<_Tp>::work_type _wTp;
    cv::Mat_<_wTp> Point4d(4,1);
    Point4d(0,0)=std::get<0>(PointIn);
    Point4d(1,0)=std::get<1>(PointIn);
    Point4d(2,0)=std::get<2>(PointIn);
    Point4d(3,0)=1.;
    cv::Mat_<_wTp> TransPoint=Pose*Point4d;

    cv::Mat_<_wTp> Point3d(3,1);
    Point3d(0,0)=TransPoint(0,0);
    Point3d(1,0)=TransPoint(1,0);
    Point3d(2,0)=TransPoint(2,0);

    return Point3d;
}

template <typename _Tp>
inline cv::Point project3dCC(cv::Mat_<_Tp> Point3d,cv::Mat_<_Tp> ProjectionMatrix)
{
    typedef typename cv::DataType<_Tp>::work_type _wTp;
    assert(Point3d.size()==cv::Size(1,3) && "wrong point size for projection");
    assert(ProjectionMatrix.size()==cv::Size(3,3) && "Intrinsicsmatrix has wrong size");
    cv::Mat_<_wTp> BackProj=ProjectionMatrix*Point3d;
    BackProj=1/BackProj(2,0)*BackProj;

    return cv::Point(BackProj(0,0),BackProj(1,0));
}

template <typename _Tp>
inline cv::Point project3dCC(std::tuple<double,double,double,double> PointIn,cv::Mat_<_Tp> Pose,cv::Mat_<_Tp> ProjectionMatrix)
{
    typedef typename cv::DataType<_Tp>::work_type _wTp;
    assert(ProjectionMatrix.size()==cv::Size(3,3) && "Intrinsicsmatrix has wrong size");
    assert(Pose.size()==cv::Size(4,4) && "Pose has wrong size");
    assert(std::get<3>(PointIn)!=-1.);

    cv::Mat_<_wTp> TransPoint=transform_laser_point(PointIn,Pose);

    return project3dCC(TransPoint,ProjectionMatrix);
}

template <typename _Tp>
inline cv::Point project3dCC(std::tuple<double,double,double,double> PointIn,cv::Mat_<_Tp> ProjectionMatrix)
{
    typedef typename cv::DataType<_Tp>::work_type _wTp;

    cv::Mat_<_wTp> Point(3,1);
    Point(0,0)=std::get<0>(PointIn);
    Point(1,0)=std::get<1>(PointIn);
    Point(2,0)=std::get<2>(PointIn);

    return project3dCC(Point,ProjectionMatrix);
}

inline cv::Point project3dCC(Eigen::Vector3d Point,Eigen::Matrix4d Pose,Eigen::Matrix3d ProjectionMatrix)
{
    cv::Mat_<double> PoseCV;
    cv::Mat_<double> ProjMatCV;
    cv::eigen2cv(Pose,PoseCV);
    cv::eigen2cv(ProjectionMatrix,ProjMatCV);

    std::tuple<double,double,double,double> PTuple=std::make_tuple(Point(0),Point(1),Point(2),1.);

    return project3dCC(PTuple,PoseCV,ProjMatCV);
}


inline Eigen::Matrix3d rotation_matrix(Eigen::Vector3d axis,double theta)
{
    axis = -axis/axis.norm();
    double a,b,c,d;
    a = cos(theta/2.);
    b = -axis(0)*sin(theta/2.);
    c= -axis(1)*sin(theta/2.);
    d = -axis(2)*sin(theta/2.);
    Eigen::Matrix3d Mat;
    Mat<<a*a+b*b-c*c-d*d, 2.*(b*c-a*d), 2.*(b*d+a*c),
            2.*(b*c+a*d), a*a+c*c-b*b-d*d, 2.*(c*d-a*b),
            2.*(b*d-a*c), 2.*(c*d+a*b), a*a+d*d-b*b-c*c;

    return Mat;
}

inline Eigen::Matrix4d convert_VectorRotTrans_to_homogenous(Eigen::Vector3d VecRot,Eigen::Vector3d Trans)
{
    double Angle=VecRot.norm();
    Eigen::Matrix3d Rot=rotation_matrix((1./Angle)*VecRot,Angle);
    Eigen::Matrix4d Out=Eigen::Matrix4d::Identity();
    Out.topLeftCorner(3,3)=Rot;
    Out.topRightCorner(3,1)=Trans;

    return Out;
}
template<typename T>
Eigen::Matrix<T,2,1> back_project(const Eigen::Matrix<T,3,1>& P, const Eigen::Matrix<T,3,3>& ProjMatrix,const Eigen::Matrix<T,4,4>& Pose)
{
    Eigen::Matrix<T,4,1> P1(P(0,0),P(1,0),P(2,0),1.);
    //        P1=Pose.inverse()*P1;
    //        Eigen::Matrix<T,3,1> PTrans(P1(0),P1(1),P1(2));
    auto PCamCoords=Pose*P1;
    Eigen::Matrix<T,3,1> HeadPCamCoords(PCamCoords(0,0),PCamCoords(1,0),PCamCoords(2,0));
    auto BackProj=(ProjMatrix*HeadPCamCoords)/HeadPCamCoords(2,0);

    return Eigen::Matrix<T,2,1>(BackProj(0,0),BackProj(1,0));
}

} //end of ns
#endif // HELPERS_HPP
