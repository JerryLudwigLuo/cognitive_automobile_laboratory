/*
 * Copyright 2014. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */

#ifndef TRIANGULATION_FUNCTIONS_HPP
#define TRIANGULATION_FUNCTIONS_HPP

//standard includes
#include <string>
#include <vector>
#include <tuple>
#include <memory>
#include <cassert>
#include <iostream>
#include <math.h>

// #include <MRT/libPointMatching/pointMatching.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Triangulation{
/**
 * @brief triangulateCheiral, triangulation a la Geiger
 * Thsi is an approach to calculate the trinagulate 3D point by an implementation of the "Direct linear transoframtion" algoithm, minimizing the algebraic error e_alg=cross( (u,v,1)^T , ProjMatrix*(x,y,z)^T )
 * (Multiple View Geometry, Hartley and Zissermann, p.312)
 * @param p_matched
 * @param K, 3x3 intrinsic matrix
 * @param R, rotation, 3x3
 * @param t, translation 3
 * @param X, output: trinagulated 3d points, !normed! pionts
 * @return
 */
// std::vector<std::pair<Eigen::Vector3d,PointMatching::cMatcher::tPosition > > triangulateCheiral(const PointMatching::cMatcher::tCleanMatches &p_matched,const Eigen::Matrix3d &K,const Eigen::Matrix3d &R,const Eigen::Vector3d &T)
// {
//
//     //translation must be normed
//     Eigen::Vector3d t=T/T.norm();
//     // init 3d point matrix
//     Eigen::MatrixXd X(4,p_matched.size());
//
//     // projection matrices
//     Eigen::MatrixXd P1=Eigen::MatrixXd::Zero(3,4);
//     Eigen::MatrixXd P2=Eigen::MatrixXd::Zero(3,4);
//
//     P1.topLeftCorner(3,3)=K;
//     P2.topLeftCorner(3,3)=R;
//
//     P2(0,3)=t(0); P2(1,3)=t(1); P2(2,3)=t(2);
//     P2 = K*P2;
//
//     // triangulation via orthogonal regression
//     Eigen::Matrix4d J;
//     for (int32_t i=0; i<(int)p_matched.size(); i++) {
//         /* HERE WE SOLVE cross( (u,v,1)^T , ProjMatrix*(x,y,z)^T )=0 (algebraic error) from that we obtain to linear independant equations with X=(x,y,z)^T:
//         u*(P(2,:)*X)-P(0,:)*X=0 and v*(P(2,:)*X)-P(1,:)*X=0
//         The third row of the cross product is linearly dependant. taking this equation for a second point (u',v',1) with P' and reforming it yields A*X=0, with
//         A=[   u*P(2,:)-P(0,:) ;
//             v*P(2,:)-P(1,:) ;
//             u'*P'(2,:)-P'(0,:) ;
//             v*'P(2,:)-P'(1,:) ]
//             whoch can be solved using SVD (This system is overdetermined since we only can calculate the point up to scale). The least square solution is hence the vector corresponding to the minimum singular value.
// */
//         for (int32_t j=0; j<4; j++) {
//             J(0,j) = P1(2,j)*p_matched[i][0].first - P1(0,j);
//             J(1,j) = P1(2,j)*p_matched[i][0].second - P1(1,j);
//             J(2,j) = P2(2,j)*p_matched[i][1].first - P2(0,j);
//             J(3,j) = P2(2,j)*p_matched[i][1].second - P2(1,j);
//         }
//         Eigen::JacobiSVD<Eigen::Matrix4d> svd(J,Eigen::ComputeFullV);
//         //        auto U = svd.matrixU();
//         Eigen::Matrix4d V = svd.matrixV();
//         //        auto S = svd.singularValues();
//
//         //singular values are sorted in descending order by default -> solution is vector corresponding to minimum singular value
//         X.col(i)=V.col(3);
//     }
//
//     std::vector<std::pair<Eigen::Vector3d,PointMatching::cMatcher::tPosition>> Out;
//     Out.reserve(X.cols());
//     for (size_t i = 0; i < X.cols(); ++i)
//     {
//         Eigen::Vector3d LM(X(0,i),X(1,i),X(2,i));
//         //normalize Points(if not they lie on sphere)
//         LM=LM/X(3,i);
//         PointMatching::cMatcher::tPosition Meas=p_matched[i][1];
//
//         Out.push_back(std::make_pair(LM,Meas));
//     }
//
//     return Out;
//
//     //    // compute inliers
//     //    viso::Matrix  AX1 = P1*X;
//     //    viso::Matrix  BX1 = P2*X;
//     //    int32_t num = 0;
//     //    for (int32_t i=0; i<X.n; i++)
//     //        if (AX1.val[2][i]*X.val[3][i]>0 && BX1.val[2][i]*X.val[3][i]>0)
//     //            num++;
//
//     //    // return number of inliers
//     //    return num;
//
// }

struct PMatchEig
{
    PMatchEig(){;}
    PMatchEig(Eigen::Vector2d Cur,Eigen::Vector2d Prev):
    Current(Cur)
    ,Previous(Prev)
    {;}
    Eigen::Vector2d Current;
    Eigen::Vector2d Previous;
};

/**
 * @brief triangulateCheiral, triangulation a la Geiger
 * Thsi is an approach to calculate the trinagulate 3D point by an implementation of the "Direct linear transoframtion" algoithm, minimizing the algebraic error e_alg=cross( (u,v,1)^T , ProjMatrix*(x,y,z)^T )
 * (Multiple View Geometry, Hartley and Zissermann, p.312)
 * @param p_matched, point matches reconstructed points will be expressed in coordinate system correspodning to frame of p_matched[i].first, motion direction doesn't matter
 * @param K, 3x3 intrinsic matrix
 * @param R, rotation, 3x3
 * @param t, translation 3
 * @param X, output: triangulated 3d points, points in COS of previous match
 * @return triangulated point cloud
 */
inline std::vector<Eigen::Vector3d> triangulateCheiral(const std::vector<PMatchEig> &p_matched,const Eigen::Matrix3d &K,const Eigen::Matrix3d &R,const Eigen::Vector3d &t)
{

    //translation must be normed
//    Eigen::Vector3d t=T/T.norm();
    // init 3d point matrix
    Eigen::MatrixXd X(4,p_matched.size());

    // projection matrices
    Eigen::MatrixXd P1=Eigen::MatrixXd::Zero(3,4);
    Eigen::MatrixXd P2=Eigen::MatrixXd::Zero(3,4);

    P1.topLeftCorner(3,3)=K;
    P2.topLeftCorner(3,3)=R;

    P2(0,3)=t(0); P2(1,3)=t(1); P2(2,3)=t(2);
    P2 = K*P2;

    // triangulation via orthogonal regression
    Eigen::Matrix4d J;
    for (int32_t i=0; i<(int)p_matched.size(); i++) {
        /* HERE WE SOLVE cross( (u,v,1)^T , ProjMatrix*(x,y,z)^T )=0 (algebraic error) from that we obtain two linear independant equations with X=(x,y,z)^T:
        u*(P(2,:)*X)-P(0,:)*X=0 and v*(P(2,:)*X)-P(1,:)*X=0
        The third row of the cross product is linearly dependant. taking this equation for a second point (u',v',1) with P' and reforming it yields A*X=0, with
        A=[   u*P(2,:)-P(0,:) ;
            v*P(2,:)-P(1,:) ;
            u'*P'(2,:)-P'(0,:) ;
            v*'P(2,:)-P'(1,:) ]
            whoch can be solved using SVD (This system is overdetermined since we only can calculate the point up to scale). The least square solution is hence the vector corresponding to the minimum singular value.
*/
        for (int32_t j=0; j<4; j++) {
            J(0,j) = P1(2,j)*p_matched[i].Previous(0) - P1(0,j);
            J(1,j) = P1(2,j)*p_matched[i].Previous(1) - P1(1,j);
            J(2,j) = P2(2,j)*p_matched[i].Current(0) - P2(0,j);
            J(3,j) = P2(2,j)*p_matched[i].Current(1) - P2(1,j);
        }
        Eigen::JacobiSVD<Eigen::Matrix4d> svd(J,Eigen::ComputeFullV);
        //        auto U = svd.matrixU();
        Eigen::Matrix4d V = svd.matrixV();
        //        auto S = svd.singularValues();

        //singular values are sorted in descending order by default -> solution is vector corresponding to minimum singular value
        X.col(i)=V.col(3);
    }

    std::vector<Eigen::Vector3d> Out;
    Out.reserve(X.cols());
    for (size_t i = 0; i < X.cols(); ++i)
    {
        Eigen::Vector3d LM(X(0,i),X(1,i),X(2,i));
        //normalize Points(if not they lie on sphere)
        LM=LM/X(3,i);

        Out.push_back(LM);
    }

    return Out;
}

inline std::vector<Eigen::Vector3d> triangulateCheiral(const std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d> > &p_matched,const Eigen::Matrix3d &K,const Eigen::Matrix3d &R,const Eigen::Vector3d &T)
{
    std::vector<PMatchEig> MatchEig;MatchEig.reserve(p_matched.size());

    for ( auto & el:p_matched)
    {
        MatchEig.push_back(PMatchEig(el.first,el.second));
    }

    return triangulateCheiral(MatchEig,K,R,T);
}

//inline Eigen::Vector3d triangulate_tracklets(const std::vector<const Eigen::Vector2d*> &features, const std::vector<std::pair<const tCamera*, const Vector6d*> > &camPoseCombinations) {
//  //author: MarcSons
//    //posen global definiert
//    Vector3d lm;
//    assert(features.size() == camPoseCombinations.size() && "triangulateLandmark: unequal number of camPoses and tracklet points");
//    Matrix3d sumRrT;
//    Vector3d temp;
//    sumRrT.setZero();
//    temp.setZero();
//    for(int i = 0; i < features.size(); i++) {

//        Vector3d p0, r;


//        // sichtstrahl in kameracoordinaten (auflpunkt=p0,richtung.normalize()=r0)
//(camPoseCombinations[i].first)->getIntrinsicModel()->getViewingRay(*(features[i]), p0, r);
////        cout << "p0 " << p0.transpose() << " r " << r.transpose() << endl;
//        T_se3 pose;
//        rodXyz_2_qt(*(camPoseCombinations[i].second), pose);
//        pose = pose * camPoseCombinations[i].first->getPose();
//        p0 = pose * p0;
//        r = pose.rotation() * r;
//        Matrix3d curRrT = Matrix3d::Identity() - r * r.transpose();
//        sumRrT += curRrT;
//        temp += curRrT * p0;
//    }
//    lm = sumRrT.inverse() * temp;
//    return lm;
//}
inline Eigen::Vector3d triangulate_tracklet(const std::vector<Eigen::Vector2d> &FeatureTracklet, const std::vector<Eigen::Affine3d> &CamPoses,Eigen::Matrix3d InvIntrinsics)
{
    //author: MarcSons, modified by JG
    //posen global definiert
    Eigen::Vector3d lm;
    assert(FeatureTracklet.size() == CamPoses.size() && "triangulateLandmark: unequal number of camPoses and tracklet points");
    Eigen::Matrix3d sumRrT;
    Eigen::Vector3d temp;
    sumRrT.setZero();
    temp.setZero();
    for(int i = 0; i < FeatureTracklet.size(); i++) {

        Eigen::Vector3d p0;p0.setZero();
        // sichtstrahl in kameracoordinaten (aufpunkt=p0,richtung.normalize()=r)
       // (CamPoses[i].first)->getIntrinsicModel()->getViewingRay(*(features[i]), p0, r);
        Eigen::Vector3d r=InvIntrinsics*Eigen::Vector3d(FeatureTracklet[i](0),FeatureTracklet[i](1),1.);
        r.normalize();
//        cout << "p0 " << p0.transpose() << " r " << r.transpose() << endl;
        Eigen::Affine3d pose=CamPoses[i];
        p0 = pose * p0;
        r = pose.rotation() * r;
        Eigen::Matrix3d curRrT = Eigen::Matrix3d::Identity() - r * r.transpose();
        sumRrT += curRrT;
        temp += curRrT * p0;
    }
    lm = sumRrT.inverse() * temp;
    return lm;
}

inline std::vector<Eigen::Vector3d> triangulate_tracklets(const std::vector<std::vector<Eigen::Vector2d> >& FeatureTracklets, const std::vector<std::vector<Eigen::Affine3d> >& CamPoses,Eigen::Matrix3d InvIntrinsics)
{
    std::vector<Eigen::Vector3d> Out;Out.reserve(FeatureTracklets.size());

    assert(FeatureTracklets.size()==CamPoses.size());
    for (size_t i = 0; i < FeatureTracklets.size(); ++i)
    {
        auto P=triangulate_tracklet(FeatureTracklets[i],CamPoses[i],InvIntrinsics);
        Out.push_back(P);
    }
    return Out;
}

} //end of namespace
#endif // TRIANGULATION_FUNCTIONS_HPP
