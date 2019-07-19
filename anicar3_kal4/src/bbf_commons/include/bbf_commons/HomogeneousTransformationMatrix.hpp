#ifndef HomogeneousTransformationMatrix_hpp__
#define HomogeneousTransformationMatrix_hpp__

#include <Eigen/Core> // provide overloaded functions for Eigen-Matrices and Vectors
#include <assert.h>


namespace HTM {  // HTM = 4x4 homogeneous transformation matrix

  // given a base coordinate system "base_cs" that is first translated by tx,ty,tz,
  // then rotated by yaw,pitch,roll (in rad!) results in the new coord-sys "new_cs"
  // the following functions will create HTMs for the two tasks:
  
  // with the resulting HTM, a point in the new CS p_new_cs = (x,y,z,1) can be transformed into p_base_cs by   p_base_cs = HTM * p_new_cs   or   R * p_new_cs + t
  template <class Mat44_T> // Mat assumed to have  operator(row,col)
  inline void              YawPitchRollXYZ_2_HTM( const double yawRAD, const double pitchRAD, const double rollRAD, const double tx, const double ty, const double tz, Mat44_T &htm );
  inline Eigen::Matrix4d   YawPitchRollXYZ_2_HTM( const double yawRAD, const double pitchRAD, const double rollRAD, const double tx, const double ty, const double tz ) { Eigen::Matrix4d htm; YawPitchRollXYZ_2_HTM( yawRAD, pitchRAD, rollRAD, tx, ty, tz, htm ); return htm; };
  template <class Mat44_T, class Vec6_T>
  inline void              YawPitchRollXYZ_2_HTM( const Vec6_T &params, Mat44_T &htm ) {YawPitchRollXYZ_2_HTM(params(0), params(1), params(2), params(3), params(4), params(5), htm);};
  template <class Vec6_T>
  inline Eigen::Matrix4d   YawPitchRollXYZ_2_HTM( const Vec6_T &params ) {return YawPitchRollXYZ_2_HTM(params(0), params(1), params(2), params(3), params(4), params(5));};
  template <class Mat33_T, class Vec3_T>
  inline void              YawPitchRollXYZ_2_Rt( const double yawRAD, const double pitchRAD, const double rollRAD, const double tx, const double ty, const double tz, Mat33_T &R, Vec3_T &t );
  template <class Mat33_T, class Vec3_T, class Vec6_T>
  inline void              YawPitchRollXYZ_2_Rt( const Vec6_T &params, Mat33_T &R, Vec3_T &t ) {YawPitchRollXYZ_2_Rt(params(0), params(1), params(2), params(3), params(4), params(5), R, t);};
  
  template <class Mat44_T>
  inline void              HTM_2_YawPitchRollXYZ( const Mat44_T &htm, double &yawRAD, double &pitchRAD, double &rollRAD, double &tx, double &ty, double &tz );
  template <class Mat44_T, class Vec6_T>
  inline void              HTM_2_YawPitchRollXYZ( const Mat44_T &htm, Vec6_T &params) {HTM_2_YawPitchRollXYZ(htm, params(0), params(1), params(2), params(3), params(4), params(5));};
  inline Eigen::VectorXd   HTM_2_YawPitchRollXYZ( const Eigen::Matrix4d &htm) {Eigen::VectorXd params(6); HTM_2_YawPitchRollXYZ(htm, params); return params;};
  template <class Mat33_T, class Vec3_T, class Vec6_T>
  inline void              Rt_2_YawPitchRollXYZ( const Mat33_T &R, const Vec3_T &t, double &yawRAD, double &pitchRAD, double &rollRAD, double &tx, double &ty, double &tz );
  
  // with the resulting HTM, a point p_base_cs = (x,y,z,1) can be transformed into the transformed CS by   p_new_cs = HTM * p_base_cs   or   R * p_base_cs + t
  template <class Mat44_T>
  inline void            YawPitchRollXYZ_2_HTM_inv( const double yawRAD, const double pitchRAD, const double rollRAD, const double tx, const double ty, const double tz, Mat44_T &htm );
  inline Eigen::Matrix4d YawPitchRollXYZ_2_HTM_inv( const double yawRAD, const double pitchRAD, const double rollRAD, const double tx, const double ty, const double tz ) {Eigen::Matrix4d htm; YawPitchRollXYZ_2_HTM_inv( yawRAD, pitchRAD, rollRAD, tx, ty, tz, htm ); return htm;};
  template <class Mat33_T, class Vec3_T>
  inline void            YawPitchRollXYZ_2_Rt_inv( const double yawRAD, const double pitchRAD, const double rollRAD, const double tx, const double ty, const double tz, Mat33_T &R, Vec3_T &t );
  
  // inverts the transformation
  template <class Mat44_T>
  inline Mat44_T         invert_HTM( const Mat44_T& htm );
  template <class Mat33_T, class Vec3_T>
  inline void            invert_Rt( Mat33_T &R, Vec3_T &t );
  
  // conversion:
  //inline Eigen::Matrix4d Rt_2_HTM(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
  //inline void            HTM_2_Rt(const Eigen::Matrix4d &htm, Eigen::Matrix3d &R, Eigen::Vector3d &t);
};

template <class Mat44_T>
void HTM::YawPitchRollXYZ_2_HTM( const double yawRAD, const double pitchRAD, const double rollRAD, const double tx, const double ty, const double tz, Mat44_T &htm )
{
  double c_a = cos( yawRAD );
  double s_a = sin( yawRAD );
  double c_b = cos( pitchRAD );
  double s_b = sin( pitchRAD );
  double c_c = cos( rollRAD );
  double s_c = sin( rollRAD );
  
  htm(0, 0) = c_a * c_b;
  htm(1, 0) = s_a * c_b;
  htm(2, 0) = -s_b;
  htm(3, 0) = 0;
  
  htm(0, 1) = c_a * s_b * s_c - s_a * c_c;
  htm(1, 1) = s_a * s_b * s_c + c_a * c_c;
  htm(2, 1) = c_b * s_c;
  htm(3, 1) = 0;
  
  htm(0, 2) = c_a * s_b * c_c + s_a * s_c;
  htm(1, 2) = s_a * s_b * c_c - c_a * s_c;
  htm(2, 2) = c_b * c_c;
  htm(3, 2) = 0;
  
  htm(0, 3) = tx;
  htm(1, 3) = ty;
  htm(2, 3) = tz;
  htm(3, 3) = 1;
}

template <class Mat33_T, class Vec3_T>
void HTM::YawPitchRollXYZ_2_Rt( const double yawRAD, const double pitchRAD, const double rollRAD, const double tx, const double ty, const double tz, Mat33_T &R, Vec3_T &t )
{
  double c_a = cos( yawRAD );
  double s_a = sin( yawRAD );
  double c_b = cos( pitchRAD );
  double s_b = sin( pitchRAD );
  double c_c = cos( rollRAD );
  double s_c = sin( rollRAD );
  
  R(0, 0) = c_a * c_b;
  R(1, 0) = s_a * c_b;
  R(2, 0) = -s_b;
  
  R(0, 1) = c_a * s_b * s_c - s_a * c_c;
  R(1, 1) = s_a * s_b * s_c + c_a * c_c;
  R(2, 1) = c_b * s_c;
  
  R(0, 2) = c_a * s_b * c_c + s_a * s_c;
  R(1, 2) = s_a * s_b * c_c - c_a * s_c;
  R(2, 2) = c_b * c_c;
  
  t(0) = tx;
  t(1) = ty;
  t(2) = tz;
}

template <class Mat44_T>
void HTM::HTM_2_YawPitchRollXYZ( const Mat44_T &matrix, double &yawRAD, double &pitchRAD, double &rollRAD, double &tx, double &ty, double &tz )
{
  pitchRAD = atan2(-matrix(2,0), sqrt(matrix(2,1)*matrix(2,1)+matrix(2,2)*matrix(2,2)));
  if ((M_PI - fabs(pitchRAD)) > 0.0001) { // no singularity!
      yawRAD = atan2(matrix(1,0), matrix(0,0));
      rollRAD = atan2(matrix(2,1), matrix(2,2));
  } else { // singularity! in this case either yaw or roll can be arbitrarily selected
      yawRAD = 0;
      rollRAD = atan2(matrix(0,1), matrix(1,1));
  }
  
  tx = matrix(0,3);
  ty = matrix(1,3);
  tz = matrix(2,3);
}

template <class Mat33_T, class Vec3_T, class Vec6_T>
void HTM::Rt_2_YawPitchRollXYZ( const Mat33_T &R, const Vec3_T &t, double &yawRAD, double &pitchRAD, double &rollRAD, double &tx, double &ty, double &tz )
{
  pitchRAD = atan2(-R(2,0), sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
  if ((M_PI - fabs(pitchRAD)) > 0.0001) { // no singularity!
      yawRAD = atan2(R(1,0), R(0,0));
      rollRAD = atan2(R(2,1), R(2,2));
  } else { // singularity! in this case either yaw or roll can be arbitrarily selected
      yawRAD = 0;
      rollRAD = atan2(R(0,1), R(1,1));
  }
  
  tx = t(0);
  ty = t(1);
  tz = t(2);
}

template <class Mat44_T>
void HTM::YawPitchRollXYZ_2_HTM_inv( const double yawRAD, const double pitchRAD, const double rollRAD, const double tx, const double ty, const double tz, Mat44_T &htm )
{
  double c_a = cos( yawRAD );
  double s_a = sin( yawRAD );
  
  double c_b = cos( pitchRAD );
  double s_b = sin( pitchRAD );
  
  double c_c = cos( rollRAD );
  double s_c = sin( rollRAD );
  
  htm(0, 0) = c_a * c_b;
  htm(0, 1) = s_a * c_b;
  htm(0, 2) = -s_b;
  htm(0, 3) = 0;
  
  htm(1, 0) = c_a * s_b * s_c - s_a * c_c;
  htm(1, 1) = s_a * s_b * s_c + c_a * c_c;
  htm(1, 2) = c_b * s_c;
  htm(1, 3) = 0;
  
  htm(2, 0) = c_a * s_b * c_c + s_a * s_c;
  htm(2, 1) = s_a * s_b * c_c - c_a * s_c;
  htm(2, 2) = c_b * c_c;
  htm(2, 3) = 0;
  
  htm(0, 3) = -tx * htm(0, 0) - ty * htm(0, 1) - tz * htm(0, 2);
  htm(1, 3) = -tx * htm(1, 0) - ty * htm(1, 1) - tz * htm(1, 2);
  htm(2, 3) = -tx * htm(2, 0) - ty * htm(2, 1) - tz * htm(2, 2);
  htm(3, 3) = 1.;
 
}

template <class Mat33_T, class Vec3_T>
void HTM::YawPitchRollXYZ_2_Rt_inv( const double yawRAD, const double pitchRAD, const double rollRAD, const double tx, const double ty, const double tz, Mat33_T &R, Vec3_T &t )
{
  double c_a = cos( yawRAD );
  double s_a = sin( yawRAD );
  double c_b = cos( pitchRAD );
  double s_b = sin( pitchRAD );
  double c_c = cos( rollRAD );
  double s_c = sin( rollRAD );
  
  R(0, 0) = c_a * c_b;
  R(0, 1) = s_a * c_b;
  R(0, 2) = -s_b;
  
  R(1, 0) = c_a * s_b * s_c - s_a * c_c;
  R(1, 1) = s_a * s_b * s_c + c_a * c_c;
  R(1, 2) = c_b * s_c;
  
  R(2, 0) = c_a * s_b * c_c + s_a * s_c;
  R(2, 1) = s_a * s_b * c_c - c_a * s_c;
  R(2, 2) = c_b * c_c;
  
  t(0)=-tx;
  t(1)=-ty;
  t(2)=-tz;
  t = R * t;
}

template <class Mat44_T>
Mat44_T HTM::invert_HTM( const Mat44_T& htm )
{
  Mat44_T inverse(htm);
  
  inverse(0, 0) = htm(0, 0);
  inverse(1, 1) = htm(1, 1);
  inverse(2, 2) = htm(2, 2);
  
  inverse(1, 0) = htm(0, 1);
  inverse(2, 0) = htm(0, 2);
  
  inverse(0, 1) = htm(1, 0);
  inverse(2, 1) = htm(1, 2);
  
  inverse(0, 2) = htm(2, 0);
  inverse(1, 2) = htm(2, 1);
  
  inverse(0, 3) = -( inverse(0, 0) * htm(0, 3) + inverse(0, 1) * htm(1, 3) + inverse(0, 2) * htm(2, 3) );
  inverse(1, 3) = -( inverse(1, 0) * htm(0, 3) + inverse(1, 1) * htm(1, 3) + inverse(1, 2) * htm(2, 3) );
  inverse(2, 3) = -( inverse(2, 0) * htm(0, 3) + inverse(2, 1) * htm(1, 3) + inverse(2, 2) * htm(2, 3) );
  inverse(3, 3) = 1.;
  
  inverse(3, 0) = 0.;
  inverse(3, 1) = 0.;
  inverse(3, 2) = 0.;
  
  return inverse;
}

template <class Mat33_T, class Vec3_T>
void HTM::invert_Rt( Mat33_T &R, Vec3_T &t )
{
  // transpose R
  double s;
  s = R(0,1); R(0,1) = R(1,0); R(1,0) = s;
  s = R(0,2); R(0,2) = R(2,0); R(2,0) = s;
  s = R(1,2); R(1,2) = R(2,1); R(2,1) = s;
	// adjust t
	t = -R*t;
}



#endif
