/*
 * Copyright 2014. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors: 
 *  Henning Lategahn (henning.lategahn@kit.edu)
 *  and others
 */

#pragma once

#include <tuple>

#include "basicTypes.h"

  /*@class cAffinity
   *
   * This class will estimate a triangle to triangle mapping (affinity).
   *
   * It is constructed with three point to point correspondences. Thereafter
   * it allows to loop over either the input or output triangle (all integer
   * positions within the triangle) and map back and forth between triangle
   * coordinates.
   *
   */
  class cAffinity
  {

    public: /* public classes/enums/types etc... */

      struct tPoint
        : public pair<float,float>
      {

        tPoint()
        {
          this->first = 0;
          this->second = 0;
        }

        tPoint(float u, float v)
        {
          this->first = u;
          this->second = v;
        }
        
        float u() const
        {
          return this->first;
        }

        float v() const
        {
          return this->second;
        }

      };

      typedef vector<tPoint> tPoints;
      
      struct tTriangle
        : public tuple<tPoint, tPoint, tPoint> 
      {

        tTriangle( tPoint pt1, tPoint pt2, tPoint pt3 )
        {
          get<0>(*this) = pt1;
          get<1>(*this) = pt2;
          get<2>(*this) = pt3;
        }

        tPoint operator[](int i)
        {

          if (i == 0)
          {
            return get<0>(*this);
          }

          if (i == 1)
          {
            return get<1>(*this);
          }

          if (i == 2)
          {
            return get<2>(*this);
          }

          assert( false && "trying to acces non-valid point index of the triangle (e.g. outside the range {0,1,2}" );

          return tPoint {-1, -1};

        }
      };

      typedef Eigen::Matrix<double, 3, 3> tMatrix3x3;
      typedef Eigen::Matrix<double, 6, 6> tMatrix6x6;
      typedef Eigen::Matrix<double, 6, 1> tVector6;
     
    public: /* public methods */

      /**
       * construct a cAffinity object from scratch
       * three point-to-point correspondences
       */
      cAffinity( tTriangle inTriangle, tTriangle outTriangle );

      /**, "vehicle"
       * destruct a cAffinity object
       */
      virtual ~cAffinity();

      /**
       * @brief computes all (integer) pixel positions of the output triangle 
       * @return set of points 
       * @param none
       */
      virtual tPoints outTriangle(  );

      /**
       * @brief computes all (integer) pixel positions of the input triangle 
       * @return set of points 
       * @param none
       */
      virtual tPoints inTriangle(  );

      /**
       * @brief computes all (integer) pixel positions of the specified triangle
       * @return set of points 
       * @param triangle triangle in question 
       * @param epsilon least distance to the leg of the triangle in [px] (if set to low the boundries can be missed)
       */
      virtual tPoints triangle( tTriangle triangle, float epsilon = .1 );

      /**
       * @brief applies the inverse map to the given point (that is from output triangle to input triangle)
       * @return none
       * @param ptIn the point to be transformed
       */
      virtual tPoint inverseMap( tPoint ptIn );
      
      /**
       * @brief applies the affinity map to the given point (that is from input triangle to output triangle)
       * @return none
       * @param ptIn the point to be transformed
       */
      virtual tPoint map( tPoint ptIn );
      
      /**
       * signum function
       */
      template <typename T> int sgn(T val) 
      {
        return (T(0) < val) - (val < T(0));
      }



    public: /* attributes */

      /**
       * @brief input triangle
       */
      tTriangle inTriangle_;
      
      /**
       * @brief output triangle
       */
      tTriangle outTriangle_;

      /**
       * @brief homogenous matrix which is the affinity from input to output triangle
       */
      tMatrix3x3 transform_;
      tMatrix3x3 transform_inv_;
      
      
      
      
      
  };

  cAffinity::cAffinity( tTriangle inTriangle, tTriangle outTriangle ) 
    : inTriangle_(inTriangle), outTriangle_(outTriangle)
  {

    // estimate affinity

    tMatrix6x6 A;
    A <<
      inTriangle[0].u(), inTriangle[0].v(), 1, 0, 0, 0,
      0, 0, 0, inTriangle[0].u(), inTriangle[0].v(), 1,
      inTriangle[1].u(), inTriangle[1].v(), 1, 0, 0, 0,
      0, 0, 0, inTriangle[1].u(), inTriangle[1].v(), 1,
      inTriangle[2].u(), inTriangle[2].v(), 1, 0, 0, 0,
      0, 0, 0, inTriangle[2].u(), inTriangle[2].v(), 1;


    tVector6 b;
    b <<
      outTriangle[0].u(),
      outTriangle[0].v(),
      outTriangle[1].u(),
      outTriangle[1].v(),
      outTriangle[2].u(),
      outTriangle[2].v();

    // solve
    tVector6 a;
    // a = A.llt().solve(b);
    a = A.inverse() * b;

    // create transform
    transform_ <<
      a(0), a(1), a(2),
      a(3), a(4), a(5),
      0,0,1;
    transform_inv_ = transform_.inverse();

  }

  cAffinity::~cAffinity()
  {

  }
      
  cAffinity::tPoints cAffinity::outTriangle(  )
  {
    return triangle(outTriangle_);
  }

  cAffinity::tPoints cAffinity::inTriangle(  )
  {
    return triangle(inTriangle_);
  }

  inline bool sign (const cAffinity::tPoint& p1,const cAffinity::tPoint& p2, const cAffinity::tPoint& p3)
  {
      return ((p1.u() - p3.u()) * (p2.v() - p3.v()) - (p2.u() - p3.u()) * (p1.v() - p3.v())) < 0.0f;
  }

  cAffinity::tPoints cAffinity::triangle( tTriangle triangle, float epsilon /* = .1 */ )
  {

    tPoints pointsOfTriangle;

    // get squared bounding box first
    float uMin = std::floor(min( triangle[0].u(),
        min(triangle[1].u(), triangle[2].u()) ));
    float uMax = std::ceil(max( triangle[0].u(),
        max(triangle[1].u(), triangle[2].u()) ));

    float vMin = std::floor(min( triangle[0].v(),
        min(triangle[1].v(), triangle[2].v()) ));
    float vMax = std::ceil(max( triangle[0].v(),
        max(triangle[1].v(), triangle[2].v()) ));

    // loop over bounding box and check if inside
    for (float v = vMin; v < vMax; v+=1)
    {
      bool found = false;
      for (float u = uMin; u < uMax; u+=1)
      {
        cAffinity::tPoint p{u,v};
        const auto b1 = sign(p, triangle[0], triangle[1]);
        const auto b2 = sign(p, triangle[1], triangle[2]);
        const auto b3 = sign(p, triangle[2], triangle[0]);
        if ((b1 == b2) && (b2 == b3))
        {
          pointsOfTriangle.push_back( tPoint {u,v} );
          found = true;
        }
        else if( found )
         break;
      }
    }

    return pointsOfTriangle;
  }
      
  cAffinity::tPoint cAffinity::inverseMap( tPoint ptIn )
  {
    // homogenous coordinates
    cVector3 vector;
    vector << ptIn.first,ptIn.second,1;
    vector = transform_inv_ * vector;
    return {(float)vector(0), (float)vector(1)};
  }

  cAffinity::tPoint cAffinity::map( tPoint ptIn )
  {
    // homogenous coordinates
    cVector3 vector;
    vector << ptIn.first,ptIn.second,1;
    vector = transform_ * vector;
    return {(float)vector(0), (float)vector(1)};
  }

