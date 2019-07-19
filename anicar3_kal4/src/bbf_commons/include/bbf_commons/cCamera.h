/*
 * Copyright 2012. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors: 
 *  Henning Lategahn (henning.latgahn@kit.edu)
 *  and others
 */

#pragma once
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include "basicTypes.h"

/*@class cCamera
 *
 * @par Purpose
 * 
 * 
 * @todo: write more comments
 *
 */

using namespace std;
class cCamera
{



public:

  /**
   * @brief base width
   */
  float baseWidth_;

  /**
   * @brief principial point (x-position)
   */
  float cx_;

  /**
   * @brief principial point (y-position)
   */
  float cy_;

  /**
   * @brief focal length of the cameara
   */
  float focalLength_;

  /**
   * @brief height of image
   */
  int height_;

  /**
   * @brief widht of image
   */
  int width_;

  /**
   * @brief standart deviation of stereo correspondence
   */
  float sigmaDisp_;


  public: /* public classes/enums/types etc... */

  public: /* public methods */

    /**
     * construct a cCamera object from scratch
     */
    cCamera()
    {
      cx_ = 0;
      cy_ = 0;
      focalLength_ = 0;
      baseWidth_ = 0;

      width_ = 0;
      height_ = 0;

      sigmaDisp_ = 1.0f;
    }

    /**
     * construct a cCamera object from text file
     * @param strPath the path to the camera.txt file
     */
    cCamera( std::string strPath )
    {

      std::string line;
      ifstream myfile ( strPath + "/camera.txt" );
      if (myfile.is_open())
      {

        for (int16_t i = 1; i <= 6; i++)
        {
          getline(myfile,line);
          switch (i)
          {
            case 2: 
              {
                cx_ = atof(line.c_str());
                break;
              }
            case 3: 
              {
                cy_ = atof(line.c_str());
                break;
              }
            case 1: 
              {
                focalLength_ = atof(line.c_str());
                break;
              }
            case 4: 
              {
                baseWidth_ = atof(line.c_str());
                break;
              }
            case 5: 
              {
                width_ = atof(line.c_str());
                break;
              }
            case 6: 
              {
                height_ = atof(line.c_str());
                break;
              }
          }
        }
        myfile.close();
      }
      else
      {
        cerr << "couldnt find camera file" << "\n";
        exit(0);
      }

      cout << "Read Camera from file:\n"
        << "focal_length: " << focalLength_ << "\n"
        << "center_x: " << cx_ << "\n"
        << "center_y: " << cy_ << "\n"
        << "base_length: " << baseWidth_ << "\n"
        << "width: " << width_ << "\n"
        << "height: " << height_ << "\n"
        << "\n";

      sigmaDisp_ = 1.0f;

    }

    /**
     * destruct a cCamera object
     */
    virtual ~cCamera()
    {

    }

    /**
     * @brief reconstructs a point from pixel position and disparity (in camera coordinate system)
     * @return none
     * @param u pixel position
     * @param v pixel position
     * @param d disparity
     * @param x x-coordinate of reconstructed point
     * @param y y-coordinate of reconstructed point
     * @param z z-coordinate of reconstructed point
     */
    virtual void reconstructCC( float u, float v, float d, float & x, float & y, float & z ) const
    {

      z = baseWidth_ * focalLength_ / d;
      x = (u - cx_) * z / focalLength_;
      y = (v - cy_) * z / focalLength_;

    }

    /**
     * @brief reconstructs a point from pixel position and disparity (in camera coordinate system)
     * @return homogeneous 3D point in sensor coordinates
     * @param u pixel position
     * @param v pixel position
     * @param d disparity
     */
    virtual cPoint4 reconstruct4dCC( float u, float v, float d ) const
    {
      cPoint4 ret;
      ret.block<3,1>(0,0) = reconstructCC(u,v,d);
      ret(3) = 1;
      return ret;
    }

    /**
     * @brief reconstructs a point from pixel position and disparity (in camera coordinate system)
     * @return 3D point in sensor coordinates
     * @param u pixel position
     * @param v pixel position
     * @param d disparity
     */
    virtual cPoint3 reconstructCC( float u, float v, float d ) const
    {
      float x,y,z;
      reconstructCC(u,v,d,x,y,z);
      cPoint3 vecReturn(x,y,z);
      return vecReturn;
    }


    /**
     * @brief reconstructs a point from pixel position and disparity 
     * @return none
     * @param u pixel position
     * @param v pixel position
     * @param d disparity
     * @param x x-coordinate of reconstructed point
     * @param y y-coordinate of reconstructed point
     * @param z z-coordinate of reconstructed point
     */
    virtual void reconstruct( float u, float v, float d, float & x, float & y, float & z ) const
    {

      x = baseWidth_ * focalLength_ / d;
      y = -(u - cx_) * x / focalLength_;
      z = -(v - cy_) * x / focalLength_;

    }

    /**
     * @brief reconstructs a point from pixel position and disparity 
     * @return homogeneous 3D point in sensor coordinates
     * @param u pixel position
     * @param v pixel position
     * @param d disparity
     */
    virtual cPoint4 reconstruct4d( float u, float v, float d ) const
    {
      cPoint4 ret;
      ret.block<3,1>(0,0) = reconstruct(u,v,d);
      ret(3) = 1;
      return ret;
    }

    /**
     * @brief reconstructs a point from pixel position and disparity 
     * @return 3D point in sensor coordinates
     * @param u pixel position
     * @param v pixel position
     * @param d disparity
     */
    virtual cPoint3 reconstruct( float u, float v, float d ) const
    {
      float x,y,z;
      reconstruct(u,v,d,x,y,z);
      cPoint3 vecReturn(x,y,z);
      return vecReturn;
    }

    /**
     * @brief reconstructs a point from pixel position and disparity 
     * @return 2D (no height) point in sensor coordinates 
     * @param u pixel position
     * @param v pixel position
     * @param d disparity
     */
    virtual cPoint2 reconstruct2d( float u, float v, float d ) const
    {
      float x,y,z;
      reconstruct(u,v,d,x,y,z);
      cPoint2 vecReturn(x,y);
      return vecReturn;
    }

    /**
     * @brief transforms 2d index into linear index
     * @return linear index
     * @param u pixel position
     * @param v pixel position
     */
    inline long toLinear( int u, int v ) const
    {
      return v * width_ + u;
    }

    /**
     * @brief computes the field of view of the camera from camera parameters
     * @return field of view of the camera [rad]
     * @param none
     */
    virtual float getHorizontalFieldOfView(  ) const
    {
      return 2*atan( width_ / (2*focalLength_));
    }

    /**
     * @brief project a point from homogeneous 3d coordinates to image coordinates
     * @return none
     * @param p4 homogeneous 3d point (in camera coordinates)
     * @param u pixel position
     * @param v pixel position
     * @param d disparity
     */
    virtual void project4d( cPoint4 p4, float & u, float & v, float & d ) const
    {
      u = -p4(1) * focalLength_ / p4(0) + cx_; 
      v = -p4(2) * focalLength_ / p4(0) + cy_;

      d = baseWidth_ * focalLength_ / p4(0) ;
    }

    /**
     * @brief project a point from 3d camera coordinates to image coordinates
     * @return none
     * @param p2 3d point (in camera coordinates)
     * @param u pixel position
     * @param v pixel position
     * @param d disparity
     */
    virtual void project3dCC( cPoint3 p3, float & u, float & v, float & d ) const
    {
      u = p3(0) * focalLength_ / p3(2) + cx_; 
      v = p3(1) * focalLength_ / p3(2) + cy_;

      d = baseWidth_ * focalLength_ / p3(2) ;
    }

    /**
     * @brief equal to operator
     * @return true if cameras are equal
     * @param other other camera
     */
    bool operator==( const cCamera & other ) const
    {
      return 
        (baseWidth_ == other.baseWidth_) &&
        (focalLength_ == other.focalLength_) &&
        (width_ == other.width_) &&
        (height_ == other.height_) &&
        (cx_ == other.cx_) &&
        (cy_ == other.cy_) 
        ;
    }

    /**
     * this function can be used to weight pixel measurements on towards the boundry of the image less. This 
     * is usuful for least square estimates to reduce the information (increase measurement variances). 
     * It is a little hack to account for imperfect radial distortion calibration 
     * @brief returns a weight (0...1) with 1 in the center and 1/dampingDegree on the most left/right column.
     * @return weight for given pixel position
     * @param u pixel position
     * @param dampingDegree degree by which the weight is decrease. must be greater than 1. (something like 2,3,4)
     */
    virtual double informationWeight( float u, float dampingDegree )
    {
      return 1.0/(fabs((u - cx_)/cx_) + 1.0/(dampingDegree-1.0)) / (dampingDegree-1.0);
    }
    



  protected: /* protected methods */

};

