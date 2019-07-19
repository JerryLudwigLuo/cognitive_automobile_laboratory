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

#include "basicTypes.h"
#include <fstream>
#include <vector>
#include <cassert>
#include <stdio.h>
using namespace std;

struct stringHelper
{
  
  static string num2str( int i )
  {
    char buf[100];
    sprintf(buf, "%010i", i);
    string fileBase( buf );
    return fileBase;
  }

  static string n2s( int i )
  {
    char buf[100];
    sprintf(buf, "%i", i);
    string fileBase( buf );
    return fileBase;
  }

  static string n2sf( float i )
  {
    char buf[100];
    sprintf(buf, "%f", i);
    string fileBase( buf );
    return fileBase;
  }

  static vector<string> splitString( string str )
  {
    vector<string> vecRV;

    istringstream iss(str, istringstream::in);
    string word;

    while ( iss >> word )
    {
      vecRV.push_back(word);
    }

    return vecRV;
  }

  static void plot( std::ostream& os, vector<double> vecValues, vector<double> vecAxis )
  {
    vector<string> vecAxisString;
    for (auto a:vecAxis)
    {
      char buf[100];
      sprintf( buf, "%f", a);
      vecAxisString.push_back(buf);
    }

    plot( os, vecValues, vecAxisString);
  }

  static void plot( std::ostream& os, vector<double> vecValues, vector<string> vecAxis )
  {

    assert( vecValues.size( ) == vecAxis.size() && "vectors of unequal size detected" );
    assert( vecValues.size() != 0 && "empty vector detected" );

    double minValue = vecValues[0];
    double maxValue = vecValues[0];
    double maxLength = vecAxis[0].size();
    for (size_t i = 0; i < vecValues.size(); ++i)
    {
      maxLength = std::max( (double)vecAxis[i].size(), maxLength );
      maxValue = std::max( maxValue, vecValues[i] );
      minValue = std::min( minValue, vecValues[i] );
    }

    double range = maxValue - minValue;
    double maxNumChar = 100 - maxLength;

    for (size_t i = 0; i < vecValues.size(); ++i)
    {
      os << "[";
      for (size_t j = 0; j < maxLength - vecAxis[i].size(); ++j)
      {
        os << " ";
      }
      os << vecAxis[i] << "]";
      for (size_t j = 0; j < (vecValues[i]-minValue)/range * maxNumChar; ++j)
      {
        os << "=";
      }
      os << "\n";
    }

  }

  static vector<double> getAxisFromValue( vector<double> vecValues, int n )
  {
    double minValue = vecValues[0];
    double maxValue = vecValues[0];
    for (size_t i = 0; i < vecValues.size(); ++i)
    {
      maxValue = std::max( maxValue, vecValues[i] );
      minValue = std::min( minValue, vecValues[i] );
    }

    double range = maxValue - minValue;
    double step = range/(n-1);

    vector<double> vecAxis;
    for (int i = 0; i < n; ++i)
    {
      vecAxis.push_back( i*step + minValue);
    }

    return vecAxis;
  }

  static void saveEigenMatrix( Eigen::MatrixXd mat, ofstream & file )
  {
    file << mat.rows() << "\n";
    file << mat.cols() << "\n";

    for (uint32_t i = 0; i < mat.rows(); ++i)
    {
      for (uint32_t j = 0; j < mat.cols(); ++j)
      {
        file << (float) mat(i,j) << " ";
      }
      file << "" << "\n";
    }

  }

  template <typename MATRIXTYPE>
 	static void loadEigenMatrix(  MATRIXTYPE  & mat, ifstream & file )
	{
    string line;
    getline( file, line );
    uint16_t numRows = atoi(line.c_str());
    getline( file, line );
    uint16_t numCols = atoi(line.c_str());

    mat.setZero( numRows, numCols );
    for (size_t r = 0; r < numRows; ++r)
    {
      getline( file, line );

      int pos1 = 0;
      int pos2 = line.find_first_of(" ");
      mat(r,0) = atof( line.substr(pos1, pos2).c_str() );

      for (size_t c = 1; c < numCols; ++c)
      {
        pos1 = pos2;
        pos2 = line.find_first_of(" ", pos2+1);
        mat(r,c) = atof( line.substr(pos1+1, pos2-pos1-1).c_str() );
      }
    }

	}

};
