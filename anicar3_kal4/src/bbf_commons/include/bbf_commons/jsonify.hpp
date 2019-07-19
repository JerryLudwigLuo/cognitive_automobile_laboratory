#pragma once

#include <Eigen/Core>
#include <jsoncpp/json/json.h>
#include <string>
#include <fstream>

namespace jsonify
{

template<typename Derived>
inline
Json::Value trajectory(const Eigen::DenseBase<Derived>& traj, double t0, double dt)
{
    Json::Value root;

    for( int i = 0; i < traj.rows(); ++i )
    {
        root["xs"].append(traj(i, 0));
        root["ys"].append(traj(i, 1));

        if( traj.cols() > 2 )
            root["psis"].append(traj(i, 2));

        root["ts"].append(t0 + i * dt);
    }

    return root;
}

template<typename Derived>
inline
Json::Value matrix(const Eigen::DenseBase<Derived>& m)
{
    Json::Value rows;
    for(int i = 0; i < m.rows(); ++i)
    {
        Json::Value cols;
        for(int j = 0; j < m.cols(); ++j)
        {
            cols.append(m(i,j));
        }
        rows.append(cols);
    }

    return rows;
}

}
