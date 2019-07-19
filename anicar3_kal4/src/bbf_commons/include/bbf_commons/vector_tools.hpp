#pragma once

#include <vector>

namespace vector_tools
{

inline
std::vector< double > linspace(double low, double hi, int32_t N)
{
    std::vector< double > result(N);

    double ds = (hi - low) / (N-1);

    for( int i = 0; i < N; ++i)
    {
        result[i] = low + i * ds;
    }

    return result;
}

}
