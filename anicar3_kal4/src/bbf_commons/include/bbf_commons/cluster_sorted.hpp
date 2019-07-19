#pragma once

#include <vector>

template<typename ITERATOR, typename COMPARE_FUN>
std::vector< int32_t > cluster_sorted(ITERATOR first, const ITERATOR last, COMPARE_FUN& same_cluster)
{
    std::vector< int32_t > clusters(std::distance(first, last));

    auto& last_item = *first;

    clusters.front() = 0;

    ++first;
    for(int32_t i = 1; first != last; ++first, ++i)
    {
        const auto& current_item = *first;
        clusters[i] = clusters[i-1];
        if(!same_cluster(last_item, current_item))
        {
            clusters[i]++;
        }

        last_item = current_item;
    }
    return clusters;
}
