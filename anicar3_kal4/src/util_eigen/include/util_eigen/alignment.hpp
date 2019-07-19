#pragma once

#include <deque>
#include <list>
#include <map>
#include <set>
#include <vector>

#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)

#if GCC_VERSION >= 70000
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#include <Eigen/Core>
#pragma GCC diagnostic pop
#else
#include <Eigen/Core>
#endif

#include <Eigen/StdVector>


template <typename V>
using EigenAlignedVec = std::vector<V, Eigen::aligned_allocator<V>>;

template <typename V>
using EigenAlignedList = std::list<V, Eigen::aligned_allocator<V>>;

template <typename K, typename V>
using EigenAlignedMap = std::map<K, V, std::less<K>, Eigen::aligned_allocator<std::pair<const K, V>>>;

template <typename V>
using EigenAlignedSet = std::set<V, std::less<V>, Eigen::aligned_allocator<V>>;

template <typename V>
using EigenAlignedDeque = std::deque<V, Eigen::aligned_allocator<V>>;

#if __has_include(<boost/multi_array.hpp>)
#include <boost/multi_array.hpp>

template <typename T, int NumDims>
using EigenAlignedBoostMultiArray = boost::multi_array<T, NumDims, Eigen::aligned_allocator<T>>;
#endif
