#pragma once

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <list>



template<typename>
struct is_std_vector : std::false_type {};

template<typename T, typename A>
struct is_std_vector<std::vector<T,A>> : std::true_type {};

template<typename AR>
inline bool checkArrayData(const AR &ar) {
    for(int i = 0; i < ar.size(); i++) {
        if(std::isnan(ar[i]) || std::isinf(ar[i])) {
            std::cout << "At least array element (" << i << " ) is " << ar[i] << std::endl;
            return false;
        }
    }
    return true;
}


template<typename K, typename V, typename C, typename A>
inline std::vector<K, A> map_2_keyVector(const std::map<K, V, C, A> &mapIn) {
    int k = 0;
    std::vector<K, A> out(mapIn.size());
    for(const auto &elem:mapIn) {
        out[k] = elem.first;
        k++;
    }
    return out;
}

template<typename K, typename V, typename C, typename A>
inline std::vector<V> map_2_valueVector(const std::map<K, V, C, A> &mapIn) {
    int k = 0;
    std::vector<V> out(mapIn.size());
    for(const auto &elem:mapIn) {
        out[k] = elem.second;
        k++;
    }
    return out;
}

template<typename K, typename V, typename C, typename A>
inline std::vector<std::pair<K, V>, A> map_2_keyValueVector(const std::map<K, V, C, A> &mapIn) {
    int k = 0;
    std::vector<std::pair<K, V>, A> out(mapIn.size());
    for(const auto &elem:mapIn) {
        out[k] = std::make_pair(elem.first, elem.second);
        k++;
    }
    return out;
}

template<typename V, typename C, typename A>
inline std::vector<V, A> set_2_vec(const std::set<V, C, A> &setIn) {
    int k = 0;
    std::vector<V, A> out(setIn.size());
    for(const auto &elem:setIn) {
        out[k] = elem;
        k++;
    }
    return out;
}

template<typename K, typename V, typename C, typename A>
inline std::set<K, C> map_2_keySet(const std::map<K, V, C, A> &mapIn) {
    std::set<K, C> s;
    for(const auto &e:mapIn) {
        s.insert(e.first);
    }
    return s;
}

template<typename T, typename A>
inline std::set<T> vec_2_set(const std::vector<T, A> &v) {
    std::set<T> s;
    for(const auto &e:v) {
        s.insert(e);
    }
    return s;
}

template<typename K, typename V>
inline std::map<K, V> vecs_2_map(
        const std::vector<K> &kv,
        const std::vector<V> &vv) {
    std::map<K, V> m;
    if(kv.size() != vv.size()) throw std::runtime_error("size of key vector != size of value vector");
    for(size_t j = 0; j < kv.size(); j++) m.emplace(kv[j], vv[j]);
    return m;
}

template<typename T, typename A>
inline std::vector<T, A> list_2_vec(const std::list<T, A> &l) {
    std::vector<T, A> v;
    v.reserve(l.size());
    std::copy(l.begin(), l.end(), std::back_inserter(v));
    return v;
}

template<typename T, typename IdxType>
inline size_t keep_indexed_vector_elements(std::vector<T>& v, const std::vector<IdxType> & vIdx) {
    auto cpy = v;
    v.clear();
    v.reserve(vIdx.size());
    for(const auto &i:vIdx) v.emplace_back(cpy[i]);
    assert(v.size() == vIdx.size());
    return cpy.size();
}

template< typename ContainerT, typename PredicateT >
inline void erase_if( ContainerT& items, const PredicateT& predicate ) {
   for( auto it = items.begin(); it != items.end(); ) {
     if( predicate(*it) ) it = items.erase(it);
     else ++it;
   }
 }

template<typename K, typename V>
inline V* getValuePtrFromMap(const K& k, std::map<K, V> &map) {
    auto it = map.find(k);
    if(it == map.end()) return nullptr;
    return &(it->second);
}

template<typename K, typename V>
inline const V* getValuePtrFromMap(const K& k, const std::map<K, V> &map) {
    const auto it = map.find(k);
    if(it == map.end()) return nullptr;
    return &(it->second);
}

template<typename K, typename V>
inline V& getValueRefFromMap(const K& k, std::map<K, V> &map) {
    auto it = map.find(k);
    if(it == map.end()) throw std::runtime_error("No entry to this key");
    return (it->second);
}

template<typename K, typename V>
inline const V& getValueRefFromMap(const K& k, const std::map<K, V> &map) {
    const auto it = map.find(k);
    if(it == map.end()) throw std::runtime_error("No entry to this key");
    return (it->second);
}

template<typename K, typename V>
inline std::vector<const V*> getValuePtrVecFromMap(const std::map<K, V> &map) {
    std::vector<const V*> v;
    v.reserve(map.size());
    for(const auto& e:map) v.emplace_back( &(e.second) );
    return v;
}

template<typename K, typename V>
inline std::vector<V*> getValuePtrVecFromMap(std::map<K, V> &map) {
    std::vector<V*> v;
    v.reserve(map.size());
    for(auto& e:map) v.emplace_back( &(e.second) );
    return v;
}

template<typename K, typename V, typename A>
inline std::vector<typename std::map<K, V, std::less<K>, A>::const_iterator> getIteratorVecFromMap(const std::map<K, V, std::less<K>, A> &map) {
    std::vector<typename std::map<K, V, std::less<K>, A>::const_iterator> v;
    v.reserve(map.size());
    for(auto it = map.begin(); it != map.end(); it++) v.emplace_back(it);
    return v;
}

template<typename K, typename V, typename A>
inline std::vector<typename std::map<K, V, std::less<K>, A>::iterator> getIteratorVecFromMap(std::map<K, V, std::less<K>, A> &map) {
    std::vector<typename std::map<K, V, std::less<K>, A>::iterator> v;
    v.reserve(map.size());
    for(auto it = map.begin(); it != map.end(); it++) v.emplace_back(it);
    return v;
}

template<typename K, typename V, typename A>
inline std::vector<typename std::map<K, V, std::less<K>, A>::iterator> getIteratorVecFromMap(
        const std::vector<K>& sel,
        std::map<K, V, std::less<K>, A> &map) {
    std::vector<typename std::map<K, V, std::less<K>, A>::iterator> v;
    v.reserve(sel.size());
    for(const auto& e:sel) {
        auto it = map.find(e);
        if(it == map.end()) continue;
        v.emplace_back(it);
    }
    return v;
}

template<typename K, typename V, typename A>
inline std::vector<typename std::map<K, V, std::less<K>, A>::const_iterator> getConstIteratorVecFromMap(const std::map<K, V, std::less<K>, A> &map) {
    std::vector<typename std::map<K, V, std::less<K>, A>::const_iterator> v;
    v.reserve(map.size());
    for(auto it = map.begin(); it != map.end(); it++) v.emplace_back(it);
    return v;
}

template<typename K, typename V, typename A>
inline std::vector<typename std::map<K, V, std::less<K>, A>::const_iterator> getConstIteratorVecFromMap(std::map<K, V, std::less<K>, A> &map) {
    std::vector<typename std::map<K, V, std::less<K>, A>::const_iterator> v;
    v.reserve(map.size());
    for(auto it = map.begin(); it != map.end(); it++) v.emplace_back(it);
    return v;
}


template<typename C1, typename C2>
using SyncedIterators = std::pair<typename C1::const_iterator, typename C2::const_iterator>;

template<typename StampType>
inline std::pair<StampType, bool> getStampDist(const StampType& s1, const StampType& s2) {
    return (s1 < s2) ? std::make_pair(s2 - s1, true) : std::make_pair(s1 - s2, false);
}

template<typename T, typename T1, typename T2>
inline T absDifference(const T1 &t1, const T2 &t2) {
   if( static_cast<T>(t1) > static_cast<T>(t2) ) return static_cast<T>(t1) - static_cast<T>(t2);
   return static_cast<T>(t2) - static_cast<T>(t1);
}

template<typename T, typename T1, typename T2>
inline T signedDifference(const T1 &t1, const T2 &t2) {
    return ( static_cast<T>(t1) > static_cast<T>(t2) )
            ? static_cast<T>(t1) - static_cast<T>(t2)
            : static_cast<T>(-1) * static_cast<T>(t2) - static_cast<T>(t1);
}

template<typename C1, typename C2, typename GetStampC1,  typename GetStampC2, typename StampType>
inline std::vector<SyncedIterators<C1, C2> > findSynchronousIterators(
    const C1& c1,
    const C2& c2,
    GetStampC1 gsC1,
    GetStampC2 gsC2,
    const StampType& precision) {

    std::vector<SyncedIterators<C1, C2> > syncedIterators;
    syncedIterators.reserve(c1.size());
    auto it1 = c1.begin();
    auto it2 = c2.begin();
    while(it1 != c1.end()) {
        const auto s1 = gsC1(it1);
        while(it2 != c2.end()) {
            const auto s2 = gsC2(it2);
            const auto dist = getStampDist(s1, s2);
            if(dist.first > precision) { ///Out of sync
                if(dist.second) break; ///S2 > S1 -> Increase it1, Keep it2
                it2++; /// S1 > S2 -> Keep it1, Increase it2
                continue;
            }///We have a potential match; however check next stamp if it fits better
            auto it2Next = std::next(it2);
            if(it2Next == c2.end()) { ///At end
                syncedIterators.emplace_back(it1, it2);
                return syncedIterators;
            }
            ///Check if next distance is smaller
            const auto distNext = getStampDist(s1, gsC2(it2Next));
            if(distNext.first < dist.first) {
                it2++;
                continue;
            }
            ///Otherwise add match
            syncedIterators.emplace_back(it1, it2);
            it2++;
            break;
        }
        it1++;
    }
    return syncedIterators;
}


