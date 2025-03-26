
#ifndef PAIR_HASH_H
#define PAIR_HASH_H

#include <functional>
#include <utility>

struct PairHash {
    template <typename T, typename U>
    std::size_t operator()(const std::pair<T, U>& p) const {
        auto h1 = std::hash<T>{}(p.first);
        auto h2 = std::hash<U>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

#endif // PAIR_HASH_H