#ifndef MPC_LIB_HELPERS_H
#define MPC_LIB_HELPERS_H
// TODO: Move to a .cpp file?

namespace mpc_lib {
    // Finds f(x) where f = coeffs[0] + coeffs[1] * x + coeffs[2] * x^2 ...
    template<typename Tc, typename Tx>
    Tx polyeval(const Tx &x, const Tc &coeffs) {
        Tx ret = 0, pow = 1;
        for (decltype(coeffs.size()) i = 0; i < coeffs.size(); i++, pow *= x) {
            ret += coeffs[i] * pow;
        }
        return ret;
    }

    template<typename Tc, typename Tx>
    Tx deriveval(const Tx &x, const Tc &coeffs) {
        Tx ret = 0, pow = 1;
        for (decltype(coeffs.size()) i = 1; i < coeffs.size(); i++, pow *= x) {
            ret += i * coeffs[i] * pow;
        }
        return ret;
    }

    /*
     * This is a class similar to Python's range builtin
     * It works with c++ for each loop
     *
     * for (auto i : Range(3, 6)) { // Do something }
     * This will loop with i = 3, 4, 5
     *
     * This functions as an iterator too.
 */
    class Range {
        size_t cur;
        const size_t last;
    public:
        Range(size_t start, const size_t end) : cur(start), last(end) {}

        // Iterable functions
        [[nodiscard]] const Range &begin() const { return *this; }

        [[nodiscard]] const Range &end() const { return *this; }

        // Iterator functions
        bool operator!=(const Range &r) const { return cur < r.last; }

        void operator++() { ++cur; }

        size_t operator*() const { return cur; }

        size_t operator[](size_t index) const {
            assert(index < length());
            return cur + index;
        }

        [[nodiscard]] size_t length() const { return last - cur; }

        friend Range operator+(const Range &a, const Range &b) {
            assert(a.last == b.cur);
            return {a.cur, b.last};
        }
    };
}


// IGNORE:
/*
// ONLY USE WITH Range
template<typename T1, typename T2>
class zip2 {
    std::pair<T1, T2> containers;
public:
    zip2(T1 t1, T2 t2) : containers(std::make_pair(t1, t2)) {
        */
/*
         * this.val_end = std::end(containers[0]));
         * this.containers = std::make_tuple(std::begin(containers[0]), std::begin(containers[1]));
         *//*

    }

    auto begin() { return *this; }

    auto end() { return *this; }

    bool operator!=(const zip2 &r) const {
        return containers.first != r.containers.first;
        */
/* return this.containers[0] != this.val_end; *//*

    }

    void operator++() {
        ++containers.first;
        ++containers.second;
    }

    auto operator*() const { return std::make_pair(*containers.first, *containers.second); }
};

template<typename T1, typename T2, typename T3>
class zip3 {
    std::tuple<T1, T2, T3> containers;
public:
    zip3(T1 t1, T2 t2, T3 t3) : containers{t1, t2, t3} {}

    auto begin() { return *this; }

    auto end() { return *this; }

    bool operator!=(const zip3 &r) const { return std::get<0>(containers) != std::get<0>(r.containers); }

    void operator++() {
        ++std::get<0>(containers);
        ++std::get<1>(containers);
        ++std::get<2>(containers);
    }

    auto operator*() const {
        return std::make_tuple(*std::get<0>(containers), *std::get<1>(containers), *std::get<2>(containers));
    }
};

*/


// DONT BE STUID

//#include <utility>
//
//template<typename ...T, size_t ...I>
//auto get_deref_helper(std::tuple<T...> &ts, std::index_sequence<I...>) {
//    return std::make_tuple(*(std::get<I>(ts)) ...);
//}
//
//template<typename ...T>
//std::tuple<> get_deref(std::tuple<T...> &ts) {
//    return get_deref_helper(ts, std::make_index_sequence<sizeof...(T)>());
//}
//
//template<typename ...T, size_t ...I>
//void get_inc_helper(std::tuple<T...> &ts, std::index_sequence<I...>) {
//    std::tie((*(std::get<I>(ts)), 1) ...);
//}
//
//template<typename ...T>
//void get_inc(std::tuple<T...> &ts) {
//    get_inc_helper(ts, std::make_index_sequence<sizeof...(T)>());
//}
//
//
//template<typename... Ts>
//class Zip {
//    // : we shouldnt require Ts to have a default constructor
//    std::tuple<decltype(std::begin(Ts()))...> tup;
////    std::tuple<std::invoke_result<std::begin,Ts>::type...> tup;
//
//    typename std::tuple_element<0, std::tuple<Ts...> >::type t_end;
//    decltype(std::end(t_end)) end_pt;
////    std::invoke_result<std::end, std::tuple_element<0, std::tuple<Ts...> >::type> end_pt;
//
//public:
//    Zip(Ts... containers) : end_pt(std::end(std::get<0>(std::make_tuple(containers...)))),
//                            tup{std::begin(containers)...} {}
//
//    auto begin() { return *this; }
//
//    auto end() { return *this; }
//
//    bool operator!=(const Zip &r) const { return std::get<0>(tup) != r.end_pt; }
//
//    bool operator++() { get_inc(tup); }
//
//    auto operator*() const { return get_deref(tup);/*return std::make_tuple(*std::get<Indices>(t));*/ }
//};


#endif //MPC_LIB_HELPERS_H
