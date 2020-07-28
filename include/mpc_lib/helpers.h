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

    // Finds f'(x) where f = coeffs[0] + coeffs[1] * x + coeffs[2] * x^2 ...
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
     * It works with c++ for each loop (range based loop)
     *
     * for (auto i : Range(3, 6)) { // Do something }
     * This will loop with i = 3, 4, and 5.
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

    // auto begin(Range &r) { return r.begin(); }
    // auto end(Range &r) { return r.end(); }

}

#endif //MPC_LIB_HELPERS_H
