#include <stdint.h>

#if defined(__wasm__) || defined(__wasm)
#  define WASM_EXPORT(name) __attribute__((export_name(name)))
#else
#  define WASM_EXPORT(name)
#endif

// add(a, b) = a + b
WASM_EXPORT("add")
int32_t add(int32_t a, int32_t b)
{
    return a + b;
}

// sum_to_n(n) = 0 + 1 + ... + n (per n >= 0)
WASM_EXPORT("sum_to_n")
int32_t sum_to_n(int32_t n)
{
    if (n <= 0) {
        return 0;
    }

    int32_t s = 0;
    for (int32_t i = 0; i <= n; ++i) {
        s += i;
    }
    return s;
}
