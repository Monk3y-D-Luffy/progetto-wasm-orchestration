#include <stdint.h>

#if defined(__wasm__) || defined(__wasm)
#  define WASM_EXPORT(name) __attribute__((export_name(name)))
#else
#  define WASM_EXPORT(name)
#endif

// Dichiarazione dell'import "env.gpio_toggle" visto dal modulo Wasm. WAMR collegherà questa funzione alla nativa gpio_toggle_native dell'agent.
__attribute__((import_module("env"), import_name("gpio_toggle")))
void gpio_toggle(void);

WASM_EXPORT("toggle_n")
void toggle_n(int32_t n)
{
    if (n <= 0) {
        return;
    }

    for (int32_t i = 0; i < n; ++i) {
        gpio_toggle();
        // Il delay è già dentro la nativa gpio_toggle_native (k_msleep), quindi qui non serve aggiungerlo
    }
}
