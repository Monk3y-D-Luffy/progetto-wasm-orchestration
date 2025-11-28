#include <stdint.h>

#if defined(__wasm__) || defined(__wasm)
#  define WASM_EXPORT(name) __attribute__((export_name(name)))
#else
#  define WASM_EXPORT(name)
#endif

// Import "env.gpio_toggle" dal runtime host
__attribute__((import_module("env"), import_name("gpio_toggle")))
void gpio_toggle(void);

// Import "env.should_stop" che ritorna i32 (0 = continua, !=0 = stop)
__attribute__((import_module("env"), import_name("should_stop")))
int32_t should_stop(void);

// Loop che fa il toggle del led e che esce quando should_stop() != 0
WASM_EXPORT("toggle_forever")
void toggle_forever(void)
{
    for (;;) {
        gpio_toggle();

        // Controlla periodicamente il flag di stop
        if (should_stop() != 0) {
            break;
        }
    }
}
