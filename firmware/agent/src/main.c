#include <stm32f4xx.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include "bh_platform.h"
#include "bh_assert.h"
#include "bh_log.h"
#include "wasm_export.h"

// UART usata per agent/orchestrator
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

// LED usato da gpio_toggle
#define LED0_NODE DT_ALIAS(led0)

// dimensione massima riga comando (LOAD ..., START ..., ecc.)
#define LINE_BUF_SIZE 256

// msgq per linee di testo da UART (4 messaggi di 256 byte)
K_MSGQ_DEFINE(uart_msgq, LINE_BUF_SIZE, 4, 4);


#define MAX_CALL_ARGS  4 

// ID logico del modulo attualmente caricato (da LOAD module_id=...)
static char g_current_module_id[32];

typedef struct {
    char     func_name[64];
    uint32_t argc;
    uint32_t argv[MAX_CALL_ARGS];
} run_request_t;

// device UART
static const struct device *uart_dev;

// buffer RX usato in ISR
static char rx_buf[LINE_BUF_SIZE];
static int  rx_buf_pos;

// Stato RX per payload binario (AOT)
typedef enum {
    RX_STATE_LINE = 0,
    RX_STATE_BINARY
} rx_state_t;

static volatile rx_state_t g_rx_state = RX_STATE_LINE;

// puntatore buffer binario e dimensioni attese
static uint8_t *g_bin_buf      = NULL;
static size_t   g_bin_expected = 0;
static size_t   g_bin_received = 0;

// semaforo per notificare al thread che il payload è completo
K_SEM_DEFINE(bin_sem, 0, 1);

// Stato modulo caricato
static uint8_t           *g_wasm_buf      = NULL;  // Module binary
static uint32_t           g_wasm_size     = 0;
static wasm_module_t      g_wasm_module   = NULL;  // Parsed module
static wasm_module_inst_t g_wasm_inst     = NULL;  // Instance with memory
static bool               g_module_loaded = false; 

// Stato esecuzione RUNNER


static run_request_t g_run_req;
static volatile bool g_runner_busy     = false;  // Execution in progress
static volatile bool g_stop_requested  = false;  // Stop signal

// semaforo usato per svegliare il RUNNER quando c'è un nuovo job
K_SEM_DEFINE(run_sem, 0, 1);

// Config WAMR
#define CONFIG_GLOBAL_HEAP_BUF_SIZE WASM_GLOBAL_HEAP_SIZE
#define CONFIG_APP_STACK_SIZE       8192
#define CONFIG_APP_HEAP_SIZE        8192

// Parametri agent
#define NUM_ITER_DEFAULT   100

// Thread config
#define COMM_THREAD_STACK_SIZE    8192
#define COMM_THREAD_PRIORITY      5

#define RUNNER_THREAD_STACK_SIZE  8192
#define RUNNER_THREAD_PRIORITY    6

K_THREAD_STACK_DEFINE(comm_thread_stack,   COMM_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(runner_thread_stack, RUNNER_THREAD_STACK_SIZE);

static struct k_thread comm_thread;
static struct k_thread runner_thread;

// Prototipi
static void agent_write_str(const char *s);
static int  agent_read_line(char *buf, size_t max_len);

// CRC32 (compatibile zlib)
static uint32_t crc32_calc(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++) {
        uint32_t byte = data[i];
        crc ^= byte;
        for (int j = 0; j < 8; j++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

// UART ISR
static void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }

    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if (g_rx_state == RX_STATE_LINE) {
            // modalità line-based: accumula fino a \n / \r
            if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
                rx_buf[rx_buf_pos] = '\0';
                // se la coda è piena, il messaggio viene scartato
                k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
                rx_buf_pos = 0;
            } else if (rx_buf_pos < (int)(sizeof(rx_buf) - 1)) {
                rx_buf[rx_buf_pos++] = (char)c;
            }
        } else if (g_rx_state == RX_STATE_BINARY) {
            // modalità binaria: copia direttamente nel buffer AOT 
            if (g_bin_buf != NULL && g_bin_received < g_bin_expected) {
                g_bin_buf[g_bin_received++] = c;

                if (g_bin_received == g_bin_expected) {
                    // payload completo: ritorna a modalità line-based e sveglia il thread che sta aspettando
                    g_rx_state = RX_STATE_LINE;
                    k_sem_give(&bin_sem);
                }
            }
        }
    }
}

// GPIO per gpio_toggle
static const struct device *gpio_dev;
static uint32_t gpio_pin;

static int gpio_init_for_wasm(void)
{
    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

    if (!device_is_ready(led.port)) {
        return -1;
    }

    if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) < 0) {
        return -1;
    }

    gpio_dev = led.port;
    gpio_pin = led.pin;
    return 0;
}

#define SLEEP_TIME_MS 1000

// funzione nativa chiamata dal Wasm: env.gpio_toggle
static void
gpio_toggle_native(wasm_exec_env_t exec_env)
{
    ARG_UNUSED(exec_env);

    if (!gpio_dev) {
        return;
    }

    gpio_pin_toggle(gpio_dev, gpio_pin);
    k_msleep(SLEEP_TIME_MS);
}

// nativa env.should_stop: ritorna 1 se STOP richiesto
static int32_t
should_stop_native(wasm_exec_env_t exec_env)
{
    ARG_UNUSED(exec_env);
    return g_stop_requested ? 1 : 0;
}

// tabella delle funzioni native esportate al modulo "env"
static NativeSymbol native_symbols[] = {
    { "gpio_toggle",
      gpio_toggle_native,
      "()"              // nessun parametro, nessun valore di ritorno
    },
    { "should_stop",
      (void *)should_stop_native,
      "()i"             // nessun parametro, ritorna i32
    },
};

// Utility parsing key=value
static const char *find_param(const char *line, const char *key)
{
    size_t key_len = strlen(key);
    const char *p = line;

    while ((p = strstr(p, key)) != NULL) {
        if (p[key_len] == '=') {
            return p + key_len + 1;
        }
        p++;
    }
    return NULL;
}

// Copia un parametro fino al primo spazio / fine stringa / newline
static void copy_param_value(const char *start, char *dst, size_t dst_len)
{
    size_t i = 0;
    while (start[i] != '\0' &&
           start[i] != ' '  &&
           start[i] != '\r' &&
           start[i] != '\n') {
        if (i + 1 < dst_len) {
            dst[i] = start[i];
        }
        i++;
    }
    if (dst_len > 0) {
        dst[i < dst_len ? i : dst_len - 1] = '\0';
    }
}

// Gestione comando LOAD
/* Formato:
      LOAD module_id=<id> size=12345 crc32=1a2b3c4d
   Poi arrivano 'size' byte di payload
*/
static void handle_load_cmd(const char *line)
{
    char size_str[16];
    char crc_str[16];
    const char *p_size = find_param(line, "size");
    const char *p_crc  = find_param(line, "crc32");
    char out_buf[160];

    if (!p_size) {
        agent_write_str("LOAD_ERR code=BAD_PARAMS msg=\"missing size\"\n");
        return;
    }
    if (!p_crc) {
        agent_write_str("LOAD_ERR code=BAD_PARAMS msg=\"missing crc32\"\n");
        return;
    }

    copy_param_value(p_size, size_str, sizeof(size_str));
    copy_param_value(p_crc,  crc_str,  sizeof(crc_str));

    uint32_t size = (uint32_t)atoi(size_str);
    if (size == 0) {
        agent_write_str("LOAD_ERR code=BAD_PARAMS msg=\"size=0\"\n");
        return;
    }

    uint32_t crc_expected = (uint32_t)strtoul(crc_str, NULL, 16);

    // libera eventuale modulo precedente
    if (g_module_loaded) {
        wasm_runtime_deinstantiate(g_wasm_inst);
        wasm_runtime_unload(g_wasm_module);
        free(g_wasm_buf);
        g_wasm_buf = NULL;
        g_module_loaded = false;
    }

    g_wasm_buf = (uint8_t *)malloc(size);
    if (!g_wasm_buf) {
        agent_write_str("LOAD_ERR code=NO_MEM\n");
        return;
    }

    g_wasm_size = size;

    // configura la ricezione binaria nell'ISR
    unsigned int key = irq_lock();
    g_bin_buf      = g_wasm_buf;
    g_bin_expected = g_wasm_size;
    g_bin_received = 0;
    g_rx_state     = RX_STATE_BINARY;
    k_sem_reset(&bin_sem);
    irq_unlock(key);

    // avvisa l'orchestratore che siamo pronti a ricevere il payload
    snprintf(out_buf, sizeof(out_buf),
             "LOAD_READY size=%lu crc32=%s\n",
             (unsigned long)g_wasm_size, crc_str);
    agent_write_str(out_buf);

    // aspetta che l'ISR abbia ricevuto tutto il payload
    if (k_sem_take(&bin_sem, K_SECONDS(5)) != 0) {
        agent_write_str("LOAD_ERR code=TIMEOUT msg=\"binary payload not received\"\n");
        key = irq_lock();
        g_rx_state = RX_STATE_LINE;
        irq_unlock(key);
        free(g_wasm_buf);
        g_wasm_buf = NULL;
        return;
    }

    // verifica CRC
    uint32_t crc_calc = crc32_calc(g_wasm_buf, g_wasm_size);
    if (crc_calc != crc_expected) {
        snprintf(out_buf, sizeof(out_buf),
                 "LOAD_ERR code=BAD_CRC msg=\"expected=%08lx got=%08lx\"\n",
                 (unsigned long)crc_expected,
                 (unsigned long)crc_calc);
        agent_write_str(out_buf);
        free(g_wasm_buf);
        g_wasm_buf = NULL;
        return;
    }

    char error_buf[128];
    g_wasm_module = wasm_runtime_load(g_wasm_buf, g_wasm_size,
                                      error_buf, sizeof(error_buf));
    if (!g_wasm_module) {
        snprintf(out_buf, sizeof(out_buf),
                 "LOAD_ERR code=LOAD_FAIL msg=\"%s\"\n", error_buf);
        agent_write_str(out_buf);
        free(g_wasm_buf);
        g_wasm_buf = NULL;
        return;
    }

    g_wasm_inst = wasm_runtime_instantiate(g_wasm_module,
                                           CONFIG_APP_STACK_SIZE,
                                           CONFIG_APP_HEAP_SIZE,
                                           error_buf, sizeof(error_buf));
    if (!g_wasm_inst) {
        snprintf(out_buf, sizeof(out_buf),
                 "LOAD_ERR code=INSTANTIATE_FAIL msg=\"%s\"\n", error_buf);
        agent_write_str(out_buf);
        wasm_runtime_unload(g_wasm_module);
        g_wasm_module = NULL;
        free(g_wasm_buf);
        g_wasm_buf = NULL;
        return;
    }

    
    // estrae module_id=... e aggiorna g_current_module_id
    char module_id_buf[32];
    const char *p_mod = find_param(line, "module_id");
    if (p_mod) {
        copy_param_value(p_mod, module_id_buf, sizeof(module_id_buf));
        strncpy(g_current_module_id, module_id_buf,
                sizeof(g_current_module_id) - 1);
        g_current_module_id[sizeof(g_current_module_id) - 1] = '\0';
    } else {
        // se per qualche motivo manca, azzera l'ID corrente
        g_current_module_id[0] = '\0';
    }

    g_module_loaded = true;

    agent_write_str("LOAD_OK\n");
}

// Gestione comando START (prepara job per RUNNER)
/* Esempi:
      START module_id=toggle_forever func=toggle_forever
      START module_id=toggle_n func=toggle_n args="n=100"
      START module_id=math_ops func=add args="a=200,b=26"
*/
static void handle_start_cmd(const char *line)
{
    char func_name[64];
    char args_buf[64];
    char module_id_buf[32];   
    uint32_t argv[MAX_CALL_ARGS];
    uint32_t argc = 0;

    if (!g_module_loaded) {
        agent_write_str("RESULT status=NO_MODULE\n");
        return;
    }

    // legge module_id=... e verifica che combaci
    const char *p_mod = find_param(line, "module_id");
    if (!p_mod) {
        agent_write_str("RESULT status=BAD_PARAMS msg=\"missing module_id\"\n");
        return;
    }
    copy_param_value(p_mod, module_id_buf, sizeof(module_id_buf));
    if (strcmp(module_id_buf, g_current_module_id) != 0) {
        agent_write_str("RESULT status=NO_MODULE msg=\"module_id mismatch\"\n");
        return;
    }

    if (g_runner_busy) {
        agent_write_str("RESULT status=BUSY\n");
        return;
    }

    // func=<nome_funzione>
    const char *p_func = find_param(line, "func");
    if (!p_func) {
        agent_write_str("RESULT status=BAD_PARAMS msg=\"missing func\"\n");
        return;
    }
    copy_param_value(p_func, func_name, sizeof(func_name));

    // args="...": parse a=1,b=2,... in argv[]
    const char *p_args = find_param(line, "args");
    if (p_args && *p_args == '\"') {
        p_args++;
        const char *p_end = strchr(p_args, '\"');
        if (p_end) {
            size_t len = (size_t)(p_end - p_args);
            if (len >= sizeof(args_buf)) {
                len = sizeof(args_buf) - 1;
            }
            memcpy(args_buf, p_args, len);
            args_buf[len] = '\0';

            char *tok = strtok(args_buf, ",");
            while (tok && argc < MAX_CALL_ARGS) {
                char *eq = strchr(tok, '=');
                if (eq) {
                    int val = atoi(eq + 1);
                    argv[argc++] = (uint32_t)val;
                }
                tok = strtok(NULL, ",");
            }
        }
    }

    // verifica subito che la funzione esista
    wasm_function_inst_t fn =
        wasm_runtime_lookup_function(g_wasm_inst, func_name);
    if (!fn) {
        char out[96];
        snprintf(out, sizeof(out),
                 "RESULT status=NO_FUNC name=%s\n", func_name);
        agent_write_str(out);
        return;
    }

    // Prepara richiesta per il RUNNER
    memset(&g_run_req, 0, sizeof(g_run_req));
    strncpy(g_run_req.func_name, func_name,
            sizeof(g_run_req.func_name) - 1);
    g_run_req.argc = argc;
    for (uint32_t i = 0; i < argc && i < MAX_CALL_ARGS; i++) {
        g_run_req.argv[i] = argv[i];
    }

    g_stop_requested = false;
    g_runner_busy    = true;

    // sveglia il runner
    k_sem_give(&run_sem);

    // conferma immediata di START
    agent_write_str("START_OK\n");
}



// Gestione comando STOP
static void handle_stop_cmd(const char *line)
{
    char module_id_buf[32];

    if (!g_runner_busy) {
        agent_write_str("STOP_OK status=IDLE\n");
        return;
    }

    // verifica che lo STOP sia per il modulo attivo
    const char *p_mod = find_param(line, "module_id");
    if (!p_mod) {
        agent_write_str("STOP_OK status=NO_JOB\n");
        return;
    }
    copy_param_value(p_mod, module_id_buf, sizeof(module_id_buf));
    if (strcmp(module_id_buf, g_current_module_id) != 0) {
        agent_write_str("STOP_OK status=NO_JOB\n");
        return;
    }

    g_stop_requested = true;
    agent_write_str("STOP_OK status=PENDING\n");
}


// Gestione comando STATUS
static void handle_status_cmd(const char *line)
{
    (void)line;
    char out_buf[128];

    if (!g_module_loaded) {
        agent_write_str("STATUS_OK modules=\"none\" runner=IDLE\n");
        return;
    }

    snprintf(out_buf, sizeof(out_buf),
             "STATUS_OK modules=\"wasm_module(loaded)\" runner=%s\n",
             g_runner_busy ? "RUNNING" : "IDLE");
    agent_write_str(out_buf);
}

// Gestione generica linea comando (COMM thread)
static void handle_command_line(char *line)
{
    // rimuove newline finale
    size_t len = strlen(line);
    if (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'))
        line[len-1] = '\0';

    // comando = prima parola
    char *cmd  = strtok(line, " ");
    char *rest = strtok(NULL, ""); // tutto il resto della linea

    if (!cmd)
        return;

    if (strcmp(cmd, "LOAD") == 0) {
        handle_load_cmd(rest ? rest : "");
    } else if (strcmp(cmd, "START") == 0) {
        handle_start_cmd(rest ? rest : "");
    } else if (strcmp(cmd, "STOP") == 0) {
        handle_stop_cmd(rest ? rest : "");
    } else if (strcmp(cmd, "STATUS") == 0) {
        handle_status_cmd(rest ? rest : "");
    } else {
        agent_write_str("ERROR code=UNKNOWN_COMMAND\n");
    }
}

// Inizializzazione runtime WAMR
static bool wasm_runtime_init_all(void)
{
    RuntimeInitArgs init_args;
    memset(&init_args, 0, sizeof(init_args));

    init_args.mem_alloc_type   = Alloc_With_System_Allocator;
    init_args.native_module_name = "env";
    init_args.native_symbols     = native_symbols;
    init_args.n_native_symbols   =
        sizeof(native_symbols) / sizeof(native_symbols[0]);

    if (!wasm_runtime_full_init(&init_args)) {
        agent_write_str("ERROR code=WAMR_INIT_FAIL\n");
        return false;
    }

#if WASM_ENABLE_LOG != 0
    bh_log_set_verbose_level(0);
#endif
    return true;
}

// Thread COMM: UART + comandi
static void comm_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready!\n");
        return;
    }

    int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    if (ret < 0) {
        printk("Error setting UART callback: %d\n", ret);
        return;
    }
    uart_irq_rx_enable(uart_dev);

    if (!wasm_runtime_init_all()) {
        return;
    }

    if (gpio_init_for_wasm() != 0) {
        agent_write_str("ERROR code=GPIO_INIT_FAIL\n");
        return;
    }

    agent_write_str("HELLO device_id=stm32f4_01 rtos=Zephyr runtime=WAMR_AOT fw_version=1.0.0\n");

    char line_buf[LINE_BUF_SIZE];
    for (;;) {
        int n = agent_read_line(line_buf, sizeof(line_buf));
        if (n <= 0) {
            continue;
        }
        handle_command_line(line_buf);
    }
}

// Thread RUNNER: esegue le funzioni Wasm
static void runner_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    // Ogni thread nativo che usa le API WAMR deve fare questo
    if (!wasm_runtime_init_thread_env()) {
        agent_write_str("ERROR code=WAMR_THREAD_ENV_INIT_FAIL\n");
        return;
    }

    for (;;) {
    // aspetta un nuovo job
    k_sem_take(&run_sem, K_FOREVER);

    if (!g_module_loaded) {
        g_runner_busy    = false;
        g_stop_requested = false;
        continue;
    }

    // snapshot della richiesta
    run_request_t req;
    memcpy(&req, &g_run_req, sizeof(req));

    // lookup funzione
    wasm_function_inst_t fn =
        wasm_runtime_lookup_function(g_wasm_inst, req.func_name);
    if (!fn) {
        char out[96];
        snprintf(out, sizeof(out),
                 "RESULT status=NO_FUNC name=%s\n", req.func_name);
        agent_write_str(out);
        g_runner_busy    = false;
        g_stop_requested = false;
        continue;
    }

    // Per moduli AOT, usa direttamente wasm_func_get_result_count
    uint32_t result_count = wasm_func_get_result_count(fn, g_wasm_inst);

    wasm_exec_env_t exec_env =
        wasm_runtime_create_exec_env(g_wasm_inst, CONFIG_APP_STACK_SIZE);
    if (!exec_env) {
        char out[96];
        snprintf(out, sizeof(out),
                 "RESULT status=NO_EXEC_ENV func=%s\n", req.func_name);
        agent_write_str(out);
        g_runner_busy    = false;
        g_stop_requested = false;
        continue;
    }

    // prepara argv locale con gli argomenti in ingresso
    uint32 argc = req.argc;
    uint32 argv_local[MAX_CALL_ARGS];
    for (uint32 i = 0; i < argc && i < MAX_CALL_ARGS; i++) {
        argv_local[i] = req.argv[i];
    }

    bool ok = wasm_runtime_call_wasm(exec_env, fn, argc, argv_local);
    const char *exc = NULL;
    if (!ok) {
        exc = wasm_runtime_get_exception(g_wasm_inst);
    }

    // prepara RESULT
    char out[192];

    if (!ok) {
        snprintf(out, sizeof(out),
                 "RESULT status=EXCEPTION func=%s msg=\"%s\"\n",
                 req.func_name,
                 exc ? exc : "<none>");
    } else if (g_stop_requested) {
        snprintf(out, sizeof(out),
                 "RESULT status=STOPPED func=%s\n",
                 req.func_name);
    } else {
        // Se la funzione ha almeno un risultato, assumiamo i32 e lo leggiamo da argv_local[0]
        if (result_count > 0) {
            uint32_t ret_i32 = argv_local[0];
            snprintf(out, sizeof(out),
                     "RESULT status=OK func=%s ret_i32=%lu\n",
                     req.func_name,
                     (unsigned long)ret_i32);
        } else {
            // Nessun risultato (void): non stampiamo ret_i32
            snprintf(out, sizeof(out),
                     "RESULT status=OK func=%s\n",
                     req.func_name);
        }
    }


    agent_write_str(out);

    wasm_runtime_destroy_exec_env(exec_env);

    // reset stato runner
    g_runner_busy    = false;
    g_stop_requested = false;
}


    wasm_runtime_destroy_thread_env();
}



// Creazione thread
bool iwasm_init(void)
{
    k_tid_t tid_comm = k_thread_create(
        &comm_thread,
        comm_thread_stack,
        COMM_THREAD_STACK_SIZE,
        comm_thread_entry,
        NULL, NULL, NULL,
        COMM_THREAD_PRIORITY,
        0,
        K_NO_WAIT);

    k_tid_t tid_runner = k_thread_create(
        &runner_thread,
        runner_thread_stack,
        RUNNER_THREAD_STACK_SIZE,
        runner_thread_entry,
        NULL, NULL, NULL,
        RUNNER_THREAD_PRIORITY,
        0,
        K_NO_WAIT);

    return tid_comm && tid_runner;
}

// Entry Zephyr
void main(void)
{
    (void)iwasm_init();
    while (1) {
        k_sleep(K_FOREVER);
    }
}

// I/O UART

static void agent_write_str(const char *buf)
{
    if (!uart_dev || !buf) {
        return;
    }

    int msg_len = strlen(buf);
    for (int i = 0; i < msg_len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
}

// agent_read_line: blocca finché arriva una riga da msgq
static int agent_read_line(char *buf, size_t max_len)
{
    if (!buf || max_len == 0) {
        return -1;
    }

    char local_buf[LINE_BUF_SIZE];

    if (k_msgq_get(&uart_msgq, &local_buf, K_FOREVER) != 0) {
        return -1;
    }

    size_t len = strlen(local_buf);
    if (len >= max_len) {
        len = max_len - 1;
    }
    memcpy(buf, local_buf, len);
    buf[len] = '\0';

    return (int)len;
}
