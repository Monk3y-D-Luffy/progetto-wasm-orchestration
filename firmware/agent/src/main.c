#include <stm32f4xx.h>    //Header HAL/LL STM32F4: definizioni di registri, interrupt, ecc. specifiche della MCU

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <zephyr/kernel.h>

// API per oggetti struct device e driver UART/GPIO 
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

// Header di WAMR (WebAssembly Micro Runtime): porting layer, assert/log, funzioni per caricare/eseguire moduli
#include "bh_platform.h"
#include "bh_assert.h"
#include "bh_log.h"
#include "wasm_export.h"


// UART usata per agent/orchestrator
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)   // DT_CHOSEN(zephyr_shell_uart) prende dalla Devicetree il nodo marcato come zephyr,shell-uart nella sezione chosen

// LED usato da gpio_toggle
#define LED0_NODE DT_ALIAS(led0)    // DT_ALIAS(led0) prende il nodo con alias led0 nella Devicetree della board

// dimensione massima riga comando (LOAD ..., START ..., ecc.)
#define LINE_BUF_SIZE 256

/*  
    K_MSGQ_DEFINE(name, msg_size, max_msgs, align) definisce staticamente una message queue di Zephyr:
        uart_msgq: coda usata per passare le linee di testo dal ISR UART al COMM thread
        msg_size = LINE_BUF_SIZE → ogni messaggio è un buffer da 256 byte
        max_msgs = 4 → la coda può contenere fino a 4 linee pendenti
        align = 4 → allineamento a 4 byte
*/
K_MSGQ_DEFINE(uart_msgq, LINE_BUF_SIZE, 4, 4); 


#define MAX_CALL_ARGS  4 

// ID logico del modulo attualmente caricato (da LOAD module_id=...)
static char g_current_module_id[32];

//  definisce un typedef struct con le informazioni necessarie per chiedere al thread RUNNER di chiamare una funzione Wasm con argomenti interi
typedef struct {
    char     func_name[64];       // Buffer per il nome della funzione esportata nel modulo Wasm da eseguire
    uint32_t argc;                // Numero di argomenti effettivi passati alla funzione
    uint32_t argv[MAX_CALL_ARGS]; // Array che contiene i valori degli argomenti (interi a 32 bit)
} run_request_t;

// device UART; struct device è un tipo definito da Zephyr
static const struct device *uart_dev;

// buffer RX usato in ISR
static char rx_buf[LINE_BUF_SIZE];
static int  rx_buf_pos;

// Stato RX per comandi testuali e payload binario (LOAD)
typedef enum {
    RX_STATE_LINE = 0,
    RX_STATE_BINARY
} rx_state_t;

static volatile rx_state_t g_rx_state = RX_STATE_LINE; // variabile globale che contiene lo stato corrente della RX UART

// puntatore buffer binario e dimensioni attese
static uint8_t *g_bin_buf      = NULL;
static size_t   g_bin_expected = 0;
static size_t   g_bin_received = 0;

// semaforo per notificare al thread che il payload è completo; valore iniziale 0 e valore massimo 1
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
#define CONFIG_APP_STACK_SIZE       8192
#define CONFIG_APP_HEAP_SIZE        8192

// Thread config
#define COMM_THREAD_STACK_SIZE    8192
#define COMM_THREAD_PRIORITY      5

#define RUNNER_THREAD_STACK_SIZE  8192
#define RUNNER_THREAD_PRIORITY    6

// K_THREAD_STACK_DEFINE(name, size) alloca staticamente un blocco di RAM allineato per usarlo come stack di un thread Zephyr
K_THREAD_STACK_DEFINE(comm_thread_stack,   COMM_THREAD_STACK_SIZE);  // stack associato al COMM thread
K_THREAD_STACK_DEFINE(runner_thread_stack, RUNNER_THREAD_STACK_SIZE);  // stack associato al RUNNER thread

// struct k_thread è la struttura kernel che contiene lo stato del thread
static struct k_thread comm_thread;
static struct k_thread runner_thread;

// Prototipi
static void agent_write_str(const char *s);
static int  agent_read_line(char *buf, size_t max_len);



// CRC32 (compatibile zlib)
static uint32_t crc32_calc(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;   // SEED standard CRC32-zlib (tutti i bit a 1)

    // Per ogni byte del payload
    for (size_t i = 0; i < len; i++) {
        uint32_t byte = data[i];    // byte corrente
        crc ^= byte;  //  XOR iniziale: mescola byte con CRC corrente

        // Processa 8 bit del byte (LSB-first, algoritmo "reversed")
        for (int j = 0; j < 8; j++) {    // 8 bit del byte
            uint32_t lsb = crc & 1u;          // LSB corrente (0 o 1)
            uint32_t mask = -(int32_t)lsb;    // Trick: LSB=1 → mask=0xFFFFFFFF, LSB=0 → mask=0
            crc = (crc >> 1) ^ (0xEDB88320u & mask);  // Shift destro (simula divisione polinomio) e XOR con polinomio CRC32 SE SOLO LSB era 1
        }
    }
    return ~crc; // NOT finale: inverte tutti i 32 bit (standard zlib)
}

// UART ISR
static void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    if (!uart_irq_update(uart_dev)) {  // aggiorna gli status dei flag di interrupt
        return;
    }

    if (!uart_irq_rx_ready(uart_dev)) {  // controlla se ci sono dati nella FIFO RX
        return;
    }

    while (uart_fifo_read(uart_dev, &c, 1) == 1) {    // Legge byte dalla FIFO uno alla volta finché uart_fifo_read restituisce 1
        if (g_rx_state == RX_STATE_LINE) {
            // modalità line-based: accumula fino a \n / \r
            if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {  // quando vede \n o \r e c’è almeno un carattere nel buffer
                rx_buf[rx_buf_pos] = '\0';          // chiude la stringa con '\0'
                k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT); // mette la linea completa nella message queue uart_msgq. Se la coda è piena, il messaggio viene scartato
                rx_buf_pos = 0;
            } else if (rx_buf_pos < (int)(sizeof(rx_buf) - 1)) {   // Se non è fine linea ed il buffer non è pieno
                rx_buf[rx_buf_pos++] = (char)c;         // aggiunge il carattere al buffer e incrementa rx_buf_pos
            }
        } else if (g_rx_state == RX_STATE_BINARY) {
            // modalità binaria: copia direttamente nel buffer WASM/AOT 
            if (g_bin_buf != NULL && g_bin_received < g_bin_expected) {
                g_bin_buf[g_bin_received++] = c;   // ogni byte ricevuto viene copiato direttamente nel buffer binario g_bin_buf e si incrementa g_bin_received

                if (g_bin_received == g_bin_expected) {
                    // payload completo: ritorna a modalità line-based e sveglia il thread che sta aspettando
                    g_rx_state = RX_STATE_LINE;
                    k_sem_give(&bin_sem);  //  incrementa il semaforo bin_sem da 0 a 1 e sblocca immediatamente il thread che stava aspettando su k_sem_take
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
     /*  legge dalla Devicetree il nodo LED0_NODE e costruisce una struct gpio_dt_spec con:
                led.port: const struct device * del controller GPIO;
                led.pin: numero di pin;
                led.dt_flags: flag di configurazione definiti in Devicetree (active low, pull‑up, ecc.)
     */
    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

    if (!device_is_ready(led.port)) { // Controlla che il driver del GPIO relativo a led.port sia stato inizializzato correttamente dal kernel
        return -1;
    }

    if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) < 0) { // configura il pin descritto da led come output e inizialmente “inactive”
        return -1;
    }

    gpio_dev = led.port; // device controller del LED 
    gpio_pin = led.pin;  // numero di pin del LED
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

    while ((p = strstr(p, key)) != NULL) {  // strstr(p, key) cerca la prima occorrenza di key a partire da p
        if (p[key_len] == '=') {   // controlla il carattere dopo key è uguale a '='
            return p + key_len + 1;  // restituisce tutto dopo il =
        }
        p++;
    }
    return NULL;
}

// estrae il valore puro dopo key= fino al primo delimitatore (spazio, \r, \n), copiandolo in modo sicuro nel buffer destinazione
static void copy_param_value(const char *start, char *dst, size_t dst_len)
{
    size_t i = 0;
    while (start[i] != '\0' &&
           start[i] != ' '  &&
           start[i] != '\r' &&
           start[i] != '\n') {
        if (i + 1 < dst_len) {
            // Copia sicura: solo se resta spazio per dst[i] + '\0' finale
            dst[i] = start[i];
        }
        i++;
    }
    if (dst_len > 0) {
        // Terminazione sicura: se i < dst_len: mette '\0' alla posizione i; se i >= dst_len: tronca a dst_len-1 (overflow protetto)
        dst[i < dst_len ? i : dst_len - 1] = '\0';
    }
}


// Gestione comando LOAD: parsa parametri, alloca buffer, riceve payload binario, verifica CRC, carica in WAMR
/* Formato:
      LOAD module_id=<id> size=12345 crc32=1a2b3c4d
   Poi arrivano 'size' byte di payload
*/
static void handle_load_cmd(const char *line)
{
    // Buffer temporanei per estrarre size e crc32 dalla riga comando
    char size_str[16];
    char crc_str[16];
    
    // find_param cerca "key=" nella stringa line e ritorna puntatore al valore
    const char *p_size = find_param(line, "size");      // es: "12345"
    const char *p_crc  = find_param(line, "crc32");     // es: "ABCD1234"
    char out_buf[160];                                  // buffer per messaggi di risposta

    // Validazione parametri obbligatori
    if (!p_size) {
        agent_write_str("LOAD_ERR code=BAD_PARAMS msg=\"missing size\"\n");
        return;
    }
    if (!p_crc) {
        agent_write_str("LOAD_ERR code=BAD_PARAMS msg=\"missing crc32\"\n");
        return;
    }

    // Estrae i valori puri (senza =) nei buffer locali
    copy_param_value(p_size, size_str, sizeof(size_str));  // size_str = "12345"
    copy_param_value(p_crc,  crc_str,  sizeof(crc_str));   // crc_str  = "ABCD1234"

    // Converte size in intero, valida > 0
    uint32_t size = (uint32_t)atoi(size_str);
    if (size == 0) {
        agent_write_str("LOAD_ERR code=BAD_PARAMS msg=\"size=0\"\n");
        return;
    }

    // Converte CRC esadecimale in intero
    uint32_t crc_expected = (uint32_t)strtoul(crc_str, NULL, 16);  // 0xABCD1234

    // Cleanup: scarica eventuale modulo Wasm/AOT già caricato
    if (g_module_loaded) {
        wasm_runtime_deinstantiate(g_wasm_inst);  // distrugge istanza (memoria, stack)
        wasm_runtime_unload(g_wasm_module);       // libera modulo parsato
        free(g_wasm_buf);                         // libera buffer binario
        g_wasm_buf = NULL;
        g_module_loaded = false;
    }

    // Alloca buffer RAM per il nuovo modulo
    g_wasm_buf = (uint8_t *)malloc(size);
    if (!g_wasm_buf) {
        agent_write_str("LOAD_ERR code=NO_MEM\n");
        return;
    }
    g_wasm_size = size;  // salva dimensione per uso successivo

    // SEZIONE CRITICA: configura ISR per ricevere payload binario
    unsigned int key = irq_lock();                    // disabilita interrupt
    g_bin_buf      = g_wasm_buf;                      // ISR scriverà qui
    g_bin_expected = g_wasm_size;                     // quanti byte attendere
    g_bin_received = 0;                               // contatore byte ricevuti
    g_rx_state     = RX_STATE_BINARY;                 // ISR: passa in modalità binaria
    k_sem_reset(&bin_sem);                            // reset semaforo (torna a 0)
    irq_unlock(key);                                  // riabilita interrupt

    // Avvisa gateway: "pronto, manda il payload binario"
    snprintf(out_buf, sizeof(out_buf),
             "LOAD_READY size=%lu crc32=%s\n",
             (unsigned long)g_wasm_size, crc_str);
    agent_write_str(out_buf);

    // BLOCCA: aspetta che ISR riceva tutto il payload (max 5s)
    if (k_sem_take(&bin_sem, K_SECONDS(5)) != 0) {
        // Timeout: ISR non ha ricevuto tutto
        agent_write_str("LOAD_ERR code=TIMEOUT msg=\"binary payload not received\"\n");
        key = irq_lock();
        g_rx_state = RX_STATE_LINE;  // torna a comandi testuali
        irq_unlock(key);
        free(g_wasm_buf);
        g_wasm_buf = NULL;
        return;
    }

    // Verifica integrità: calcola CRC32 del buffer ricevuto
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

    // Carica modulo in WAMR: parsing del binario Wasm/AOT
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

    // Crea istanza eseguibile: alloca memoria/stack/heap per il modulo
    g_wasm_inst = wasm_runtime_instantiate(g_wasm_module,
                                           CONFIG_APP_STACK_SIZE,
                                           CONFIG_APP_HEAP_SIZE,
                                           error_buf, sizeof(error_buf));
    if (!g_wasm_inst) {
        snprintf(out_buf, sizeof(out_buf),
                 "LOAD_ERR code=INSTANTIATE_FAIL msg=\"%s\"\n", error_buf);
        agent_write_str(out_buf);
        wasm_runtime_unload(g_wasm_module);  // cleanup modulo parsato
        g_wasm_module = NULL;
        free(g_wasm_buf);
        g_wasm_buf = NULL;
        return;
    }

    // Salva module_id dal comando LOAD per uso successivo (STATUS, ecc.)
    char module_id_buf[32];
    const char *p_mod = find_param(line, "module_id");
    if (p_mod) {
        copy_param_value(p_mod, module_id_buf, sizeof(module_id_buf));
        strncpy(g_current_module_id, module_id_buf,
                sizeof(g_current_module_id) - 1);
        g_current_module_id[sizeof(g_current_module_id) - 1] = '\0';
    } else {
        // se manca module_id, azzera l'ID corrente
        g_current_module_id[0] = '\0';
    }

    // Modulo caricato con successo
    g_module_loaded = true;
    agent_write_str("LOAD_OK\n");  // conferma al gateway
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

    // Parsea args="key1=val1,key2=val2,..." → riempie argv[] con valori numerici
    const char *p_args = find_param(line, "args");  // Cerca parametro args= nella riga comando
    
    if (p_args && *p_args == '\"') {   //Verifica che args= esista e inizi con "
        p_args++;      // salta " iniziale

        const char *p_end = strchr(p_args, '\"'); // Trova " finale per estrarre contenuto
        if (p_end) {
            size_t len = (size_t)(p_end - p_args);  // lunghezza contenuto ""
            if (len >= sizeof(args_buf)) {
                len = sizeof(args_buf) - 1;     // tronca se troppo lungo
            }
            memcpy(args_buf, p_args, len);
            args_buf[len] = '\0';

            char *tok = strtok(args_buf, ",");  // primo token
            while (tok && argc < MAX_CALL_ARGS) { 
                char *eq = strchr(tok, '=');  // trova =
                if (eq) {
                    int val = atoi(eq + 1);     // converte il valore stringa in intero
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
    memset(&g_run_req, 0, sizeof(g_run_req));  // Pulisce struttura richiesta (zero tutti i campi)
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
    char *cmd  = strtok(line, " "); // cerca il primo spazio nella stringa, modifica line inserendo '\0' al posto dello spazio, ritorna un puntatore al comando
    char *rest = strtok(NULL, ""); // NULL = continua dal token precedente, "" = nessun delimitatore, prende tutto il resto della linea

    if (!cmd)
        return;

    if (strcmp(cmd, "LOAD") == 0) {
        handle_load_cmd(rest ? rest : "");  // rest ? rest : "" → se rest è NULL (no argomenti), passa stringa vuota
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
    RuntimeInitArgs init_args;      // struct definita da WAMR che contiene tutti i parametri di inizializzazione del runtime
    memset(&init_args, 0, sizeof(init_args));   // azzera tutti i campi per partire da uno stato noto

    init_args.mem_alloc_type   = Alloc_With_System_Allocator;   // Dice a WAMR come allocare memoria: Alloc_With_System_Allocator = usa il malloc/free di sistema (quello fornito da Zephyr / libc) per tutte le allocazioni interne del runtime
    
    // Qui registriamo le native functions (funzioni C del firmware chiamabili dal Wasm) sotto il modulo "env"
    init_args.native_module_name = "env";
    init_args.native_symbols     = native_symbols;  //native_symbols è un array di NativeSymbol definito da noi, che contiene voci tipo nome esportato ("gpio_toggle"), puntatore alla funzione C, signature WAMR (tipi argomenti/ritorno).
    init_args.n_native_symbols   =
        sizeof(native_symbols) / sizeof(native_symbols[0]);  // n_native_symbols è il numero di elementi dell’array calcolato come numero di byte totali occupati dall’array diviso il numero di byte di un singolo elemento

    if (!wasm_runtime_full_init(&init_args)) {  // Chiama l’API WAMR “completa” di init
        agent_write_str("ERROR code=WAMR_INIT_FAIL\n");
        return false;
    }

    #if WASM_ENABLE_LOG != 0            // Se WAMR è compilato con logging abilitato
        bh_log_set_verbose_level(0);    // Impostare il livello di verbosità dei log interni a 0 (tipicamente “solo errori” o quasi niente), per non inondare la UART di log del runtime
    #endif

    return true;
}

// Thread COMM: UART + comandi
static void comm_thread_entry(void *arg1, void *arg2, void *arg3)
{
    // Qui non usiamo argomenti, quindi ARG_UNUSED serve solo a evitare warning
    ARG_UNUSED(arg1); 
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready!\n");
        return;
    }

    int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);  //Registra serial_cb come callback di interrupt per la UART RX
    if (ret < 0) {
        printk("Error setting UART callback: %d\n", ret);
        return;
    }
    uart_irq_rx_enable(uart_dev);  // uart_irq_rx_enable abilita gli interrupt di ricezione: da questo momento ogni byte arrivato da gateway attiva serial_cb

    if (!wasm_runtime_init_all()) { // Inizializza il runtime WAMR (heap, VM, tipi, ecc.)
        return;
    }

    if (gpio_init_for_wasm() != 0) {    // Configura il LED scelto (LED0_NODE) come output e prepara le strutture per gpio_toggle
        agent_write_str("ERROR code=GPIO_INIT_FAIL\n");
        return;
    }

    agent_write_str("HELLO device_id=stm32f4_01 rtos=Zephyr runtime=WAMR_AOT fw_version=1.0.0\n");

    char line_buf[LINE_BUF_SIZE];
    for (;;) {
        int n = agent_read_line(line_buf, sizeof(line_buf));  // blocca il thread finché non arriva una riga completa terminata da \n nella message queue popolata dall’ISR UART
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

    if (!wasm_runtime_init_thread_env()) {   // OGNI thread WAMR deve inizializzare il proprio ambiente thread-local
        agent_write_str("ERROR code=WAMR_THREAD_ENV_INIT_FAIL\n");
        return;
    }

    for (;;) {
    k_sem_take(&run_sem, K_FOREVER);    // BLOCCATO: aspetta job dal COMM thread

    if (!g_module_loaded) {     // Nessun modulo caricato → reset stato e riprova
        g_runner_busy    = false;
        g_stop_requested = false;
        continue;
    }

    // snapshot della richiesta; COPIA locale: evita race condition con COMM thread
    run_request_t req;
    memcpy(&req, &g_run_req, sizeof(req));

    // Cerca funzione esportata nel modulo caricato
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

    // Numero risultati funzione (i32 in argv_local[0] se > 0)
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
        &comm_thread,       // puntatore alla struct k_thread che contiene lo stato del thread
        comm_thread_stack,  // buffer stack definito con K_THREAD_STACK_DEFINE
        COMM_THREAD_STACK_SIZE,  // dimensione dello stack in byte
        comm_thread_entry,      // funzione di entry del thread
        NULL, NULL, NULL,     // tre eventuali parametri passati all’entry (qui non ne usiamo)
        COMM_THREAD_PRIORITY,  // priorità del thread
        0,                   // opzioni extra (nessuna flag speciale)
        K_NO_WAIT);         // nessun ritardo di start

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
        k_sleep(K_FOREVER); // sospende il thread indefinitamente
    }
}



// I/O UART
static void agent_write_str(const char *buf)
{
    if (!uart_dev || !buf) {    // verifica che uart_dev sia inizializzato (non NULL) e che buf punti a una stringa valida (non NULL)
        return;
    }

    int msg_len = strlen(buf);  // strlen(buf) calcola la lunghezza della stringa escludendo il '\0' finale
    for (int i = 0; i < msg_len; i++) { // Loop byte-per-byte: per ogni carattere da 0 a msg_len-1
        // buf[i] → prende l'i-esimo byte della stringa
        /* uart_poll_out(uart_dev, buf[i]) → trasmette immediatamente l'i-esimo byte sulla UART:
                Polling/blocking: aspetta che il registro TX sia libero, scrive il byte, aspetta che sia partito
                Garantito: non passa al byte successivo finché quello corrente non è trasmesso fisicamente
        */
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

    /* Lettura bloccante dalla message queue
       Blocca indefinitamente (K_FOREVER) finché l'ISR non mette una riga nella coda
       Copia il messaggio dalla coda in local_buf.
    */
    if (k_msgq_get(&uart_msgq, &local_buf, K_FOREVER) != 0) {
        return -1;
    }

    size_t len = strlen(local_buf); //lunghezza della riga ricevuta (senza '\0')
    if (len >= max_len) {
        len = max_len - 1;      // Protezione overflow: se troppo lunga per buf, tronca a max_len-1
    }
    memcpy(buf, local_buf, len);    // copia i caratteri nel buffer chiamante
    buf[len] = '\0';        // termina con null terminator

    return (int)len;    // Restituisce quanti byte ha copiato realmente nel buffer chiamante 
}