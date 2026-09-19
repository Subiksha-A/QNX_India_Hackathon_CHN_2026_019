/*
 * qnx_vehicle_gateway_realtime.c
 *
 * QNX 8.x / Raspberry Pi 4
 *
 * Vehicle Gateway with:
 *   - QNX SCHED_FIFO priority scheduling
 *   - fixed-size bounded memory queues
 *   - logical DATA BUS / ADDRESS BUS / MEMORY BUS model
 *   - CAN safety > normal > telemetry software scheduling
 *   - MCP2515 dedicated TX buffers:
 *         TXB0 = safety
 *         TXB1 = normal
 *         TXB2 = telemetry
 *   - wrong-prediction-first retention
 *   - latest-corrected-only double buffer
 *   - low-priority CSV + gzip archival
 *   - queue/load-shedding metrics
 *   - per-thread CPU-time monitoring
 *   - CPU affinity via QNX ThreadCtl()
 *   - UDP telemetry for dashboard
 *
 * IMPORTANT:
 *   "Data bus", "address bus", and "memory bus" below are LOGICAL
 *   software architecture abstractions for the hackathon, not
 *   physical Raspberry Pi CPU buses.
 *
 * Build:
 *   qcc -Wall -Wextra -O2 qnx_vehicle_gateway_realtime.c \
 *       -o qnx_gateway -lsocket
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/neutrino.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <devctl.h>
#include <hw/io-spi.h>

/* ========================================================================== */
/* Project configuration                                                     */
/* ========================================================================== */

#define CAN0_DEV                  "/dev/io-spi/spi0/dev0"
#define CAN1_DEV                  "/dev/io-spi/spi0/dev1"

#define CAN_BITRATE               500000U
#define CAN_APPROX_BITS_8BYTE     111U

#define SAFETY_DEADLINE_MS        5.0

#define UDP_PORT                  8080
#define UDP_BROADCAST             "169.254.255.255"

#define RX_POLL_US                500U
#define TX_IDLE_US                100U
#define ARCHIVE_PERIOD_MS         100U

#define SAFETY_INGRESS_Q          64U
#define SAFETY_TX_Q               64U
#define NORMAL_Q                  128U
#define TELEMETRY_Q               256U
#define ERROR_Q                   512U
#define ERROR_BATCH               32U

#define TELEMETRY_SHED_PCT        85U

#define ROOT_DIR                  "/tmp/qnx_gateway"
#define ERROR_DIR                 ROOT_DIR "/errors"
#define LATEST_DIR                ROOT_DIR "/latest"
#define STAGE_DIR                 ROOT_DIR "/staging"

#define ID_EMERGENCY_BRAKE        0x010U
#define ID_STEERING_AIRBAG        0x020U
#define ID_FRONT_RADAR            0x110U
#define ID_VEHICLE_STATE          0x120U
#define ID_REAR_STATUS            0x210U
#define ID_PREDICTION             0x310U
#define ID_CONGESTION_NOISE       0x450U

/* ========================================================================== */
/* MCP2515 registers                                                         */
/* ========================================================================== */

#define MCP_RESET                 0xC0U
#define MCP_READ                  0x03U
#define MCP_WRITE                 0x02U
#define MCP_BITMOD                0x05U

#define REG_CANSTAT               0x0EU
#define REG_CANCTRL               0x0FU
#define REG_CNF1                  0x2AU
#define REG_CNF2                  0x29U
#define REG_CNF3                  0x28U

#define REG_CANINTF              0x2CU
#define REG_CANINTE               0x2BU

#define REG_RXB0CTRL              0x60U
#define REG_RXB0SIDH              0x61U
#define REG_RXB1CTRL              0x70U
#define REG_RXB1SIDH              0x71U

#define REG_RXM0SIDH              0x20U
#define REG_RXM0SIDL              0x21U
#define REG_RXM1SIDH              0x24U
#define REG_RXM1SIDL              0x25U

#define REG_TXB0CTRL              0x30U
#define REG_TXB0SIDH              0x31U
#define REG_TXB1CTRL              0x40U
#define REG_TXB1SIDH              0x41U
#define REG_TXB2CTRL              0x50U
#define REG_TXB2SIDH              0x51U

/* ========================================================================== */
/* Logical memory architecture                                               */
/* ========================================================================== */

typedef enum {
    MEM_BANK_SAFETY_INGRESS = 0,
    MEM_BANK_SAFETY_TX,
    MEM_BANK_NORMAL,
    MEM_BANK_TELEMETRY,
    MEM_BANK_ERRORS,
    MEM_BANK_CORRECTED
} mem_bank_t;

#define BASE_SAFETY_INGRESS       0x0000U
#define BASE_SAFETY_TX            0x0800U
#define BASE_NORMAL               0x1000U
#define BASE_TELEMETRY            0x2000U
#define BASE_ERRORS               0x3000U
#define BASE_CORRECTED            0x5000U

static uint32_t mem_base(mem_bank_t bank)
{
    switch (bank) {
        case MEM_BANK_SAFETY_INGRESS: return BASE_SAFETY_INGRESS;
        case MEM_BANK_SAFETY_TX:      return BASE_SAFETY_TX;
        case MEM_BANK_NORMAL:         return BASE_NORMAL;
        case MEM_BANK_TELEMETRY:      return BASE_TELEMETRY;
        case MEM_BANK_ERRORS:         return BASE_ERRORS;
        case MEM_BANK_CORRECTED:      return BASE_CORRECTED;
        default:                      return 0U;
    }
}

/* ========================================================================== */
/* CAN / prediction records                                                  */
/* ========================================================================== */

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    uint64_t timestamp_ns;

    uint32_t logical_addr;
    uint8_t  bus_source;      /* 0 = CAN0, 1 = CAN1 */
    uint8_t  priority_class;  /* 0 = safety, 1 = normal, 2 = telemetry */
    uint16_t slot;
} can_frame_t;

typedef struct {
    uint64_t seq;
    uint64_t timestamp_ns;

    uint32_t can_id;
    uint32_t logical_addr;
    uint32_t feature_hash;

    uint8_t predicted;
    uint8_t corrected;
    uint8_t xor_diff;
    uint8_t reason;           /* 0 = correct, 1 = wrong */

    uint8_t payload[8];
    uint8_t reserved[8];
} prediction_record_t;

/* ========================================================================== */
/* Fixed queues                                                              */
/* ========================================================================== */

typedef struct {
    can_frame_t ring[SAFETY_INGRESS_Q];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} safety_ingress_queue_t;

typedef struct {
    can_frame_t ring[SAFETY_TX_Q];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} safety_tx_queue_t;

typedef struct {
    can_frame_t ring[NORMAL_Q];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} normal_queue_t;

typedef struct {
    can_frame_t ring[TELEMETRY_Q];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} telemetry_queue_t;

typedef struct {
    prediction_record_t ring[ERROR_Q];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} error_queue_t;

/* ========================================================================== */
/* Latest-corrected double buffer                                             */
/* ========================================================================== */

typedef struct {
    prediction_record_t slot[2];
    unsigned active;
    bool valid;
    bool dirty;
    pthread_mutex_t lock;
} latest_corrected_t;

/* ========================================================================== */
/* Metrics                                                                    */
/* ========================================================================== */

typedef struct {
    pthread_mutex_t lock;

    uint64_t rx_frames;
    uint64_t tx_frames;
    uint64_t routed_frames;

    uint64_t safety_frames;
    uint64_t normal_frames;
    uint64_t telemetry_frames;

    uint64_t dropped_noise;
    uint64_t tx_busy;
    uint64_t tx_failures;

    uint64_t prediction_total;
    uint64_t prediction_wrong;
    uint64_t prediction_correct;

    uint64_t error_overflow;
    uint64_t error_archived;

    uint64_t deadline_misses;

    uint64_t safety_latency_sum_ns;
    uint64_t safety_latency_min_ns;
    uint64_t safety_latency_max_ns;

    uint64_t memory_bus_writes;

    uint64_t last_rx;
    uint64_t last_tx;
    uint64_t safety_frames_at_last_diag;
} gateway_stats_t;

/* ========================================================================== */
/* Globals                                                                    */
/* ========================================================================== */

static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_noise_mode = 0;

static int g_fd_can0 = -1;
static int g_fd_can1 = -1;
static int g_udp_sock = -1;

static struct sockaddr_in g_host_addr;

static pthread_t g_th_safety;
static pthread_t g_th_tx;
static pthread_t g_th_rx;
static pthread_t g_th_diag;
static pthread_t g_th_archive;
static pthread_t g_th_noise;

static safety_ingress_queue_t g_q_safety_ingress;
static safety_tx_queue_t      g_q_safety_tx;
static normal_queue_t         g_q_normal;
static telemetry_queue_t      g_q_telemetry;
static error_queue_t          g_q_errors;

static latest_corrected_t g_latest_corrected;
static gateway_stats_t g_stats;

static pthread_mutex_t g_memory_bus_lock;

static uint64_t g_prediction_seq = 1U;
static uint64_t g_archive_seq = 1U;

static unsigned g_cpu_count = 1U;

/* ========================================================================== */
/* Utilities                                                                  */
/* ========================================================================== */

static uint64_t now_ns(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }

    return ((uint64_t)ts.tv_sec * 1000000000ULL) +
           (uint64_t)ts.tv_nsec;
}

static double ns_to_ms(uint64_t ns)
{
    return (double)ns / 1000000.0;
}

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

static uint32_t hash32(const uint8_t *data, size_t len)
{
    uint32_t h = 2166136261U;
    size_t i;

    for (i = 0U; i < len; ++i) {
        h ^= data[i];
        h *= 16777619U;
    }

    return h;
}

static void make_dir_if_needed(const char *path)
{
    if (mkdir(path, 0777) != 0 && errno != EEXIST) {
        fprintf(stderr, "mkdir(%s): %s\n", path, strerror(errno));
    }
}

static void init_directories(void)
{
    make_dir_if_needed(ROOT_DIR);
    make_dir_if_needed(ERROR_DIR);
    make_dir_if_needed(LATEST_DIR);
    make_dir_if_needed(STAGE_DIR);
}

/* ========================================================================== */
/* QNX CPU affinity                                                           */
/* ========================================================================== */

static void pin_current_thread(unsigned cpu)
{
    unsigned runmask;

    if (g_cpu_count == 0U) {
        return;
    }

    cpu %= g_cpu_count;

    runmask = 1U << cpu;

    if (ThreadCtl(_NTO_TCTL_RUNMASK, &runmask) != 0) {
        fprintf(stderr,
                "[WARN] CPU affinity CPU%u failed: %s\n",
                cpu,
                strerror(errno));
    }
}

/* ========================================================================== */
/* Mutex / condition helpers                                                  */
/* ========================================================================== */

static void init_priority_inherit_mutex(pthread_mutex_t *m)
{
    pthread_mutexattr_t a;

    (void)pthread_mutexattr_init(&a);
    (void)pthread_mutexattr_setprotocol(&a, PTHREAD_PRIO_INHERIT);
    (void)pthread_mutex_init(m, &a);
    (void)pthread_mutexattr_destroy(&a);
}

static void init_monotonic_cond(pthread_cond_t *c)
{
    pthread_condattr_t a;

    (void)pthread_condattr_init(&a);
    (void)pthread_condattr_setclock(&a, CLOCK_MONOTONIC);
    (void)pthread_cond_init(c, &a);
    (void)pthread_condattr_destroy(&a);
}

/* ========================================================================== */
/* Queue initialization                                                       */
/* ========================================================================== */

static void init_queues(void)
{
    memset(&g_q_safety_ingress, 0, sizeof(g_q_safety_ingress));
    memset(&g_q_safety_tx, 0, sizeof(g_q_safety_tx));
    memset(&g_q_normal, 0, sizeof(g_q_normal));
    memset(&g_q_telemetry, 0, sizeof(g_q_telemetry));
    memset(&g_q_errors, 0, sizeof(g_q_errors));

    init_priority_inherit_mutex(&g_q_safety_ingress.lock);
    init_priority_inherit_mutex(&g_q_safety_tx.lock);
    init_priority_inherit_mutex(&g_q_normal.lock);
    init_priority_inherit_mutex(&g_q_telemetry.lock);
    init_priority_inherit_mutex(&g_q_errors.lock);

    init_monotonic_cond(&g_q_safety_ingress.not_empty);
    init_monotonic_cond(&g_q_safety_tx.not_empty);
    init_monotonic_cond(&g_q_normal.not_empty);
    init_monotonic_cond(&g_q_telemetry.not_empty);
    init_monotonic_cond(&g_q_errors.not_empty);

    memset(&g_latest_corrected, 0, sizeof(g_latest_corrected));
    init_priority_inherit_mutex(&g_latest_corrected.lock);
}

/* ========================================================================== */
/* Queue size functions                                                       */
/* ========================================================================== */

static unsigned safety_ingress_size(void)
{
    unsigned n;

    pthread_mutex_lock(&g_q_safety_ingress.lock);
    n = g_q_safety_ingress.count;
    pthread_mutex_unlock(&g_q_safety_ingress.lock);

    return n;
}

static unsigned safety_tx_size(void)
{
    unsigned n;

    pthread_mutex_lock(&g_q_safety_tx.lock);
    n = g_q_safety_tx.count;
    pthread_mutex_unlock(&g_q_safety_tx.lock);

    return n;
}

static unsigned normal_size(void)
{
    unsigned n;

    pthread_mutex_lock(&g_q_normal.lock);
    n = g_q_normal.count;
    pthread_mutex_unlock(&g_q_normal.lock);

    return n;
}

static unsigned telemetry_size(void)
{
    unsigned n;

    pthread_mutex_lock(&g_q_telemetry.lock);
    n = g_q_telemetry.count;
    pthread_mutex_unlock(&g_q_telemetry.lock);

    return n;
}

static unsigned error_size(void)
{
    unsigned n;

    pthread_mutex_lock(&g_q_errors.lock);
    n = g_q_errors.count;
    pthread_mutex_unlock(&g_q_errors.lock);

    return n;
}

/* ========================================================================== */
/* Safety ingress queue                                                       */
/* ========================================================================== */

static bool safety_ingress_push(const can_frame_t *frame)
{
    bool ok = false;
    can_frame_t x;

    pthread_mutex_lock(&g_q_safety_ingress.lock);

    if (g_q_safety_ingress.count < SAFETY_INGRESS_Q) {
        x = *frame;

        x.slot = g_q_safety_ingress.tail;
        x.logical_addr =
            mem_base(MEM_BANK_SAFETY_INGRESS) +
            ((uint32_t)x.slot * (uint32_t)sizeof(can_frame_t));

        g_q_safety_ingress.ring[g_q_safety_ingress.tail] = x;
        g_q_safety_ingress.tail =
            (uint16_t)((g_q_safety_ingress.tail + 1U) %
                       SAFETY_INGRESS_Q);
        ++g_q_safety_ingress.count;

        ok = true;

        pthread_cond_signal(&g_q_safety_ingress.not_empty);
    }

    pthread_mutex_unlock(&g_q_safety_ingress.lock);

    return ok;
}

static bool safety_ingress_pop(can_frame_t *frame)
{
    bool ok = false;

    pthread_mutex_lock(&g_q_safety_ingress.lock);

    while (g_q_safety_ingress.count == 0U && g_running) {
        pthread_cond_wait(&g_q_safety_ingress.not_empty,
                          &g_q_safety_ingress.lock);
    }

    if (g_q_safety_ingress.count != 0U) {
        *frame = g_q_safety_ingress.ring[g_q_safety_ingress.head];

        g_q_safety_ingress.head =
            (uint16_t)((g_q_safety_ingress.head + 1U) %
                       SAFETY_INGRESS_Q);
        --g_q_safety_ingress.count;

        ok = true;
    }

    pthread_mutex_unlock(&g_q_safety_ingress.lock);

    return ok;
}

/* ========================================================================== */
/* Safety TX queue                                                            */
/* ========================================================================== */

static bool safety_tx_push(const can_frame_t *frame)
{
    bool ok = false;
    can_frame_t x;

    pthread_mutex_lock(&g_q_safety_tx.lock);

    if (g_q_safety_tx.count < SAFETY_TX_Q) {
        x = *frame;

        x.slot = g_q_safety_tx.tail;
        x.logical_addr =
            mem_base(MEM_BANK_SAFETY_TX) +
            ((uint32_t)x.slot * (uint32_t)sizeof(can_frame_t));

        g_q_safety_tx.ring[g_q_safety_tx.tail] = x;
        g_q_safety_tx.tail =
            (uint16_t)((g_q_safety_tx.tail + 1U) %
                       SAFETY_TX_Q);
        ++g_q_safety_tx.count;

        ok = true;

        pthread_cond_signal(&g_q_safety_tx.not_empty);
    }

    pthread_mutex_unlock(&g_q_safety_tx.lock);

    return ok;
}

static bool safety_tx_peek(can_frame_t *frame)
{
    bool ok = false;

    pthread_mutex_lock(&g_q_safety_tx.lock);

    if (g_q_safety_tx.count != 0U) {
        *frame = g_q_safety_tx.ring[g_q_safety_tx.head];
        ok = true;
    }

    pthread_mutex_unlock(&g_q_safety_tx.lock);

    return ok;
}

static void safety_tx_drop(void)
{
    pthread_mutex_lock(&g_q_safety_tx.lock);

    if (g_q_safety_tx.count != 0U) {
        g_q_safety_tx.head =
            (uint16_t)((g_q_safety_tx.head + 1U) %
                       SAFETY_TX_Q);
        --g_q_safety_tx.count;
    }

    pthread_mutex_unlock(&g_q_safety_tx.lock);
}

/* ========================================================================== */
/* Normal queue                                                               */
/* ========================================================================== */

static bool normal_push(const can_frame_t *frame)
{
    bool ok = false;
    can_frame_t x;

    pthread_mutex_lock(&g_q_normal.lock);

    if (g_q_normal.count < NORMAL_Q) {
        x = *frame;

        x.slot = g_q_normal.tail;
        x.logical_addr =
            mem_base(MEM_BANK_NORMAL) +
            ((uint32_t)x.slot * (uint32_t)sizeof(can_frame_t));

        g_q_normal.ring[g_q_normal.tail] = x;
        g_q_normal.tail =
            (uint16_t)((g_q_normal.tail + 1U) % NORMAL_Q);
        ++g_q_normal.count;

        ok = true;

        pthread_cond_signal(&g_q_normal.not_empty);
    }

    pthread_mutex_unlock(&g_q_normal.lock);

    return ok;
}

static bool normal_peek(can_frame_t *frame)
{
    bool ok = false;

    pthread_mutex_lock(&g_q_normal.lock);

    if (g_q_normal.count != 0U) {
        *frame = g_q_normal.ring[g_q_normal.head];
        ok = true;
    }

    pthread_mutex_unlock(&g_q_normal.lock);

    return ok;
}

static void normal_drop(void)
{
    pthread_mutex_lock(&g_q_normal.lock);

    if (g_q_normal.count != 0U) {
        g_q_normal.head =
            (uint16_t)((g_q_normal.head + 1U) %
                       NORMAL_Q);
        --g_q_normal.count;
    }

    pthread_mutex_unlock(&g_q_normal.lock);
}

/* ========================================================================== */
/* Telemetry queue                                                            */
/* ========================================================================== */

static bool telemetry_push(const can_frame_t *frame)
{
    bool ok = false;
    can_frame_t x;

    pthread_mutex_lock(&g_q_telemetry.lock);

    if (g_q_telemetry.count < TELEMETRY_Q) {
        x = *frame;

        x.slot = g_q_telemetry.tail;
        x.logical_addr =
            mem_base(MEM_BANK_TELEMETRY) +
            ((uint32_t)x.slot * (uint32_t)sizeof(can_frame_t));

        g_q_telemetry.ring[g_q_telemetry.tail] = x;
        g_q_telemetry.tail =
            (uint16_t)((g_q_telemetry.tail + 1U) %
                       TELEMETRY_Q);
        ++g_q_telemetry.count;

        ok = true;

        pthread_cond_signal(&g_q_telemetry.not_empty);
    }

    pthread_mutex_unlock(&g_q_telemetry.lock);

    return ok;
}

static bool telemetry_peek(can_frame_t *frame)
{
    bool ok = false;

    pthread_mutex_lock(&g_q_telemetry.lock);

    if (g_q_telemetry.count != 0U) {
        *frame = g_q_telemetry.ring[g_q_telemetry.head];
        ok = true;
    }

    pthread_mutex_unlock(&g_q_telemetry.lock);

    return ok;
}

static void telemetry_drop(void)
{
    pthread_mutex_lock(&g_q_telemetry.lock);

    if (g_q_telemetry.count != 0U) {
        g_q_telemetry.head =
            (uint16_t)((g_q_telemetry.head + 1U) %
                       TELEMETRY_Q);
        --g_q_telemetry.count;
    }

    pthread_mutex_unlock(&g_q_telemetry.lock);
}

/* ========================================================================== */
/* Error queue                                                                */
/* ========================================================================== */

static bool error_push(const prediction_record_t *record)
{
    bool ok = false;
    prediction_record_t x;

    pthread_mutex_lock(&g_q_errors.lock);

    if (g_q_errors.count < ERROR_Q) {
        x = *record;

        x.logical_addr =
            mem_base(MEM_BANK_ERRORS) +
            ((uint32_t)g_q_errors.tail *
             (uint32_t)sizeof(prediction_record_t));

        g_q_errors.ring[g_q_errors.tail] = x;
        g_q_errors.tail =
            (uint16_t)((g_q_errors.tail + 1U) %
                       ERROR_Q);
        ++g_q_errors.count;

        ok = true;

        pthread_cond_signal(&g_q_errors.not_empty);
    }

    pthread_mutex_unlock(&g_q_errors.lock);

    if (!ok) {
        pthread_mutex_lock(&g_stats.lock);
        ++g_stats.error_overflow;
        pthread_mutex_unlock(&g_stats.lock);
    }

    return ok;
}

static unsigned error_take_batch(prediction_record_t *dst,
                                 unsigned max_count)
{
    unsigned n = 0U;

    pthread_mutex_lock(&g_q_errors.lock);

    while (n < max_count && g_q_errors.count != 0U) {
        dst[n] = g_q_errors.ring[g_q_errors.head];

        g_q_errors.head =
            (uint16_t)((g_q_errors.head + 1U) %
                       ERROR_Q);
        --g_q_errors.count;

        ++n;
    }

    pthread_mutex_unlock(&g_q_errors.lock);

    return n;
}

/* ========================================================================== */
/* Latest corrected double buffer                                             */
/* ========================================================================== */

static void latest_corrected_publish(const prediction_record_t *record)
{
    pthread_mutex_lock(&g_latest_corrected.lock);

    g_latest_corrected.active ^= 1U;

    g_latest_corrected.slot[g_latest_corrected.active] =
        *record;

    g_latest_corrected.slot[g_latest_corrected.active].logical_addr =
        mem_base(MEM_BANK_CORRECTED);

    g_latest_corrected.valid = true;
    g_latest_corrected.dirty = true;

    pthread_mutex_unlock(&g_latest_corrected.lock);
}

static bool latest_corrected_snapshot(prediction_record_t *record)
{
    bool valid;
    bool dirty;

    pthread_mutex_lock(&g_latest_corrected.lock);

    valid = g_latest_corrected.valid;
    dirty = g_latest_corrected.dirty;

    if (valid) {
        *record =
            g_latest_corrected.slot[g_latest_corrected.active];
    }

    g_latest_corrected.dirty = false;

    pthread_mutex_unlock(&g_latest_corrected.lock);

    return valid && dirty;
}

/* ========================================================================== */
/* MCP2515 SPI -- fixed buffer, no malloc in RT path                         */
/* ========================================================================== */

#define MAX_SPI_BYTES 32U

typedef union {
    uint64_t alignment;
    uint8_t raw[sizeof(spi_xchng_t) + MAX_SPI_BYTES];
} spi_storage_t;

static int spi_xfer(int fd,
                    const void *tx,
                    void *rx,
                    size_t len)
{
    spi_storage_t storage;
    spi_xchng_t *x;
    uint32_t total;
    int rc;

    if (len > MAX_SPI_BYTES) {
        return EINVAL;
    }

    memset(&storage, 0, sizeof(storage));

    x = (spi_xchng_t *)storage.raw;

    total = (uint32_t)(sizeof(spi_xchng_t) + len);
    x->nbytes = (uint32_t)len;

    if (tx != NULL) {
        memcpy(x->data, tx, len);
    }

    rc = devctl(fd,
                DCMD_SPI_DATA_XCHNG,
                x,
                total,
                NULL);

    if (rc == 0 && rx != NULL) {
        memcpy(rx, x->data, len);
    }

    return rc;
}

static int mcp_write(int fd,
                     uint8_t reg,
                     uint8_t value)
{
    struct __attribute__((packed)) {
        uint8_t cmd;
        uint8_t reg;
        uint8_t value;
    } packet;

    packet.cmd = MCP_WRITE;
    packet.reg = reg;
    packet.value = value;

    return spi_xfer(fd,
                    &packet,
                    NULL,
                    sizeof(packet));
}

static int mcp_read(int fd,
                    uint8_t reg,
                    uint8_t *value)
{
    struct __attribute__((packed)) {
        uint8_t cmd;
        uint8_t reg;
        uint8_t dummy;
    } tx;

    struct __attribute__((packed)) {
        uint8_t cmd;
        uint8_t reg;
        uint8_t dummy;
    } rx;

    memset(&tx, 0, sizeof(tx));
    memset(&rx, 0, sizeof(rx));

    tx.cmd = MCP_READ;
    tx.reg = reg;

    if (spi_xfer(fd,
                 &tx,
                 &rx,
                 sizeof(tx)) != 0) {
        return EIO;
    }

    *value = rx.dummy;

    return 0;
}

static int mcp_bit_modify(int fd,
                          uint8_t reg,
                          uint8_t mask,
                          uint8_t value)
{
    struct __attribute__((packed)) {
        uint8_t cmd;
        uint8_t reg;
        uint8_t mask;
        uint8_t value;
    } packet;

    packet.cmd = MCP_BITMOD;
    packet.reg = reg;
    packet.mask = mask;
    packet.value = value;

    return spi_xfer(fd,
                    &packet,
                    NULL,
                    sizeof(packet));
}

/* ========================================================================== */
/* CAN initialization                                                        */
/* ========================================================================== */

static int init_can_channel(const char *device,
                            const char *name)
{
    int fd;
    spi_cfg_t cfg;
    uint8_t canstat = 0U;

    fd = open(device, O_RDWR);

    if (fd < 0) {
        fprintf(stderr,
                "[-] open %s failed: %s\n",
                device,
                strerror(errno));
        return -1;
    }

    memset(&cfg, 0, sizeof(cfg));

    cfg.mode = 8U | (1U << 10);
    cfg.clock_rate = 5000000U;

    if (devctl(fd,
               DCMD_SPI_SET_CONFIG,
               &cfg,
               sizeof(cfg),
               NULL) != 0) {
        fprintf(stderr,
                "[-] SPI configuration failed: %s\n",
                strerror(errno));

        close(fd);
        return -1;
    }

    {
        uint8_t reset_cmd = MCP_RESET;

        if (spi_xfer(fd,
                     &reset_cmd,
                     NULL,
                     1U) != 0) {
            fprintf(stderr,
                    "[-] MCP2515 reset failed on %s\n",
                    name);

            close(fd);
            return -1;
        }
    }

    usleep(10000);

    /*
     * Existing project assumption:
     * 16-MHz MCP2515 oscillator / 500-kbps nominal CAN.
     * Verify against the physical board before claiming exact timing.
     */
    (void)mcp_write(fd, REG_CNF1, 0x01U);
    (void)mcp_write(fd, REG_CNF2, 0x90U);
    (void)mcp_write(fd, REG_CNF3, 0x02U);

    /* Accept all standard identifiers on both RX buffers. */
    (void)mcp_write(fd, REG_RXM0SIDH, 0x00U);
    (void)mcp_write(fd, REG_RXM0SIDL, 0x00U);
    (void)mcp_write(fd, REG_RXM1SIDH, 0x00U);
    (void)mcp_write(fd, REG_RXM1SIDL, 0x00U);

    /* Enable RX0 / RX1 interrupts. */
    (void)mcp_write(fd, REG_CANINTE, 0x03U);

    /* RX0 rollover to RX1. */
    (void)mcp_write(fd, REG_RXB0CTRL, 0x64U);

    /* RX1 accepts all valid standard frames. */
    (void)mcp_write(fd, REG_RXB1CTRL, 0x60U);

    /* Normal operation. */
    (void)mcp_bit_modify(fd,
                         REG_CANCTRL,
                         0xE0U,
                         0x00U);

    (void)mcp_read(fd,
                   REG_CANSTAT,
                   &canstat);

    printf("[+] %s initialized: CANSTAT=0x%02X\n",
           name,
           canstat);

    return fd;
}

/* ========================================================================== */
/* MCP2515 RX                                                                  */
/* ========================================================================== */

static int receive_can_frame(int fd,
                             uint8_t source,
                             can_frame_t *frame)
{
    uint8_t intf;
    uint8_t base_reg;

    struct __attribute__((packed)) {
        uint8_t cmd;
        uint8_t reg;
        uint8_t sidh;
        uint8_t sidl;
        uint8_t eid8;
        uint8_t eid0;
        uint8_t dlc;
        uint8_t data[8];
    } tx;

    struct __attribute__((packed)) {
        uint8_t cmd;
        uint8_t reg;
        uint8_t sidh;
        uint8_t sidl;
        uint8_t eid8;
        uint8_t eid0;
        uint8_t dlc;
        uint8_t data[8];
    } rx;

    if (mcp_read(fd,
                 REG_CANINTF,
                 &intf) != 0) {
        return -1;
    }

    if ((intf & 0x01U) != 0U) {
        base_reg = REG_RXB0SIDH;
    } else if ((intf & 0x02U) != 0U) {
        base_reg = REG_RXB1SIDH;
    } else {
        return 0;
    }

    memset(&tx, 0, sizeof(tx));
    memset(&rx, 0, sizeof(rx));

    tx.cmd = MCP_READ;
    tx.reg = base_reg;

    if (spi_xfer(fd,
                 &tx,
                 &rx,
                 sizeof(tx)) != 0) {
        return -1;
    }

    memset(frame, 0, sizeof(*frame));

    frame->id =
        ((uint32_t)rx.sidh << 3) |
        ((uint32_t)rx.sidl >> 5);

    frame->dlc = (uint8_t)(rx.dlc & 0x0FU);

    if (frame->dlc > 8U) {
        frame->dlc = 8U;
    }

    memcpy(frame->data,
           rx.data,
           frame->dlc);

    frame->timestamp_ns = now_ns();
    frame->bus_source = source;

    if (base_reg == REG_RXB0SIDH) {
        (void)mcp_bit_modify(fd,
                             REG_CANINTF,
                             0x01U,
                             0x00U);
    } else {
        (void)mcp_bit_modify(fd,
                             REG_CANINTF,
                             0x02U,
                             0x00U);
    }

    return 1;
}

/* ========================================================================== */
/* MCP2515 TX buffers                                                         */
/* ========================================================================== */

static int send_can_frame_buffer(int fd,
                                 unsigned tx_buffer,
                                 const can_frame_t *frame)
{
    uint8_t ctrl_reg;
    uint8_t data_reg;
    uint8_t start_tx_value;
    uint8_t ctrl_value;

    struct __attribute__((packed)) {
        uint8_t cmd;
        uint8_t reg;
        uint8_t sidh;
        uint8_t sidl;
        uint8_t eid8;
        uint8_t eid0;
        uint8_t dlc;
        uint8_t data[8];
    } packet;

    if (frame == NULL ||
        frame->dlc > 8U ||
        tx_buffer > 2U) {
        return EINVAL;
    }

    if (tx_buffer == 0U) {
        ctrl_reg = REG_TXB0CTRL;
        data_reg = REG_TXB0SIDH;
        start_tx_value = 0x0BU; /* TXP=3 + TXREQ */
    } else if (tx_buffer == 1U) {
        ctrl_reg = REG_TXB1CTRL;
        data_reg = REG_TXB1SIDH;
        start_tx_value = 0x0AU; /* TXP=2 + TXREQ */
    } else {
        ctrl_reg = REG_TXB2CTRL;
        data_reg = REG_TXB2SIDH;
        start_tx_value = 0x08U; /* TXP=0 + TXREQ */
    }

    if (mcp_read(fd,
                 ctrl_reg,
                 &ctrl_value) != 0) {
        return EIO;
    }

    if ((ctrl_value & 0x08U) != 0U) {
        return EBUSY;
    }

    memset(&packet, 0, sizeof(packet));

    packet.cmd = MCP_WRITE;
    packet.reg = data_reg;

    packet.sidh = (uint8_t)(frame->id >> 3);
    packet.sidl = (uint8_t)(frame->id << 5);

    packet.dlc = frame->dlc;

    memcpy(packet.data,
           frame->data,
           frame->dlc);

    if (spi_xfer(fd,
                 &packet,
                 NULL,
                 7U + frame->dlc) != 0) {
        return EIO;
    }

    if (mcp_write(fd,
                  ctrl_reg,
                  start_tx_value) != 0) {
        return EIO;
    }

    return 0;
}

/* ========================================================================== */
/* Prediction handling                                                        */
/* ========================================================================== */

static void process_prediction(const can_frame_t *frame)
{
    prediction_record_t record;

    if (frame == NULL ||
        frame->id != ID_PREDICTION ||
        frame->dlc < 2U) {
        return;
    }

    memset(&record, 0, sizeof(record));

    record.seq = g_prediction_seq++;
    record.timestamp_ns = now_ns();

    record.can_id = frame->id;

    record.predicted = frame->data[0];
    record.corrected = frame->data[1];

    record.xor_diff =
        (uint8_t)(record.predicted ^
                  record.corrected);

    record.reason =
        (record.predicted == record.corrected) ? 0U : 1U;

    if (frame->dlc > 2U) {
        memcpy(record.payload,
               &frame->data[2],
               frame->dlc - 2U);
    }

    record.feature_hash =
        hash32(frame->data + 2,
               (frame->dlc > 2U) ?
                   (size_t)(frame->dlc - 2U) : 0U);

    pthread_mutex_lock(&g_stats.lock);
    ++g_stats.prediction_total;
    pthread_mutex_unlock(&g_stats.lock);

    if (record.predicted != record.corrected) {
        /*
         * WRONG = high-retention path.
         * No filesystem access here.
         */
        if (error_push(&record)) {
            pthread_mutex_lock(&g_memory_bus_lock);
            pthread_mutex_lock(&g_stats.lock);

            ++g_stats.memory_bus_writes;
            ++g_stats.prediction_wrong;

            pthread_mutex_unlock(&g_stats.lock);
            pthread_mutex_unlock(&g_memory_bus_lock);
        }
    } else {
        /*
         * CORRECT = latest-only.
         * Previous correct record is overwritten.
         */
        latest_corrected_publish(&record);

        pthread_mutex_lock(&g_memory_bus_lock);
        pthread_mutex_lock(&g_stats.lock);

        ++g_stats.memory_bus_writes;
        ++g_stats.prediction_correct;

        pthread_mutex_unlock(&g_stats.lock);
        pthread_mutex_unlock(&g_memory_bus_lock);
    }
}

/* ========================================================================== */
/* THREAD 1: Safety task P50                                                  */
/* ========================================================================== */

static void *safety_thread(void *arg)
{
    (void)arg;

    pin_current_thread(0U);

    while (g_running) {
        can_frame_t frame;

        if (!safety_ingress_pop(&frame)) {
            continue;
        }

        /*
         * Safety processing remains intentionally tiny:
         * no malloc, no file I/O, no gzip.
         */
        frame.priority_class = 0U;

        if (!safety_tx_push(&frame)) {
            /*
             * Safety queue overflow is a visible failure.
             * It is NOT converted into low-priority traffic.
             */
            pthread_mutex_lock(&g_stats.lock);
            ++g_stats.tx_failures;
            pthread_mutex_unlock(&g_stats.lock);
        }
    }

    return NULL;
}

/* ========================================================================== */
/* THREAD 2: CAN TX priority arbiter P48                                      */
/* ========================================================================== */

static void *tx_arbiter_thread(void *arg)
{
    (void)arg;

    pin_current_thread(0U);

    while (g_running) {
        can_frame_t frame;

        /*
         * LEVEL 1: SAFETY
         */
        if (safety_tx_peek(&frame)) {
            int rc =
                send_can_frame_buffer(g_fd_can1,
                                      0U,
                                      &frame);

            if (rc == 0) {
                uint64_t latency_ns;

                safety_tx_drop();

                latency_ns =
                    now_ns() - frame.timestamp_ns;

                pthread_mutex_lock(&g_stats.lock);

                ++g_stats.tx_frames;
                ++g_stats.routed_frames;
                ++g_stats.safety_frames;

                g_stats.safety_latency_sum_ns +=
                    latency_ns;

                if (g_stats.safety_latency_min_ns ==
                        UINT64_MAX ||
                    latency_ns <
                        g_stats.safety_latency_min_ns) {
                    g_stats.safety_latency_min_ns =
                        latency_ns;
                }

                if (latency_ns >
                    g_stats.safety_latency_max_ns) {
                    g_stats.safety_latency_max_ns =
                        latency_ns;
                }

                if (ns_to_ms(latency_ns) >
                    SAFETY_DEADLINE_MS) {
                    ++g_stats.deadline_misses;
                }

                pthread_mutex_unlock(&g_stats.lock);

                continue;
            }

            pthread_mutex_lock(&g_stats.lock);

            if (rc == EBUSY) {
                ++g_stats.tx_busy;
            } else {
                ++g_stats.tx_failures;
            }

            pthread_mutex_unlock(&g_stats.lock);

            usleep(TX_IDLE_US);

            continue;
        }

        /*
         * LEVEL 2: NORMAL
         */
        if (normal_peek(&frame)) {
            int rc =
                send_can_frame_buffer(g_fd_can1,
                                      1U,
                                      &frame);

            if (rc == 0) {
                normal_drop();

                pthread_mutex_lock(&g_stats.lock);

                ++g_stats.tx_frames;
                ++g_stats.routed_frames;
                ++g_stats.normal_frames;

                pthread_mutex_unlock(&g_stats.lock);

                continue;
            }

            if (rc != EBUSY) {
                pthread_mutex_lock(&g_stats.lock);
                ++g_stats.tx_failures;
                pthread_mutex_unlock(&g_stats.lock);
            }
        }

        /*
         * LEVEL 3: TELEMETRY / NOISE
         */
        if (telemetry_peek(&frame)) {
            int rc =
                send_can_frame_buffer(g_fd_can1,
                                      2U,
                                      &frame);

            if (rc == 0) {
                telemetry_drop();

                pthread_mutex_lock(&g_stats.lock);

                ++g_stats.tx_frames;
                ++g_stats.routed_frames;
                ++g_stats.telemetry_frames;

                pthread_mutex_unlock(&g_stats.lock);

                continue;
            }

            if (rc != EBUSY) {
                pthread_mutex_lock(&g_stats.lock);
                ++g_stats.tx_failures;
                pthread_mutex_unlock(&g_stats.lock);
            }
        }

        usleep(TX_IDLE_US);
    }

    return NULL;
}

/* ========================================================================== */
/* THREAD 3: CAN RX P40                                                      */
/* ========================================================================== */

static void *rx_thread(void *arg)
{
    (void)arg;

    pin_current_thread(1U);

    while (g_running) {
        bool did_work = false;

        /*
         * CAN0 = front / main ingress.
         */
        {
            can_frame_t frame;
            int rc =
                receive_can_frame(g_fd_can0,
                                  0U,
                                  &frame);

            if (rc > 0) {
                did_work = true;

                pthread_mutex_lock(&g_stats.lock);
                ++g_stats.rx_frames;
                pthread_mutex_unlock(&g_stats.lock);

                /*
                 * Prediction path is independent from CAN priority
                 * classification and therefore can analyze ID 0x310.
                 */
                process_prediction(&frame);

                if (frame.id ==
                        ID_EMERGENCY_BRAKE ||
                    frame.id ==
                        ID_STEERING_AIRBAG) {

                    frame.priority_class = 0U;

                    if (!safety_ingress_push(&frame)) {
                        pthread_mutex_lock(&g_stats.lock);
                        ++g_stats.tx_failures;
                        pthread_mutex_unlock(&g_stats.lock);
                    }

                } else if (frame.id ==
                               ID_FRONT_RADAR ||
                           frame.id ==
                               ID_VEHICLE_STATE ||
                           frame.id ==
                               ID_REAR_STATUS ||
                           frame.id ==
                               ID_PREDICTION) {

                    frame.priority_class = 1U;

                    if (!normal_push(&frame)) {
                        pthread_mutex_lock(&g_stats.lock);
                        ++g_stats.tx_failures;
                        pthread_mutex_unlock(&g_stats.lock);
                    }

                } else {

                    frame.priority_class = 2U;

                    if (telemetry_size() >=
                        (TELEMETRY_Q *
                         TELEMETRY_SHED_PCT) / 100U) {

                        pthread_mutex_lock(&g_stats.lock);
                        ++g_stats.dropped_noise;
                        pthread_mutex_unlock(&g_stats.lock);

                    } else if (!telemetry_push(&frame)) {

                        pthread_mutex_lock(&g_stats.lock);
                        ++g_stats.dropped_noise;
                        pthread_mutex_unlock(&g_stats.lock);
                    }
                }
            }
        }

        /*
         * CAN1 = rear ingress monitoring.
         */
        {
            can_frame_t frame;
            int rc =
                receive_can_frame(g_fd_can1,
                                  1U,
                                  &frame);

            if (rc > 0) {
                did_work = true;

                pthread_mutex_lock(&g_stats.lock);
                ++g_stats.rx_frames;
                pthread_mutex_unlock(&g_stats.lock);

                process_prediction(&frame);

                if (frame.id ==
                        ID_EMERGENCY_BRAKE ||
                    frame.id ==
                        ID_STEERING_AIRBAG) {

                    frame.priority_class = 0U;
                    (void)safety_ingress_push(&frame);

                } else if (frame.id ==
                               ID_FRONT_RADAR ||
                           frame.id ==
                               ID_VEHICLE_STATE ||
                           frame.id ==
                               ID_REAR_STATUS ||
                           frame.id ==
                               ID_PREDICTION) {

                    frame.priority_class = 1U;
                    (void)normal_push(&frame);

                } else {

                    frame.priority_class = 2U;

                    if (telemetry_size() <
                        (TELEMETRY_Q *
                         TELEMETRY_SHED_PCT) / 100U) {
                        (void)telemetry_push(&frame);
                    } else {
                        pthread_mutex_lock(&g_stats.lock);
                        ++g_stats.dropped_noise;
                        pthread_mutex_unlock(&g_stats.lock);
                    }
                }
            }
        }

        if (!did_work) {
            usleep(RX_POLL_US);
        }
    }

    return NULL;
}

/* ========================================================================== */
/* THREAD 4: Noise generator P10                                              */
/* ========================================================================== */

static void *noise_thread(void *arg)
{
    (void)arg;

    pin_current_thread(2U);

    while (g_running) {
        if (!g_noise_mode) {
            usleep(1000U);
            continue;
        }

        {
            can_frame_t frame;
            unsigned i;

            memset(&frame, 0, sizeof(frame));

            frame.id = ID_CONGESTION_NOISE;
            frame.dlc = 8U;
            frame.timestamp_ns = now_ns();
            frame.priority_class = 2U;

            for (i = 0U; i < 8U; ++i) {
                frame.data[i] =
                    (uint8_t)(i ^
                              (unsigned)(frame.timestamp_ns &
                                         0xFFU));
            }

            if (telemetry_size() >=
                (TELEMETRY_Q *
                 TELEMETRY_SHED_PCT) / 100U) {

                pthread_mutex_lock(&g_stats.lock);
                ++g_stats.dropped_noise;
                pthread_mutex_unlock(&g_stats.lock);

            } else {
                (void)telemetry_push(&frame);
            }
        }

        usleep(100U);
    }

    return NULL;
}

/* ========================================================================== */
/* Archive helpers                                                            */
/* ========================================================================== */

static int write_all(int fd,
                     const char *buffer,
                     size_t len)
{
    size_t offset = 0U;

    while (offset < len) {
        ssize_t written =
            write(fd,
                  buffer + offset,
                  len - offset);

        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }

            return -1;
        }

        offset += (size_t)written;
    }

    return 0;
}

static bool gzip_available(void)
{
    return access("/usr/bin/gzip", X_OK) == 0 ||
           access("/bin/gzip", X_OK) == 0;
}

static void gzip_file(const char *path)
{
    char command[512];

    if (!gzip_available()) {
        return;
    }

    if (access("/usr/bin/gzip", X_OK) == 0) {
        (void)snprintf(command,
                       sizeof(command),
                       "/usr/bin/gzip -f \"%s\"",
                       path);
    } else {
        (void)snprintf(command,
                       sizeof(command),
                       "/bin/gzip -f \"%s\"",
                       path);
    }

    /*
     * This function is ONLY called by the P5 archive thread.
     * It is never part of the safety path.
     */
    (void)system(command);
}

static void archive_error_batch(prediction_record_t *records,
                                unsigned count)
{
    char path[256];
    char gz_path[256];
    char final_path[256];

    int fd;

    unsigned i;

    if (count == 0U) {
        return;
    }

    (void)snprintf(path,
                   sizeof(path),
                   STAGE_DIR "/error_batch_%06llu.csv",
                   (unsigned long long)g_archive_seq);

    fd = open(path,
              O_WRONLY | O_CREAT | O_TRUNC,
              0666);

    if (fd < 0) {
        fprintf(stderr,
                "[ARCHIVE] open failed: %s\n",
                strerror(errno));
        return;
    }

    {
        static const char header[] =
            "seq,timestamp_ns,can_id,logical_address,"
            "predicted,corrected,xor_diff,feature_hash\n";

        (void)write_all(fd,
                        header,
                        strlen(header));
    }

    for (i = 0U; i < count; ++i) {
        char line[256];

        (void)snprintf(
            line,
            sizeof(line),
            "%llu,%llu,0x%03X,0x%08X,%u,%u,%u,%u\n",
            (unsigned long long)records[i].seq,
            (unsigned long long)records[i].timestamp_ns,
            records[i].can_id,
            records[i].logical_addr,
            records[i].predicted,
            records[i].corrected,
            records[i].xor_diff,
            records[i].feature_hash);

        (void)write_all(fd,
                        line,
                        strlen(line));
    }

    (void)fsync(fd);
    close(fd);

    gzip_file(path);

    (void)snprintf(gz_path,
                   sizeof(gz_path),
                   "%s.gz",
                   path);

    if (access(gz_path, F_OK) == 0) {
        (void)snprintf(final_path,
                       sizeof(final_path),
                       ERROR_DIR "/error_batch_%06llu.csv.gz",
                       (unsigned long long)g_archive_seq);

        (void)rename(gz_path,
                     final_path);
    } else {
        (void)snprintf(final_path,
                       sizeof(final_path),
                       ERROR_DIR "/error_batch_%06llu.csv",
                       (unsigned long long)g_archive_seq);

        (void)rename(path,
                     final_path);
    }

    pthread_mutex_lock(&g_stats.lock);
    g_stats.error_archived += count;
    pthread_mutex_unlock(&g_stats.lock);

    ++g_archive_seq;
}

static void write_latest_corrected(const prediction_record_t *record)
{
    char tmp_path[256];
    char final_path[256];
    char json[768];

    int fd;

    (void)snprintf(
        tmp_path,
        sizeof(tmp_path),
        LATEST_DIR "/corrected_latest.tmp");

    (void)snprintf(
        final_path,
        sizeof(final_path),
        LATEST_DIR "/corrected_latest.json");

    fd = open(tmp_path,
              O_WRONLY | O_CREAT | O_TRUNC,
              0666);

    if (fd < 0) {
        return;
    }

    (void)snprintf(
        json,
        sizeof(json),
        "{\n"
        "  \"sequence\": %llu,\n"
        "  \"timestamp_ns\": %llu,\n"
        "  \"can_id\": \"0x%03X\",\n"
        "  \"logical_address\": \"0x%08X\",\n"
        "  \"predicted\": %u,\n"
        "  \"corrected\": %u,\n"
        "  \"xor_diff\": %u,\n"
        "  \"feature_hash\": %u\n"
        "}\n",
        (unsigned long long)record->seq,
        (unsigned long long)record->timestamp_ns,
        record->can_id,
        record->logical_addr,
        record->predicted,
        record->corrected,
        record->xor_diff,
        record->feature_hash);

    (void)write_all(fd,
                    json,
                    strlen(json));

    (void)fsync(fd);
    close(fd);

    (void)rename(tmp_path,
                 final_path);
}

/* ========================================================================== */
/* THREAD 5: Error archive P5                                                 */
/* ========================================================================== */

static void *archive_thread(void *arg)
{
    (void)arg;

    pin_current_thread(2U);

    while (g_running || error_size() != 0U) {
        prediction_record_t batch[ERROR_BATCH];
        unsigned count;

        /*
         * ERROR DATA ALWAYS DRAINS FIRST.
         */
        count =
            error_take_batch(batch,
                             ERROR_BATCH);

        if (count != 0U) {
            archive_error_batch(batch,
                                count);
            continue;
        }

        /*
         * Correct history is deliberately not accumulated.
         * Only the newest corrected snapshot is written.
         */
        {
            prediction_record_t latest;

            if (latest_corrected_snapshot(&latest)) {
                write_latest_corrected(&latest);
            }
        }

        usleep(ARCHIVE_PERIOD_MS * 1000U);
    }

    /* Final error drain on clean shutdown. */
    while (1) {
        prediction_record_t batch[ERROR_BATCH];
        unsigned count =
            error_take_batch(batch,
                             ERROR_BATCH);

        if (count == 0U) {
            break;
        }

        archive_error_batch(batch,
                            count);
    }

    return NULL;
}

/* ========================================================================== */
/* CPU metrics                                                                */
/* ========================================================================== */

static double thread_cpu_percent(pthread_t tid,
                                  uint64_t *last_cpu_ns,
                                  uint64_t wall_delta_ns)
{
    clockid_t clock_id;
    struct timespec ts;

    uint64_t current_cpu_ns;
    uint64_t delta_cpu_ns;

    if (pthread_getcpuclockid(tid,
                              &clock_id) != 0) {
        return 0.0;
    }

    if (clock_gettime(clock_id,
                      &ts) != 0) {
        return 0.0;
    }

    current_cpu_ns =
        ((uint64_t)ts.tv_sec * 1000000000ULL) +
        (uint64_t)ts.tv_nsec;

    if (*last_cpu_ns == 0U) {
        *last_cpu_ns = current_cpu_ns;
        return 0.0;
    }

    delta_cpu_ns =
        current_cpu_ns - *last_cpu_ns;

    *last_cpu_ns = current_cpu_ns;

    if (wall_delta_ns == 0U) {
        return 0.0;
    }

    {
        double pct =
            ((double)delta_cpu_ns /
             (double)wall_delta_ns) * 100.0;

        if (pct < 0.0) {
            pct = 0.0;
        }

        if (pct > 100.0) {
            pct = 100.0;
        }

        return pct;
    }
}

/* ========================================================================== */
/* THREAD 6: Diagnostics P15                                                  */
/* ========================================================================== */

static void *diagnostics_thread(void *arg)
{
    (void)arg;

    uint64_t last_wall_ns = now_ns();

    uint64_t cpu_safety = 0U;
    uint64_t cpu_tx = 0U;
    uint64_t cpu_rx = 0U;
    uint64_t cpu_archive = 0U;
    uint64_t cpu_noise = 0U;

    pin_current_thread(3U);

    while (g_running) {
        uint64_t wall_now_ns;
        uint64_t wall_delta_ns;

        double safety_cpu;
        double tx_cpu;
        double rx_cpu;
        double archive_cpu;
        double noise_cpu;

        uint64_t rx_total;
        uint64_t tx_total;

        uint64_t safety_sum;
        uint64_t safety_min;
        uint64_t safety_max;
        uint64_t safety_count;

        uint64_t deadline_misses;
        uint64_t wrong_predictions;
        uint64_t correct_predictions;
        uint64_t error_archived;
        uint64_t error_overflow;

        uint64_t rx_delta;
        uint64_t tx_delta;

        unsigned safety_ingress_count;
        unsigned safety_tx_count;
        unsigned normal_count;
        unsigned telemetry_count;
        unsigned error_count;

        double rx_fps;
        double tx_fps;
        double busload_pct;
        double avg_latency_ms;
        double jitter_ms;

        double core0_app;
        double core1_app;
        double core2_app;
        double core3_app;

        sleep(1);

        wall_now_ns = now_ns();

        wall_delta_ns =
            wall_now_ns - last_wall_ns;

        safety_cpu =
            thread_cpu_percent(g_th_safety,
                               &cpu_safety,
                               wall_delta_ns);

        tx_cpu =
            thread_cpu_percent(g_th_tx,
                               &cpu_tx,
                               wall_delta_ns);

        rx_cpu =
            thread_cpu_percent(g_th_rx,
                               &cpu_rx,
                               wall_delta_ns);

        archive_cpu =
            thread_cpu_percent(g_th_archive,
                               &cpu_archive,
                               wall_delta_ns);

        noise_cpu =
            thread_cpu_percent(g_th_noise,
                               &cpu_noise,
                               wall_delta_ns);

        pthread_mutex_lock(&g_stats.lock);

        rx_total = g_stats.rx_frames;
        tx_total = g_stats.tx_frames;

        safety_sum =
            g_stats.safety_latency_sum_ns;

        safety_min =
            g_stats.safety_latency_min_ns;

        safety_max =
            g_stats.safety_latency_max_ns;

        safety_count =
            g_stats.safety_frames;

        deadline_misses =
            g_stats.deadline_misses;

        wrong_predictions =
            g_stats.prediction_wrong;

        correct_predictions =
            g_stats.prediction_correct;

        error_archived =
            g_stats.error_archived;

        error_overflow =
            g_stats.error_overflow;

        pthread_mutex_unlock(&g_stats.lock);

        rx_delta =
            rx_total - g_stats.last_rx;

        tx_delta =
            tx_total - g_stats.last_tx;

        rx_fps = (double)rx_delta;
        tx_fps = (double)tx_delta;

        /*
         * Estimated occupancy:
         *     TX FPS * approximate bits/frame / nominal bitrate.
         *
         * This is NOT a physical bus analyzer measurement.
         */
        busload_pct =
            (tx_fps *
             (double)CAN_APPROX_BITS_8BYTE *
             100.0) /
            (double)CAN_BITRATE;

        if (busload_pct > 100.0) {
            busload_pct = 100.0;
        }

        avg_latency_ms = 0.0;

        if (safety_count != 0U) {
            avg_latency_ms =
                ns_to_ms(safety_sum /
                         safety_count);
        }

        if (safety_min == UINT64_MAX ||
            safety_max < safety_min) {
            jitter_ms = 0.0;
        } else {
            jitter_ms =
                ns_to_ms(safety_max -
                         safety_min);
        }

        core0_app =
            safety_cpu + tx_cpu;

        core1_app =
            rx_cpu;

        core2_app =
            archive_cpu + noise_cpu;

        /*
         * Diagnostics itself is the P15 thread.
         * Its CPU contribution is measured by this thread's own CPU clock
         * on the next interval, but it is kept separate from the other
         * roles for presentation clarity.
         */
        core3_app = 0.0;

        if (core0_app > 100.0) core0_app = 100.0;
        if (core1_app > 100.0) core1_app = 100.0;
        if (core2_app > 100.0) core2_app = 100.0;

        safety_ingress_count =
            safety_ingress_size();

        safety_tx_count =
            safety_tx_size();

        normal_count =
            normal_size();

        telemetry_count =
            telemetry_size();

        error_count =
            error_size();

        printf(
            "\n"
            "================ REAL-TIME GATEWAY ================\n"
            "RX FPS                       : %.0f\n"
            "TX FPS                       : %.0f\n"
            "Estimated CAN load           : %.1f %%\n"
            "Safety latency min/avg/max   : %.3f / %.3f / %.3f ms\n"
            "Safety jitter                : %.3f ms\n"
            "Deadline misses > %.1f ms   : %llu\n"
            "Queues S_in/S_tx/N/T/E       : %u/%u/%u/%u/%u\n"
            "Wrong predictions            : %llu\n"
            "Correct predictions          : %llu (latest only)\n"
            "Wrong records archived      : %llu\n"
            "Error-ring overflow          : %llu\n"
            "CPU0 Safety+TX               : %.1f %%\n"
            "CPU1 CAN RX                  : %.1f %%\n"
            "CPU2 Archive+Noise           : %.1f %%\n"
            "CPU3 Diagnostics             : %.1f %%\n"
            "====================================================\n",
            rx_fps,
            tx_fps,
            busload_pct,
            safety_min == UINT64_MAX ?
                0.0 : ns_to_ms(safety_min),
            avg_latency_ms,
            safety_max == 0U ?
                0.0 : ns_to_ms(safety_max),
            jitter_ms,
            SAFETY_DEADLINE_MS,
            (unsigned long long)deadline_misses,
            safety_ingress_count,
            safety_tx_count,
            normal_count,
            telemetry_count,
            error_count,
            (unsigned long long)wrong_predictions,
            (unsigned long long)correct_predictions,
            (unsigned long long)error_archived,
            (unsigned long long)error_overflow,
            core0_app,
            core1_app,
            core2_app,
            core3_app);

        if (g_udp_sock >= 0) {
            char json[1024];

            (void)snprintf(
                json,
                sizeof(json),
                "{"
                "\"rx_fps\":%.0f,"
                "\"tx_fps\":%.0f,"
                "\"busload\":%.1f,"
                "\"safety_latency_ms\":%.3f,"
                "\"safety_jitter_ms\":%.3f,"
                "\"deadline_misses\":%llu,"
                "\"q_safety_ingress\":%u,"
                "\"q_safety_tx\":%u,"
                "\"q_normal\":%u,"
                "\"q_telemetry\":%u,"
                "\"q_errors\":%u,"
                "\"wrong_predictions\":%llu,"
                "\"correct_predictions\":%llu,"
                "\"errors_archived\":%llu,"
                "\"error_overflow\":%llu,"
                "\"cpu0\":%.1f,"
                "\"cpu1\":%.1f,"
                "\"cpu2\":%.1f,"
                "\"cpu3\":%.1f"
                "}\n",
                rx_fps,
                tx_fps,
                busload_pct,
                avg_latency_ms,
                jitter_ms,
                (unsigned long long)deadline_misses,
                safety_ingress_count,
                safety_tx_count,
                normal_count,
                telemetry_count,
                error_count,
                (unsigned long long)wrong_predictions,
                (unsigned long long)correct_predictions,
                (unsigned long long)error_archived,
                (unsigned long long)error_overflow,
                core0_app,
                core1_app,
                core2_app,
                core3_app);

            (void)sendto(
                g_udp_sock,
                json,
                strlen(json),
                0,
                (struct sockaddr *)&g_host_addr,
                sizeof(g_host_addr));
        }

        g_stats.last_rx = rx_total;
        g_stats.last_tx = tx_total;
        last_wall_ns = wall_now_ns;
    }

    return NULL;
}

/* ========================================================================== */
/* RT thread creation                                                         */
/* ========================================================================== */

static int create_rt_thread(pthread_t *thread_id,
                            void *(*entry)(void *),
                            int priority,
                            const char *name)
{
    pthread_attr_t attr;
    struct sched_param sched_param;
    int rc;

    (void)pthread_attr_init(&attr);

    (void)pthread_attr_setinheritsched(
        &attr,
        PTHREAD_EXPLICIT_SCHED);

    (void)pthread_attr_setschedpolicy(
        &attr,
        SCHED_FIFO);

    memset(&sched_param,
           0,
           sizeof(sched_param));

    sched_param.sched_priority =
        priority;

    (void)pthread_attr_setschedparam(
        &attr,
        &sched_param);

    rc = pthread_create(thread_id,
                        &attr,
                        entry,
                        NULL);

    (void)pthread_attr_destroy(&attr);

    if (rc != 0) {
        fprintf(stderr,
                "[-] pthread_create %s P%d: %s\n",
                name,
                priority,
                strerror(rc));
        return rc;
    }

    printf("[+] %s: SCHED_FIFO P%d\n",
           name,
           priority);

    return 0;
}

/* ========================================================================== */
/* UDP telemetry                                                              */
/* ========================================================================== */

static void init_udp(void)
{
    int broadcast = 1;

    g_udp_sock =
        socket(AF_INET,
               SOCK_DGRAM,
               0);

    if (g_udp_sock < 0) {
        fprintf(stderr,
                "[WARN] UDP socket failed: %s\n",
                strerror(errno));
        return;
    }

    (void)setsockopt(
        g_udp_sock,
        SOL_SOCKET,
        SO_BROADCAST,
        &broadcast,
        sizeof(broadcast));

    memset(&g_host_addr,
           0,
           sizeof(g_host_addr));

    g_host_addr.sin_family =
        AF_INET;

    g_host_addr.sin_port =
        htons(UDP_PORT);

    g_host_addr.sin_addr.s_addr =
        inet_addr(UDP_BROADCAST);
}

/* ========================================================================== */
/* Architecture display                                                       */
/* ========================================================================== */

static void print_architecture(void)
{
    size_t safety_ingress_bytes =
        sizeof(g_q_safety_ingress.ring);

    size_t safety_tx_bytes =
        sizeof(g_q_safety_tx.ring);

    size_t normal_bytes =
        sizeof(g_q_normal.ring);

    size_t telemetry_bytes =
        sizeof(g_q_telemetry.ring);

    size_t error_bytes =
        sizeof(g_q_errors.ring);

    size_t corrected_bytes =
        sizeof(g_latest_corrected.slot);

    size_t total_fixed_bytes =
        safety_ingress_bytes +
        safety_tx_bytes +
        normal_bytes +
        telemetry_bytes +
        error_bytes +
        corrected_bytes;

    printf(
        "\n"
        "================ MEMORY ARCHITECTURE ================\n"
        "DATA BUS:\n"
        "  CAN payload                    : 8 bytes/frame\n"
        "  Prediction record              : %zu bytes\n"
        "\n"
        "ADDRESS BUS (logical):\n"
        "  Safety ingress                 : 0x%04X\n"
        "  Safety TX                      : 0x%04X\n"
        "  Normal                         : 0x%04X\n"
        "  Telemetry                      : 0x%04X\n"
        "  Wrong predictions              : 0x%04X\n"
        "  Latest corrected               : 0x%04X\n"
        "\n"
        "MEMORY BUS:\n"
        "  Mutex-protected fixed banks\n"
        "  No RT heap allocation\n"
        "\n"
        "FIXED STORAGE:\n"
        "  Safety ingress                 : %zu bytes\n"
        "  Safety TX                      : %zu bytes\n"
        "  Normal                         : %zu bytes\n"
        "  Telemetry                      : %zu bytes\n"
        "  Wrong predictions              : %zu bytes\n"
        "  Corrected double buffer        : %zu bytes\n"
        "  Total fixed record storage     : %zu bytes\n"
        "\n"
        "RETENTION POLICY:\n"
        "  Wrong prediction               : HIGH / historical\n"
        "  Correct prediction             : LATEST ONLY\n"
        "======================================================\n",
        sizeof(prediction_record_t),
        BASE_SAFETY_INGRESS,
        BASE_SAFETY_TX,
        BASE_NORMAL,
        BASE_TELEMETRY,
        BASE_ERRORS,
        BASE_CORRECTED,
        safety_ingress_bytes,
        safety_tx_bytes,
        normal_bytes,
        telemetry_bytes,
        error_bytes,
        corrected_bytes,
        total_fixed_bytes);
}

/* ========================================================================== */
/* Demo functions                                                             */
/* ========================================================================== */

static void demo_normal(void)
{
    can_frame_t frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = ID_FRONT_RADAR;
    frame.dlc = 8U;
    frame.timestamp_ns = now_ns();
    frame.priority_class = 1U;

    frame.data[0] = 10U;
    frame.data[1] = 20U;
    frame.data[2] = 30U;
    frame.data[3] = 40U;

    if (normal_push(&frame)) {
        printf("[DEMO] Normal frame 0x110 queued\n");
    } else {
        printf("[DEMO] Normal queue full\n");
    }
}

static void demo_safety(void)
{
    can_frame_t frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = ID_EMERGENCY_BRAKE;
    frame.dlc = 8U;
    frame.timestamp_ns = now_ns();
    frame.priority_class = 0U;

    frame.data[0] = 0xFFU;
    frame.data[1] = 0xAAU;

    if (safety_ingress_push(&frame)) {
        printf("[DEMO] SAFETY 0x010 queued\n");
    } else {
        printf("[DEMO] SAFETY queue overflow\n");
    }
}

static void demo_wrong_prediction(void)
{
    can_frame_t frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = ID_PREDICTION;
    frame.dlc = 8U;
    frame.timestamp_ns = now_ns();

    frame.data[0] = 3U; /* predicted */
    frame.data[1] = 1U; /* corrected */

    frame.data[2] = 10U;
    frame.data[3] = 20U;
    frame.data[4] = 30U;
    frame.data[5] = 40U;
    frame.data[6] = 50U;
    frame.data[7] = 60U;

    process_prediction(&frame);

    printf("[DEMO] WRONG prediction: predicted=3 corrected=1\n");
}

static void demo_correct_prediction(void)
{
    can_frame_t frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = ID_PREDICTION;
    frame.dlc = 8U;
    frame.timestamp_ns = now_ns();

    frame.data[0] = 2U; /* predicted */
    frame.data[1] = 2U; /* corrected */

    frame.data[2] = 11U;
    frame.data[3] = 22U;
    frame.data[4] = 33U;
    frame.data[5] = 44U;
    frame.data[6] = 55U;
    frame.data[7] = 66U;

    process_prediction(&frame);

    printf("[DEMO] CORRECT prediction: predicted=2 corrected=2 (latest only)\n");
}

/* ========================================================================== */
/* Shutdown                                                                   */
/* ========================================================================== */

static void stop_threads(void)
{
    g_running = 0;

    pthread_cond_broadcast(
        &g_q_safety_ingress.not_empty);

    pthread_cond_broadcast(
        &g_q_safety_tx.not_empty);

    pthread_cond_broadcast(
        &g_q_normal.not_empty);

    pthread_cond_broadcast(
        &g_q_telemetry.not_empty);

    pthread_cond_broadcast(
        &g_q_errors.not_empty);
}

/* ========================================================================== */
/* Main                                                                       */
/* ========================================================================== */

int main(void)
{
    long cpu_count_result;
    int rc;

    cpu_count_result =
        sysconf(_SC_NPROCESSORS_ONLN);

    if (cpu_count_result > 0) {
        g_cpu_count =
            (unsigned)cpu_count_result;
    }

    printf(
        "============================================================\n"
        " QNX VEHICLE GATEWAY - REAL-TIME CAN + ERROR-FIRST MEMORY\n"
        "============================================================\n"
        "CPU count                   : %u\n"
        "CAN nominal bitrate         : %u bps\n"
        "Safety deadline             : %.1f ms\n"
        "------------------------------------------------------------\n",
        g_cpu_count,
        CAN_BITRATE,
        SAFETY_DEADLINE_MS);

    (void)signal(SIGINT, signal_handler);
    (void)signal(SIGTERM, signal_handler);

    init_directories();
    init_queues();

    init_priority_inherit_mutex(
        &g_stats.lock);

    init_priority_inherit_mutex(
        &g_memory_bus_lock);

    g_stats.safety_latency_min_ns =
        UINT64_MAX;

    print_architecture();

    g_fd_can0 =
        init_can_channel(
            CAN0_DEV,
            "CAN0 / FRONT");

    g_fd_can1 =
        init_can_channel(
            CAN1_DEV,
            "CAN1 / REAR");

    if (g_fd_can0 < 0 ||
        g_fd_can1 < 0) {

        if (g_fd_can0 >= 0) {
            close(g_fd_can0);
        }

        if (g_fd_can1 >= 0) {
            close(g_fd_can1);
        }

        return EXIT_FAILURE;
    }

    init_udp();

    rc =
        create_rt_thread(
            &g_th_safety,
            safety_thread,
            50,
            "Safety task");

    if (rc != 0) {
        stop_threads();
        return EXIT_FAILURE;
    }

    rc =
        create_rt_thread(
            &g_th_tx,
            tx_arbiter_thread,
            48,
            "CAN TX arbiter");

    if (rc != 0) {
        stop_threads();
        return EXIT_FAILURE;
    }

    rc =
        create_rt_thread(
            &g_th_rx,
            rx_thread,
            40,
            "CAN RX");

    if (rc != 0) {
        stop_threads();
        return EXIT_FAILURE;
    }

    rc =
        create_rt_thread(
            &g_th_diag,
            diagnostics_thread,
            15,
            "Diagnostics");

    if (rc != 0) {
        stop_threads();
        return EXIT_FAILURE;
    }

    rc =
        create_rt_thread(
            &g_th_noise,
            noise_thread,
            10,
            "Noise generator");

    if (rc != 0) {
        stop_threads();
        return EXIT_FAILURE;
    }

    rc =
        create_rt_thread(
            &g_th_archive,
            archive_thread,
            5,
            "Error archive");

    if (rc != 0) {
        stop_threads();
        return EXIT_FAILURE;
    }

    printf(
        "\n"
        "========================= MENU =========================\n"
        "1  Normal traffic (0x110)\n"
        "2  Toggle congestion/noise (0x450)\n"
        "3  Emergency safety frame (0x010)\n"
        "4  Inject WRONG prediction (0x310)\n"
        "5  Inject CORRECT prediction (latest-only)\n"
        "6  Show memory architecture\n"
        "7  Quit\n"
        "=========================================================\n");

    while (g_running) {
        int c;

        printf("\nSelect [1-7]: ");
        fflush(stdout);

        c = getchar();

        if (c == EOF) {
            break;
        }

        if (c == '\n') {
            continue;
        }

        switch (c) {
            case '1':
                demo_normal();
                break;

            case '2':
                g_noise_mode =
                    !g_noise_mode;

                printf(
                    "[DEMO] Congestion generator: %s\n",
                    g_noise_mode ?
                        "ON" : "OFF");
                break;

            case '3':
                demo_safety();
                break;

            case '4':
                demo_wrong_prediction();
                break;

            case '5':
                demo_correct_prediction();
                break;

            case '6':
                print_architecture();
                break;

            case '7':
                g_running = 0;
                break;

            default:
                printf("[DEMO] Invalid option\n");
                break;
        }

        while (c != '\n' && c != EOF) {
            c = getchar();
        }
    }

    printf("\n[+] Shutting down gateway...\n");

    stop_threads();

    (void)pthread_join(g_th_safety, NULL);
    (void)pthread_join(g_th_tx, NULL);
    (void)pthread_join(g_th_rx, NULL);
    (void)pthread_join(g_th_noise, NULL);
    (void)pthread_join(g_th_archive, NULL);
    (void)pthread_join(g_th_diag, NULL);

    if (g_fd_can0 >= 0) {
        close(g_fd_can0);
    }

    if (g_fd_can1 >= 0) {
        close(g_fd_can1);
    }

    if (g_udp_sock >= 0) {
        close(g_udp_sock);
    }

    printf(
        "[+] Gateway stopped.\n"
        "[+] Latest correct file : %s/corrected_latest.json\n"
        "[+] Wrong history       : %s/\n",
        LATEST_DIR,
        ERROR_DIR);

    return EXIT_SUCCESS;
}
