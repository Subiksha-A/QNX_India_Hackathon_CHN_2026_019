/*
 * qnx_master_code.c
 *
 * QNX 8.x / Raspberry Pi 4 vehicle gateway master
 *
 * THIS VERSION IS MATCHED TO THE PROVIDED FRDM-MCXA156 NXP NODE.
 *
 * NXP node protocol taken from its source:
 *   0x111 Temperature
 *   0x112 Humidity
 *   0x113 Speed
 *   0x114 Battery voltage
 *   0x115 Battery current
 *   0x116 Ultrasonic distance
 *   0x117 IR obstacle status
 *
 * Payload for 0x111..0x116:
 *   byte 0 = value high byte, value = raw / 100
 *   byte 1 = value low byte
 *   byte 2 = sensor type
 *   byte 3 = node ID
 *   byte 4..7 = reserved
 *
 * Payload for 0x117:
 *   byte 0 = obstacle: 0 clear, 1 obstacle
 *   byte 2 = 0x07
 *   byte 3 = node ID
 *
 * QNX -> NXP command protocol:
 *   0x201 = Node 1 command
 *   0x202 = Node 2 command
 *   0x200 = broadcast command
 *   byte 0 = 0x01 START, 0x00 STOP
 *
 * NXP node uses:
 *   CAN = 500 kbps
 *   sensor loop = 500 ms
 *
 * Gateway functions:
 *   - Receive/decode NXP sensor frames on CAN0.
 *   - Forward normal telemetry to CAN1.
 *   - Detect safety conditions from ultrasonic/IR.
 *   - Send high-priority STOP command 0x201 back to Node 1.
 *   - Maintain fixed-size memory banks and logical addresses.
 *   - Run a lightweight one-step trend predictor at the gateway.
 *   - WRONG prediction -> retain history in a fixed error ring and archive.
 *   - CORRECT prediction -> retain ONLY the latest corrected snapshot.
 *   - Archive runs at P5 and never in the safety path.
 *   - Congestion generator creates low-priority 0x450 traffic on CAN0.
 *   - QNX SCHED_FIFO + runmask/core display + CPU-time monitoring.
 *
 * DATA / ADDRESS / MEMORY BUS MODEL:
 *   The memory banks printed by this application are a logical software
 *   architecture used for deterministic allocation and hackathon evaluation.
 *   They are not physical Raspberry Pi CPU buses.
 *
 * IMPORTANT:
 *   DATA BUS / ADDRESS BUS / MEMORY BUS below are logical software
 *   architecture abstractions for the hackathon. They are not physical
 *   Raspberry Pi CPU buses.
 *
 * Build on QNX:
 *   qcc -Wall -Wextra -O2 qnx_master_code.c -o qnx_master -lsocket
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
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/neutrino.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <devctl.h>
#include <hw/io-spi.h>

/* ========================================================================= */
/* CAN / gateway configuration                                               */
/* ========================================================================= */

#define CAN0_DEV                    "/dev/io-spi/spi0/dev0"
#define CAN1_DEV                    "/dev/io-spi/spi0/dev1"

#define CAN_BITRATE                 500000U
#define CAN_APPROX_BITS_8BYTE       111U

#define UDP_PORT                    8080
#define UDP_BROADCAST               "169.254.255.255"

#define RX_POLL_US                  500U
#define TX_IDLE_US                  100U
#define ARCHIVE_PERIOD_MS           100U

#define SAFETY_DEADLINE_MS          5.0

/* NXP node protocol */
#define NODE1_ID                    1U
#define NODE1_COMMAND_ID            0x201U
#define NODE2_COMMAND_ID            0x202U
#define BROADCAST_COMMAND_ID       0x200U

#define CMD_STOP                    0x00U
#define CMD_START                   0x01U

/* Sensor IDs from the supplied NXP code */
#define ID_TEMPERATURE              0x111U
#define ID_HUMIDITY                 0x112U
#define ID_SPEED                    0x113U
#define ID_BATTERY_VOLTAGE          0x114U
#define ID_BATTERY_CURRENT          0x115U
#define ID_ULTRASONIC               0x116U
#define ID_IR                       0x117U

/* Gateway-only congestion traffic */
#define ID_CONGESTION_NOISE         0x450U

/* The NXP node uses 500 ms sensor transmission period. */
#define SENSOR_PERIOD_MS            500U

/* Safety thresholds used by the gateway. Tune for the physical demo. */
#define ULTRASONIC_SAFETY_CM        50.0
#define IR_OBSTACLE_IS_SAFETY       1

/* Fixed memory sizes */
#define ERROR_Q_SIZE                512U
#define ERROR_BATCH_SIZE            32U
#define NORMAL_Q_SIZE               128U
#define SAFETY_Q_SIZE               64U
#define NOISE_Q_SIZE                256U

/* ========================================================================= */
/* Logical memory map                                                       */
/* ========================================================================= */

#define ADDR_SENSOR_TEMP             0x1000U
#define ADDR_SENSOR_HUM              0x1010U
#define ADDR_SENSOR_SPEED            0x1020U
#define ADDR_SENSOR_VOLT             0x1030U
#define ADDR_SENSOR_CURR             0x1040U
#define ADDR_SENSOR_ULTRASONIC       0x1050U
#define ADDR_SENSOR_IR               0x1060U

#define ADDR_SAFETY_INGRESS          0x0800U
#define ADDR_SAFETY_TX               0x0900U
#define ADDR_NORMAL_QUEUE             0x2000U
#define ADDR_NOISE_QUEUE              0x2500U
#define ADDR_ERROR_BANK               0x3000U
#define ADDR_LATEST_CORRECTED         0x5000U

/* ========================================================================= */
/* MCP2515                                                                     */
/* ========================================================================= */

#define MCP_RESET                    0xC0U
#define MCP_READ                     0x03U
#define MCP_WRITE                    0x02U
#define MCP_BITMOD                   0x05U

#define REG_CANSTAT                  0x0EU
#define REG_CANCTRL                  0x0FU
#define REG_CNF1                     0x2AU
#define REG_CNF2                     0x29U
#define REG_CNF3                     0x28U

#define REG_CANINTF                 0x2CU
#define REG_CANINTE                  0x2BU

#define REG_RXB0CTRL                 0x60U
#define REG_RXB0SIDH                 0x61U
#define REG_RXB1CTRL                 0x70U
#define REG_RXB1SIDH                 0x71U

#define REG_RXM0SIDH                 0x20U
#define REG_RXM0SIDL                 0x21U
#define REG_RXM1SIDH                 0x24U
#define REG_RXM1SIDL                 0x25U

#define REG_TXB0CTRL                 0x30U
#define REG_TXB0SIDH                 0x31U
#define REG_TXB1CTRL                 0x40U
#define REG_TXB1SIDH                 0x41U
#define REG_TXB2CTRL                 0x50U
#define REG_TXB2SIDH                 0x51U

#define MAX_SPI_BYTES                32U

/* ========================================================================= */
/* Data records                                                              */
/* ========================================================================= */

typedef enum {
    SENSOR_TEMP = 0,
    SENSOR_HUM,
    SENSOR_SPEED,
    SENSOR_VOLT,
    SENSOR_CURR,
    SENSOR_ULTRASONIC,
    SENSOR_IR,
    SENSOR_COUNT
} sensor_type_t;

typedef struct {
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  data[8];
    uint64_t rx_timestamp_ns;

    uint32_t logical_addr;

    uint8_t node_id;
    uint8_t sensor_type;
} gateway_can_frame_t;

typedef struct {
    bool valid;
    uint8_t node_id;
    sensor_type_t sensor;
    float actual_value;
    float predicted_value;
    float error_value;
    float tolerance;

    uint64_t timestamp_ns;
    uint32_t can_id;
    uint32_t logical_addr;

    uint32_t cycle;
} prediction_record_t;

typedef struct {
    bool valid;
    uint8_t node_id;
    sensor_type_t sensor;
    float latest_value;
    uint64_t timestamp_ns;
    uint32_t can_id;
    uint32_t logical_addr;
} sensor_state_t;

/* ========================================================================= */
/* Fixed-memory queues                                                       */
/* ========================================================================= */

typedef struct {
    gateway_can_frame_t ring[SAFETY_Q_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} safety_queue_t;

typedef struct {
    gateway_can_frame_t ring[SAFETY_Q_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} safety_tx_queue_t;

typedef struct {
    gateway_can_frame_t ring[NORMAL_Q_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} normal_queue_t;

typedef struct {
    gateway_can_frame_t ring[NOISE_Q_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    pthread_mutex_t lock;
} noise_queue_t;

typedef struct {
    prediction_record_t ring[ERROR_Q_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} error_queue_t;

/* Only the newest correct prediction is retained. */
typedef struct {
    prediction_record_t slot[2];
    unsigned active;
    bool valid;
    bool dirty;
    pthread_mutex_t lock;
} latest_corrected_t;

/* ========================================================================= */
/* Statistics                                                                */
/* ========================================================================= */

typedef struct {
    pthread_mutex_t lock;

    uint64_t rx_frames;
    uint64_t tx_frames;
    uint64_t forwarded_frames;

    uint64_t sensor_frames;
    uint64_t safety_events;
    uint64_t safety_commands;

    uint64_t safety_deadline_misses;
    uint64_t safety_latency_sum_ns;
    uint64_t safety_latency_min_ns;
    uint64_t safety_latency_max_ns;

    uint64_t prediction_total;
    uint64_t prediction_wrong;
    uint64_t prediction_correct;
    uint64_t wrong_archived;
    uint64_t error_overflow;

    uint64_t dropped_noise;
    uint64_t tx_busy;
    uint64_t tx_failures;

    uint64_t memory_bus_writes;

    uint64_t last_rx;
    uint64_t last_tx;
} gateway_stats_t;

/* ========================================================================= */
/* Globals                                                                   */
/* ========================================================================= */

static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_congestion = 0;

static int g_fd_can0 = -1;
static int g_fd_can1 = -1;
static int g_udp_sock = -1;

static struct sockaddr_in g_dashboard;

static pthread_t g_th_safety;
static pthread_t g_th_tx;
static pthread_t g_th_rx;
static pthread_t g_th_archive;
static pthread_t g_th_noise;
static pthread_t g_th_diag;

static safety_queue_t g_q_safety;
static safety_tx_queue_t g_q_safety_tx;
static normal_queue_t g_q_normal;
static noise_queue_t g_q_noise;
static error_queue_t g_q_errors;

static latest_corrected_t g_latest_corrected;

static sensor_state_t g_sensor_state[SENSOR_COUNT];

static gateway_stats_t g_stats;

static pthread_mutex_t g_memory_bus_lock;

static uint64_t g_prediction_sequence = 1U;
static uint64_t g_archive_sequence = 1U;

static unsigned g_cpu_count = 1U;

static bool g_safety_latched = false;

/* Previous values for the lightweight gateway predictor. */
static float g_prev_value[SENSOR_COUNT];
static float g_last_value[SENSOR_COUNT];
static bool g_have_prev[SENSOR_COUNT];

/* ========================================================================= */
/* Utility functions                                                         */
/* ========================================================================= */

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

static void mkdir_if_needed(const char *path)
{
    if (mkdir(path, 0777) != 0 && errno != EEXIST) {
        fprintf(stderr,
                "[WARN] mkdir(%s): %s\n",
                path,
                strerror(errno));
    }
}

static void init_directories(void)
{
    mkdir_if_needed("/tmp/qnx_master");
    mkdir_if_needed("/tmp/qnx_master/errors");
    mkdir_if_needed("/tmp/qnx_master/errors/archive");
    mkdir_if_needed("/tmp/qnx_master/corrected");
    mkdir_if_needed("/tmp/qnx_master/staging");
}

/* ========================================================================= */
/* CPU affinity                                                              */
/* ========================================================================= */

static void pin_current_thread(unsigned cpu)
{
    uint64_t runmask;

    if (g_cpu_count == 0U) {
        return;
    }

    cpu %= g_cpu_count;

    runmask = 1ULL << cpu;

    /*
     * QNX _NTO_TCTL_RUNMASK expects the runmask value passed through
     * the data argument. This follows the QNX API form.
     */
    if (ThreadCtl(_NTO_TCTL_RUNMASK,
                  (void *)runmask) != 0) {

        fprintf(stderr,
                "[WARN] ThreadCtl CPU%u failed: %s\n",
                cpu,
                strerror(errno));
    }
}

/* ========================================================================= */
/* RT synchronization                                                        */
/* ========================================================================= */

static void init_rt_mutex(pthread_mutex_t *mutex)
{
    pthread_mutexattr_t attr;

    (void)pthread_mutexattr_init(&attr);

    /*
     * Priority inheritance prevents a lower-priority thread holding
     * a shared queue lock from unnecessarily blocking a higher-priority
     * real-time thread.
     */
    (void)pthread_mutexattr_setprotocol(
        &attr,
        PTHREAD_PRIO_INHERIT);

    (void)pthread_mutex_init(
        mutex,
        &attr);

    (void)pthread_mutexattr_destroy(&attr);
}

static void init_mono_cond(pthread_cond_t *cond)
{
    pthread_condattr_t attr;

    (void)pthread_condattr_init(&attr);

    (void)pthread_condattr_setclock(
        &attr,
        CLOCK_MONOTONIC);

    (void)pthread_cond_init(
        cond,
        &attr);

    (void)pthread_condattr_destroy(&attr);
}

/* ========================================================================= */
/* Queue initialization                                                      */
/* ========================================================================= */

static void init_queues(void)
{
    memset(&g_q_safety, 0, sizeof(g_q_safety));
    memset(&g_q_safety_tx, 0, sizeof(g_q_safety_tx));
    memset(&g_q_normal, 0, sizeof(g_q_normal));
    memset(&g_q_noise, 0, sizeof(g_q_noise));
    memset(&g_q_errors, 0, sizeof(g_q_errors));

    init_rt_mutex(&g_q_safety.lock);
    init_rt_mutex(&g_q_safety_tx.lock);
    init_rt_mutex(&g_q_normal.lock);
    init_rt_mutex(&g_q_noise.lock);
    init_rt_mutex(&g_q_errors.lock);

    init_mono_cond(&g_q_safety.not_empty);
    init_mono_cond(&g_q_safety_tx.not_empty);
    init_mono_cond(&g_q_normal.not_empty);
    init_mono_cond(&g_q_errors.not_empty);

    memset(&g_latest_corrected,
           0,
           sizeof(g_latest_corrected));

    init_rt_mutex(&g_latest_corrected.lock);

    memset(g_sensor_state,
           0,
           sizeof(g_sensor_state));

    memset(g_prev_value,
           0,
           sizeof(g_prev_value));

    memset(g_last_value,
           0,
           sizeof(g_last_value));

    memset(g_have_prev,
           0,
           sizeof(g_have_prev));
}

/* ========================================================================= */
/* Queue size helpers                                                        */
/* ========================================================================= */

static unsigned safety_size(void)
{
    unsigned n;

    pthread_mutex_lock(&g_q_safety.lock);
    n = g_q_safety.count;
    pthread_mutex_unlock(&g_q_safety.lock);

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

static unsigned noise_size(void)
{
    unsigned n;

    pthread_mutex_lock(&g_q_noise.lock);
    n = g_q_noise.count;
    pthread_mutex_unlock(&g_q_noise.lock);

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

/* ========================================================================= */
/* Safety queue                                                              */
/* ========================================================================= */

static bool safety_push(const gateway_can_frame_t *frame)
{
    bool ok = false;
    gateway_can_frame_t copy;

    pthread_mutex_lock(&g_q_safety.lock);

    if (g_q_safety.count < SAFETY_Q_SIZE) {
        copy = *frame;

        copy.logical_addr =
            ADDR_SAFETY_INGRESS +
            ((uint32_t)g_q_safety.tail *
             (uint32_t)sizeof(gateway_can_frame_t));

        g_q_safety.ring[g_q_safety.tail] = copy;

        g_q_safety.tail =
            (uint16_t)((g_q_safety.tail + 1U) %
                       SAFETY_Q_SIZE);

        ++g_q_safety.count;

        pthread_cond_signal(
            &g_q_safety.not_empty);

        ok = true;
    }

    pthread_mutex_unlock(&g_q_safety.lock);

    return ok;
}

static bool safety_pop(gateway_can_frame_t *frame)
{
    bool ok = false;

    pthread_mutex_lock(&g_q_safety.lock);

    if (g_q_safety.count != 0U) {

        *frame =
            g_q_safety.ring[g_q_safety.head];

        g_q_safety.head =
            (uint16_t)((g_q_safety.head + 1U) %
                       SAFETY_Q_SIZE);

        --g_q_safety.count;

        ok = true;
    }

    pthread_mutex_unlock(&g_q_safety.lock);

    return ok;
}

static bool safety_tx_push(const gateway_can_frame_t *frame)
{
    bool ok = false;
    gateway_can_frame_t copy;

    pthread_mutex_lock(&g_q_safety_tx.lock);

    if (g_q_safety_tx.count < SAFETY_Q_SIZE) {

        copy = *frame;

        copy.logical_addr =
            ADDR_SAFETY_TX +
            ((uint32_t)g_q_safety_tx.tail *
             (uint32_t)sizeof(gateway_can_frame_t));

        g_q_safety_tx.ring[g_q_safety_tx.tail] =
            copy;

        g_q_safety_tx.tail =
            (uint16_t)((g_q_safety_tx.tail + 1U) %
                       SAFETY_Q_SIZE);

        ++g_q_safety_tx.count;

        pthread_cond_signal(
            &g_q_safety_tx.not_empty);

        ok = true;
    }

    pthread_mutex_unlock(&g_q_safety_tx.lock);

    return ok;
}

static bool safety_tx_peek(gateway_can_frame_t *frame)
{
    bool ok = false;

    pthread_mutex_lock(&g_q_safety_tx.lock);

    if (g_q_safety_tx.count != 0U) {
        *frame =
            g_q_safety_tx.ring[g_q_safety_tx.head];

        ok = true;
    }

    pthread_mutex_unlock(&g_q_safety_tx.lock);

    return ok;
}

static bool safety_tx_pop(gateway_can_frame_t *frame)
{
    bool ok = false;

    pthread_mutex_lock(&g_q_safety_tx.lock);

    if (g_q_safety_tx.count != 0U) {

        *frame =
            g_q_safety_tx.ring[g_q_safety_tx.head];

        g_q_safety_tx.head =
            (uint16_t)((g_q_safety_tx.head + 1U) %
                       SAFETY_Q_SIZE);

        --g_q_safety_tx.count;

        ok = true;
    }

    pthread_mutex_unlock(&g_q_safety_tx.lock);

    return ok;
}

/* ========================================================================= */
/* Normal queue                                                              */
/* ========================================================================= */

static bool normal_push(const gateway_can_frame_t *frame)
{
    bool ok = false;
    gateway_can_frame_t copy;

    pthread_mutex_lock(&g_q_normal.lock);

    if (g_q_normal.count < NORMAL_Q_SIZE) {
        copy = *frame;

        copy.logical_addr =
            0x2000U +
            ((uint32_t)g_q_normal.tail *
             (uint32_t)sizeof(gateway_can_frame_t));

        g_q_normal.ring[g_q_normal.tail] = copy;

        g_q_normal.tail =
            (uint16_t)((g_q_normal.tail + 1U) %
                       NORMAL_Q_SIZE);

        ++g_q_normal.count;

        ok = true;
    }

    pthread_mutex_unlock(&g_q_normal.lock);

    return ok;
}

static bool normal_peek(gateway_can_frame_t *frame)
{
    bool ok = false;

    pthread_mutex_lock(&g_q_normal.lock);

    if (g_q_normal.count != 0U) {
        *frame =
            g_q_normal.ring[g_q_normal.head];

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
                       NORMAL_Q_SIZE);

        --g_q_normal.count;
    }

    pthread_mutex_unlock(&g_q_normal.lock);
}

/* ========================================================================= */
/* Noise queue                                                               */
/* ========================================================================= */

static bool noise_push(const gateway_can_frame_t *frame)
{
    bool ok = false;

    pthread_mutex_lock(&g_q_noise.lock);

    if (g_q_noise.count < NOISE_Q_SIZE) {
        gateway_can_frame_t copy = *frame;

        copy.logical_addr =
            0x2500U +
            ((uint32_t)g_q_noise.tail *
             (uint32_t)sizeof(gateway_can_frame_t));

        g_q_noise.ring[g_q_noise.tail] = copy;

        g_q_noise.tail =
            (uint16_t)((g_q_noise.tail + 1U) %
                       NOISE_Q_SIZE);

        ++g_q_noise.count;

        ok = true;
    }

    pthread_mutex_unlock(&g_q_noise.lock);

    return ok;
}

static bool noise_peek(gateway_can_frame_t *frame)
{
    bool ok = false;

    pthread_mutex_lock(&g_q_noise.lock);

    if (g_q_noise.count != 0U) {
        *frame =
            g_q_noise.ring[g_q_noise.head];

        ok = true;
    }

    pthread_mutex_unlock(&g_q_noise.lock);

    return ok;
}

static void noise_drop(void)
{
    pthread_mutex_lock(&g_q_noise.lock);

    if (g_q_noise.count != 0U) {
        g_q_noise.head =
            (uint16_t)((g_q_noise.head + 1U) %
                       NOISE_Q_SIZE);

        --g_q_noise.count;
    }

    pthread_mutex_unlock(&g_q_noise.lock);
}

/* ========================================================================= */
/* Error queue                                                               */
/* ========================================================================= */

static bool error_push(const prediction_record_t *record)
{
    bool ok = false;

    pthread_mutex_lock(&g_q_errors.lock);

    if (g_q_errors.count < ERROR_Q_SIZE) {
        prediction_record_t copy = *record;

        copy.logical_addr =
            ADDR_ERROR_BANK +
            ((uint32_t)g_q_errors.tail *
             (uint32_t)sizeof(prediction_record_t));

        g_q_errors.ring[g_q_errors.tail] =
            copy;

        g_q_errors.tail =
            (uint16_t)((g_q_errors.tail + 1U) %
                       ERROR_Q_SIZE);

        ++g_q_errors.count;

        pthread_cond_signal(
            &g_q_errors.not_empty);

        ok = true;
    }

    pthread_mutex_unlock(&g_q_errors.lock);

    if (!ok) {
        pthread_mutex_lock(&g_stats.lock);
        ++g_stats.error_overflow;
        pthread_mutex_unlock(&g_stats.lock);
    }

    return ok;
}

static unsigned error_take_batch(
    prediction_record_t *records,
    unsigned max_count)
{
    unsigned count = 0U;

    pthread_mutex_lock(&g_q_errors.lock);

    while (count < max_count &&
           g_q_errors.count != 0U) {

        records[count] =
            g_q_errors.ring[g_q_errors.head];

        g_q_errors.head =
            (uint16_t)((g_q_errors.head + 1U) %
                       ERROR_Q_SIZE);

        --g_q_errors.count;
        ++count;
    }

    pthread_mutex_unlock(&g_q_errors.lock);

    return count;
}

/* ========================================================================= */
/* Latest corrected double buffer                                             */
/* ========================================================================= */

static void latest_corrected_publish(
    const prediction_record_t *record)
{
    pthread_mutex_lock(
        &g_latest_corrected.lock);

    g_latest_corrected.active ^= 1U;

    g_latest_corrected.slot[
        g_latest_corrected.active] =
        *record;

    g_latest_corrected.slot[
        g_latest_corrected.active].logical_addr =
        ADDR_LATEST_CORRECTED;

    g_latest_corrected.valid = true;
    g_latest_corrected.dirty = true;

    pthread_mutex_unlock(
        &g_latest_corrected.lock);
}

static bool latest_corrected_snapshot(
    prediction_record_t *record)
{
    bool valid;
    bool dirty;

    pthread_mutex_lock(
        &g_latest_corrected.lock);

    valid = g_latest_corrected.valid;
    dirty = g_latest_corrected.dirty;

    if (valid) {
        *record =
            g_latest_corrected.slot[
                g_latest_corrected.active];
    }

    g_latest_corrected.dirty = false;

    pthread_mutex_unlock(
        &g_latest_corrected.lock);

    return valid && dirty;
}

/* ========================================================================= */
/* MCP2515 SPI                                                                */
/* ========================================================================= */

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
    spi_xchng_t *exchange;
    uint32_t total;
    int rc;

    if (len > MAX_SPI_BYTES) {
        return EINVAL;
    }

    memset(&storage,
           0,
           sizeof(storage));

    exchange =
        (spi_xchng_t *)storage.raw;

    total =
        (uint32_t)(sizeof(spi_xchng_t) +
                   len);

    exchange->nbytes =
        (uint32_t)len;

    if (tx != NULL) {
        memcpy(exchange->data,
               tx,
               len);
    }

    rc =
        devctl(fd,
               DCMD_SPI_DATA_XCHNG,
               exchange,
               total,
               NULL);

    if (rc == 0 && rx != NULL) {
        memcpy(rx,
               exchange->data,
               len);
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

    memset(&tx,
           0,
           sizeof(tx));

    memset(&rx,
           0,
           sizeof(rx));

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

/* ========================================================================= */
/* MCP2515 initialization                                                     */
/* ========================================================================= */

static int init_can_channel(const char *device,
                            const char *name)
{
    int fd;
    spi_cfg_t cfg;
    uint8_t canstat = 0U;

    fd =
        open(device,
             O_RDWR);

    if (fd < 0) {
        fprintf(stderr,
                "[-] open(%s): %s\n",
                device,
                strerror(errno));
        return -1;
    }

    memset(&cfg,
           0,
           sizeof(cfg));

    cfg.mode =
        8U | (1U << 10);

    cfg.clock_rate =
        5000000U;

    if (devctl(fd,
               DCMD_SPI_SET_CONFIG,
               &cfg,
               sizeof(cfg),
               NULL) != 0) {

        fprintf(stderr,
                "[-] SPI config failed on %s\n",
                name);

        close(fd);
        return -1;
    }

    {
        uint8_t reset_command =
            MCP_RESET;

        if (spi_xfer(fd,
                     &reset_command,
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
     * 16-MHz MCP2515 oscillator, 500-kbps nominal CAN.
     * Verify the oscillator on the actual MCP2515 boards.
     */
    (void)mcp_write(fd,
                    REG_CNF1,
                    0x01U);

    (void)mcp_write(fd,
                    REG_CNF2,
                    0x90U);

    (void)mcp_write(fd,
                    REG_CNF3,
                    0x02U);

    /* Accept all standard IDs. */
    (void)mcp_write(fd,
                    REG_RXM0SIDH,
                    0x00U);

    (void)mcp_write(fd,
                    REG_RXM0SIDL,
                    0x00U);

    (void)mcp_write(fd,
                    REG_RXM1SIDH,
                    0x00U);

    (void)mcp_write(fd,
                    REG_RXM1SIDL,
                    0x00U);

    /* RX0 rollover + RX1 accept all. */
    (void)mcp_write(fd,
                    REG_RXB0CTRL,
                    0x64U);

    (void)mcp_write(fd,
                    REG_RXB1CTRL,
                    0x60U);

    /* RX0 + RX1 interrupt flags. */
    (void)mcp_write(fd,
                    REG_CANINTE,
                    0x03U);

    /* Normal operation. */
    (void)mcp_bit_modify(fd,
                         REG_CANCTRL,
                         0xE0U,
                         0x00U);

    (void)mcp_read(fd,
                   REG_CANSTAT,
                   &canstat);

    printf("[+] %s: CANSTAT=0x%02X\n",
           name,
           canstat);

    return fd;
}

/* ========================================================================= */
/* MCP2515 RX                                                                */
/* ========================================================================= */

static int receive_can_frame(
    int fd,
    gateway_can_frame_t *frame)
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
        base_reg =
            REG_RXB0SIDH;
    } else if ((intf & 0x02U) != 0U) {
        base_reg =
            REG_RXB1SIDH;
    } else {
        return 0;
    }

    memset(&tx,
           0,
           sizeof(tx));

    memset(&rx,
           0,
           sizeof(rx));

    tx.cmd = MCP_READ;
    tx.reg = base_reg;

    if (spi_xfer(fd,
                 &tx,
                 &rx,
                 sizeof(tx)) != 0) {
        return -1;
    }

    memset(frame,
           0,
           sizeof(*frame));

    frame->can_id =
        ((uint32_t)rx.sidh << 3) |
        ((uint32_t)rx.sidl >> 5);

    frame->dlc =
        (uint8_t)(rx.dlc & 0x0FU);

    if (frame->dlc > 8U) {
        frame->dlc = 8U;
    }

    memcpy(frame->data,
           rx.data,
           frame->dlc);

    frame->rx_timestamp_ns =
        now_ns();

    if (base_reg ==
        REG_RXB0SIDH) {

        (void)mcp_bit_modify(
            fd,
            REG_CANINTF,
            0x01U,
            0x00U);
    } else {

        (void)mcp_bit_modify(
            fd,
            REG_CANINTF,
            0x02U,
            0x00U);
    }

    return 1;
}

/* ========================================================================= */
/* MCP2515 TX                                                                 */
/* ========================================================================= */

static int send_can_buffer(
    int fd,
    unsigned tx_buffer,
    const gateway_can_frame_t *frame)
{
    uint8_t ctrl_reg;
    uint8_t data_reg;
    uint8_t tx_value;
    uint8_t ctrl;

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

        ctrl_reg =
            REG_TXB0CTRL;

        data_reg =
            REG_TXB0SIDH;

        tx_value =
            0x0BU;       /* TXP=3 + TXREQ */

    } else if (tx_buffer == 1U) {

        ctrl_reg =
            REG_TXB1CTRL;

        data_reg =
            REG_TXB1SIDH;

        tx_value =
            0x0AU;       /* TXP=2 + TXREQ */

    } else {

        ctrl_reg =
            REG_TXB2CTRL;

        data_reg =
            REG_TXB2SIDH;

        tx_value =
            0x08U;       /* TXP=0 + TXREQ */
    }

    if (mcp_read(fd,
                 ctrl_reg,
                 &ctrl) != 0) {
        return EIO;
    }

    if ((ctrl & 0x08U) != 0U) {
        return EBUSY;
    }

    memset(&packet,
           0,
           sizeof(packet));

    packet.cmd = MCP_WRITE;
    packet.reg = data_reg;

    packet.sidh =
        (uint8_t)(frame->can_id >> 3);

    packet.sidl =
        (uint8_t)(frame->can_id << 5);

    packet.dlc =
        frame->dlc;

    memcpy(packet.data,
           frame->data,
           frame->dlc);

    if (spi_xfer(fd,
                 &packet,
                 NULL,
                 7U + frame->dlc) != 0) {
        return EIO;
    }

    return mcp_write(fd,
                     ctrl_reg,
                     tx_value);
}

/* ========================================================================= */
/* NXP protocol mapping                                                      */
/* ========================================================================= */

static uint32_t sensor_logical_address(
    sensor_type_t sensor)
{
    switch (sensor) {
        case SENSOR_TEMP:
            return ADDR_SENSOR_TEMP;

        case SENSOR_HUM:
            return ADDR_SENSOR_HUM;

        case SENSOR_SPEED:
            return ADDR_SENSOR_SPEED;

        case SENSOR_VOLT:
            return ADDR_SENSOR_VOLT;

        case SENSOR_CURR:
            return ADDR_SENSOR_CURR;

        case SENSOR_ULTRASONIC:
            return ADDR_SENSOR_ULTRASONIC;

        case SENSOR_IR:
            return ADDR_SENSOR_IR;

        default:
            return 0U;
    }
}

static const char *sensor_name(
    sensor_type_t sensor)
{
    switch (sensor) {
        case SENSOR_TEMP:       return "Temperature";
        case SENSOR_HUM:        return "Humidity";
        case SENSOR_SPEED:      return "Speed";
        case SENSOR_VOLT:       return "BatteryVoltage";
        case SENSOR_CURR:       return "BatteryCurrent";
        case SENSOR_ULTRASONIC: return "Ultrasonic";
        case SENSOR_IR:         return "IR";
        default:                return "Unknown";
    }
}

static bool decode_sensor_frame(
    const gateway_can_frame_t *frame,
    sensor_type_t *sensor,
    float *value,
    uint8_t *node_id)
{
    uint16_t raw;

    if (frame == NULL ||
        frame->dlc < 4U ||
        sensor == NULL ||
        value == NULL ||
        node_id == NULL) {
        return false;
    }

    *node_id =
        frame->data[3];

    switch (frame->can_id) {

        case ID_TEMPERATURE:
            *sensor = SENSOR_TEMP;
            break;

        case ID_HUMIDITY:
            *sensor = SENSOR_HUM;
            break;

        case ID_SPEED:
            *sensor = SENSOR_SPEED;
            break;

        case ID_BATTERY_VOLTAGE:
            *sensor = SENSOR_VOLT;
            break;

        case ID_BATTERY_CURRENT:
            *sensor = SENSOR_CURR;
            break;

        case ID_ULTRASONIC:
            *sensor = SENSOR_ULTRASONIC;
            break;

        case ID_IR:
            *sensor = SENSOR_IR;

            *value =
                (frame->data[0] != 0U) ?
                    1.0f : 0.0f;

            return true;

        default:
            return false;
    }

    raw =
        ((uint16_t)frame->data[0] << 8) |
        (uint16_t)frame->data[1];

    *value =
        (float)raw / 100.0f;

    return true;
}

/* ========================================================================= */
/* Prediction/error analysis                                                 */
/* ========================================================================= */

/*
 * The supplied NXP node sends sensor measurements, not prediction/correction
 * pairs. Therefore the gateway adds a lightweight one-step trend predictor.
 *
 * Prediction:
 *     predicted[n] = actual[n-1] + (actual[n-1] - actual[n-2])
 *
 * Current NXP measurement becomes the reference/actual value.
 *
 * Wrong:
 *     |actual - predicted| > tolerance
 *
 * Correct:
 *     |actual - predicted| <= tolerance
 *
 * This is a gateway-side analysis feature. It is NOT an AI model embedded
 * in the NXP node.
 */

static float sensor_tolerance(
    sensor_type_t sensor)
{
    switch (sensor) {
        case SENSOR_TEMP:
            return 0.50f;

        case SENSOR_HUM:
            return 2.0f;

        case SENSOR_SPEED:
            return 5.0f;

        case SENSOR_VOLT:
            return 1.0f;

        case SENSOR_CURR:
            return 2.0f;

        case SENSOR_ULTRASONIC:
            return 15.0f;

        case SENSOR_IR:
            return 0.0f;

        default:
            return 0.0f;
    }
}

static void update_sensor_state(
    sensor_type_t sensor,
    uint8_t node_id,
    float value,
    uint32_t can_id,
    uint64_t timestamp_ns)
{
    sensor_state_t *state;

    state =
        &g_sensor_state[sensor];

    state->valid = true;
    state->node_id = node_id;
    state->sensor = sensor;
    state->latest_value = value;
    state->timestamp_ns = timestamp_ns;
    state->can_id = can_id;
    state->logical_addr =
        sensor_logical_address(sensor);
}

static void analyze_sensor_sample(
    sensor_type_t sensor,
    uint8_t node_id,
    float actual,
    uint32_t can_id,
    uint64_t timestamp_ns)
{
    float predicted = actual;
    float error_value = 0.0f;
    float tolerance = sensor_tolerance(sensor);

    prediction_record_t record;

    memset(&record,
           0,
           sizeof(record));

    record.seq =
        g_prediction_sequence++;

    record.timestamp_ns =
        timestamp_ns;

    record.can_id =
        can_id;

    record.logical_addr =
        sensor_logical_address(sensor);

    record.node_id =
        node_id;

    record.sensor =
        sensor;

    record.actual_value =
        actual;

    record.tolerance =
        tolerance;

    /*
     * First measurement is warm-up; there isn't enough history
     * for a trend prediction yet.
     */
    if (!g_have_prev[sensor]) {

        g_prev_value[sensor] =
            actual;

        g_last_value[sensor] =
            actual;

        g_have_prev[sensor] =
            true;

        return;
    }

    predicted =
        g_last_value[sensor] +
        (g_last_value[sensor] -
         g_prev_value[sensor]);

    error_value =
        fabsf(actual -
              predicted);

    record.predicted_value =
        predicted;

    record.error_value =
        error_value;

    record.valid = true;

    /*
     * Advance fixed-size history.
     */
    g_prev_value[sensor] =
        g_last_value[sensor];

    g_last_value[sensor] =
        actual;

    pthread_mutex_lock(&g_stats.lock);
    ++g_stats.prediction_total;
    pthread_mutex_unlock(&g_stats.lock);

    if (error_value > tolerance) {

        /*
         * WRONG:
         * retain full history in the fixed error bank.
         * No filesystem work happens here.
         */
        if (error_push(&record)) {

            pthread_mutex_lock(
                &g_memory_bus_lock);

            pthread_mutex_lock(
                &g_stats.lock);

            ++g_stats.memory_bus_writes;
            ++g_stats.prediction_wrong;

            pthread_mutex_unlock(
                &g_stats.lock);

            pthread_mutex_unlock(
                &g_memory_bus_lock);
        }

    } else {

        /*
         * CORRECT:
         * retain ONLY the newest record.
         * Older correct predictions are overwritten.
         */
        latest_corrected_publish(
            &record);

        pthread_mutex_lock(
            &g_memory_bus_lock);

        pthread_mutex_lock(
            &g_stats.lock);

        ++g_stats.memory_bus_writes;
        ++g_stats.prediction_correct;

        pthread_mutex_unlock(
            &g_stats.lock);

        pthread_mutex_unlock(
            &g_memory_bus_lock);
    }
}

/* ========================================================================= */
/* Safety condition from NXP readings                                         */
/* ========================================================================= */

static void request_safety_stop(
    uint8_t node_id,
    const char *reason,
    uint64_t source_timestamp_ns)
{
    gateway_can_frame_t command;

    /*
     * Safety is latched so the same STOP command isn't generated on
     * every incoming frame.
     */
    if (g_safety_latched) {
        return;
    }

    memset(&command,
           0,
           sizeof(command));

    command.can_id =
        (node_id == 1U) ?
            NODE1_COMMAND_ID :
            NODE2_COMMAND_ID;

    command.dlc = 8U;

    command.data[0] =
        CMD_STOP;

    command.logical_addr =
        0x0800U;

    command.rx_timestamp_ns =
        source_timestamp_ns;

    command.sensor_type =
        0U;

    if (safety_push(&command)) {

        g_safety_latched = true;

        pthread_mutex_lock(
            &g_stats.lock);

        ++g_stats.safety_events;

        pthread_mutex_unlock(
            &g_stats.lock);

        printf(
            "\n[SAFETY] STOP requested for Node %u: %s\n",
            node_id,
            reason);
    }
}

static void clear_safety_latch(void)
{
    g_safety_latched = false;

    printf(
        "[SAFETY] Latch cleared; next configured hazard may issue STOP.\n");
}

/* ========================================================================= */
/* THREAD 1: Safety task P60                                                 */
/* ========================================================================= */

static void *safety_thread(void *arg)
{
    (void)arg;

    pin_current_thread(0U);

    while (g_running) {

        gateway_can_frame_t command;

        /*
         * P60 is the safety preparation stage.
         * It moves safety commands from ingress memory into the dedicated
         * TX memory bank. If the TX bank is temporarily full, the command
         * is retained locally and retried rather than silently dropped.
         */
        if (!safety_pop(&command)) {
            usleep(100U);
            continue;
        }

        command.priority_class = 0U;

        while (g_running &&
               !safety_tx_push(&command)) {

            usleep(10U);
        }
    }

    return NULL;
}

/* ========================================================================= */
/* THREAD 2: CAN TX priority arbiter P55                                     */
/* ========================================================================= */

static void *tx_thread(void *arg)
{
    (void)arg;

    pin_current_thread(0U);

    while (g_running) {

        gateway_can_frame_t frame;

        /*
         * Priority 1:
         * SAFETY -> CAN0 -> MCP2515 TXB0
         * 0x201 STOP has a lower CAN identifier than the 0x450 noise
         * frame and therefore also has the stronger native CAN arbitration
         * priority if transmitted simultaneously.
         */
        if (safety_tx_peek(&frame)) {

            int rc =
                send_can_buffer(g_fd_can0,
                                0U,
                                &frame);

            if (rc == 0) {

                uint64_t latency_ns;

                (void)safety_tx_pop(
                    &frame);

                latency_ns =
                    now_ns() -
                    frame.rx_timestamp_ns;

                pthread_mutex_lock(
                    &g_stats.lock);

                ++g_stats.tx_frames;
                ++g_stats.safety_commands;

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

                    ++g_stats.safety_deadline_misses;
                }

                pthread_mutex_unlock(
                    &g_stats.lock);

                continue;
            }

            pthread_mutex_lock(
                &g_stats.lock);

            if (rc == EBUSY) {
                ++g_stats.tx_busy;
            } else {
                ++g_stats.tx_failures;
            }

            pthread_mutex_unlock(
                &g_stats.lock);

            usleep(TX_IDLE_US);

            continue;
        }

        /*
         * Priority 2:
         * Normal NXP telemetry -> CAN1 -> TXB1
         */
        if (normal_peek(&frame)) {

            int rc =
                send_can_buffer(g_fd_can1,
                                1U,
                                &frame);

            if (rc == 0) {

                normal_drop();

                pthread_mutex_lock(
                    &g_stats.lock);

                ++g_stats.tx_frames;
                ++g_stats.forwarded_frames;

                pthread_mutex_unlock(
                    &g_stats.lock);

                continue;
            }

            if (rc != EBUSY) {

                pthread_mutex_lock(
                    &g_stats.lock);

                ++g_stats.tx_failures;

                pthread_mutex_unlock(
                    &g_stats.lock);
            }
        }

        /*
         * Priority 3:
         * Congestion/noise -> CAN0 -> TXB2
         */
        if (noise_peek(&frame)) {

            int rc =
                send_can_buffer(g_fd_can0,
                                2U,
                                &frame);

            if (rc == 0) {

                noise_drop();

                pthread_mutex_lock(
                    &g_stats.lock);

                ++g_stats.tx_frames;

                pthread_mutex_unlock(
                    &g_stats.lock);

                continue;
            }

            if (rc != EBUSY) {

                pthread_mutex_lock(
                    &g_stats.lock);

                ++g_stats.tx_failures;

                pthread_mutex_unlock(
                    &g_stats.lock);
            }
        }

        usleep(TX_IDLE_US);
    }

    return NULL;
}

/* ========================================================================= */
/* THREAD 3: CAN RX / decoder P50                                            */
/* ========================================================================= */

static void *rx_thread(void *arg)
{
    (void)arg;

    pin_current_thread(1U);

    while (g_running) {

        gateway_can_frame_t frame;
        sensor_type_t sensor;
        float value;
        uint8_t node_id;

        int rc =
            receive_can_frame(g_fd_can0,
                              &frame);

        if (rc <= 0) {

            usleep(RX_POLL_US);
            continue;
        }

        pthread_mutex_lock(
            &g_stats.lock);

        ++g_stats.rx_frames;

        pthread_mutex_unlock(
            &g_stats.lock);

        /*
         * Decode only the protocol generated by the provided NXP node.
         */
        if (!decode_sensor_frame(
                &frame,
                &sensor,
                &value,
                &node_id)) {

            continue;
        }

        frame.sensor_type =
            (uint8_t)sensor;

        frame.logical_addr =
            sensor_logical_address(sensor);

        pthread_mutex_lock(
            &g_stats.lock);

        ++g_stats.sensor_frames;

        pthread_mutex_unlock(
            &g_stats.lock);

        update_sensor_state(
            sensor,
            node_id,
            value,
            frame.can_id,
            frame.rx_timestamp_ns);

        /*
         * Gateway-side prediction/error analysis.
         */
        analyze_sensor_sample(
            sensor,
            node_id,
            value,
            frame.can_id,
            frame.rx_timestamp_ns);

        /*
         * Safety logic derived from the exact NXP sensor semantics:
         *
         *   0x116 = ultrasonic distance in cm
         *   0x117 byte0 = obstacle status
         */
        if (sensor == SENSOR_ULTRASONIC) {

            if (value > 0.0f &&
                value <= ULTRASONIC_SAFETY_CM) {

                request_safety_stop(
                    node_id,
                    "Ultrasonic distance below safety threshold",
                    frame.rx_timestamp_ns);
            }

        } else if (sensor == SENSOR_IR) {

            if ((int)value ==
                IR_OBSTACLE_IS_SAFETY) {

                request_safety_stop(
                    node_id,
                    "IR obstacle detected",
                    frame.rx_timestamp_ns);
            }
        }

        /*
         * Forward every valid NXP sensor frame to the secondary CAN bus.
         */
        if (!normal_push(&frame)) {

            pthread_mutex_lock(
                &g_stats.lock);

            ++g_stats.tx_failures;

            pthread_mutex_unlock(
                &g_stats.lock);
        }
    }

    return NULL;
}

/* ========================================================================= */
/* THREAD 4: Error archive P5                                                */
/* ========================================================================= */

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

        (void)snprintf(
            command,
            sizeof(command),
            "/usr/bin/gzip -f \"%s\"",
            path);

    } else {

        (void)snprintf(
            command,
            sizeof(command),
            "/bin/gzip -f \"%s\"",
            path);
    }

    /*
     * LOW PRIORITY ONLY.
     * Never called from Safety or CAN RX.
     */
    (void)system(command);
}

static int write_all(int fd,
                     const char *buffer,
                     size_t length)
{
    size_t offset = 0U;

    while (offset < length) {

        ssize_t written =
            write(fd,
                  buffer + offset,
                  length - offset);

        if (written < 0) {

            if (errno == EINTR) {
                continue;
            }

            return -1;
        }

        offset +=
            (size_t)written;
    }

    return 0;
}

static void archive_error_batch(
    prediction_record_t *records,
    unsigned count)
{
    char raw_path[256];
    char gz_path[256];
    char final_path[256];

    int fd;

    unsigned i;

    if (count == 0U) {
        return;
    }

    (void)snprintf(
        raw_path,
        sizeof(raw_path),
        "/tmp/qnx_master/staging/error_batch_%06llu.csv",
        (unsigned long long)g_archive_sequence);

    fd =
        open(raw_path,
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
            "seq,timestamp_ns,node_id,can_id,"
            "logical_address,sensor,predicted,actual,"
            "difference,tolerance\n";

        (void)write_all(
            fd,
            header,
            strlen(header));
    }

    for (i = 0U; i < count; ++i) {

        char line[512];

        (void)snprintf(
            line,
            sizeof(line),
            "%llu,%llu,%u,0x%03X,0x%08X,%s,"
            "%.3f,%.3f,%.3f,%.3f\n",
            (unsigned long long)
                records[i].seq,

            (unsigned long long)
                records[i].timestamp_ns,

            records[i].node_id,

            records[i].can_id,

            records[i].logical_addr,

            sensor_name(
                records[i].sensor),

            records[i].predicted_value,

            records[i].actual_value,

            records[i].error_value,

            records[i].tolerance);

        (void)write_all(
            fd,
            line,
            strlen(line));
    }

    (void)fsync(fd);
    close(fd);

    gzip_file(raw_path);

    (void)snprintf(
        gz_path,
        sizeof(gz_path),
        "%s.gz",
        raw_path);

    if (access(gz_path,
               F_OK) == 0) {

        (void)snprintf(
            final_path,
            sizeof(final_path),
            "/tmp/qnx_master/errors/archive/"
            "error_batch_%06llu.csv.gz",
            (unsigned long long)
                g_archive_sequence);

        (void)rename(
            gz_path,
            final_path);

    } else {

        (void)snprintf(
            final_path,
            sizeof(final_path),
            "/tmp/qnx_master/errors/archive/"
            "error_batch_%06llu.csv",
            (unsigned long long)
                g_archive_sequence);

        (void)rename(
            raw_path,
            final_path);
    }

    pthread_mutex_lock(
        &g_stats.lock);

    g_stats.wrong_archived +=
        count;

    pthread_mutex_unlock(
        &g_stats.lock);

    ++g_archive_sequence;
}

static void write_latest_corrected(
    const prediction_record_t *record)
{
    char temp_path[256];
    char final_path[256];
    char json[1024];

    int fd;

    (void)snprintf(
        temp_path,
        sizeof(temp_path),
        "/tmp/qnx_master/corrected/"
        "latest.tmp");

    (void)snprintf(
        final_path,
        sizeof(final_path),
        "/tmp/qnx_master/corrected/"
        "latest_corrected.json");

    fd =
        open(temp_path,
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
        "  \"node_id\": %u,\n"
        "  \"sensor\": \"%s\",\n"
        "  \"can_id\": \"0x%03X\",\n"
        "  \"logical_address\": \"0x%08X\",\n"
        "  \"predicted\": %.3f,\n"
        "  \"actual\": %.3f,\n"
        "  \"difference\": %.3f,\n"
        "  \"tolerance\": %.3f,\n"
        "  \"timestamp_ns\": %llu\n"
        "}\n",

        (unsigned long long)
            record->seq,

        record->node_id,

        sensor_name(
            record->sensor),

        record->can_id,

        record->logical_addr,

        record->predicted_value,

        record->actual_value,

        record->error_value,

        record->tolerance,

        (unsigned long long)
            record->timestamp_ns);

    (void)write_all(
        fd,
        json,
        strlen(json));

    (void)fsync(fd);
    close(fd);

    (void)rename(
        temp_path,
        final_path);
}

static void *archive_thread(void *arg)
{
    (void)arg;

    pin_current_thread(2U);

    while (g_running ||
           error_size() != 0U) {

        prediction_record_t batch[
            ERROR_BATCH_SIZE];

        unsigned count;

        /*
         * ERROR-FIRST POLICY:
         * never process the latest-corrected snapshot before pending
         * wrong records have been drained.
         */
        count =
            error_take_batch(
                batch,
                ERROR_BATCH_SIZE);

        if (count != 0U) {

            archive_error_batch(
                batch,
                count);

            continue;
        }

        {
            prediction_record_t latest;

            if (latest_corrected_snapshot(
                    &latest)) {

                write_latest_corrected(
                    &latest);
            }
        }

        usleep(
            ARCHIVE_PERIOD_MS * 1000U);
    }

    /* Final wrong-record drain. */
    while (error_size() != 0U) {

        prediction_record_t batch[
            ERROR_BATCH_SIZE];

        unsigned count =
            error_take_batch(
                batch,
                ERROR_BATCH_SIZE);

        if (count == 0U) {
            break;
        }

        archive_error_batch(
            batch,
            count);
    }

    return NULL;
}

/* ========================================================================= */
/* THREAD 5: Noise generator P10                                              */
/* ========================================================================= */

static void *noise_thread(void *arg)
{
    (void)arg;

    pin_current_thread(2U);

    while (g_running) {

        if (!g_congestion) {

            usleep(1000U);

            continue;
        }

        {
            gateway_can_frame_t frame;

            memset(&frame,
                   0,
                   sizeof(frame));

            frame.can_id =
                ID_CONGESTION_NOISE;

            frame.dlc = 8U;

            frame.rx_timestamp_ns =
                now_ns();

            frame.data[0] =
                0xA5U;

            frame.data[1] =
                0x5AU;

            frame.data[2] =
                (uint8_t)(frame.rx_timestamp_ns &
                          0xFFU);

            if (noise_size() >=
                (NOISE_Q_SIZE * 85U) / 100U) {

                pthread_mutex_lock(
                    &g_stats.lock);

                ++g_stats.dropped_noise;

                pthread_mutex_unlock(
                    &g_stats.lock);

            } else if (!noise_push(
                           &frame)) {

                pthread_mutex_lock(
                    &g_stats.lock);

                ++g_stats.dropped_noise;

                pthread_mutex_unlock(
                    &g_stats.lock);
            }
        }

        usleep(100U);
    }

    return NULL;
}

/* ========================================================================= */
/* CPU utilization                                                           */
/* ========================================================================= */

static double thread_cpu_percent(
    pthread_t thread,
    uint64_t *last_cpu_ns,
    uint64_t wall_delta_ns)
{
    clockid_t clock_id;

    struct timespec ts;

    uint64_t current_cpu_ns;
    uint64_t delta_cpu_ns;

    if (pthread_getcpuclockid(
            thread,
            &clock_id) != 0) {

        return 0.0;
    }

    if (clock_gettime(
            clock_id,
            &ts) != 0) {

        return 0.0;
    }

    current_cpu_ns =
        ((uint64_t)ts.tv_sec *
         1000000000ULL) +
        (uint64_t)ts.tv_nsec;

    if (*last_cpu_ns == 0U) {

        *last_cpu_ns =
            current_cpu_ns;

        return 0.0;
    }

    delta_cpu_ns =
        current_cpu_ns -
        *last_cpu_ns;

    *last_cpu_ns =
        current_cpu_ns;

    if (wall_delta_ns == 0U) {
        return 0.0;
    }

    {
        double percent =
            ((double)delta_cpu_ns /
             (double)wall_delta_ns) *
            100.0;

        if (percent < 0.0) {
            percent = 0.0;
        }

        if (percent > 100.0) {
            percent = 100.0;
        }

        return percent;
    }
}

/* ========================================================================= */
/* THREAD 6: Diagnostics P15                                                  */
/* ========================================================================= */

static void *diagnostics_thread(void *arg)
{
    (void)arg;

    uint64_t last_wall_ns =
        now_ns();

    uint64_t cpu_safety = 0U;
    uint64_t cpu_tx = 0U;
    uint64_t cpu_rx = 0U;
    uint64_t cpu_archive = 0U;
    uint64_t cpu_noise = 0U;
    uint64_t cpu_diag = 0U;

    pin_current_thread(3U);

    while (g_running) {

        uint64_t wall_now_ns;
        uint64_t wall_delta_ns;

        uint64_t rx_total;
        uint64_t tx_total;

        uint64_t safety_sum;
        uint64_t safety_min;
        uint64_t safety_max;
        uint64_t safety_count;

        uint64_t deadline_misses;
        uint64_t wrong_predictions;
        uint64_t correct_predictions;
        uint64_t archived;
        uint64_t overflow;

        unsigned safety_count_q;
        unsigned safety_tx_count_q;
        unsigned normal_count_q;
        unsigned noise_count_q;
        unsigned error_count_q;

        double cpu0;
        double cpu1;
        double cpu2;
        double cpu3;

        double rx_fps;
        double tx_fps;
        double observed_busload;

        double avg_latency;
        double jitter;

        sleep(1);

        wall_now_ns =
            now_ns();

        wall_delta_ns =
            wall_now_ns -
            last_wall_ns;

        cpu0 =
            thread_cpu_percent(
                g_th_safety,
                &cpu_safety,
                wall_delta_ns);

        cpu0 +=
            thread_cpu_percent(
                g_th_tx,
                &cpu_tx,
                wall_delta_ns);

        cpu1 =
            thread_cpu_percent(
                g_th_rx,
                &cpu_rx,
                wall_delta_ns);

        cpu2 =
            thread_cpu_percent(
                g_th_archive,
                &cpu_archive,
                wall_delta_ns);

        cpu2 +=
            thread_cpu_percent(
                g_th_noise,
                &cpu_noise,
                wall_delta_ns);

        cpu3 =
            thread_cpu_percent(
                g_th_diag,
                &cpu_diag,
                wall_delta_ns);

        pthread_mutex_lock(
            &g_stats.lock);

        rx_total =
            g_stats.rx_frames;

        tx_total =
            g_stats.tx_frames;

        safety_sum =
            g_stats.safety_latency_sum_ns;

        safety_min =
            g_stats.safety_latency_min_ns;

        safety_max =
            g_stats.safety_latency_max_ns;

        safety_count =
            g_stats.safety_commands;

        deadline_misses =
            g_stats.safety_deadline_misses;

        wrong_predictions =
            g_stats.prediction_wrong;

        correct_predictions =
            g_stats.prediction_correct;

        archived =
            g_stats.wrong_archived;

        overflow =
            g_stats.error_overflow;

        pthread_mutex_unlock(
            &g_stats.lock);

        rx_fps =
            (double)(rx_total -
                     g_stats.last_rx);

        tx_fps =
            (double)(tx_total -
                     g_stats.last_tx);

        /*
         * Observed occupancy estimate:
         *
         *   (observed CAN frames/s * approximate frame bits) / bitrate
         *
         * It is only an estimate of traffic observed by this gateway.
         */
        observed_busload =
            ((rx_fps + tx_fps) *
             (double)CAN_APPROX_BITS_8BYTE *
             100.0) /
            (double)CAN_BITRATE;

        if (observed_busload > 100.0) {
            observed_busload =
                100.0;
        }

        if (safety_count != 0U) {

            avg_latency =
                ns_to_ms(
                    safety_sum /
                    safety_count);

        } else {

            avg_latency =
                0.0;
        }

        if (safety_min ==
            UINT64_MAX ||
            safety_max <
            safety_min) {

            jitter =
                0.0;

        } else {

            jitter =
                ns_to_ms(
                    safety_max -
                    safety_min);
        }

        safety_count_q =
            safety_size();

        safety_tx_count_q =
            safety_tx_size();

        normal_count_q =
            normal_size();

        noise_count_q =
            noise_size();

        error_count_q =
            error_size();

        printf(
            "\n"
            "================ QNX MASTER ================\n"
            "RX FPS                         : %.0f\n"
            "TX FPS                         : %.0f\n"
            "Observed CAN load estimate    : %.1f %%\n"
            "Safety latency min/avg/max    : %.3f / %.3f / %.3f ms\n"
            "Safety jitter                  : %.3f ms\n"
            "Safety deadline misses >%.1f  : %llu\n"
            "Queues S/N/Noise/Error        : %u/%u/%u/%u\n"
            "Wrong predictions             : %llu\n"
            "Correct predictions           : %llu (latest only)\n"
            "Wrong records archived       : %llu\n"
            "Error-buffer overflow         : %llu\n"
            "CPU0 Safety+TX                : %.1f %%\n"
            "CPU1 CAN RX                   : %.1f %%\n"
            "CPU2 Archive+Noise            : %.1f %%\n"
            "CPU3 Diagnostics              : %.1f %%\n"
            "Safety latch                  : %s\n"
            "=============================================\n",

            rx_fps,
            tx_fps,
            observed_busload,

            safety_min ==
                UINT64_MAX ?
                0.0 :
                ns_to_ms(safety_min),

            avg_latency,

            safety_max == 0U ?
                0.0 :
                ns_to_ms(safety_max),

            jitter,

            SAFETY_DEADLINE_MS,

            (unsigned long long)
                deadline_misses,

            safety_count_q,
            safety_tx_count_q,
            normal_count_q,
            noise_count_q,
            error_count_q,

            (unsigned long long)
                wrong_predictions,

            (unsigned long long)
                correct_predictions,

            (unsigned long long)
                archived,

            (unsigned long long)
                overflow,

            cpu0,
            cpu1,
            cpu2,
            cpu3,

            g_safety_latched ?
                "ACTIVE" :
                "CLEAR");

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
                "\"q_safety\":%u,"
                "\"q_normal\":%u,"
                "\"q_noise\":%u,"
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
                observed_busload,
                avg_latency,
                jitter,

                (unsigned long long)
                    deadline_misses,

                safety_count_q,
                safety_tx_count_q,
                normal_count_q,
                noise_count_q,
                error_count_q,

                (unsigned long long)
                    wrong_predictions,

                (unsigned long long)
                    correct_predictions,

                (unsigned long long)
                    archived,

                (unsigned long long)
                    overflow,

                cpu0,
                cpu1,
                cpu2,
                cpu3);

            (void)sendto(
                g_udp_sock,
                json,
                strlen(json),
                0,
                (struct sockaddr *)&g_dashboard,
                sizeof(g_dashboard));
        }

        g_stats.last_rx =
            rx_total;

        g_stats.last_tx =
            tx_total;

        last_wall_ns =
            wall_now_ns;
    }

    return NULL;
}

/* ========================================================================= */
/* QNX RT thread creation                                                    */
/* ========================================================================= */

static int create_rt_thread(
    pthread_t *thread_id,
    void *(*entry)(void *),
    int priority,
    const char *name)
{
    pthread_attr_t attr;
    struct sched_param param;

    int rc;

    (void)pthread_attr_init(
        &attr);

    (void)pthread_attr_setinheritsched(
        &attr,
        PTHREAD_EXPLICIT_SCHED);

    (void)pthread_attr_setschedpolicy(
        &attr,
        SCHED_FIFO);

    memset(&param,
           0,
           sizeof(param));

    param.sched_priority =
        priority;

    (void)pthread_attr_setschedparam(
        &attr,
        &param);

    rc =
        pthread_create(
            thread_id,
            &attr,
            entry,
            NULL);

    (void)pthread_attr_destroy(
        &attr);

    if (rc != 0) {

        fprintf(stderr,
                "[-] create %s P%d: %s\n",
                name,
                priority,
                strerror(rc));

        return rc;
    }

    printf(
        "[+] %s -> SCHED_FIFO P%d\n",
        name,
        priority);

    return 0;
}

/* ========================================================================= */
/* UDP setup                                                                 */
/* ========================================================================= */

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

    memset(&g_dashboard,
           0,
           sizeof(g_dashboard));

    g_dashboard.sin_family =
        AF_INET;

    g_dashboard.sin_port =
        htons(UDP_PORT);

    g_dashboard.sin_addr.s_addr =
        inet_addr(UDP_BROADCAST);
}

/* ========================================================================= */
/* NXP command creation                                                      */
/* ========================================================================= */

static bool queue_node_command(
    uint8_t node_id,
    uint8_t command,
    bool safety,
    uint64_t source_timestamp_ns)
{
    gateway_can_frame_t frame;

    memset(&frame,
           0,
           sizeof(frame));

    frame.can_id =
        (node_id == 1U) ?
            NODE1_COMMAND_ID :
            NODE2_COMMAND_ID;

    frame.dlc = 8U;

    frame.data[0] =
        command;

    frame.rx_timestamp_ns =
        source_timestamp_ns;

    frame.logical_addr =
        safety ?
            0x0800U :
            0x0900U;

    if (safety) {
        return safety_push(&frame);
    }

    return normal_push(&frame);
}

/* ========================================================================= */
/* Human-readable latest sensor state                                        */
/* ========================================================================= */

static void print_sensor_states(void)
{
    unsigned i;

    printf(
        "\n================ SENSOR STATE ================\n");

    for (i = 0U;
         i < SENSOR_COUNT;
         ++i) {

        if (g_sensor_state[i].valid) {

            if (g_sensor_state[i].sensor ==
                SENSOR_IR) {

                printf(
                    "Node %u | %-16s | %s | CAN 0x%03X | addr 0x%08X\n",
                    g_sensor_state[i].node_id,
                    sensor_name(
                        g_sensor_state[i].sensor),
                    g_sensor_state[i].latest_value >
                        0.5f ?
                        "OBSTACLE" :
                        "CLEAR",
                    g_sensor_state[i].can_id,
                    g_sensor_state[i].logical_addr);

            } else {

                printf(
                    "Node %u | %-16s | %.2f | CAN 0x%03X | addr 0x%08X\n",
                    g_sensor_state[i].node_id,
                    sensor_name(
                        g_sensor_state[i].sensor),
                    g_sensor_state[i].latest_value,
                    g_sensor_state[i].can_id,
                    g_sensor_state[i].logical_addr);
            }
        }
    }

    printf(
        "================================================\n");
}

/* ========================================================================= */
/* Memory architecture display                                               */
/* ========================================================================= */

static void print_memory_architecture(void)
{
    size_t safety_bytes =
        sizeof(g_q_safety.ring);

    size_t safety_tx_bytes =
        sizeof(g_q_safety_tx.ring);

    size_t normal_bytes =
        sizeof(g_q_normal.ring);

    size_t noise_bytes =
        sizeof(g_q_noise.ring);

    size_t error_bytes =
        sizeof(g_q_errors.ring);

    size_t corrected_bytes =
        sizeof(g_latest_corrected.slot);

    size_t sensor_state_bytes =
        sizeof(g_sensor_state);

    size_t total_fixed =
        safety_bytes +
        safety_tx_bytes +
        normal_bytes +
        noise_bytes +
        error_bytes +
        corrected_bytes +
        sensor_state_bytes;

    printf(
        "\n"
        "================ ARCHITECTURE =================\n"
        "DATA BUS\n"
        "  CAN payload                 : 8 bytes\n"
        "  NXP value encoding         : raw uint16 / 100\n"
        "  IR encoding                : byte0 = obstacle\n"
        "\n"
        "ADDRESS BUS (logical software map)\n"
        "  Temperature                : 0x%04X\n"
        "  Humidity                   : 0x%04X\n"
        "  Speed                      : 0x%04X\n"
        "  Battery Voltage            : 0x%04X\n"
        "  Battery Current            : 0x%04X\n"
        "  Ultrasonic                 : 0x%04X\n"
        "  IR                         : 0x%04X\n"
        "  Wrong prediction bank     : 0x%04X\n"
        "  Latest corrected bank     : 0x%04X\n"
        "\n"
        "MEMORY BUS\n"
        "  Fixed queues + mutexes    : bounded\n"
        "  RT heap allocation        : 0 bytes in CAN/SPI path\n"
        "\n"
        "FIXED STORAGE\n"
        "  Safety queue              : %zu bytes\n"
        "  Normal queue              : %zu bytes\n"
        "  Noise queue               : %zu bytes\n"
        "  Error history             : %zu bytes\n"
        "  Corrected double buffer   : %zu bytes\n"
        "  Sensor state              : %zu bytes\n"
        "  Total fixed record memory : %zu bytes\n"
        "\n"
        "RETENTION\n"
        "  WRONG predictions         : historical + gzip archive\n"
        "  CORRECT predictions       : latest only\n"
        "===============================================\n",

        ADDR_SAFETY_INGRESS,
        ADDR_SAFETY_TX,
        ADDR_SENSOR_TEMP,
        ADDR_SENSOR_HUM,
        ADDR_SENSOR_SPEED,
        ADDR_SENSOR_VOLT,
        ADDR_SENSOR_CURR,
        ADDR_SENSOR_ULTRASONIC,
        ADDR_SENSOR_IR,

        ADDR_ERROR_BANK,
        ADDR_LATEST_CORRECTED,

        safety_bytes,
        safety_tx_bytes,
        normal_bytes,
        noise_bytes,
        error_bytes,
        corrected_bytes,
        sensor_state_bytes,
        total_fixed);
}

/* ========================================================================= */
/* Main                                                                      */
/* ========================================================================= */

int main(void)
{
    long cpu_result;

    cpu_result =
        sysconf(_SC_NPROCESSORS_ONLN);

    if (cpu_result > 0L) {
        g_cpu_count =
            (unsigned)cpu_result;
    }

    (void)signal(SIGINT,
                 signal_handler);

    (void)signal(SIGTERM,
                 signal_handler);

    printf(
        "============================================================\n"
        " QNX MASTER - FRDM-MCXA156 SENSOR GATEWAY\n"
        "============================================================\n"
        "NXP CAN bitrate                 : 500 kbps\n"
        "NXP sensor period               : 500 ms\n"
        "Node 1 command ID               : 0x201\n"
        "Gateway CAN0                    : %s\n"
        "Gateway CAN1                    : %s\n"
        "CPUs detected                   : %u\n"
        "============================================================\n",

        CAN0_DEV,
        CAN1_DEV,
        g_cpu_count);

    init_directories();
    init_queues();

    init_rt_mutex(
        &g_stats.lock);

    init_rt_mutex(
        &g_memory_bus_lock);

    g_stats.safety_latency_min_ns =
        UINT64_MAX;

    print_memory_architecture();

    g_fd_can0 =
        init_can_channel(
            CAN0_DEV,
            "CAN0 / NXP FRONT");

    g_fd_can1 =
        init_can_channel(
            CAN1_DEV,
            "CAN1 / REAR");

    if (g_fd_can0 < 0 ||
        g_fd_can1 < 0) {

        fprintf(stderr,
                "[-] CAN initialization failed.\n");

        if (g_fd_can0 >= 0) {
            close(g_fd_can0);
        }

        if (g_fd_can1 >= 0) {
            close(g_fd_can1);
        }

        return EXIT_FAILURE;
    }

    init_udp();

    /*
     * Priority structure:
     *
     * P60 Safety task
     * P55 CAN TX arbiter
     * P50 CAN RX / decode
     * P15 Diagnostics
     * P10 Noise
     * P5  Error archive
     */
    if (create_rt_thread(
            &g_th_safety,
            safety_thread,
            60,
            "Safety") != 0) {

        return EXIT_FAILURE;
    }

    if (create_rt_thread(
            &g_th_tx,
            tx_thread,
            55,
            "CAN TX Arbiter") != 0) {

        return EXIT_FAILURE;
    }

    if (create_rt_thread(
            &g_th_rx,
            rx_thread,
            50,
            "CAN RX / Decoder") != 0) {

        return EXIT_FAILURE;
    }

    if (create_rt_thread(
            &g_th_diag,
            diagnostics_thread,
            15,
            "Diagnostics") != 0) {

        return EXIT_FAILURE;
    }

    if (create_rt_thread(
            &g_th_noise,
            noise_thread,
            10,
            "Noise") != 0) {

        return EXIT_FAILURE;
    }

    if (create_rt_thread(
            &g_th_archive,
            archive_thread,
            5,
            "Error Archive") != 0) {

        return EXIT_FAILURE;
    }

    printf(
        "\n=================== QNX MASTER MENU ===================\n"
        "1  START Node 1 (0x201, byte0=0x01)\n"
        "2  STOP Node 1 (0x201, byte0=0x00)\n"
        "3  Show latest decoded NXP sensor readings\n"
        "4  Toggle CAN congestion generator (0x450)\n"
        "5  Clear safety latch\n"
        "6  Show memory/address/data-bus architecture\n"
        "7  Quit\n"
        "========================================================\n");

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

                if (queue_node_command(
                        NODE1_ID,
                        CMD_START,
                        false,
                        now_ns())) {

                    clear_safety_latch();

                    printf(
                        "[MASTER] START command queued for Node 1 (0x201)\n");
                }

                break;

            case '2':

                if (queue_node_command(
                        NODE1_ID,
                        CMD_STOP,
                        true,
                        now_ns())) {

                    printf(
                        "[MASTER] SAFETY STOP queued for Node 1 (0x201)\n");
                }

                break;

            case '3':

                print_sensor_states();

                break;

            case '4':

                g_congestion =
                    !g_congestion;

                printf(
                    "[MASTER] congestion/noise: %s\n",
                    g_congestion ?
                        "ON" :
                        "OFF");

                break;

            case '5':

                clear_safety_latch();

                break;

            case '6':

                print_memory_architecture();

                break;

            case '7':

                g_running = 0;

                break;

            default:

                printf(
                    "[MASTER] Invalid menu option\n");

                break;
        }

        while (c != '\n' &&
               c != EOF) {

            c = getchar();
        }
    }

    printf(
        "\n[MASTER] Shutting down...\n");

    g_running = 0;

    pthread_cond_broadcast(
        &g_q_safety.not_empty);

    pthread_cond_broadcast(
        &g_q_safety_tx.not_empty);

    pthread_cond_broadcast(
        &g_q_normal.not_empty);

    pthread_cond_broadcast(
        &g_q_errors.not_empty);

    (void)pthread_join(
        g_th_safety,
        NULL);

    (void)pthread_join(
        g_th_tx,
        NULL);

    (void)pthread_join(
        g_th_rx,
        NULL);

    (void)pthread_join(
        g_th_noise,
        NULL);

    (void)pthread_join(
        g_th_archive,
        NULL);

    (void)pthread_join(
        g_th_diag,
        NULL);

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
        "[MASTER] stopped.\n"
        "[MASTER] Wrong history : /tmp/qnx_master/errors/archive/\n"
        "[MASTER] Latest correct : /tmp/qnx_master/corrected/latest_corrected.json\n");

    return EXIT_SUCCESS;
}
