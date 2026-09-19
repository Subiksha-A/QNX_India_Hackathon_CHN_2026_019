/******************************************************************************
 *
 * ============================================================================
 *              VEHICLE GATEWAY WITH SAFETY-CRITICAL CAN SCHEDULING
 * ============================================================================
 *
 * Platform:
 *      Raspberry Pi 4
 *      QNX Neutrino RTOS 8.0
 *
 * Hardware:
 *      Dual MCP2515 CAN controllers
 *      MCP2515 controllers accessed through QNX io-spi
 *
 * CAN:
 *      CAN 2.0B
 *      500 kbps
 *      11-bit standard identifiers
 *
 * ============================================================================
 *                           SYSTEM ARCHITECTURE
 * ============================================================================
 *
 *
 *                         CAN0 / FRONT ZONE
 *                                |
 *                                v
 *                    +-----------------------+
 *                    |     CAN RX THREAD     |
 *                    |       Priority 40     |
 *                    +-----------+-----------+
 *                                |
 *              +-----------------+------------------+
 *              |                 |                  |
 *              v                 v                  v
 *        +-----------+     +-----------+     +-------------+
 *        |  SAFETY   |     |  NORMAL   |     | TELEMETRY   |
 *        |   QUEUE   |     |   QUEUE   |     |    QUEUE    |
 *        +-----+-----+     +-----+-----+     +-------------+
 *              |                 |
 *              v                 v
 *       +-------------+    +-------------+
 *       | SAFETY TASK |    |   GATEWAY   |
 *       | Priority 50 |    | Priority 30 |
 *       +------+------+    +------+------+
 *              |                  |
 *              +--------+---------+
 *                       |
 *                       v
 *                    CAN1 / REAR
 *
 *
 *                    +----------------------+
 *                    | DIAGNOSTICS THREAD   |
 *                    |      Priority 15     |
 *                    +----------+-----------+
 *                               |
 *              +----------------+----------------+
 *              |                |                |
 *              v                v                v
 *         CPU usage         CAN metrics       UDP / CSV
 *
 *
 * ============================================================================
 *                         QNX CONCEPTS USED
 * ============================================================================
 *
 *  1. SCHED_FIFO real-time scheduling
 *  2. Explicit thread priorities
 *  3. Priority-based preemption
 *  4. Priority inheritance
 *  5. POSIX mutexes
 *  6. POSIX condition variables
 *  7. QNX native channels
 *  8. QNX pulses
 *  9. POSIX timers
 * 10. CLOCK_MONOTONIC
 * 11. CLOCK_PROCESS_CPUTIME_ID
 * 12. QNX SPI resource manager
 * 13. devctl()
 * 14. Isolated queues
 * 15. Deterministic load shedding
 * 16. Resource-aware diagnostics
 * 17. User-space hardware access through resource managers
 *
 ******************************************************************************/

#define _GNU_SOURCE

/*
 * BUILD NOTE:
 * QNX socket APIs are provided by libsocket. The executable must be linked
 * with: -lsocket
 * In QNX IDE: Project Properties -> Linker -> Libraries -> socket.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <signal.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <devctl.h>
#include <hw/io-spi.h>

#include <sys/neutrino.h>
#include <sys/socket.h>

#include <netinet/in.h>
#include <arpa/inet.h>


/******************************************************************************
 * ============================================================================
 *                           CONFIGURATION
 * ============================================================================
 ******************************************************************************/

/* CAN bitrate */
#define CAN_BITRATE                     500000ULL

/*
 * Approximate standard CAN 8-byte frame length.
 *
 * This is an engineering estimate for bus-load visualization.
 * Actual CAN wire occupancy varies because of bit stuffing and frame content.
 */
#define CAN_FRAME_BITS                  111ULL


/* Safety deadline */
#define SAFETY_DEADLINE_NS              5000000ULL
#define SAFETY_DEADLINE_MS              5.0


/* Queue configuration */
#define QUEUE_SIZE                      256U

/*
 * Once telemetry queue reaches this occupancy,
 * low-priority traffic is discarded.
 */
#define LOAD_SHED_THRESHOLD_PERCENT     85U


/* Thread priorities */

#define PRIORITY_SAFETY                 50
#define PRIORITY_CAN_RX                 40
#define PRIORITY_GATEWAY                30
#define PRIORITY_DIAGNOSTICS            15


/* QNX SPI resource-manager paths */

#define CAN0_DEVICE                     "/dev/io-spi/spi0/dev0"
#define CAN1_DEVICE                     "/dev/io-spi/spi0/dev1"


/* UDP telemetry */

#define UDP_PORT                        8080
#define UDP_BROADCAST_ADDRESS           "169.254.255.255"


/* QNX pulse */

#define DIAGNOSTIC_PULSE_CODE          \
        (_PULSE_CODE_MINAVAIL + 1)


/******************************************************************************
 * ============================================================================
 *                         MCP2515 COMMANDS
 * ============================================================================
 ******************************************************************************/

#define MCP_RESET                       0xC0
#define MCP_READ                        0x03
#define MCP_WRITE                       0x02
#define MCP_BITMOD                      0x05


/******************************************************************************
 * ============================================================================
 *                         MCP2515 REGISTERS
 * ============================================================================
 ******************************************************************************/

#define REG_CANSTAT                     0x0E
#define REG_CANCTRL                     0x0F

#define REG_CNF1                        0x2A
#define REG_CNF2                        0x29
#define REG_CNF3                        0x28

#define REG_CANINTF                     0x2C
#define REG_CANINTE                     0x2B

#define REG_TXB0CTRL                    0x30
#define REG_TXB0SIDH                    0x31
#define REG_TXB0DLC                     0x35
#define REG_TXB0D0                      0x36

#define REG_RXB0CTRL                    0x60
#define REG_RXB0SIDH                    0x61
#define REG_RXB0DLC                     0x65
#define REG_RXB0D0                      0x66

#define REG_RXM0SIDH                    0x20
#define REG_RXM0SIDL                    0x21


/******************************************************************************
 * ============================================================================
 *                         CAN MESSAGE IDENTIFIERS
 * ============================================================================
 ******************************************************************************/

#define ID_EMERGENCY_BRAKE              0x010
#define ID_STEERING_AIRBAG              0x020

#define ID_FRONT_RADAR                  0x110
#define ID_VEHICLE_STATE                0x120

#define ID_REAR_STATUS                  0x210

#define ID_CONGESTION_NOISE             0x450


/******************************************************************************
 * ============================================================================
 *                             CAN FRAME
 * ============================================================================
 ******************************************************************************/

/*
 * Internal representation used by the QNX application.
 *
 * timestamp_ns:
 *      Time at which the frame entered the gateway.
 *
 * bus_source:
 *      0 = CAN0
 *      1 = CAN1
 */

typedef struct
{
    uint32_t id;

    uint8_t dlc;

    uint8_t data[8];

    uint64_t timestamp_ns;

    uint8_t bus_source;

} can_frame_t;


/******************************************************************************
 * ============================================================================
 *                           PRIORITY QUEUE
 * ============================================================================
 ******************************************************************************/

/*
 * Three independent queues are used.
 *
 * SAFETY:
 *      Never shares capacity with flood traffic.
 *
 * NORMAL:
 *      Used for regular routed traffic.
 *
 * TELEMETRY:
 *      Disposable under resource pressure.
 *
 * This is the software mechanism used to prevent Head-of-Line blocking.
 */

typedef struct
{
    can_frame_t ring[QUEUE_SIZE];

    unsigned int head;

    unsigned int tail;

    unsigned int count;

    pthread_mutex_t lock;

    pthread_cond_t not_empty;

} priority_queue_t;


/******************************************************************************
 * ============================================================================
 *                       PERFORMANCE STATISTICS
 * ============================================================================
 ******************************************************************************/

/*
 * All timing values are retained in nanoseconds internally.
 *
 * This avoids unnecessary precision loss.
 */

typedef struct
{
    /* ------------------------------------------------------------------------
     * Traffic counters
     * --------------------------------------------------------------------- */

    uint64_t rx_frames;

    uint64_t tx_frames;

    uint64_t routed_frames;

    uint64_t safety_frames;

    uint64_t dropped_noise;

    uint64_t safety_drops;


    /* ------------------------------------------------------------------------
     * Safety timing
     * --------------------------------------------------------------------- */

    uint64_t latency_samples;

    uint64_t total_latency_ns;

    uint64_t min_latency_ns;

    uint64_t max_latency_ns;

    uint64_t deadline_misses;


    /*
     * Previous latency is used to calculate sample-to-sample jitter.
     */
    uint64_t previous_latency_ns;

    uint64_t jitter_total_ns;

    uint64_t jitter_max_ns;


    /* ------------------------------------------------------------------------
     * Throughput
     * --------------------------------------------------------------------- */

    uint64_t previous_rx_frames;

    uint64_t previous_tx_frames;

    uint64_t previous_time_ns;

    double rx_fps;

    double tx_fps;

    double busload_pct;


    /* ------------------------------------------------------------------------
     * CPU utilization
     * --------------------------------------------------------------------- */

    double cpu_utilization_pct;


    /* ------------------------------------------------------------------------
     * Latest safety result
     * --------------------------------------------------------------------- */

    double latest_latency_ms;

    bool safety_alert;


    /*
     * Shared statistics lock.
     *
     * Priority inheritance prevents a high-priority thread from being
     * unnecessarily blocked behind a lower-priority statistics operation.
     */
    pthread_mutex_t stat_lock;

} gateway_stats_t;


/******************************************************************************
 * ============================================================================
 *                             GLOBAL STATE
 * ============================================================================
 ******************************************************************************/

static priority_queue_t q_safety;

static priority_queue_t q_normal;

static priority_queue_t q_telemetry;


static gateway_stats_t g_stats;


static volatile bool g_running = true;


/* CAN file descriptors */

static int g_fd_can0 = -1;

static int g_fd_can1 = -1;


/* UDP telemetry */

static int g_udp_sock = -1;

static struct sockaddr_in g_host_addr;


/* Flight recorder */

static FILE *g_log_fp = NULL;


/* QNX diagnostics channel */

static int g_diag_chid = -1;

static int g_diag_coid = -1;


/* POSIX timer */

static timer_t g_diag_timer;


/* CPU monitoring state */

typedef struct
{
    struct timespec cpu_previous;

    struct timespec wall_previous;

} cpu_monitor_t;


static cpu_monitor_t g_cpu_monitor;


/******************************************************************************
 * ============================================================================
 *                              TIME FUNCTIONS
 * ============================================================================
 ******************************************************************************/

/*
 * Return monotonic time in nanoseconds.
 *
 * CLOCK_MONOTONIC is used because wall-clock adjustments must not affect
 * safety latency measurements.
 */

static uint64_t get_time_ns(void)
{
    struct timespec ts;


    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        return 0;
    }


    return ((uint64_t)ts.tv_sec * 1000000000ULL) +
           (uint64_t)ts.tv_nsec;
}


/*
 * Convert timespec to seconds.
 */

static double timespec_to_seconds(const struct timespec *ts)
{
    if (ts == NULL)
    {
        return 0.0;
    }


    return (double)ts->tv_sec +
           ((double)ts->tv_nsec / 1000000000.0);
}


/******************************************************************************
 * ============================================================================
 *                         CPU UTILIZATION
 * ============================================================================
 ******************************************************************************/

/*
 * Initialize CPU utilization measurement.
 *
 * CLOCK_PROCESS_CPUTIME_ID measures CPU execution time consumed by this
 * process.
 *
 * CLOCK_MONOTONIC measures actual elapsed time.
 */

static int cpu_monitor_init(void)
{
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID,
                      &g_cpu_monitor.cpu_previous) != 0)
    {
        perror("clock_gettime(CLOCK_PROCESS_CPUTIME_ID)");

        return -1;
    }


    if (clock_gettime(CLOCK_MONOTONIC,
                      &g_cpu_monitor.wall_previous) != 0)
    {
        perror("clock_gettime(CLOCK_MONOTONIC)");

        return -1;
    }


    return 0;
}


/*
 * Calculate process CPU utilization since the previous sample.
 *
 *
 *                     CPU execution time
 * CPU utilization = ------------------------- x 100
 *                     wall-clock time
 *
 *
 * This measures the computational demand of the complete gateway process.
 *
 * It does NOT claim that one particular CPU core is being used.
 */

static double cpu_monitor_update(void)
{
    struct timespec current_cpu;

    struct timespec current_wall;


    double cpu_previous;

    double cpu_current;

    double wall_previous;

    double wall_current;

    double cpu_elapsed;

    double wall_elapsed;

    double utilization;


    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID,
                      &current_cpu) != 0)
    {
        return -1.0;
    }


    if (clock_gettime(CLOCK_MONOTONIC,
                      &current_wall) != 0)
    {
        return -1.0;
    }


    cpu_previous =
        timespec_to_seconds(
            &g_cpu_monitor.cpu_previous);


    cpu_current =
        timespec_to_seconds(
            &current_cpu);


    wall_previous =
        timespec_to_seconds(
            &g_cpu_monitor.wall_previous);


    wall_current =
        timespec_to_seconds(
            &current_wall);


    cpu_elapsed =
        cpu_current - cpu_previous;


    wall_elapsed =
        wall_current - wall_previous;


    if (wall_elapsed <= 0.0)
    {
        return 0.0;
    }


    utilization =
        (cpu_elapsed / wall_elapsed) * 100.0;


    /*
     * Update reference points.
     */
    g_cpu_monitor.cpu_previous =
        current_cpu;


    g_cpu_monitor.wall_previous =
        current_wall;


    if (utilization < 0.0)
    {
        utilization = 0.0;
    }


    return utilization;
}


/******************************************************************************
 * ============================================================================
 *                         SPI RESOURCE MANAGER
 * ============================================================================
 ******************************************************************************/

/*
 * Perform one complete SPI transaction.
 *
 * QNX architecture:
 *
 *
 *       Application
 *            |
 *            v
 *       /dev/io-spi/...
 *            |
 *            v
 *       QNX SPI resource manager
 *            |
 *            v
 *       SPI controller
 *            |
 *            v
 *         MCP2515
 *
 *
 * DCMD_SPI_DATA_XCHNG keeps the complete transaction together.
 */

static int spi_xfer(int fd,
                    const void *tx,
                    void *rx,
                    size_t len)
{
    uint32_t total;

    spi_xchng_t *x;

    int err;


    if (fd < 0 || len == 0)
    {
        return EINVAL;
    }


    total =
        (uint32_t)(sizeof(spi_xchng_t) + len);


    x =
        (spi_xchng_t *)malloc(total);


    if (x == NULL)
    {
        return ENOMEM;
    }


    x->nbytes =
        (uint32_t)len;


    if (tx != NULL)
    {
        memcpy(x->data,
               tx,
               len);
    }
    else
    {
        memset(x->data,
               0,
               len);
    }


    err =
        devctl(fd,
               DCMD_SPI_DATA_XCHNG,
               x,
               total,
               NULL);


    if (err == 0 && rx != NULL)
    {
        memcpy(rx,
               x->data,
               len);
    }


    free(x);


    return err;
}


/******************************************************************************
 * ============================================================================
 *                         MCP2515 ACCESS
 * ============================================================================
 ******************************************************************************/

/*
 * Write one MCP2515 register.
 */

static int mcp_write(int fd,
                     uint8_t reg,
                     uint8_t value)
{
    struct __attribute__((packed))
    {
        uint8_t cmd;

        uint8_t reg;

        uint8_t value;

    } packet;


    packet.cmd =
        MCP_WRITE;


    packet.reg =
        reg;


    packet.value =
        value;


    return spi_xfer(fd,
                    &packet,
                    NULL,
                    sizeof(packet));
}


/*
 * Read one MCP2515 register.
 */

static int mcp_read(int fd,
                    uint8_t reg,
                    uint8_t *value)
{
    struct __attribute__((packed))
    {
        uint8_t cmd;

        uint8_t reg;

        uint8_t dummy;

    } tx, rx;


    int err;


    if (value == NULL)
    {
        return EINVAL;
    }


    memset(&tx, 0, sizeof(tx));

    memset(&rx, 0, sizeof(rx));


    tx.cmd =
        MCP_READ;


    tx.reg =
        reg;


    err =
        spi_xfer(fd,
                 &tx,
                 &rx,
                 sizeof(tx));


    if (err != 0)
    {
        return err;
    }


    *value =
        rx.dummy;


    return 0;
}


/*
 * Modify selected bits in an MCP2515 register.
 */

static int mcp_mod(int fd,
                   uint8_t reg,
                   uint8_t mask,
                   uint8_t value)
{
    struct __attribute__((packed))
    {
        uint8_t cmd;

        uint8_t reg;

        uint8_t mask;

        uint8_t value;

    } packet;


    packet.cmd =
        MCP_BITMOD;


    packet.reg =
        reg;


    packet.mask =
        mask;


    packet.value =
        value;


    return spi_xfer(fd,
                    &packet,
                    NULL,
                    sizeof(packet));
}


/******************************************************************************
 * ============================================================================
 *                         MCP2515 INITIALIZATION
 * ============================================================================
 ******************************************************************************/

static int init_can_channel(const char *devpath,
                            const char *name)
{
    int fd;

    spi_cfg_t cfg;

    uint8_t status;

    int err;


    printf("\n[INIT] Opening %s\n",
           devpath);


    fd =
        open(devpath,
             O_RDWR);


    if (fd < 0)
    {
        printf("[ERROR] Cannot open %s: %s\n",
               devpath,
               strerror(errno));

        return -1;
    }


    /*
     * Configure QNX SPI resource manager.
     *
     * 8-bit words
     * MSB first
     * 5 MHz SPI clock
     */
    memset(&cfg,
           0,
           sizeof(cfg));


    cfg.mode =
        8 | (1 << 10);


    cfg.clock_rate =
        5000000;


    err =
        devctl(fd,
               DCMD_SPI_SET_CONFIG,
               &cfg,
               sizeof(cfg),
               NULL);


    if (err != 0)
    {
        printf("[ERROR] SPI configuration failed for %s: %s\n",
               name,
               strerror(err));

        close(fd);

        return -1;
    }


    /*
     * Reset MCP2515.
     */
    {
        uint8_t reset_cmd =
            MCP_RESET;


        err =
            spi_xfer(fd,
                     &reset_cmd,
                     NULL,
                     sizeof(reset_cmd));


        if (err != 0)
        {
            printf("[ERROR] MCP2515 reset failed on %s\n",
                   name);

            close(fd);

            return -1;
        }
    }


    /*
     * Allow MCP2515 reset to complete.
     */
    usleep(10000);


    /*
     * Configure CAN timing.
     *
     * Existing project configuration:
     *
     *      16 MHz oscillator
     *      500 kbps CAN
     */
    if (mcp_write(fd,
                  REG_CNF1,
                  0x01) != 0 ||

        mcp_write(fd,
                  REG_CNF2,
                  0x90) != 0 ||

        mcp_write(fd,
                  REG_CNF3,
                  0x02) != 0)
    {
        printf("[ERROR] CAN timing configuration failed on %s\n",
               name);

        close(fd);

        return -1;
    }


    /*
     * Disable receive masks.
     *
     * All IDs reach software classification.
     */
    if (mcp_write(fd,
                  REG_RXM0SIDH,
                  0x00) != 0 ||

        mcp_write(fd,
                  REG_RXM0SIDL,
                  0x00) != 0)
    {
        printf("[ERROR] RX mask configuration failed on %s\n",
               name);

        close(fd);

        return -1;
    }


    /*
     * RXB0:
     *
     *      0x64
     *
     * enables rollover and disables filtering for the software
     * classification architecture.
     */
    if (mcp_write(fd,
                  REG_RXB0CTRL,
                  0x64) != 0)
    {
        printf("[ERROR] RXB0 configuration failed on %s\n",
               name);

        close(fd);

        return -1;
    }


    /*
     * Enable RX buffer 0 interrupt flag.
     */
    if (mcp_write(fd,
                  REG_CANINTE,
                  0x01) != 0)
    {
        printf("[ERROR] CAN interrupt configuration failed on %s\n",
               name);

        close(fd);

        return -1;
    }


    /*
     * Enter Normal Mode.
     */
    if (mcp_mod(fd,
                REG_CANCTRL,
                0xE0,
                0x00) != 0)
    {
        printf("[ERROR] Failed to enter Normal Mode on %s\n",
               name);

        close(fd);

        return -1;
    }


    /*
     * Verify controller mode.
     */
    if (mcp_read(fd,
                 REG_CANSTAT,
                 &status) != 0)
    {
        printf("[ERROR] Cannot read CANSTAT on %s\n",
               name);

        close(fd);

        return -1;
    }


    printf("[OK] %s active - CANSTAT = 0x%02X\n",
           name,
           status & 0xE0);


    return fd;
}


/******************************************************************************
 * ============================================================================
 *                           CAN TRANSMISSION
 * ============================================================================
 ******************************************************************************/

/*
 * Send one standard CAN frame through MCP2515 TXB0.
 *
 * This function performs the hardware operation.
 *
 * It does not perform logging or diagnostics.
 */

static int send_hardware_can(int fd,
                             const can_frame_t *frame)
{
    struct __attribute__((packed))
    {
        uint8_t cmd;

        uint8_t reg;

        uint8_t sidh;

        uint8_t sidl;

        uint8_t eid8;

        uint8_t eid0;

        uint8_t dlc;

        uint8_t data[8];

    } packet;


    size_t transfer_size;


    if (fd < 0 ||
        frame == NULL)
    {
        return EINVAL;
    }


    if (frame->dlc > 8)
    {
        return EINVAL;
    }


    memset(&packet,
           0,
           sizeof(packet));


    packet.cmd =
        MCP_WRITE;


    packet.reg =
        REG_TXB0SIDH;


    /*
     * Convert 11-bit CAN ID to MCP2515 standard-ID representation.
     */
    packet.sidh =
        (uint8_t)((frame->id >> 3) & 0xFF);


    packet.sidl =
        (uint8_t)((frame->id & 0x07) << 5);


    packet.eid8 =
        0;


    packet.eid0 =
        0;


    packet.dlc =
        frame->dlc & 0x0F;


    if (frame->dlc > 0)
    {
        memcpy(packet.data,
               frame->data,
               frame->dlc);
    }


    /*
     * MCP2515 sequential write:
     *
     * command
     * register
     * SIDH
     * SIDL
     * EID8
     * EID0
     * DLC
     * DATA
     */
    transfer_size =
        7U + (size_t)frame->dlc;


    if (spi_xfer(fd,
                 &packet,
                 NULL,
                 transfer_size) != 0)
    {
        return EIO;
    }


    /*
     * Request transmission.
     *
     * 0x0B:
     *
     *      TXREQ = 1
     *      TX priority = highest
     */
    if (mcp_write(fd,
                  REG_TXB0CTRL,
                  0x0B) != 0)
    {
        return EIO;
    }


    return 0;
}


/******************************************************************************
 * ============================================================================
 *                             QUEUE FUNCTIONS
 * ============================================================================
 ******************************************************************************/

/*
 * Initialize a queue with priority inheritance.
 */

static int queue_init(priority_queue_t *queue)
{
    pthread_mutexattr_t attr;

    int result;


    if (queue == NULL)
    {
        return EINVAL;
    }


    memset(queue,
           0,
           sizeof(*queue));


    result =
        pthread_mutexattr_init(&attr);


    if (result != 0)
    {
        return result;
    }


    result =
        pthread_mutexattr_setprotocol(
            &attr,
            PTHREAD_PRIO_INHERIT);


    if (result != 0)
    {
        pthread_mutexattr_destroy(&attr);

        return result;
    }


    result =
        pthread_mutex_init(
            &queue->lock,
            &attr);


    pthread_mutexattr_destroy(&attr);


    if (result != 0)
    {
        return result;
    }


    result =
        pthread_cond_init(
            &queue->not_empty,
            NULL);


    if (result != 0)
    {
        pthread_mutex_destroy(&queue->lock);

        return result;
    }


    return 0;
}


/*
 * Push frame into queue.
 */

static bool queue_push(priority_queue_t *queue,
                       const can_frame_t *frame)
{
    bool success = false;


    if (queue == NULL ||
        frame == NULL)
    {
        return false;
    }


    pthread_mutex_lock(
        &queue->lock);


    if (queue->count < QUEUE_SIZE)
    {
        queue->ring[queue->tail] =
            *frame;


        queue->tail++;


        if (queue->tail >= QUEUE_SIZE)
        {
            queue->tail = 0;
        }


        queue->count++;


        success = true;


        /*
         * Wake a waiting consumer.
         */
        pthread_cond_signal(
            &queue->not_empty);
    }


    pthread_mutex_unlock(
        &queue->lock);


    return success;
}


/*
 * Pop frame from queue.
 */

static bool queue_pop(priority_queue_t *queue,
                      can_frame_t *frame)
{
    bool success = false;


    if (queue == NULL ||
        frame == NULL)
    {
        return false;
    }


    pthread_mutex_lock(
        &queue->lock);


    while (queue->count == 0 &&
           g_running)
    {
        /*
         * The thread blocks rather than busy-spinning.
         *
         * This is important for CPU resource efficiency.
         */
        pthread_cond_wait(
            &queue->not_empty,
            &queue->lock);
    }


    if (queue->count > 0)
    {
        *frame =
            queue->ring[queue->head];


        queue->head++;


        if (queue->head >= QUEUE_SIZE)
        {
            queue->head = 0;
        }


        queue->count--;


        success = true;
    }


    pthread_mutex_unlock(
        &queue->lock);


    return success;
}


/*
 * Calculate queue occupancy.
 */

static unsigned int queue_occupancy_percent(
                    priority_queue_t *queue)
{
    unsigned int count;


    if (queue == NULL)
    {
        return 0;
    }


    pthread_mutex_lock(
        &queue->lock);


    count =
        queue->count;


    pthread_mutex_unlock(
        &queue->lock);


    return
        (count * 100U) /
        QUEUE_SIZE;
}


/******************************************************************************
 * ============================================================================
 *                       SAFETY LATENCY MONITOR
 * ============================================================================
 ******************************************************************************/

/*
 * Record one completed safety transaction.
 *
 * Measured path:
 *
 *
 *       CAN RX
 *          |
 *          v
 *     classification
 *          |
 *          v
 *     safety queue
 *          |
 *          v
 *     safety thread
 *          |
 *          v
 *       CAN1 TX
 *
 *
 * latency =
 *
 *       transmission timestamp
 *       -
 *       gateway RX timestamp
 */

static void record_safety_latency(
                    uint64_t rx_time_ns,
                    uint64_t tx_time_ns)
{
    uint64_t latency_ns;

    uint64_t jitter_ns;


    if (tx_time_ns <
        rx_time_ns)
    {
        return;
    }


    latency_ns =
        tx_time_ns - rx_time_ns;


    pthread_mutex_lock(
        &g_stats.stat_lock);


    g_stats.safety_frames++;

    g_stats.latency_samples++;

    g_stats.total_latency_ns +=
        latency_ns;


    if (g_stats.latency_samples == 1)
    {
        g_stats.min_latency_ns =
            latency_ns;

        g_stats.max_latency_ns =
            latency_ns;
    }
    else
    {
        if (latency_ns <
            g_stats.min_latency_ns)
        {
            g_stats.min_latency_ns =
                latency_ns;
        }


        if (latency_ns >
            g_stats.max_latency_ns)
        {
            g_stats.max_latency_ns =
                latency_ns;
        }


        /*
         * Absolute difference between consecutive samples.
         */
        if (latency_ns >
            g_stats.previous_latency_ns)
        {
            jitter_ns =
                latency_ns -
                g_stats.previous_latency_ns;
        }
        else
        {
            jitter_ns =
                g_stats.previous_latency_ns -
                latency_ns;
        }


        g_stats.jitter_total_ns +=
            jitter_ns;


        if (jitter_ns >
            g_stats.jitter_max_ns)
        {
            g_stats.jitter_max_ns =
                jitter_ns;
        }
    }


    g_stats.previous_latency_ns =
        latency_ns;


    g_stats.latest_latency_ms =
        (double)latency_ns /
        1000000.0;


    /*
     * Hard deadline verification.
     */
    if (latency_ns >
        SAFETY_DEADLINE_NS)
    {
        g_stats.deadline_misses++;
    }


    g_stats.safety_alert =
        true;


    pthread_mutex_unlock(
        &g_stats.stat_lock);
}


/******************************************************************************
 * ============================================================================
 *                         THROUGHPUT MONITOR
 * ============================================================================
 ******************************************************************************/

/*
 * Update frame-rate and bus-load calculations.
 */

static void update_throughput(void)
{
    uint64_t now;

    uint64_t elapsed_ns;

    uint64_t rx_delta;

    uint64_t tx_delta;


    now =
        get_time_ns();


    pthread_mutex_lock(
        &g_stats.stat_lock);


    if (g_stats.previous_time_ns == 0)
    {
        g_stats.previous_time_ns =
            now;


        g_stats.previous_rx_frames =
            g_stats.rx_frames;


        g_stats.previous_tx_frames =
            g_stats.tx_frames;


        pthread_mutex_unlock(
            &g_stats.stat_lock);


        return;
    }


    elapsed_ns =
        now -
        g_stats.previous_time_ns;


    if (elapsed_ns == 0)
    {
        pthread_mutex_unlock(
            &g_stats.stat_lock);

        return;
    }


    rx_delta =
        g_stats.rx_frames -
        g_stats.previous_rx_frames;


    tx_delta =
        g_stats.tx_frames -
        g_stats.previous_tx_frames;


    g_stats.rx_fps =
        ((double)rx_delta *
         1000000000.0) /
        (double)elapsed_ns;


    g_stats.tx_fps =
        ((double)tx_delta *
         1000000000.0) /
        (double)elapsed_ns;


    /*
     * Engineering estimate:
     *
     *                  FPS x bits/frame
     * Busload = ----------------------------- x 100
     *                  CAN bitrate
     */
    g_stats.busload_pct =
        ((g_stats.rx_fps *
          (double)CAN_FRAME_BITS) /
         (double)CAN_BITRATE) *
        100.0;


    if (g_stats.busload_pct >
        100.0)
    {
        g_stats.busload_pct =
            100.0;
    }


    g_stats.previous_time_ns =
        now;


    g_stats.previous_rx_frames =
        g_stats.rx_frames;


    g_stats.previous_tx_frames =
        g_stats.tx_frames;


    pthread_mutex_unlock(
        &g_stats.stat_lock);
}


/******************************************************************************
 * ============================================================================
 *                         PERFORMANCE REPORT
 * ============================================================================
 ******************************************************************************/

static void print_performance_report(void)
{
    uint64_t latency_samples;

    uint64_t rx_frames;

    uint64_t tx_frames;

    uint64_t routed_frames;

    uint64_t safety_frames;

    uint64_t dropped_noise;

    uint64_t safety_drops;

    uint64_t deadline_misses;

    uint64_t total_latency_ns;

    uint64_t min_latency_ns;

    uint64_t max_latency_ns;

    uint64_t jitter_total_ns;

    uint64_t jitter_max_ns;


    double rx_fps;

    double tx_fps;

    double busload;

    double cpu_usage;


    double average_latency_ms;

    double minimum_latency_ms;

    double maximum_latency_ms;

    double average_jitter_ms;

    double maximum_jitter_ms;


    cpu_usage =
        cpu_monitor_update();


    pthread_mutex_lock(
        &g_stats.stat_lock);


    latency_samples =
        g_stats.latency_samples;


    rx_frames =
        g_stats.rx_frames;


    tx_frames =
        g_stats.tx_frames;


    routed_frames =
        g_stats.routed_frames;


    safety_frames =
        g_stats.safety_frames;


    dropped_noise =
        g_stats.dropped_noise;


    safety_drops =
        g_stats.safety_drops;


    deadline_misses =
        g_stats.deadline_misses;


    total_latency_ns =
        g_stats.total_latency_ns;


    min_latency_ns =
        g_stats.min_latency_ns;


    max_latency_ns =
        g_stats.max_latency_ns;


    jitter_total_ns =
        g_stats.jitter_total_ns;


    jitter_max_ns =
        g_stats.jitter_max_ns;


    rx_fps =
        g_stats.rx_fps;


    tx_fps =
        g_stats.tx_fps;


    busload =
        g_stats.busload_pct;


    pthread_mutex_unlock(
        &g_stats.stat_lock);


    /*
     * Calculate latency statistics.
     */
    if (latency_samples > 0)
    {
        average_latency_ms =
            ((double)total_latency_ns /
             (double)latency_samples) /
            1000000.0;


        minimum_latency_ms =
            (double)min_latency_ns /
            1000000.0;


        maximum_latency_ms =
            (double)max_latency_ns /
            1000000.0;
    }
    else
    {
        average_latency_ms = 0.0;

        minimum_latency_ms = 0.0;

        maximum_latency_ms = 0.0;
    }


    /*
     * Jitter is the average absolute difference between consecutive
     * latency samples.
     */
    if (latency_samples > 1)
    {
        average_jitter_ms =
            ((double)jitter_total_ns /
             (double)(latency_samples - 1)) /
            1000000.0;
    }
    else
    {
        average_jitter_ms = 0.0;
    }


    maximum_jitter_ms =
        (double)jitter_max_ns /
        1000000.0;


    printf("\n");

    printf("================================================================\n");

    printf("             QNX VEHICLE GATEWAY PERFORMANCE\n");

    printf("================================================================\n");


    printf("\n");

    printf("---------------------- QNX SCHEDULING -------------------------\n");

    printf("Scheduler             : SCHED_FIFO\n");

    printf("Safety Priority       : %d\n",
           PRIORITY_SAFETY);

    printf("CAN RX Priority       : %d\n",
           PRIORITY_CAN_RX);

    printf("Gateway Priority      : %d\n",
           PRIORITY_GATEWAY);

    printf("Diagnostics Priority  : %d\n",
           PRIORITY_DIAGNOSTICS);


    printf("\n");

    printf("---------------------- TRAFFIC -------------------------------\n");

    printf("RX Frames             : %llu\n",
           (unsigned long long)rx_frames);

    printf("TX Frames             : %llu\n",
           (unsigned long long)tx_frames);

    printf("Routed Frames         : %llu\n",
           (unsigned long long)routed_frames);

    printf("RX Throughput         : %.2f FPS\n",
           rx_fps);

    printf("TX Throughput         : %.2f FPS\n",
           tx_fps);

    printf("Estimated CAN Load    : %.2f %%\n",
           busload);

    printf("Noise Dropped         : %llu\n",
           (unsigned long long)dropped_noise);


    printf("\n");

    printf("---------------------- SAFETY -------------------------------\n");

    printf("Safety Frames         : %llu\n",
           (unsigned long long)safety_frames);

    printf("Safety Drops          : %llu\n",
           (unsigned long long)safety_drops);

    printf("Deadline Misses       : %llu\n",
           (unsigned long long)deadline_misses);


    printf("\n");

    printf("---------------------- LATENCY ------------------------------\n");

    printf("Samples               : %llu\n",
           (unsigned long long)latency_samples);

    printf("Minimum               : %.3f ms\n",
           minimum_latency_ms);

    printf("Average               : %.3f ms\n",
           average_latency_ms);

    printf("Maximum               : %.3f ms\n",
           maximum_latency_ms);

    printf("Average Jitter        : %.3f ms\n",
           average_jitter_ms);

    printf("Maximum Jitter        : %.3f ms\n",
           maximum_jitter_ms);

    printf("Safety Deadline       : %.3f ms\n",
           SAFETY_DEADLINE_MS);


    printf("\n");

    printf("---------------------- RESOURCE USAGE -----------------------\n");

    if (cpu_usage >= 0.0)
    {
        printf("Gateway Process CPU   : %.2f %%\n",
               cpu_usage);
    }
    else
    {
        printf("Gateway Process CPU   : unavailable\n");
    }


    printf("\n");

    printf("---------------------- QUEUE STATUS --------------------------\n");

    printf("Safety Queue          : %u %%\n",
           queue_occupancy_percent(
               &q_safety));

    printf("Normal Queue          : %u %%\n",
           queue_occupancy_percent(
               &q_normal));

    printf("Telemetry Queue       : %u %%\n",
           queue_occupancy_percent(
               &q_telemetry));


    printf("\n");

    if (latency_samples == 0)
    {
        printf("Safety Status         : NO SAMPLE\n");
    }
    else if (deadline_misses == 0)
    {
        printf("Safety Status         : PASS\n");
    }
    else
    {
        printf("Safety Status         : DEADLINE MISS\n");
    }


    if (safety_drops == 0)
    {
        printf("Safety Frame Loss     : PASS (0 drops)\n");
    }
    else
    {
        printf("Safety Frame Loss     : FAIL\n");
    }


    printf("================================================================\n");
}


/******************************************************************************
 * ============================================================================
 *                         FLIGHT RECORDER
 * ============================================================================
 ******************************************************************************/

/*
 * Logging is deliberately separated from the primary hardware forwarding
 * operation.
 */

static int init_flight_recorder(void)
{
    g_log_fp =
        fopen("/tmp/can_safety_log.csv",
              "w");


    if (g_log_fp == NULL)
    {
        printf("[WARNING] Flight recorder unavailable: %s\n",
               strerror(errno));

        return -1;
    }


    fprintf(g_log_fp,
            "Timestamp_ns,"
            "CAN_ID,"
            "Event,"
            "Latency_ms,"
            "Status\n");


    fflush(g_log_fp);


    return 0;
}


/*
 * Record a safety event after transmission.
 */

static void log_safety_event(
                    const can_frame_t *frame,
                    uint64_t tx_time_ns,
                    double latency_ms)
{
    if (g_log_fp == NULL ||
        frame == NULL)
    {
        return;
    }


    fprintf(g_log_fp,
            "%llu,0x%03X,SAFETY_EVENT,%.3f,%s\n",

            (unsigned long long)tx_time_ns,

            frame->id,

            latency_ms,

            (latency_ms <
             SAFETY_DEADLINE_MS)
                ? "PASS"
                : "FAIL");


    fflush(g_log_fp);
}


/******************************************************************************
 * ============================================================================
 *                              UDP
 * ============================================================================
 ******************************************************************************/

static int init_udp(void)
{
    int broadcast = 1;


    g_udp_sock =
        socket(AF_INET,
               SOCK_DGRAM,
               0);


    if (g_udp_sock < 0)
    {
        printf("[WARNING] UDP unavailable: %s\n",
               strerror(errno));

        return -1;
    }


    if (setsockopt(g_udp_sock,
                   SOL_SOCKET,
                   SO_BROADCAST,
                   &broadcast,
                   sizeof(broadcast)) != 0)
    {
        printf("[WARNING] UDP broadcast option failed: %s\n",
               strerror(errno));
        close(g_udp_sock);
        g_udp_sock = -1;
        return -1;
    }


    memset(&g_host_addr,
           0,
           sizeof(g_host_addr));


    g_host_addr.sin_family =
        AF_INET;


    g_host_addr.sin_port =
        htons(UDP_PORT);


    if (inet_aton(UDP_BROADCAST_ADDRESS,
                  &g_host_addr.sin_addr) == 0)
    {
        printf("[WARNING] Invalid UDP destination address: %s\n",
               UDP_BROADCAST_ADDRESS);
        close(g_udp_sock);
        g_udp_sock = -1;
        return -1;
    }

    printf("[OK] UDP telemetry initialized: %s:%d\n",
           UDP_BROADCAST_ADDRESS,
           UDP_PORT);

    return 0;
}


/*
 * Send current metrics to the engineering laptop.
 *
 * This function is ONLY called by diagnostics.
 */

static void send_udp_telemetry(void)
{
    char json[768];


    uint64_t samples;

    uint64_t safety_drops;

    uint64_t noise_drops;

    uint64_t deadline_misses;


    double average_latency;

    double maximum_latency;

    double busload;

    double rx_fps;

    double cpu_usage;


    if (g_udp_sock < 0)
    {
        return;
    }


    pthread_mutex_lock(
        &g_stats.stat_lock);


    samples =
        g_stats.latency_samples;


    safety_drops =
        g_stats.safety_drops;


    noise_drops =
        g_stats.dropped_noise;


    deadline_misses =
        g_stats.deadline_misses;


    busload =
        g_stats.busload_pct;


    rx_fps =
        g_stats.rx_fps;


    cpu_usage =
        g_stats.cpu_utilization_pct;


    if (samples > 0)
    {
        average_latency =
            ((double)g_stats.total_latency_ns /
             (double)samples) /
            1000000.0;


        maximum_latency =
            (double)g_stats.max_latency_ns /
            1000000.0;
    }
    else
    {
        average_latency = 0.0;

        maximum_latency = 0.0;
    }


    pthread_mutex_unlock(
        &g_stats.stat_lock);


    snprintf(
        json,
        sizeof(json),

        "{"
        "\"rx_fps\":%.2f,"
        "\"busload\":%.2f,"
        "\"cpu_usage\":%.2f,"
        "\"safety_avg_ms\":%.3f,"
        "\"safety_max_ms\":%.3f,"
        "\"safety_drops\":%llu,"
        "\"noise_dropped\":%llu,"
        "\"deadline_misses\":%llu,"
        "\"status\":\"%s\""
        "}",

        rx_fps,

        busload,

        cpu_usage,

        average_latency,

        maximum_latency,

        (unsigned long long)safety_drops,

        (unsigned long long)noise_drops,

        (unsigned long long)deadline_misses,

        (samples > 0 &&
         deadline_misses == 0)
            ? "PASS"
            : "WAIT");


    /*
     * UDP is best-effort.
     *
     * Failure here must never stop CAN processing.
     */
    if (sendto(g_udp_sock,
               json,
               strlen(json),
               0,
               (struct sockaddr *)&g_host_addr,
               sizeof(g_host_addr)) < 0)
    {
        /* UDP telemetry is non-critical; CAN processing continues. */
        printf("[WARNING] UDP telemetry send failed: %s\n",
               strerror(errno));
    }
}


/******************************************************************************
 * ============================================================================
 *                       THREAD 1: SAFETY
 * ============================================================================
 ******************************************************************************/

/*
 * Highest-priority application thread.
 *
 * Priority:
 *
 *      50
 *
 * Responsibilities:
 *
 *      - Consume safety queue
 *      - Forward safety frame
 *      - Measure latency
 *      - Record safety failure
 *
 * No UDP is performed here.
 *
 * No diagnostics are performed here.
 *
 * The objective is to keep the safety critical path short.
 */

static void *thread_safety(void *arg)
{
    can_frame_t frame;


    (void)arg;


    printf("[THREAD] Safety thread started - Priority %d\n",
           PRIORITY_SAFETY);


    while (g_running)
    {
        if (!queue_pop(&q_safety,
                       &frame))
        {
            continue;
        }


        uint64_t rx_time_ns =
            frame.timestamp_ns;


        /*
         * Hardware forwarding is performed immediately.
         */
        int result =
            send_hardware_can(
                g_fd_can1,
                &frame);


        uint64_t tx_time_ns =
            get_time_ns();


        if (result != 0)
        {
            pthread_mutex_lock(
                &g_stats.stat_lock);


            g_stats.safety_drops++;


            pthread_mutex_unlock(
                &g_stats.stat_lock);


            continue;
        }


        /*
         * Record measured gateway latency.
         */
        record_safety_latency(
            rx_time_ns,
            tx_time_ns);


        pthread_mutex_lock(
            &g_stats.stat_lock);


        g_stats.tx_frames++;

        g_stats.routed_frames++;


        double latency_ms =
            g_stats.latest_latency_ms;


        pthread_mutex_unlock(
            &g_stats.stat_lock);


        /*
         * Logging occurs only AFTER hardware forwarding.
         */
        log_safety_event(
            &frame,
            tx_time_ns,
            latency_ms);
    }


    return NULL;
}


/******************************************************************************
 * ============================================================================
 *                       THREAD 2: CAN RX
 * ============================================================================
 ******************************************************************************/

/*
 * Hardware ingestion and classification thread.
 *
 * Priority:
 *
 *      40
 *
 * Responsibilities:
 *
 *      1. Poll MCP2515 status
 *      2. Read received CAN frame
 *      3. Timestamp
 *      4. Classify
 *      5. Apply load shedding
 *      6. Push into isolated queue
 *
 */

static void *thread_can_rx(void *arg)
{
    (void)arg;


    printf("[THREAD] CAN RX thread started - Priority %d\n",
           PRIORITY_CAN_RX);


    while (g_running)
    {
        uint8_t interrupt_flags;


        /*
         * Read MCP2515 interrupt flags.
         */
        if (mcp_read(
                g_fd_can0,
                REG_CANINTF,
                &interrupt_flags) != 0)
        {
            usleep(100);

            continue;
        }


        /*
         * No frame waiting.
         */
        if ((interrupt_flags & 0x01U) == 0)
        {
            usleep(100);

            continue;
        }


        can_frame_t frame;


        memset(&frame,
               0,
               sizeof(frame));


        /*
         * Timestamp as close to hardware detection as possible.
         */
        frame.timestamp_ns =
            get_time_ns();


        frame.bus_source =
            0;


        /*
         * MCP2515 RXB0 sequential read.
         */
        struct __attribute__((packed))
        {
            uint8_t cmd;

            uint8_t reg;

            uint8_t sidh;

            uint8_t sidl;

            uint8_t eid8;

            uint8_t eid0;

            uint8_t dlc;

            uint8_t data[8];

        } tx, rx;


        memset(&tx, 0, sizeof(tx));

        memset(&rx, 0, sizeof(rx));


        tx.cmd =
            MCP_READ;


        tx.reg =
            REG_RXB0SIDH;


        if (spi_xfer(
                g_fd_can0,
                &tx,
                &rx,
                sizeof(tx)) != 0)
        {
            usleep(100);

            continue;
        }


        /*
         * Decode standard 11-bit CAN ID.
         */
        frame.id =
            ((uint32_t)rx.sidh << 3) |
            ((uint32_t)rx.sidl >> 5);


        frame.dlc =
            rx.dlc & 0x0F;


        if (frame.dlc > 8)
        {
            frame.dlc = 8;
        }


        if (frame.dlc > 0)
        {
            memcpy(frame.data,
                   rx.data,
                   frame.dlc);
        }


        /*
         * Clear RX interrupt flag.
         */
        (void)mcp_mod(
            g_fd_can0,
            REG_CANINTF,
            0x01,
            0x00);


        pthread_mutex_lock(
            &g_stats.stat_lock);


        g_stats.rx_frames++;


        pthread_mutex_unlock(
            &g_stats.stat_lock);


        /**********************************************************************
         * SAFETY CLASS
         **********************************************************************/

        if (frame.id == ID_EMERGENCY_BRAKE ||
            frame.id == ID_STEERING_AIRBAG)
        {
            /*
             * Dedicated safety queue.
             */
            if (!queue_push(
                    &q_safety,
                    &frame))
            {
                pthread_mutex_lock(
                    &g_stats.stat_lock);


                g_stats.safety_drops++;


                pthread_mutex_unlock(
                    &g_stats.stat_lock);
            }


            continue;
        }


        /**********************************************************************
         * NORMAL CLASS
         **********************************************************************/

        if (frame.id == ID_FRONT_RADAR ||
            frame.id == ID_VEHICLE_STATE)
        {
            if (!queue_push(
                    &q_normal,
                    &frame))
            {
                pthread_mutex_lock(
                    &g_stats.stat_lock);


                g_stats.dropped_noise++;


                pthread_mutex_unlock(
                    &g_stats.stat_lock);
            }


            continue;
        }


        /**********************************************************************
         * LOW-PRIORITY / TELEMETRY
         **********************************************************************/

        /*
         * Synthetic congestion traffic is deliberately disposable.
         */
        if (frame.id ==
            ID_CONGESTION_NOISE)
        {
            pthread_mutex_lock(
                &g_stats.stat_lock);


            g_stats.dropped_noise++;


            pthread_mutex_unlock(
                &g_stats.stat_lock);


            continue;
        }


        /*
         * Check queue resource pressure.
         */
        if (queue_occupancy_percent(
                &q_telemetry) >=
            LOAD_SHED_THRESHOLD_PERCENT)
        {
            pthread_mutex_lock(
                &g_stats.stat_lock);


            g_stats.dropped_noise++;


            pthread_mutex_unlock(
                &g_stats.stat_lock);


            continue;
        }


        if (!queue_push(
                &q_telemetry,
                &frame))
        {
            pthread_mutex_lock(
                &g_stats.stat_lock);


            g_stats.dropped_noise++;


            pthread_mutex_unlock(
                &g_stats.stat_lock);
        }
    }


    return NULL;
}


/******************************************************************************
 * ============================================================================
 *                       THREAD 3: GATEWAY ROUTER
 * ============================================================================
 ******************************************************************************/

/*
 * Priority:
 *
 *      30
 *
 * Handles normal traffic.
 *
 * Safety traffic NEVER enters this path.
 */

static void *thread_gateway(void *arg)
{
    can_frame_t frame;


    (void)arg;


    printf("[THREAD] Gateway thread started - Priority %d\n",
           PRIORITY_GATEWAY);


    while (g_running)
    {
        if (!queue_pop(
                &q_normal,
                &frame))
        {
            continue;
        }


        if (send_hardware_can(
                g_fd_can1,
                &frame) == 0)
        {
            pthread_mutex_lock(
                &g_stats.stat_lock);


            g_stats.tx_frames++;

            g_stats.routed_frames++;


            pthread_mutex_unlock(
                &g_stats.stat_lock);
        }
        else
        {
            pthread_mutex_lock(
                &g_stats.stat_lock);


            g_stats.dropped_noise++;


            pthread_mutex_unlock(
                &g_stats.stat_lock);
        }
    }


    return NULL;
}


/******************************************************************************
 * ============================================================================
 *                     QNX DIAGNOSTIC CHANNEL
 * ============================================================================
 ******************************************************************************/

/*
 * Create QNX native channel.
 */

static int create_diagnostic_channel(void)
{
    g_diag_chid =
        ChannelCreate(0);


    if (g_diag_chid == -1)
    {
        printf("[ERROR] ChannelCreate failed: %s\n",
               strerror(errno));

        return -1;
    }


    g_diag_coid =
        ConnectAttach(
            0,
            0,
            g_diag_chid,
            _NTO_SIDE_CHANNEL,
            0);


    if (g_diag_coid == -1)
    {
        printf("[ERROR] ConnectAttach failed: %s\n",
               strerror(errno));


        ChannelDestroy(
            g_diag_chid);


        g_diag_chid = -1;


        return -1;
    }


    return 0;
}


/******************************************************************************
 * ============================================================================
 *                       QNX PERIODIC TIMER
 * ============================================================================
 ******************************************************************************/

/*
 * Timer does NOT directly execute diagnostics.
 *
 * Instead:
 *
 *
 *      POSIX TIMER
 *           |
 *           v
 *      QNX PULSE
 *           |
 *           v
 *      CHANNEL
 *           |
 *           v
 *      DIAGNOSTIC THREAD
 *
 *
 * This keeps timing and execution responsibilities separate.
 */

static int create_diagnostic_timer(void)
{
    struct sigevent event;

    struct itimerspec timer_spec;


    SIGEV_PULSE_INIT(
        &event,

        g_diag_coid,

        PRIORITY_DIAGNOSTICS,

        DIAGNOSTIC_PULSE_CODE,

        0);


    if (timer_create(
            CLOCK_MONOTONIC,
            &event,
            &g_diag_timer) != 0)
    {
        printf("[ERROR] timer_create failed: %s\n",
               strerror(errno));

        return -1;
    }


    memset(&timer_spec,
           0,
           sizeof(timer_spec));


    /*
     * First report after one second.
     *
     * Then one report every second.
     */
    timer_spec.it_value.tv_sec =
        1;


    timer_spec.it_interval.tv_sec =
        1;


    if (timer_settime(
            g_diag_timer,
            0,
            &timer_spec,
            NULL) != 0)
    {
        printf("[ERROR] timer_settime failed: %s\n",
               strerror(errno));


        timer_delete(
            g_diag_timer);


        return -1;
    }


    return 0;
}


/******************************************************************************
 * ============================================================================
 *                       THREAD 4: DIAGNOSTICS
 * ============================================================================
 ******************************************************************************/

/*
 * Priority:
 *
 *      15
 *
 * This thread deliberately runs below the safety and gateway threads.
 *
 * Responsibilities:
 *
 *      - Performance calculation
 *      - CPU measurement
 *      - Busload
 *      - FPS
 *      - Queue occupancy
 *      - UDP telemetry
 *
 */

static void *thread_diagnostics(void *arg)
{
    struct _pulse pulse;


    (void)arg;


    printf("[THREAD] Diagnostics thread started - Priority %d\n",
           PRIORITY_DIAGNOSTICS);


    if (create_diagnostic_channel() != 0)
    {
        return NULL;
    }


    if (create_diagnostic_timer() != 0)
    {
        ConnectDetach(
            g_diag_coid);

        g_diag_coid = -1;

        return NULL;
    }


    while (g_running)
    {
        int rcvid;


        rcvid =
            MsgReceive(
                g_diag_chid,

                &pulse,

                sizeof(pulse),

                NULL);


        if (rcvid == 0)
        {
            /*
             * Pulse received.
             */
            if (pulse.code ==
                DIAGNOSTIC_PULSE_CODE)
            {
                update_throughput();


                /*
                 * CPU utilization is calculated here because this is
                 * deliberately the low-criticality measurement thread.
                 */
                double cpu_usage =
                    cpu_monitor_update();


                if (cpu_usage >= 0.0)
                {
                    pthread_mutex_lock(
                        &g_stats.stat_lock);


                    g_stats.cpu_utilization_pct =
                        cpu_usage;


                    pthread_mutex_unlock(
                        &g_stats.stat_lock);
                }


                print_performance_report();


                send_udp_telemetry();
            }
        }
        else if (rcvid > 0)
        {
            /*
             * Unexpected client message.
             *
             * Reply so the sender does not remain blocked.
             */
            MsgReply(
                rcvid,
                EOK,
                NULL,
                0);
        }
        else
        {
            if (errno == EINTR)
            {
                continue;
            }


            break;
        }
    }


    /*
     * Timer belongs to this diagnostic thread.
     */
    timer_delete(
        g_diag_timer);


    if (g_diag_coid != -1)
    {
        ConnectDetach(
            g_diag_coid);

        g_diag_coid = -1;
    }


    if (g_diag_chid != -1)
    {
        ChannelDestroy(
            g_diag_chid);

        g_diag_chid = -1;
    }


    return NULL;
}


/******************************************************************************
 * ============================================================================
 *                       REAL-TIME THREAD CREATION
 * ============================================================================
 ******************************************************************************/

/*
 * Create an explicitly configured SCHED_FIFO thread.
 *
 * PTHREAD_EXPLICIT_SCHED prevents accidental inheritance of the creator's
 * scheduling policy.
 */

static int create_rt_thread(
                    pthread_t *thread,
                    void *(*entry)(void *),
                    int priority)
{
    pthread_attr_t attr;

    struct sched_param param;

    int result;


    if (thread == NULL ||
        entry == NULL)
    {
        return EINVAL;
    }


    result =
        pthread_attr_init(
            &attr);


    if (result != 0)
    {
        return result;
    }


    memset(&param,
           0,
           sizeof(param));


    param.sched_priority =
        priority;


    result =
        pthread_attr_setinheritsched(
            &attr,
            PTHREAD_EXPLICIT_SCHED);


    if (result != 0)
    {
        pthread_attr_destroy(
            &attr);

        return result;
    }


    result =
        pthread_attr_setschedpolicy(
            &attr,
            SCHED_FIFO);


    if (result != 0)
    {
        pthread_attr_destroy(
            &attr);

        return result;
    }


    result =
        pthread_attr_setschedparam(
            &attr,
            &param);


    if (result != 0)
    {
        pthread_attr_destroy(
            &attr);

        return result;
    }


    result =
        pthread_create(
            thread,
            &attr,
            entry,
            NULL);


    pthread_attr_destroy(
        &attr);


    return result;
}


/******************************************************************************
 * ============================================================================
 *                       DEMONSTRATION HELPERS
 * ============================================================================
 ******************************************************************************/

/*
 * Send a normal frame.
 */

static void demo_send_normal(void)
{
    can_frame_t frame;


    memset(&frame,
           0,
           sizeof(frame));


    frame.id =
        ID_FRONT_RADAR;


    frame.dlc =
        8;


    frame.data[0] =
        10;

    frame.data[1] =
        20;

    frame.data[2] =
        30;

    frame.data[3] =
        40;


    frame.timestamp_ns =
        get_time_ns();


    frame.bus_source =
        0;


    if (send_hardware_can(
            g_fd_can0,
            &frame) == 0)
    {
        printf("[DEMO] Normal frame 0x%03X transmitted\n",
               frame.id);
    }
    else
    {
        printf("[DEMO] Normal frame transmission failed\n");
    }
}


/*
 * Trigger emergency brake event.
 *
 * This demonstrates the isolated safety queue.
 */

static void demo_trigger_safety(void)
{
    can_frame_t frame;


    memset(&frame,
           0,
           sizeof(frame));


    frame.id =
        ID_EMERGENCY_BRAKE;


    frame.dlc =
        8;


    /*
     * Example payload.
     */
    frame.data[0] =
        0xFF;


    frame.data[1] =
        0x00;


    frame.data[2] =
        0xAA;


    frame.data[3] =
        0x55;


    /*
     * Timestamp at the gateway entry point.
     */
    frame.timestamp_ns =
        get_time_ns();


    frame.bus_source =
        0;


    if (queue_push(
            &q_safety,
            &frame))
    {
        printf("\n");

        printf("******************************************************\n");

        printf("*** SAFETY EVENT: EMERGENCY BRAKE 0x%03X          ***\n",
               frame.id);

        printf("*** Isolated safety queue                           ***\n");

        printf("******************************************************\n");
    }
    else
    {
        pthread_mutex_lock(
            &g_stats.stat_lock);


        g_stats.safety_drops++;


        pthread_mutex_unlock(
            &g_stats.stat_lock);


        printf("[ERROR] Safety queue full\n");
    }
}


/*
 * Inject low-priority congestion traffic.
 *
 * This is a software demonstration of resource pressure.
 */

static void demo_inject_congestion(void)
{
    unsigned int i;

    unsigned int accepted = 0;

    unsigned int dropped = 0;


    printf("\n[DEMO] Injecting congestion traffic...\n");


    for (i = 0;
         i < 300;
         i++)
    {
        can_frame_t frame;


        memset(&frame,
               0,
               sizeof(frame));


        frame.id =
            ID_CONGESTION_NOISE;


        frame.dlc =
            8;


        frame.timestamp_ns =
            get_time_ns();


        frame.bus_source =
            0;


        /*
         * Load-shedding decision at queue boundary.
         */
        if (queue_occupancy_percent(
                &q_telemetry) >=
            LOAD_SHED_THRESHOLD_PERCENT)
        {
            dropped++;

            continue;
        }


        if (queue_push(
                &q_telemetry,
                &frame))
        {
            accepted++;
        }
        else
        {
            dropped++;
        }
    }


    pthread_mutex_lock(
        &g_stats.stat_lock);


    g_stats.dropped_noise +=
        dropped;


    pthread_mutex_unlock(
        &g_stats.stat_lock);


    printf("[DEMO] Accepted : %u\n",
           accepted);


    printf("[DEMO] Dropped  : %u\n",
           dropped);
}


/******************************************************************************
 * ============================================================================
 *                             SHUTDOWN
 * ============================================================================
 ******************************************************************************/

/*
 * Clean shutdown.
 *
 * An important detail here is that the diagnostic thread may be blocked in
 * MsgReceive(). Therefore, simply setting g_running = false is insufficient.
 *
 * We send one QNX pulse to wake it.
 */

static void shutdown_gateway(
                    pthread_t *threads,
                    size_t thread_count)
{
    size_t i;


    g_running =
        false;


    /*
     * Wake queue consumers.
     */
    pthread_mutex_lock(
        &q_safety.lock);

    pthread_cond_broadcast(
        &q_safety.not_empty);

    pthread_mutex_unlock(
        &q_safety.lock);


    pthread_mutex_lock(
        &q_normal.lock);

    pthread_cond_broadcast(
        &q_normal.not_empty);

    pthread_mutex_unlock(
        &q_normal.lock);


    pthread_mutex_lock(
        &q_telemetry.lock);

    pthread_cond_broadcast(
        &q_telemetry.not_empty);

    pthread_mutex_unlock(
        &q_telemetry.lock);


    /*
     * Wake diagnostic thread from MsgReceive().
     */
    if (g_diag_coid != -1)
    {
        (void)MsgSendPulse(
            g_diag_coid,
            PRIORITY_DIAGNOSTICS,
            DIAGNOSTIC_PULSE_CODE,
            0);
    }


    /*
     * Wait for all application threads.
     */
    for (i = 0;
         i < thread_count;
         i++)
    {
        pthread_join(
            threads[i],
            NULL);
    }


    /*
     * Close flight recorder.
     */
    if (g_log_fp != NULL)
    {
        fclose(
            g_log_fp);

        g_log_fp = NULL;
    }


    /*
     * Close UDP.
     */
    if (g_udp_sock >= 0)
    {
        close(
            g_udp_sock);

        g_udp_sock = -1;
    }


    /*
     * Close CAN controllers.
     */
    if (g_fd_can0 >= 0)
    {
        close(
            g_fd_can0);

        g_fd_can0 = -1;
    }


    if (g_fd_can1 >= 0)
    {
        close(
            g_fd_can1);

        g_fd_can1 = -1;
    }


    /*
     * Destroy queues.
     */
    pthread_cond_destroy(
        &q_safety.not_empty);

    pthread_cond_destroy(
        &q_normal.not_empty);

    pthread_cond_destroy(
        &q_telemetry.not_empty);


    pthread_mutex_destroy(
        &q_safety.lock);

    pthread_mutex_destroy(
        &q_normal.lock);

    pthread_mutex_destroy(
        &q_telemetry.lock);


    pthread_mutex_destroy(
        &g_stats.stat_lock);
}


/******************************************************************************
 * ============================================================================
 *                              MAIN
 * ============================================================================
 ******************************************************************************/

int main(void)
{
    pthread_t threads[4];

    int result;

    pthread_mutexattr_t stat_attr;


    printf("\n");

    printf("================================================================\n");

    printf("       QNX VEHICLE GATEWAY - REAL-TIME CAN ENGINE\n");

    printf("================================================================\n");

    printf("Platform            : Raspberry Pi 4\n");

    printf("OS                  : QNX Neutrino RTOS 8.0\n");

    printf("CAN                 : 500 kbps\n");

    printf("Scheduling          : SCHED_FIFO\n");

    printf("Safety Deadline     : %.2f ms\n",
           SAFETY_DEADLINE_MS);

    printf("================================================================\n");


    /**************************************************************************
     * INITIALIZE GLOBAL STATE
     **************************************************************************/

    memset(&g_stats,
           0,
           sizeof(g_stats));


    /**************************************************************************
     * INITIALIZE CPU MONITOR
     **************************************************************************/

    if (cpu_monitor_init() != 0)
    {
        printf("[FATAL] CPU monitor initialization failed\n");

        return EXIT_FAILURE;
    }


    /**************************************************************************
     * INITIALIZE STATISTICS MUTEX
     **************************************************************************/

    result =
        pthread_mutexattr_init(
            &stat_attr);


    if (result != 0)
    {
        printf("[FATAL] Statistics mutex attribute initialization failed\n");

        return EXIT_FAILURE;
    }


    result =
        pthread_mutexattr_setprotocol(
            &stat_attr,
            PTHREAD_PRIO_INHERIT);


    if (result != 0)
    {
        pthread_mutexattr_destroy(
            &stat_attr);

        printf("[FATAL] Priority inheritance configuration failed\n");

        return EXIT_FAILURE;
    }


    result =
        pthread_mutex_init(
            &g_stats.stat_lock,
            &stat_attr);


    pthread_mutexattr_destroy(
        &stat_attr);


    if (result != 0)
    {
        printf("[FATAL] Statistics mutex initialization failed\n");

        return EXIT_FAILURE;
    }


    /**************************************************************************
     * INITIALIZE QUEUES
     **************************************************************************/

    if (queue_init(&q_safety) != 0 ||
        queue_init(&q_normal) != 0 ||
        queue_init(&q_telemetry) != 0)
    {
        printf("[FATAL] Queue initialization failed\n");

        pthread_mutex_destroy(
            &g_stats.stat_lock);

        return EXIT_FAILURE;
    }


    printf("[OK] Isolated priority queues initialized\n");


    /**************************************************************************
     * INITIALIZE CAN0
     **************************************************************************/

    g_fd_can0 =
        init_can_channel(
            CAN0_DEVICE,
            "CAN0 / Front Safety");


    if (g_fd_can0 < 0)
    {
        printf("[FATAL] CAN0 initialization failed\n");

        return EXIT_FAILURE;
    }


    /**************************************************************************
     * INITIALIZE CAN1
     **************************************************************************/

    g_fd_can1 =
        init_can_channel(
            CAN1_DEVICE,
            "CAN1 / Rear Actuator");


    if (g_fd_can1 < 0)
    {
        close(
            g_fd_can0);

        return EXIT_FAILURE;
    }


    /**************************************************************************
     * INITIALIZE ENGINEERING TELEMETRY
     **************************************************************************/

    (void)init_udp();

    (void)init_flight_recorder();


    /**************************************************************************
     * CREATE REAL-TIME THREADS
     **************************************************************************/

    result =
        create_rt_thread(
            &threads[0],
            thread_safety,
            PRIORITY_SAFETY);


    if (result != 0)
    {
        printf("[FATAL] Safety thread creation failed: %s\n",
               strerror(result));

        g_running = false;

        shutdown_gateway(
            threads,
            0);

        return EXIT_FAILURE;
    }


    result =
        create_rt_thread(
            &threads[1],
            thread_can_rx,
            PRIORITY_CAN_RX);


    if (result != 0)
    {
        printf("[FATAL] CAN RX thread creation failed: %s\n",
               strerror(result));

        g_running = false;

        pthread_join(
            threads[0],
            NULL);

        return EXIT_FAILURE;
    }


    result =
        create_rt_thread(
            &threads[2],
            thread_gateway,
            PRIORITY_GATEWAY);


    if (result != 0)
    {
        printf("[FATAL] Gateway thread creation failed: %s\n",
               strerror(result));

        g_running = false;

        return EXIT_FAILURE;
    }


    result =
        create_rt_thread(
            &threads[3],
            thread_diagnostics,
            PRIORITY_DIAGNOSTICS);


    if (result != 0)
    {
        printf("[FATAL] Diagnostics thread creation failed: %s\n",
               strerror(result));

        g_running = false;

        return EXIT_FAILURE;
    }


    printf("\n");

    printf("[OK] Real-time execution plane started\n");

    printf("[OK] Safety thread       : Priority %d\n",
           PRIORITY_SAFETY);

    printf("[OK] CAN RX thread       : Priority %d\n",
           PRIORITY_CAN_RX);

    printf("[OK] Gateway thread      : Priority %d\n",
           PRIORITY_GATEWAY);

    printf("[OK] Diagnostics thread  : Priority %d\n",
           PRIORITY_DIAGNOSTICS);


    /**************************************************************************
     * DEMONSTRATION MENU
     **************************************************************************/

    printf("\n");

    printf("================================================================\n");

    printf("                     DEMONSTRATION MENU\n");

    printf("================================================================\n");

    printf("  [1] Normal CAN traffic\n");

    printf("  [2] Inject congestion / load shedding\n");

    printf("  [3] Trigger Emergency Brake\n");

    printf("  [4] Display performance report\n");

    printf("  [5] Quit\n");

    printf("================================================================\n");


    /**************************************************************************
     * INTERACTIVE LOOP
     **************************************************************************/

    while (g_running)
    {
        int command;


        printf("\nSelect option [1-5]: ");

        fflush(stdout);


        command =
            getchar();


        if (command == EOF)
        {
            break;
        }


        if (command == '\n')
        {
            continue;
        }


        switch (command)
        {
            case '1':

                demo_send_normal();

                break;


            case '2':

                demo_inject_congestion();

                break;


            case '3':

                demo_trigger_safety();

                break;


            case '4':

                update_throughput();

                print_performance_report();

                break;


            case '5':

                g_running =
                    false;

                break;


            default:

                printf("Unknown command.\n");

                break;
        }


        /*
         * Consume remaining characters from input line.
         */
        while (command != '\n' &&
               command != EOF)
        {
            command =
                getchar();
        }
    }


    /**************************************************************************
     * CLEAN SHUTDOWN
     **************************************************************************/

    printf("\n[INFO] Shutting down gateway...\n");


    shutdown_gateway(
        threads,
        4);


    printf("[OK] Gateway shutdown complete.\n");


    return EXIT_SUCCESS;
}
