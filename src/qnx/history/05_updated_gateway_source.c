/******************************************************************************
 *
 *  VEHICLE GATEWAY WITH SAFETY-CRITICAL CAN SCHEDULING
 *
 *  Platform:
 *      Raspberry Pi 4
 *      QNX Neutrino RTOS 8.0
 *
 *  Hardware:
 *      Dual MCP2515 CAN controllers over QNX io-spi
 *
 *  Architecture:
 *
 *       CAN0 / Front Zone
 *              |
 *              v
 *       +-----------------------+
 *       |   CAN RX Thread       | Priority 40
 *       +-----------+-----------+
 *                   |
 *          +--------+--------+
 *          |        |        |
 *          v        v        v
 *       SAFETY   NORMAL   TELEMETRY
 *        QUEUE    QUEUE      QUEUE
 *          |
 *          v
 *       +-----------------------+
 *       |   SAFETY Thread       | Priority 50
 *       +-----------+-----------+
 *                   |
 *                   v
 *               CAN1 / Rear
 *
 *       NORMAL QUEUE
 *             |
 *             v
 *       +-----------------------+
 *       |  Gateway Thread       | Priority 30
 *       +-----------------------+
 *             |
 *             v
 *           CAN1
 *
 *       +-----------------------+
 *       | Diagnostics Thread    | Priority 15
 *       | QNX Pulse + Timer     |
 *       +-----------------------+
 *             |
 *             +----> UDP telemetry
 *             |
 *             +----> CSV flight recorder
 *
 *
 *  QNX CONCEPTS DEMONSTRATED:
 *
 *      1. Priority-driven preemptive scheduling
 *      2. SCHED_FIFO real-time threads
 *      3. Priority inheritance
 *      4. POSIX mutexes
 *      5. POSIX condition variables
 *      6. QNX native channels
 *      7. QNX pulses
 *      8. POSIX timers / QNX timer infrastructure
 *      9. CLOCK_MONOTONIC timing
 *     10. User-space SPI resource manager (/dev/io-spi)
 *     11. devctl() resource-manager interface
 *     12. Isolated priority queues
 *     13. Deterministic load shedding
 *     14. Resource-aware diagnostics
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <devctl.h>
#include <hw/io-spi.h>

#include <sys/neutrino.h>
#include <sys/syspage.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


/******************************************************************************
 *                           CONFIGURATION
 ******************************************************************************/

/*----------------------------- CAN bitrate --------------------------------*/

#define CAN_BITRATE                 500000ULL

/*
 * Approximate number of bits occupied by a standard 8-byte CAN frame.
 *
 * This is an engineering estimate used for the live dashboard.
 * Actual wire length varies due to bit stuffing and frame content.
 */
#define CAN_FRAME_BITS              111ULL


/*----------------------------- Safety -------------------------------------*/

#define SAFETY_DEADLINE_NS          5000000ULL
#define SAFETY_DEADLINE_MS          5.0


/*----------------------------- Queue --------------------------------------*/

#define QUEUE_SIZE                  256U

/*
 * Load shedding threshold.
 *
 * Once the telemetry queue reaches 85%, low-priority traffic is discarded.
 */
#define LOAD_SHED_THRESHOLD_PERCENT 85U


/*----------------------------- Thread priorities --------------------------*/

#define PRIORITY_SAFETY             50
#define PRIORITY_CAN_RX             40
#define PRIORITY_GATEWAY            30
#define PRIORITY_DIAGNOSTICS        15


/*----------------------------- CAN devices --------------------------------*/

#define CAN0_DEVICE "/dev/io-spi/spi0/dev0"
#define CAN1_DEVICE "/dev/io-spi/spi0/dev1"


/*----------------------------- UDP ----------------------------------------*/

#define UDP_PORT                    8080
#define UDP_BROADCAST_ADDRESS       "169.254.255.255"


/*----------------------------- Diagnostics --------------------------------*/

#define DIAGNOSTIC_PULSE_CODE       (_PULSE_CODE_MINAVAIL + 1)


/******************************************************************************
 *                       MCP2515 COMMANDS
 ******************************************************************************/

#define MCP_RESET                   0xC0
#define MCP_READ                    0x03
#define MCP_WRITE                   0x02
#define MCP_BITMOD                  0x05


/******************************************************************************
 *                       MCP2515 REGISTERS
 ******************************************************************************/

#define REG_CANSTAT                 0x0E
#define REG_CANCTRL                 0x0F

#define REG_CNF1                    0x2A
#define REG_CNF2                    0x29
#define REG_CNF3                    0x28

#define REG_CANINTF                 0x2C
#define REG_CANINTE                 0x2B

#define REG_TXB0CTRL                0x30
#define REG_TXB0SIDH                0x31
#define REG_TXB0DLC                 0x35
#define REG_TXB0D0                  0x36

#define REG_RXB0CTRL                0x60
#define REG_RXB0SIDH                0x61
#define REG_RXB0DLC                 0x65
#define REG_RXB0D0                  0x66

#define REG_RXM0SIDH                0x20
#define REG_RXM0SIDL                0x21


/******************************************************************************
 *                       CAN MESSAGE IDENTIFIERS
 ******************************************************************************/

#define ID_EMERGENCY_BRAKE          0x010
#define ID_STEERING_AIRBAG          0x020

#define ID_FRONT_RADAR              0x110
#define ID_VEHICLE_STATE            0x120

#define ID_REAR_STATUS              0x210

#define ID_CONGESTION_NOISE         0x450


/******************************************************************************
 *                           CAN FRAME
 ******************************************************************************/

/*
 * Internal representation of a CAN frame.
 *
 * timestamp_ns:
 *      Timestamp taken when the frame enters the QNX gateway.
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
 *                           PRIORITY QUEUE
 ******************************************************************************/

/*
 * Each traffic class receives an independent queue.
 *
 * This is the main anti-Head-of-Line-blocking mechanism:
 *
 *      SAFETY       -> isolated queue
 *      NORMAL       -> normal routing queue
 *      TELEMETRY    -> disposable queue
 *
 * A flood of 0x450 frames therefore cannot occupy the safety queue.
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
 *                         PERFORMANCE STATISTICS
 ******************************************************************************/

/*
 * All latency values are maintained in nanoseconds internally.
 *
 * This avoids loss of precision during calculations.
 */
typedef struct
{
    /* Traffic counters */

    uint64_t rx_frames;

    uint64_t tx_frames;

    uint64_t routed_frames;

    uint64_t safety_frames;

    uint64_t dropped_noise;

    uint64_t safety_drops;


    /* Safety timing */

    uint64_t latency_samples;

    uint64_t total_latency_ns;

    uint64_t min_latency_ns;

    uint64_t max_latency_ns;

    uint64_t deadline_misses;


    /* Throughput */

    uint64_t previous_rx_frames;

    uint64_t previous_tx_frames;

    uint64_t previous_time_ns;

    double rx_fps;

    double tx_fps;

    double busload_pct;


    /* Latest safety result */

    double latest_latency_ms;

    bool safety_alert;


    /*
     * Protects statistics shared by multiple threads.
     *
     * Priority inheritance is used because the safety thread can access
     * these statistics while lower-priority diagnostic activity exists.
     */
    pthread_mutex_t stat_lock;

} gateway_stats_t;


/******************************************************************************
 *                           GLOBAL STATE
 ******************************************************************************/

static priority_queue_t q_safety;
static priority_queue_t q_normal;
static priority_queue_t q_telemetry;

static gateway_stats_t g_stats;

static volatile bool g_running = true;


/* CAN file descriptors */

static int g_fd_can0 = -1;
static int g_fd_can1 = -1;


/* UDP */

static int g_udp_sock = -1;

static struct sockaddr_in g_host_addr;


/* Safety flight recorder */

static FILE *g_log_fp = NULL;


/* QNX diagnostic channel */

static int g_diag_chid = -1;
static int g_diag_coid = -1;


/* POSIX timer */

static timer_t g_diag_timer;


/******************************************************************************
 *                           TIME FUNCTIONS
 ******************************************************************************/

/*
 * Return QNX monotonic time in nanoseconds.
 *
 * CLOCK_MONOTONIC is deliberately used instead of CLOCK_REALTIME.
 *
 * Reason:
 *      Wall-clock corrections must not affect latency measurements.
 *
 * This is the timing basis for the safety-path benchmark.
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


/******************************************************************************
 *                    SPI RESOURCE-MANAGER INTERFACE
 ******************************************************************************/

/*
 * Perform one atomic SPI transaction.
 *
 * QNX io-spi exposes the SPI controller through a resource-manager path.
 *
 * DCMD_SPI_DATA_XCHNG is used instead of a separate write() + read()
 * sequence so the transaction remains one SPI operation.
 */
static int spi_xfer(int fd,
                    const void *tx,
                    void *rx,
                    size_t len)
{
    uint32_t total;

    spi_xchng_t *x;

    int err;


    total = (uint32_t)(sizeof(spi_xchng_t) + len);

    x = (spi_xchng_t *)malloc(total);

    if (x == NULL)
    {
        return ENOMEM;
    }


    x->nbytes = (uint32_t)len;


    if (tx != NULL)
    {
        memcpy(x->data, tx, len);
    }
    else
    {
        memset(x->data, 0, len);
    }


    err = devctl(fd,
                 DCMD_SPI_DATA_XCHNG,
                 x,
                 total,
                 NULL);


    if (err == 0 && rx != NULL)
    {
        memcpy(rx, x->data, len);
    }


    free(x);

    return err;
}


/******************************************************************************
 *                         MCP2515 ACCESS
 ******************************************************************************/

/*
 * Write one MCP2515 register.
 */
static int mcp_write(int fd,
                     uint8_t reg,
                     uint8_t val)
{
    struct __attribute__((packed))
    {
        uint8_t cmd;
        uint8_t reg;
        uint8_t val;

    } pkt;


    pkt.cmd = MCP_WRITE;
    pkt.reg = reg;
    pkt.val = val;


    return spi_xfer(fd,
                    &pkt,
                    NULL,
                    sizeof(pkt));
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


    if (value == NULL)
    {
        return EINVAL;
    }


    tx.cmd = MCP_READ;
    tx.reg = reg;
    tx.dummy = 0;


    rx.cmd = 0;
    rx.reg = 0;
    rx.dummy = 0;


    int err = spi_xfer(fd,
                       &tx,
                       &rx,
                       sizeof(tx));


    if (err != 0)
    {
        return err;
    }


    *value = rx.dummy;

    return 0;
}


/*
 * MCP2515 register bit modification.
 */
static int mcp_mod(int fd,
                   uint8_t reg,
                   uint8_t mask,
                   uint8_t val)
{
    struct __attribute__((packed))
    {
        uint8_t cmd;
        uint8_t reg;
        uint8_t mask;
        uint8_t val;

    } pkt;


    pkt.cmd = MCP_BITMOD;
    pkt.reg = reg;
    pkt.mask = mask;
    pkt.val = val;


    return spi_xfer(fd,
                    &pkt,
                    NULL,
                    sizeof(pkt));
}


/******************************************************************************
 *                     MCP2515 CAN INITIALIZATION
 ******************************************************************************/

/*
 * Initialize one MCP2515 controller.
 *
 * QNX relevance:
 *
 *      Application
 *          |
 *          v
 *      /dev/io-spi/spi0/devX
 *          |
 *          v
 *      QNX SPI resource manager
 *          |
 *          v
 *      Hardware SPI controller
 *
 * The application therefore interacts with hardware through the QNX
 * resource-manager abstraction rather than directly manipulating the
 * BCM2711 SPI registers.
 */
static int init_can_channel(const char *devpath,
                            const char *name)
{
    int fd;

    spi_cfg_t cfg;

    uint8_t stat;

    int err;


    printf("\n[INIT] Opening %s\n", devpath);


    fd = open(devpath, O_RDWR);

    if (fd < 0)
    {
        printf("[ERROR] Cannot open %s: %s\n",
               devpath,
               strerror(errno));

        return -1;
    }


    /*
     * Configure the QNX SPI resource manager.
     *
     * 8-bit words
     * MSB first
     * 5 MHz SPI clock
     */
    memset(&cfg, 0, sizeof(cfg));

    cfg.mode = 8 | (1 << 10);

    cfg.clock_rate = 5000000;


    err = devctl(fd,
                 DCMD_SPI_SET_CONFIG,
                 &cfg,
                 sizeof(cfg),
                 NULL);


    if (err != 0)
    {
        printf("[ERROR] DCMD_SPI_SET_CONFIG failed for %s: %s\n",
               name,
               strerror(err));

        close(fd);

        return -1;
    }


    /*
     * Reset MCP2515.
     */
    {
        uint8_t reset_cmd = MCP_RESET;

        err = spi_xfer(fd,
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
     * Give the controller time to complete reset.
     */
    usleep(10000);


    /*
     * Configure CAN timing.
     *
     * 16 MHz MCP2515 oscillator.
     *
     * These values correspond to the existing hardware configuration
     * used by the project.
     */
    if (mcp_write(fd, REG_CNF1, 0x01) != 0 ||
        mcp_write(fd, REG_CNF2, 0x90) != 0 ||
        mcp_write(fd, REG_CNF3, 0x02) != 0)
    {
        printf("[ERROR] CAN timing configuration failed on %s\n",
               name);

        close(fd);

        return -1;
    }


    /*
     * Disable receive masks.
     *
     * All standard IDs can enter the software classification layer.
     */
    mcp_write(fd, REG_RXM0SIDH, 0x00);
    mcp_write(fd, REG_RXM0SIDL, 0x00);


    /*
     * Enable rollover from RXB0 to RXB1.
     */
    mcp_write(fd,
              REG_RXB0CTRL,
              0x64);


    /*
     * Enable RX buffer 0 interrupt.
     */
    mcp_write(fd,
              REG_CANINTE,
              0x01);


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


    if (mcp_read(fd,
                 REG_CANSTAT,
                 &stat) != 0)
    {
        printf("[ERROR] Cannot read CANSTAT on %s\n",
               name);

        close(fd);

        return -1;
    }


    printf("[OK] %s active - CANSTAT = 0x%02X\n",
           name,
           stat & 0xE0);


    return fd;
}


/******************************************************************************
 *                         CAN TRANSMISSION
 ******************************************************************************/

/*
 * Send a standard CAN frame through MCP2515 TXB0.
 *
 * Important:
 *
 *      CAN ID
 *          |
 *          v
 *      MCP2515 TXB0
 *          |
 *          v
 *      CAN1 physical bus
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

    } pkt;


    size_t transfer_size;


    if (frame == NULL)
    {
        return EINVAL;
    }


    if (frame->dlc > 8)
    {
        return EINVAL;
    }


    memset(&pkt, 0, sizeof(pkt));


    pkt.cmd = MCP_WRITE;

    pkt.reg = REG_TXB0SIDH;


    /*
     * Convert 11-bit CAN ID into MCP2515 standard-ID format.
     */
    pkt.sidh =
        (uint8_t)((frame->id >> 3) & 0xFF);


    pkt.sidl =
        (uint8_t)((frame->id & 0x07) << 5);


    pkt.eid8 = 0;
    pkt.eid0 = 0;


    pkt.dlc = frame->dlc & 0x0F;


    if (frame->dlc > 0)
    {
        memcpy(pkt.data,
               frame->data,
               frame->dlc);
    }


    /*
     * The sequential write begins at TXB0SIDH and fills:
     *
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
                 &pkt,
                 NULL,
                 transfer_size) != 0)
    {
        return EIO;
    }


    /*
     * Set TXREQ and highest transmit priority.
     *
     * The MCP2515 then requests transmission.
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
 *                           QUEUE FUNCTIONS
 ******************************************************************************/

/*
 * Initialize one queue.
 *
 * PTHREAD_PRIO_INHERIT is important here.
 *
 * If a high-priority safety thread becomes blocked behind a lower-priority
 * thread holding this mutex, QNX can temporarily raise the mutex owner's
 * effective priority.
 */
static int queue_init(priority_queue_t *q)
{
    pthread_mutexattr_t attr;


    if (q == NULL)
    {
        return EINVAL;
    }


    memset(q, 0, sizeof(*q));


    if (pthread_mutexattr_init(&attr) != 0)
    {
        return EINVAL;
    }


    if (pthread_mutexattr_setprotocol(&attr,
                                      PTHREAD_PRIO_INHERIT) != 0)
    {
        pthread_mutexattr_destroy(&attr);

        return EINVAL;
    }


    if (pthread_mutex_init(&q->lock,
                           &attr) != 0)
    {
        pthread_mutexattr_destroy(&attr);

        return EINVAL;
    }


    pthread_mutexattr_destroy(&attr);


    if (pthread_cond_init(&q->not_empty,
                          NULL) != 0)
    {
        pthread_mutex_destroy(&q->lock);

        return EINVAL;
    }


    return 0;
}


/*
 * Push a frame into a queue.
 */
static bool queue_push(priority_queue_t *q,
                       const can_frame_t *frame)
{
    bool success = false;


    if (q == NULL || frame == NULL)
    {
        return false;
    }


    pthread_mutex_lock(&q->lock);


    if (q->count < QUEUE_SIZE)
    {
        q->ring[q->tail] = *frame;

        q->tail++;

        if (q->tail >= QUEUE_SIZE)
        {
            q->tail = 0;
        }


        q->count++;

        success = true;


        /*
         * Wake the highest-priority waiting queue consumer.
         *
         * In this project, the safety consumer is the highest priority
         * consumer associated with its isolated queue.
         */
        pthread_cond_signal(&q->not_empty);
    }


    pthread_mutex_unlock(&q->lock);


    return success;
}


/*
 * Pop a frame.
 *
 * The consumer sleeps while the queue is empty.
 *
 * This is preferable to busy-spinning because a blocked QNX thread does
 * not continuously consume CPU.
 */
static bool queue_pop(priority_queue_t *q,
                      can_frame_t *frame)
{
    bool success = false;


    if (q == NULL || frame == NULL)
    {
        return false;
    }


    pthread_mutex_lock(&q->lock);


    while (q->count == 0 && g_running)
    {
        pthread_cond_wait(&q->not_empty,
                          &q->lock);
    }


    if (q->count > 0)
    {
        *frame = q->ring[q->head];

        q->head++;

        if (q->head >= QUEUE_SIZE)
        {
            q->head = 0;
        }


        q->count--;

        success = true;
    }


    pthread_mutex_unlock(&q->lock);


    return success;
}


/*
 * Obtain queue occupancy safely.
 */
static unsigned int queue_occupancy_percent(priority_queue_t *q)
{
    unsigned int count;


    if (q == NULL)
    {
        return 0;
    }


    pthread_mutex_lock(&q->lock);

    count = q->count;

    pthread_mutex_unlock(&q->lock);


    return (count * 100U) / QUEUE_SIZE;
}


/******************************************************************************
 *                      PERFORMANCE MONITORING
 ******************************************************************************/

/*
 * Record one completed safety transaction.
 *
 * Measured path:
 *
 *      QNX CAN RX timestamp
 *                 |
 *                 v
 *           classification
 *                 |
 *                 v
 *           safety queue
 *                 |
 *                 v
 *           safety thread
 *                 |
 *                 v
 *          CAN1 transmission
 *
 * Therefore:
 *
 *      latency = TX_time - RX_time
 *
 * This is a rigorous QNX gateway processing metric.
 */
static void record_safety_latency(uint64_t rx_time_ns,
                                  uint64_t tx_time_ns)
{
    uint64_t latency_ns;


    if (tx_time_ns < rx_time_ns)
    {
        return;
    }


    latency_ns =
        tx_time_ns - rx_time_ns;


    pthread_mutex_lock(&g_stats.stat_lock);


    g_stats.safety_frames++;

    g_stats.latency_samples++;

    g_stats.total_latency_ns += latency_ns;


    if (g_stats.latency_samples == 1)
    {
        g_stats.min_latency_ns = latency_ns;
        g_stats.max_latency_ns = latency_ns;
    }
    else
    {
        if (latency_ns < g_stats.min_latency_ns)
        {
            g_stats.min_latency_ns = latency_ns;
        }


        if (latency_ns > g_stats.max_latency_ns)
        {
            g_stats.max_latency_ns = latency_ns;
        }
    }


    g_stats.latest_latency_ms =
        (double)latency_ns / 1000000.0;


    /*
     * Deadline analysis.
     *
     * This is more meaningful than simply printing average latency.
     */
    if (latency_ns > SAFETY_DEADLINE_NS)
    {
        g_stats.deadline_misses++;
    }


    g_stats.safety_alert = true;


    pthread_mutex_unlock(&g_stats.stat_lock);
}


/******************************************************************************
 *                         THROUGHPUT MONITOR
 ******************************************************************************/

/*
 * Calculate frame throughput once every diagnostic period.
 */
static void update_throughput(void)
{
    uint64_t now;

    uint64_t elapsed_ns;

    uint64_t rx_delta;

    uint64_t tx_delta;


    now = get_time_ns();


    pthread_mutex_lock(&g_stats.stat_lock);


    if (g_stats.previous_time_ns == 0)
    {
        g_stats.previous_time_ns = now;

        g_stats.previous_rx_frames =
            g_stats.rx_frames;

        g_stats.previous_tx_frames =
            g_stats.tx_frames;

        pthread_mutex_unlock(&g_stats.stat_lock);

        return;
    }


    elapsed_ns =
        now - g_stats.previous_time_ns;


    if (elapsed_ns == 0)
    {
        pthread_mutex_unlock(&g_stats.stat_lock);

        return;
    }


    rx_delta =
        g_stats.rx_frames -
        g_stats.previous_rx_frames;


    tx_delta =
        g_stats.tx_frames -
        g_stats.previous_tx_frames;


    g_stats.rx_fps =
        ((double)rx_delta * 1000000000.0) /
        (double)elapsed_ns;


    g_stats.tx_fps =
        ((double)tx_delta * 1000000000.0) /
        (double)elapsed_ns;


    /*
     * Engineering estimate of CAN bus utilization.
     *
     * U = FPS * bits/frame / bitrate
     */
    g_stats.busload_pct =
        ((g_stats.rx_fps *
          (double)CAN_FRAME_BITS) /
         (double)CAN_BITRATE) *
        100.0;


    if (g_stats.busload_pct > 100.0)
    {
        g_stats.busload_pct = 100.0;
    }


    g_stats.previous_time_ns = now;

    g_stats.previous_rx_frames =
        g_stats.rx_frames;

    g_stats.previous_tx_frames =
        g_stats.tx_frames;


    pthread_mutex_unlock(&g_stats.stat_lock);
}


/******************************************************************************
 *                     PERFORMANCE REPORT
 ******************************************************************************/

/*
 * Print the complete live performance report.
 */
static void print_performance_report(void)
{
    double average_latency_ms;

    double minimum_latency_ms;

    double maximum_latency_ms;

    double jitter_ms;

    uint64_t latency_samples;

    uint64_t safety_frames;

    uint64_t safety_drops;

    uint64_t deadline_misses;

    uint64_t rx_frames;

    uint64_t tx_frames;

    uint64_t routed_frames;

    uint64_t dropped_noise;

    double rx_fps;

    double tx_fps;

    double busload_pct;


    pthread_mutex_lock(&g_stats.stat_lock);


    latency_samples =
        g_stats.latency_samples;


    safety_frames =
        g_stats.safety_frames;


    safety_drops =
        g_stats.safety_drops;


    deadline_misses =
        g_stats.deadline_misses;


    rx_frames =
        g_stats.rx_frames;


    tx_frames =
        g_stats.tx_frames;


    routed_frames =
        g_stats.routed_frames;


    dropped_noise =
        g_stats.dropped_noise;


    rx_fps =
        g_stats.rx_fps;


    tx_fps =
        g_stats.tx_fps;


    busload_pct =
        g_stats.busload_pct;


    if (latency_samples > 0)
    {
        average_latency_ms =
            ((double)g_stats.total_latency_ns /
             (double)latency_samples) /
            1000000.0;


        minimum_latency_ms =
            (double)g_stats.min_latency_ns /
            1000000.0;


        maximum_latency_ms =
            (double)g_stats.max_latency_ns /
            1000000.0;


        jitter_ms =
            maximum_latency_ms -
            minimum_latency_ms;
    }
    else
    {
        average_latency_ms = 0.0;
        minimum_latency_ms = 0.0;
        maximum_latency_ms = 0.0;
        jitter_ms = 0.0;
    }


    pthread_mutex_unlock(&g_stats.stat_lock);


    printf("\n");
    printf("============================================================\n");
    printf("             QNX VEHICLE GATEWAY PERFORMANCE\n");
    printf("============================================================\n");

    printf("QNX Scheduler       : SCHED_FIFO\n");

    printf("Safety Priority     : %d\n",
           PRIORITY_SAFETY);

    printf("CAN RX Priority     : %d\n",
           PRIORITY_CAN_RX);

    printf("Gateway Priority    : %d\n",
           PRIORITY_GATEWAY);

    printf("Diagnostics Priority: %d\n",
           PRIORITY_DIAGNOSTICS);


    printf("\n---------------------- TRAFFIC -----------------------------\n");

    printf("RX Frames           : %llu\n",
           (unsigned long long)rx_frames);

    printf("TX Frames           : %llu\n",
           (unsigned long long)tx_frames);

    printf("RX Throughput       : %.2f FPS\n",
           rx_fps);

    printf("TX Throughput       : %.2f FPS\n",
           tx_fps);

    printf("Estimated Bus Load  : %.2f %%\n",
           busload_pct);

    printf("Noise Dropped       : %llu\n",
           (unsigned long long)dropped_noise);

    printf("Total Routed        : %llu\n",
           (unsigned long long)routed_frames);


    printf("\n---------------------- SAFETY ------------------------------\n");

    printf("Safety Frames       : %llu\n",
           (unsigned long long)safety_frames);

    printf("Safety Drops        : %llu\n",
           (unsigned long long)safety_drops);

    printf("Deadline Misses     : %llu\n",
           (unsigned long long)deadline_misses);


    printf("\n---------------------- LATENCY -----------------------------\n");

    printf("Minimum             : %.3f ms\n",
           minimum_latency_ms);

    printf("Average             : %.3f ms\n",
           average_latency_ms);

    printf("Maximum             : %.3f ms\n",
           maximum_latency_ms);

    printf("Peak-to-Peak Jitter : %.3f ms\n",
           jitter_ms);

    printf("Deadline            : %.3f ms\n",
           SAFETY_DEADLINE_MS);


    if (latency_samples == 0)
    {
        printf("Safety Status       : NO SAMPLES\n");
    }
    else if (deadline_misses == 0)
    {
        printf("Safety Status       : PASS\n");
    }
    else
    {
        printf("Safety Status       : DEADLINE MISS\n");
    }


    printf("\n---------------------- QUEUES -------------------------------\n");

    printf("Safety Queue        : %u %%\n",
           queue_occupancy_percent(&q_safety));

    printf("Normal Queue        : %u %%\n",
           queue_occupancy_percent(&q_normal));

    printf("Telemetry Queue     : %u %%\n",
           queue_occupancy_percent(&q_telemetry));


    printf("============================================================\n");
}


/******************************************************************************
 *                     FLIGHT RECORDER
 ******************************************************************************/

/*
 * Record one safety event.
 *
 * File I/O is intentionally NOT performed before the safety CAN forwarding.
 *
 * The real-time operation happens first.
 * Logging happens afterward.
 */
static void log_safety_event(const can_frame_t *frame,
                             uint64_t tx_time_ns,
                             double latency_ms)
{
    if (g_log_fp == NULL ||
        frame == NULL)
    {
        return;
    }


    fprintf(g_log_fp,
            "%llu,0x%03X,SAFETY_EMERGENCY,%.3f,%s\n",
            (unsigned long long)tx_time_ns,
            frame->id,
            latency_ms,
            (latency_ms < SAFETY_DEADLINE_MS)
                ? "PASS"
                : "FAIL");


    /*
     * Flush only after the safety transmission has completed.
     *
     * This prevents the filesystem operation from becoming part of the
     * critical CAN forwarding path.
     */
    fflush(g_log_fp);
}


/******************************************************************************
 *                         THREAD PRIORITY
 ******************************************************************************/

/*
 * Configure POSIX scheduling for one thread.
 */
static int configure_thread_priority(int priority)
{
    struct sched_param param;


    memset(&param, 0, sizeof(param));

    param.sched_priority = priority;


    /*
     * Apply SCHED_FIFO to the calling thread.
     */
    if (pthread_setschedparam(pthread_self(),
                              SCHED_FIFO,
                              &param) != 0)
    {
        return -1;
    }


    return 0;
}


/******************************************************************************
 *                         THREAD 1
 *
 *                         SAFETY THREAD
 *
 *                         Priority 50
 ******************************************************************************/

/*
 * Highest-priority application thread.
 *
 * Responsibilities:
 *
 *      - Process only safety-critical messages.
 *      - Forward them immediately to CAN1.
 *      - Measure QNX gateway latency.
 *      - Record deadline violations.
 *
 * The safety path does NOT inspect or wait on the normal queue.
 *
 * This is the architectural mechanism that prevents Head-of-Line blocking.
 */
static void *thread_safety(void *arg)
{
    can_frame_t frame;


    (void)arg;


    configure_thread_priority(PRIORITY_SAFETY);


    printf("[THREAD] Safety thread started - Priority %d\n",
           PRIORITY_SAFETY);


    while (g_running)
    {
        if (!queue_pop(&q_safety,
                       &frame))
        {
            continue;
        }


        /*
         * The timestamp was taken when the frame entered the QNX gateway.
         */
        uint64_t rx_time_ns =
            frame.timestamp_ns;


        /*
         * Forward immediately.
         *
         * No diagnostic printing.
         * No UDP.
         * No file I/O.
         *
         * This keeps the critical path short.
         */
        int result =
            send_hardware_can(g_fd_can1,
                              &frame);


        uint64_t tx_time_ns =
            get_time_ns();


        if (result != 0)
        {
            pthread_mutex_lock(&g_stats.stat_lock);

            g_stats.safety_drops++;

            pthread_mutex_unlock(&g_stats.stat_lock);

            continue;
        }


        /*
         * Measure actual QNX processing latency.
         */
        record_safety_latency(rx_time_ns,
                              tx_time_ns);


        pthread_mutex_lock(&g_stats.stat_lock);

        g_stats.tx_frames++;

        g_stats.routed_frames++;

        double latency_ms =
            g_stats.latest_latency_ms;

        pthread_mutex_unlock(&g_stats.stat_lock);


        /*
         * Flight recording is deliberately performed after transmission.
         */
        log_safety_event(&frame,
                         tx_time_ns,
                         latency_ms);
    }


    return NULL;
}


/******************************************************************************
 *                         THREAD 2
 *
 *                         CAN RX THREAD
 *
 *                         Priority 40
 ******************************************************************************/

/*
 * Hardware ingestion and classification thread.
 *
 * Responsibilities:
 *
 *      1. Check MCP2515 receive interrupt flag.
 *      2. Read frame.
 *      3. Timestamp frame.
 *      4. Classify by CAN ID.
 *      5. Apply load shedding.
 *      6. Push into isolated queue.
 *
 * It does NOT perform expensive routing or diagnostics.
 */
static void *thread_can_rx(void *arg)
{
    (void)arg;


    configure_thread_priority(PRIORITY_CAN_RX);


    printf("[THREAD] CAN RX thread started - Priority %d\n",
           PRIORITY_CAN_RX);


    while (g_running)
    {
        uint8_t interrupt_flags;


        /*
         * Check MCP2515 receive interrupt flag.
         */
        if (mcp_read(g_fd_can0,
                     REG_CANINTF,
                     &interrupt_flags) != 0)
        {
            usleep(100);

            continue;
        }


        if ((interrupt_flags & 0x01U) == 0)
        {
            /*
             * No CAN frame waiting.
             *
             * A short bounded polling interval is used because the
             * current hardware integration exposes the MCP2515 through
             * SPI rather than a dedicated QNX CAN resource manager.
             */
            usleep(100);

            continue;
        }


        can_frame_t frame;

        memset(&frame,
               0,
               sizeof(frame));


        /*
         * Timestamp immediately after detecting the frame.
         */
        frame.timestamp_ns =
            get_time_ns();


        frame.bus_source = 0;


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


        tx.cmd = MCP_READ;
        tx.reg = REG_RXB0SIDH;


        if (spi_xfer(g_fd_can0,
                     &tx,
                     &rx,
                     sizeof(tx)) != 0)
        {
            usleep(100);

            continue;
        }


        /*
         * Decode standard 11-bit CAN identifier.
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
         * Clear RXB0 interrupt flag.
         */
        mcp_mod(g_fd_can0,
                REG_CANINTF,
                0x01,
                0x00);


        pthread_mutex_lock(&g_stats.stat_lock);

        g_stats.rx_frames++;

        pthread_mutex_unlock(&g_stats.stat_lock);


        /**********************************************************************
         * SAFETY CLASS
         **********************************************************************/

        if (frame.id == ID_EMERGENCY_BRAKE ||
            frame.id == ID_STEERING_AIRBAG)
        {
            /*
             * Safety traffic gets its own queue.
             *
             * A telemetry flood therefore cannot fill this queue.
             */
            if (!queue_push(&q_safety,
                            &frame))
            {
                pthread_mutex_lock(&g_stats.stat_lock);

                g_stats.safety_drops++;

                pthread_mutex_unlock(&g_stats.stat_lock);
            }


            continue;
        }


        /**********************************************************************
         * NORMAL CLASS
         **********************************************************************/

        if (frame.id == ID_FRONT_RADAR ||
            frame.id == ID_VEHICLE_STATE)
        {
            if (!queue_push(&q_normal,
                            &frame))
            {
                /*
                 * Normal frames can be discarded when the normal queue
                 * reaches capacity.
                 */
                pthread_mutex_lock(&g_stats.stat_lock);

                g_stats.dropped_noise++;

                pthread_mutex_unlock(&g_stats.stat_lock);
            }


            continue;
        }


        /**********************************************************************
         * LOW-PRIORITY / TELEMETRY CLASS
         **********************************************************************/

        unsigned int telemetry_occupancy =
            queue_occupancy_percent(&q_telemetry);


        /*
         * Active load shedding.
         *
         * This is a deliberate resource-management decision:
         *
         *      Protect critical capacity
         *              >
         *      Preserve low-priority traffic
         */
        if (telemetry_occupancy >=
            LOAD_SHED_THRESHOLD_PERCENT)
        {
            pthread_mutex_lock(&g_stats.stat_lock);

            g_stats.dropped_noise++;

            pthread_mutex_unlock(&g_stats.stat_lock);

            continue;
        }


        /*
         * Specifically classify synthetic congestion frames as disposable.
         */
        if (frame.id == ID_CONGESTION_NOISE)
        {
            pthread_mutex_lock(&g_stats.stat_lock);

            g_stats.dropped_noise++;

            pthread_mutex_unlock(&g_stats.stat_lock);

            continue;
        }


        /*
         * Remaining telemetry is retained only while queue capacity exists.
         */
        if (!queue_push(&q_telemetry,
                        &frame))
        {
            pthread_mutex_lock(&g_stats.stat_lock);

            g_stats.dropped_noise++;

            pthread_mutex_unlock(&g_stats.stat_lock);
        }
    }


    return NULL;
}


/******************************************************************************
 *                         THREAD 3
 *
 *                         GATEWAY ROUTER
 *
 *                         Priority 30
 ******************************************************************************/

/*
 * Routes normal-priority traffic from CAN0 to CAN1.
 *
 * Safety traffic never enters this thread.
 */
static void *thread_gateway(void *arg)
{
    can_frame_t frame;


    (void)arg;


    configure_thread_priority(PRIORITY_GATEWAY);


    printf("[THREAD] Gateway thread started - Priority %d\n",
           PRIORITY_GATEWAY);


    while (g_running)
    {
        if (!queue_pop(&q_normal,
                       &frame))
        {
            continue;
        }


        if (send_hardware_can(g_fd_can1,
                               &frame) == 0)
        {
            pthread_mutex_lock(&g_stats.stat_lock);

            g_stats.tx_frames++;

            g_stats.routed_frames++;

            pthread_mutex_unlock(&g_stats.stat_lock);
        }
        else
        {
            pthread_mutex_lock(&g_stats.stat_lock);

            g_stats.dropped_noise++;

            pthread_mutex_unlock(&g_stats.stat_lock);
        }
    }


    return NULL;
}


/******************************************************************************
 *                       QNX DIAGNOSTIC TIMER
 ******************************************************************************/

/*
 * Create a QNX channel used by the diagnostics thread.
 *
 * The channel allows the timer to deliver a lightweight pulse.
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
        ConnectAttach(0,
                      0,
                      g_diag_chid,
                      _NTO_SIDE_CHANNEL,
                      0);


    if (g_diag_coid == -1)
    {
        printf("[ERROR] ConnectAttach failed: %s\n",
               strerror(errno));

        ChannelDestroy(g_diag_chid);

        g_diag_chid = -1;

        return -1;
    }


    return 0;
}


/*
 * Configure a periodic POSIX timer to deliver a QNX pulse.
 */
static int create_diagnostic_timer(void)
{
    struct sigevent event;

    struct itimerspec timer_spec;


    SIGEV_PULSE_INIT(&event,
                     g_diag_coid,
                     PRIORITY_DIAGNOSTICS,
                     DIAGNOSTIC_PULSE_CODE,
                     0);


    if (timer_create(CLOCK_MONOTONIC,
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
     * First expiry: 1 second.
     * Period: 1 second.
     */
    timer_spec.it_value.tv_sec = 1;

    timer_spec.it_interval.tv_sec = 1;


    if (timer_settime(g_diag_timer,
                      0,
                      &timer_spec,
                      NULL) != 0)
    {
        printf("[ERROR] timer_settime failed: %s\n",
               strerror(errno));

        timer_delete(g_diag_timer);

        return -1;
    }


    return 0;
}


/******************************************************************************
 *                         THREAD 4
 *
 *                         DIAGNOSTICS
 *
 *                         Priority 15
 ******************************************************************************/

/*
 * Low-priority supervision and telemetry thread.
 *
 * QNX mechanism:
 *
 *      Timer
 *        |
 *        v
 *      QNX Pulse
 *        |
 *        v
 *      Channel
 *        |
 *        v
 *      Diagnostics thread
 *
 * This avoids continuously polling the diagnostic timer.
 */
static void *thread_diagnostics(void *arg)
{
    struct _pulse pulse;


    (void)arg;


    configure_thread_priority(PRIORITY_DIAGNOSTICS);


    printf("[THREAD] Diagnostics thread started - Priority %d\n",
           PRIORITY_DIAGNOSTICS);


    if (create_diagnostic_channel() != 0)
    {
        return NULL;
    }


    if (create_diagnostic_timer() != 0)
    {
        return NULL;
    }


    while (g_running)
    {
        int rcvid;


        rcvid =
            MsgReceive(g_diag_chid,
                       &pulse,
                       sizeof(pulse),
                       NULL);


        if (rcvid == 0)
        {
            /*
             * A pulse has arrived.
             */
            if (pulse.code ==
                DIAGNOSTIC_PULSE_CODE)
            {
                update_throughput();

                print_performance_report();


                /*
                 * Send lightweight JSON telemetry to the engineering
                 * laptop.
                 */
                if (g_udp_sock >= 0)
                {
                    char json[512];

                    double avg_latency = 0.0;
                    double max_latency = 0.0;

                    uint64_t samples = 0;
                    uint64_t drops = 0;
                    uint64_t safety = 0;
                    double busload = 0.0;
                    double rx_fps = 0.0;


                    pthread_mutex_lock(
                        &g_stats.stat_lock);


                    samples =
                        g_stats.latency_samples;

                    drops =
                        g_stats.dropped_noise;

                    safety =
                        g_stats.safety_frames;

                    busload =
                        g_stats.busload_pct;

                    rx_fps =
                        g_stats.rx_fps;


                    if (samples > 0)
                    {
                        avg_latency =
                            ((double)
                             g_stats.total_latency_ns /
                             (double)samples) /
                            1000000.0;


                        max_latency =
                            (double)
                            g_stats.max_latency_ns /
                            1000000.0;
                    }


                    pthread_mutex_unlock(
                        &g_stats.stat_lock);


                    snprintf(
                        json,
                        sizeof(json),

                        "{"
                        "\"rx_fps\":%.2f,"
                        "\"busload\":%.2f,"
                        "\"safety_frames\":%llu,"
                        "\"safety_avg_ms\":%.3f,"
                        "\"safety_max_ms\":%.3f,"
                        "\"noise_dropped\":%llu,"
                        "\"status\":\"%s\""
                        "}",

                        rx_fps,

                        busload,

                        (unsigned long long)
                            safety,

                        avg_latency,

                        max_latency,

                        (unsigned long long)
                            drops,

                        (samples > 0 &&
                         max_latency < SAFETY_DEADLINE_MS)
                            ? "PASS"
                            : "WAIT");


                    sendto(
                        g_udp_sock,
                        json,
                        strlen(json),
                        0,
                        (struct sockaddr *)&g_host_addr,
                        sizeof(g_host_addr));
                }
            }
        }
        else if (rcvid > 0)
        {
            /*
             * The diagnostic server currently expects pulses only.
             *
             * Any unexpected message is acknowledged so that the sender
             * does not remain blocked.
             */
            MsgReply(rcvid,
                     EOK,
                     NULL,
                     0);
        }
        else
        {
            /*
             * Channel error.
             */
            if (errno == EINTR)
            {
                continue;
            }

            break;
        }
    }


    timer_delete(g_diag_timer);


    if (g_diag_coid != -1)
    {
        ConnectDetach(g_diag_coid);

        g_diag_coid = -1;
    }


    if (g_diag_chid != -1)
    {
        ChannelDestroy(g_diag_chid);

        g_diag_chid = -1;
    }


    return NULL;
}


/******************************************************************************
 *                       THREAD CREATION
 ******************************************************************************/

/*
 * Create one SCHED_FIFO thread.
 *
 * PTHREAD_EXPLICIT_SCHED is essential.
 *
 * Without it, the thread may inherit the creator's scheduling policy
 * instead of using the explicitly configured real-time priority.
 */
static int create_rt_thread(pthread_t *thread,
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


    if (pthread_attr_init(&attr) != 0)
    {
        return EINVAL;
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
        pthread_attr_destroy(&attr);

        return result;
    }


    result =
        pthread_attr_setschedpolicy(
            &attr,
            SCHED_FIFO);


    if (result != 0)
    {
        pthread_attr_destroy(&attr);

        return result;
    }


    result =
        pthread_attr_setschedparam(
            &attr,
            &param);


    if (result != 0)
    {
        pthread_attr_destroy(&attr);

        return result;
    }


    result =
        pthread_create(thread,
                       &attr,
                       entry,
                       NULL);


    pthread_attr_destroy(&attr);


    return result;
}


/******************************************************************************
 *                         UDP INITIALIZATION
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
        printf("[WARNING] UDP socket unavailable: %s\n",
               strerror(errno));

        return -1;
    }


    if (setsockopt(g_udp_sock,
                   SOL_SOCKET,
                   SO_BROADCAST,
                   &broadcast,
                   sizeof(broadcast)) != 0)
    {
        printf("[WARNING] Cannot enable UDP broadcast\n");
    }


    memset(&g_host_addr,
           0,
           sizeof(g_host_addr));


    g_host_addr.sin_family =
        AF_INET;


    g_host_addr.sin_port =
        htons(UDP_PORT);


    g_host_addr.sin_addr.s_addr =
        inet_addr(UDP_BROADCAST_ADDRESS);


    return 0;
}


/******************************************************************************
 *                         FLIGHT RECORDER
 ******************************************************************************/

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
            "Timestamp_ns,CAN_ID,Event,Latency_ms,Status\n");


    fflush(g_log_fp);


    return 0;
}


/******************************************************************************
 *                           SHUTDOWN
 ******************************************************************************/

static void shutdown_gateway(pthread_t *threads,
                             size_t thread_count)
{
    size_t i;


    g_running = false;


    /*
     * Wake all queue consumers so that they can exit cleanly.
     */
    pthread_mutex_lock(&q_safety.lock);
    pthread_cond_broadcast(&q_safety.not_empty);
    pthread_mutex_unlock(&q_safety.lock);


    pthread_mutex_lock(&q_normal.lock);
    pthread_cond_broadcast(&q_normal.not_empty);
    pthread_mutex_unlock(&q_normal.lock);


    pthread_mutex_lock(&q_telemetry.lock);
    pthread_cond_broadcast(&q_telemetry.not_empty);
    pthread_mutex_unlock(&q_telemetry.lock);


    /*
     * Stop diagnostic timer.
     */
    if (g_diag_chid != -1)
    {
        timer_delete(g_diag_timer);
    }


    for (i = 0;
         i < thread_count;
         i++)
    {
        pthread_join(threads[i],
                     NULL);
    }


    if (g_log_fp != NULL)
    {
        fclose(g_log_fp);

        g_log_fp = NULL;
    }


    if (g_udp_sock >= 0)
    {
        close(g_udp_sock);

        g_udp_sock = -1;
    }


    if (g_fd_can0 >= 0)
    {
        close(g_fd_can0);

        g_fd_can0 = -1;
    }


    if (g_fd_can1 >= 0)
    {
        close(g_fd_can1);

        g_fd_can1 = -1;
    }


    pthread_cond_destroy(&q_safety.not_empty);
    pthread_cond_destroy(&q_normal.not_empty);
    pthread_cond_destroy(&q_telemetry.not_empty);


    pthread_mutex_destroy(&q_safety.lock);
    pthread_mutex_destroy(&q_normal.lock);
    pthread_mutex_destroy(&q_telemetry.lock);


    pthread_mutex_destroy(&g_stats.stat_lock);
}


/******************************************************************************
 *                              DEMO HELPERS
 ******************************************************************************/

/*
 * Inject a normal frame directly onto CAN0.
 *
 * This is useful for validating the hardware path.
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

    frame.data[0] = 10;
    frame.data[1] = 20;
    frame.data[2] = 30;
    frame.data[3] = 40;


    frame.timestamp_ns =
        get_time_ns();


    frame.bus_source =
        0;


    if (send_hardware_can(g_fd_can0,
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
 * Inject a safety frame into the safety queue.
 *
 * This allows the QNX scheduling path to be demonstrated even without
 * physically moving the radar target.
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
     * Demo payload.
     *
     * In the real system these bytes may carry brake pressure and
     * source metadata.
     */
    frame.data[0] = 0xFF;
    frame.data[1] = 0x00;
    frame.data[2] = 0xAA;
    frame.data[3] = 0x55;


    frame.timestamp_ns =
        get_time_ns();


    frame.bus_source =
        0;


    if (queue_push(&q_safety,
                   &frame))
    {
        printf("\n");
        printf("******************************************************\n");
        printf("*** SAFETY EVENT: EMERGENCY BRAKE 0x%03X          ***\n",
               frame.id);
        printf("*** Frame inserted into isolated SAFETY queue.    ***\n");
        printf("******************************************************\n");
    }
    else
    {
        pthread_mutex_lock(&g_stats.stat_lock);

        g_stats.safety_drops++;

        pthread_mutex_unlock(&g_stats.stat_lock);


        printf("[ERROR] Safety queue full\n");
    }
}


/*
 * Inject synthetic low-priority traffic into the telemetry queue.
 *
 * This demonstrates resource pressure and load shedding.
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
         * Deliberately use the same queue boundary as the real
         * resource-management policy.
         */
        if (queue_occupancy_percent(&q_telemetry) >=
            LOAD_SHED_THRESHOLD_PERCENT)
        {
            dropped++;

            continue;
        }


        if (queue_push(&q_telemetry,
                       &frame))
        {
            accepted++;
        }
        else
        {
            dropped++;
        }
    }


    pthread_mutex_lock(&g_stats.stat_lock);

    g_stats.dropped_noise +=
        dropped;

    pthread_mutex_unlock(&g_stats.stat_lock);


    printf("[DEMO] Accepted : %u\n",
           accepted);

    printf("[DEMO] Dropped  : %u\n",
           dropped);
}


/******************************************************************************
 *                              MAIN
 ******************************************************************************/

int main(void)
{
    pthread_t threads[4];

    int result;


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


    /**********************************************************************
     * INITIALIZE STATISTICS
     **********************************************************************/

    memset(&g_stats,
           0,
           sizeof(g_stats));


    /*
     * Statistics mutex also uses priority inheritance.
     */
    pthread_mutexattr_t stat_attr;


    pthread_mutexattr_init(&stat_attr);

    pthread_mutexattr_setprotocol(
        &stat_attr,
        PTHREAD_PRIO_INHERIT);


    if (pthread_mutex_init(&g_stats.stat_lock,
                           &stat_attr) != 0)
    {
        printf("[FATAL] Statistics mutex initialization failed\n");

        return EXIT_FAILURE;
    }


    pthread_mutexattr_destroy(&stat_attr);


    /**********************************************************************
     * INITIALIZE QUEUES
     **********************************************************************/

    if (queue_init(&q_safety) != 0 ||
        queue_init(&q_normal) != 0 ||
        queue_init(&q_telemetry) != 0)
    {
        printf("[FATAL] Queue initialization failed\n");

        return EXIT_FAILURE;
    }


    printf("[OK] Priority queues initialized\n");


    /**********************************************************************
     * INITIALIZE CAN HARDWARE
     **********************************************************************/

    g_fd_can0 =
        init_can_channel(
            CAN0_DEVICE,
            "CAN0 / Front Safety");


    if (g_fd_can0 < 0)
    {
        printf("[FATAL] CAN0 initialization failed\n");

        return EXIT_FAILURE;
    }


    g_fd_can1 =
        init_can_channel(
            CAN1_DEVICE,
            "CAN1 / Rear Actuator");


    if (g_fd_can1 < 0)
    {
        close(g_fd_can0);

        return EXIT_FAILURE;
    }


    /**********************************************************************
     * INITIALIZE ENGINEERING TELEMETRY
     **********************************************************************/

    init_udp();

    init_flight_recorder();


    /**********************************************************************
     * CREATE REAL-TIME THREADS
     **********************************************************************/

    result =
        create_rt_thread(
            &threads[0],
            thread_safety,
            PRIORITY_SAFETY);


    if (result != 0)
    {
        printf("[FATAL] Safety thread creation failed: %s\n",
               strerror(result));

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

        pthread_join(threads[0],
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

    printf("[OK] Safety thread      : Priority %d\n",
           PRIORITY_SAFETY);

    printf("[OK] CAN RX thread      : Priority %d\n",
           PRIORITY_CAN_RX);

    printf("[OK] Gateway thread     : Priority %d\n",
           PRIORITY_GATEWAY);

    printf("[OK] Diagnostics thread : Priority %d\n",
           PRIORITY_DIAGNOSTICS);


    /**********************************************************************
     * INTERACTIVE DEMONSTRATION
     **********************************************************************/

    printf("\n");
    printf("================================================================\n");
    printf("                     DEMONSTRATION MENU\n");
    printf("================================================================\n");
    printf("  [1] Normal CAN traffic\n");
    printf("  [2] Inject congestion / low-priority traffic\n");
    printf("  [3] Trigger Emergency Brake safety event\n");
    printf("  [4] Display performance report\n");
    printf("  [5] Quit\n");
    printf("================================================================\n");


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


        /*
         * Ignore newline characters.
         */
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

                g_running = false;

                break;


            default:

                printf("Unknown command.\n");

                break;
        }


        /*
         * Consume the rest of the input line.
         */
        while (command != '\n' &&
               command != EOF)
        {
            command =
                getchar();
        }
    }


    /**********************************************************************
     * CLEAN SHUTDOWN
     **********************************************************************/

    printf("\n[INFO] Shutting down gateway...\n");


    shutdown_gateway(threads,
                     4);


    printf("[OK] Gateway shutdown complete.\n");


    return EXIT_SUCCESS;
}