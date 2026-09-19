/*
 * FRDM-MCXA156 CAN Sensor Node
 * -----------------------------------------
 * CAN telemetry + QNX command receiver
 * Ultrasonic sensor
 * IR obstacle sensor
 *
 * CAN:
 *   RX = P1_12
 *   TX = P1_13
 *
 * Ultrasonic:
 *   TRIG = P3_27
 *   ECHO = P3_28
 *
 * IR:
 *   OUT = P1_14
 *
 * Compatible with Raspberry Pi QNX gateway.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_gpio.h"

#include "board.h"
#include "app.h"


/*******************************************************************************
 * Configuration
 ******************************************************************************/

/* Logging */
#define LOG_INFO(x)                 (void)PRINTF x

/*
 * Node configuration
 *
 * Node 1:
 *   0x111 Temperature
 *   0x112 Humidity
 *   0x113 Speed
 *   0x114 Battery voltage
 *   0x115 Battery current
 *   0x116 Ultrasonic
 *   0x117 IR
 *
 * Node 2:
 *   0x121 - 0x125
 */
#define THIS_NODE_ID                1


/* CAN message buffers */
#define TX_MB_NUM                   (0)
#define RX_MB_NUM                   (1)


/*******************************************************************************
 * CAN IDs
 ******************************************************************************/

#if (THIS_NODE_ID == 1)

#define CAN_ID_TEMPERATURE          0x111U
#define CAN_ID_HUMIDITY             0x112U
#define CAN_ID_SPEED                0x113U
#define CAN_ID_BATTERY_VOLTAGE      0x114U
#define CAN_ID_BATTERY_CURRENT      0x115U

#define CAN_ID_NODE_CMD             0x201U

#else

#define CAN_ID_TEMPERATURE          0x121U
#define CAN_ID_HUMIDITY             0x122U
#define CAN_ID_SPEED                0x123U
#define CAN_ID_BATTERY_VOLTAGE      0x124U
#define CAN_ID_BATTERY_CURRENT      0x125U

#define CAN_ID_NODE_CMD             0x202U

#endif


/* Broadcast command */
#define CAN_ID_BROADCAST_CMD        0x200U


/*******************************************************************************
 * Sensor Types
 ******************************************************************************/

#define SENSOR_TYPE_TEMPERATURE     0x01U
#define SENSOR_TYPE_HUMIDITY        0x02U
#define SENSOR_TYPE_SPEED           0x03U
#define SENSOR_TYPE_BATTERY_VOLTAGE 0x04U
#define SENSOR_TYPE_BATTERY_CURRENT 0x05U

/* Ultrasonic */
#define SENSOR_TYPE_ULTRASONIC     0x06U
#define CAN_ID_ULTRASONIC           0x116U

/* IR obstacle sensor */
#define SENSOR_TYPE_IR              0x07U
#define CAN_ID_IR                   0x117U


/*******************************************************************************
 * Ultrasonic Pin Configuration
 ******************************************************************************/

/*
 * HC-SR04 style sensor
 *
 * TRIG -> P3_27
 * ECHO -> P3_28
 */

#define ULTRASONIC_TRIG_PORT        GPIO3
#define ULTRASONIC_TRIG_PIN         27U

#define ULTRASONIC_ECHO_PORT        GPIO3
#define ULTRASONIC_ECHO_PIN         28U

#define ULTRASONIC_TIMEOUT_US       30000U


/*******************************************************************************
 * IR Pin Configuration
 ******************************************************************************/

/*
 * IR obstacle sensor
 *
 * VCC -> 3.3V
 * GND -> GND
 * OUT -> P1_14
 */

#define IR_PORT                     GPIO1
#define IR_PIN                      14U


/*******************************************************************************
 * General CAN Configuration
 ******************************************************************************/

#define CAN_DLC                     8U

/* Sensor transmission interval */
#define SENSOR_INTERVAL_MS          500U


/*******************************************************************************
 * Global Variables
 ******************************************************************************/

static flexcan_frame_t txFrame;
static flexcan_frame_t rxFrame;

/*
 * Sensor streaming state.
 *
 * true  = sensors active
 * false = sensors paused
 *
 * QNX can control this using:
 *
 * 0x01 -> START
 * 0x00 -> STOP
 */
static volatile bool g_sensorActive = true;


/*******************************************************************************
 * Simulated / Built-in Sensors
 ******************************************************************************/

/*
 * Onboard temperature sensor.
 *
 * This retains the behavior from your original program.
 */
static float ReadBuiltInTemperature(void)
{
    static float currentTemp = 24.80f;
    static int dir = 1;

    currentTemp += (dir * 0.05f);

    if (currentTemp > 28.20f)
    {
        dir = -1;
    }

    if (currentTemp < 24.80f)
    {
        dir = 1;
    }

    return currentTemp;
}


/*
 * Humidity
 */
static float ReadHumidity(void)
{
    static float humidity =
        (THIS_NODE_ID == 1) ? 54.0f : 58.5f;

    humidity += 0.1f;

    if (humidity > 70.0f)
    {
        humidity = 50.0f;
    }

    return humidity;
}


/*
 * Speed
 */
static float ReadSpeed(void)
{
    static float speed =
        (THIS_NODE_ID == 1) ? 35.0f : 42.0f;

    speed += 0.25f;

    if (speed > 80.0f)
    {
        speed = 30.0f;
    }

    return speed;
}


/*
 * Battery voltage
 */
static float ReadBatteryVoltage(void)
{
    return (THIS_NODE_ID == 1) ? 48.40f : 47.90f;
}


/*
 * Battery current
 */
static float ReadBatteryCurrent(void)
{
    return (THIS_NODE_ID == 1) ? 12.10f : 10.45f;
}


/*******************************************************************************
 * Ultrasonic Sensor
 ******************************************************************************/

/*
 * Initialize ultrasonic sensor GPIO.
 */
static void Ultrasonic_Init(void)
{
    gpio_pin_config_t trigConfig =
    {
        kGPIO_DigitalOutput,
        0U
    };

    gpio_pin_config_t echoConfig =
    {
        kGPIO_DigitalInput,
        0U
    };

    GPIO_PinInit(
        ULTRASONIC_TRIG_PORT,
        ULTRASONIC_TRIG_PIN,
        &trigConfig
    );

    GPIO_PinInit(
        ULTRASONIC_ECHO_PORT,
        ULTRASONIC_ECHO_PIN,
        &echoConfig
    );

    GPIO_PinWrite(
        ULTRASONIC_TRIG_PORT,
        ULTRASONIC_TRIG_PIN,
        0U
    );
}


/*
 * Initialize DWT cycle counter.
 *
 * Used for microsecond timing.
 */
static void Ultrasonic_TimerInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0U;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


/*
 * Get elapsed microseconds from DWT counter.
 */
static uint32_t Ultrasonic_GetUs(void)
{
    uint32_t cyclesPerUs;

    cyclesPerUs =
        SystemCoreClock / 1000000U;

    if (cyclesPerUs == 0U)
    {
        cyclesPerUs = 1U;
    }

    return DWT->CYCCNT / cyclesPerUs;
}


/*
 * Read HC-SR04 distance.
 *
 * Distance in cm:
 *
 *     distance = echo_time_us / 58
 *
 * Returns:
 *
 *     > 0  = valid distance
 *     0    = timeout
 */
static float ReadUltrasonicDistance(void)
{
    uint32_t startUs;
    uint32_t echoTimeUs;


    /* Make sure TRIG starts LOW */
    GPIO_PinWrite(
        ULTRASONIC_TRIG_PORT,
        ULTRASONIC_TRIG_PIN,
        0U
    );

    SDK_DelayAtLeastUs(
        5U,
        SystemCoreClock
    );


    /* 10 us trigger pulse */
    GPIO_PinWrite(
        ULTRASONIC_TRIG_PORT,
        ULTRASONIC_TRIG_PIN,
        1U
    );

    SDK_DelayAtLeastUs(
        10U,
        SystemCoreClock
    );

    GPIO_PinWrite(
        ULTRASONIC_TRIG_PORT,
        ULTRASONIC_TRIG_PIN,
        0U
    );


    /* Wait for ECHO rising edge */
    startUs = Ultrasonic_GetUs();

    while (
        GPIO_PinRead(
            ULTRASONIC_ECHO_PORT,
            ULTRASONIC_ECHO_PIN
        ) == 0U
    )
    {
        if (
            (Ultrasonic_GetUs() - startUs)
            >= ULTRASONIC_TIMEOUT_US
        )
        {
            return 0.0f;
        }
    }


    /* Measure ECHO HIGH pulse width */
    startUs = Ultrasonic_GetUs();

    while (
        GPIO_PinRead(
            ULTRASONIC_ECHO_PORT,
            ULTRASONIC_ECHO_PIN
        ) != 0U
    )
    {
        if (
            (Ultrasonic_GetUs() - startUs)
            >= ULTRASONIC_TIMEOUT_US
        )
        {
            return 0.0f;
        }
    }


    echoTimeUs =
        Ultrasonic_GetUs() - startUs;


    return ((float)echoTimeUs / 58.0f);
}


/*******************************************************************************
 * IR Sensor
 ******************************************************************************/

/*
 * Initialize IR obstacle sensor.
 *
 * P1_14 = GPIO1[14]
 */
static void IR_Init(void)
{
    gpio_pin_config_t irConfig =
    {
        kGPIO_DigitalInput,
        0U
    };


    GPIO_PinInit(
        IR_PORT,
        IR_PIN,
        &irConfig
    );
}


/*
 * Read raw IR sensor state.
 *
 * Returns:
 *
 *     0 = LOW
 *     1 = HIGH
 */
static uint8_t ReadIRSensor(void)
{
    return GPIO_PinRead(
        IR_PORT,
        IR_PIN
    );
}


/*
 * Convert IR signal to obstacle status.
 *
 * For the common IR obstacle module:
 *
 *     LOW  = obstacle detected
 *     HIGH = no obstacle
 *
 * Returns:
 *
 *     1 = obstacle detected
 *     0 = no obstacle
 */
static uint8_t GetIRObstacleStatus(void)
{
    uint8_t irState;

    irState = ReadIRSensor();

    if (irState == 0U)
    {
        return 1U;
    }

    return 0U;
}


/*******************************************************************************
 * CAN Transmission
 ******************************************************************************/

/*
 * Build standard sensor CAN frame.
 *
 * Payload:
 *
 * Byte 0 = value high byte
 * Byte 1 = value low byte
 * Byte 2 = sensor type
 * Byte 3 = node ID
 * Byte 4 = reserved
 * Byte 5 = reserved
 * Byte 6 = reserved
 * Byte 7 = reserved
 *
 * Value is encoded as value x 100.
 */
static void BuildSensorFrame(
    uint32_t canId,
    uint8_t sensorType,
    float value
)
{
    uint16_t rawValue;


    if (value < 0.0f)
    {
        rawValue = 0U;
    }
    else if (value > 655.35f)
    {
        rawValue = 65535U;
    }
    else
    {
        rawValue =
            (uint16_t)((value * 100.0f) + 0.5f);
    }


    memset(
        &txFrame,
        0,
        sizeof(txFrame)
    );


    txFrame.id =
        FLEXCAN_ID_STD(canId);

    txFrame.format =
        (uint8_t)kFLEXCAN_FrameFormatStandard;

    txFrame.type =
        (uint8_t)kFLEXCAN_FrameTypeData;

    txFrame.length =
        CAN_DLC;


    /* Value x 100, big endian */
    txFrame.dataByte0 =
        (uint8_t)((rawValue >> 8U) & 0xFFU);

    txFrame.dataByte1 =
        (uint8_t)(rawValue & 0xFFU);


    /* Sensor type */
    txFrame.dataByte2 =
        sensorType;


    /* Node ID */
    txFrame.dataByte3 =
        (uint8_t)THIS_NODE_ID;


    /* Reserved */
    txFrame.dataByte4 = 0x00U;
    txFrame.dataByte5 = 0x00U;
    txFrame.dataByte6 = 0x00U;
    txFrame.dataByte7 = 0x00U;
}


/*
 * Send normal sensor CAN frame.
 */
static void SendSensor(
    uint32_t canId,
    uint8_t sensorType,
    float value
)
{
    BuildSensorFrame(
        canId,
        sensorType,
        value
    );


    (void)FLEXCAN_TransferSendBlocking(
        EXAMPLE_CAN,
        TX_MB_NUM,
        &txFrame
    );
}


/*******************************************************************************
 * IR CAN Transmission
 ******************************************************************************/

/*
 * Send IR obstacle status.
 *
 * CAN ID:
 *
 *     0x117
 *
 * Payload:
 *
 * Byte 0:
 *     0 = no obstacle
 *     1 = obstacle detected
 *
 * Byte 1:
 *     reserved
 *
 * Byte 2:
 *     0x07 = IR sensor
 *
 * Byte 3:
 *     Node ID
 *
 * Byte 4-7:
 *     reserved
 */
static void SendIRSensor(uint8_t obstacle)
{
    memset(
        &txFrame,
        0,
        sizeof(txFrame)
    );


    txFrame.id =
        FLEXCAN_ID_STD(CAN_ID_IR);

    txFrame.format =
        (uint8_t)kFLEXCAN_FrameFormatStandard;

    txFrame.type =
        (uint8_t)kFLEXCAN_FrameTypeData;

    txFrame.length =
        CAN_DLC;


    /* IR obstacle status */
    txFrame.dataByte0 =
        obstacle;


    /* Reserved */
    txFrame.dataByte1 =
        0x00U;


    /* Sensor type */
    txFrame.dataByte2 =
        SENSOR_TYPE_IR;


    /* Node ID */
    txFrame.dataByte3 =
        (uint8_t)THIS_NODE_ID;


    /* Reserved */
    txFrame.dataByte4 = 0x00U;
    txFrame.dataByte5 = 0x00U;
    txFrame.dataByte6 = 0x00U;
    txFrame.dataByte7 = 0x00U;


    (void)FLEXCAN_TransferSendBlocking(
        EXAMPLE_CAN,
        TX_MB_NUM,
        &txFrame
    );
}


/*******************************************************************************
 * CAN Receive Message Buffer
 ******************************************************************************/

/*
 * Non-blocking CAN RX message reader.
 */
static status_t ReadRxMbNonBlocking(
    CAN_Type *base,
    uint8_t mbIdx,
    flexcan_frame_t *pRxFrame
)
{
    uint32_t cs_temp;
    uint8_t rx_code;


    cs_temp =
        base->MB[mbIdx].CS;


    rx_code =
        (uint8_t)(
            (cs_temp & CAN_CS_CODE_MASK)
            >> CAN_CS_CODE_SHIFT
        );


    if (
        (0x2U == rx_code) ||
        (0x6U == rx_code)
    )
    {
        pRxFrame->id =
            base->MB[mbIdx].ID &
            (CAN_ID_EXT_MASK | CAN_ID_STD_MASK);


        pRxFrame->format =
            (cs_temp & CAN_CS_IDE_MASK)
            != 0U
            ?
            (uint8_t)kFLEXCAN_FrameFormatExtend
            :
            (uint8_t)kFLEXCAN_FrameFormatStandard;


        pRxFrame->type =
            (cs_temp & CAN_CS_RTR_MASK)
            != 0U
            ?
            (uint8_t)kFLEXCAN_FrameTypeRemote
            :
            (uint8_t)kFLEXCAN_FrameTypeData;


        pRxFrame->length =
            (uint8_t)(
                (cs_temp & CAN_CS_DLC_MASK)
                >> CAN_CS_DLC_SHIFT
            );


        pRxFrame->timestamp =
            (uint16_t)(
                (cs_temp & CAN_CS_TIME_STAMP_MASK)
                >> CAN_CS_TIME_STAMP_SHIFT
            );


        pRxFrame->dataWord0 =
            base->MB[mbIdx].WORD0;

        pRxFrame->dataWord1 =
            base->MB[mbIdx].WORD1;


        /* Reset RX MB ID */
        base->MB[mbIdx].ID =
            FLEXCAN_ID_STD(CAN_ID_NODE_CMD);


        /* Unlock message buffer */
        (void)base->TIMER;


        return kStatus_Success;
    }


    /* Unlock message buffer */
    (void)base->TIMER;


    return kStatus_Fail;
}


/*******************************************************************************
 * QNX Command Receiver
 ******************************************************************************/

/*
 * Checks commands received from Raspberry Pi QNX.
 *
 * Direct command:
 *
 *     0x201 for Node 1
 *     0x202 for Node 2
 *
 * Broadcast:
 *
 *     0x200
 *
 * Byte 0:
 *
 *     0x01 = START
 *     0x00 = STOP
 */
static void CheckIncomingCommands(void)
{
    if (
        0U != FLEXCAN_GetMbStatusFlags(
            EXAMPLE_CAN,
            1U << RX_MB_NUM
        )
    )
    {
        if (
            ReadRxMbNonBlocking(
                EXAMPLE_CAN,
                RX_MB_NUM,
                &rxFrame
            )
            == kStatus_Success
        )
        {
            uint32_t receivedId;

            uint8_t cmd;


            receivedId =
                rxFrame.id >>
                CAN_ID_STD_SHIFT;


            cmd =
                rxFrame.dataByte0;


            if (
                (receivedId == CAN_ID_NODE_CMD) ||
                (receivedId == CAN_ID_BROADCAST_CMD)
            )
            {
                if (cmd == 0x01U)
                {
                    g_sensorActive = true;

                    LOG_INFO((
                        ">>> [CMD RECEIVED] "
                        "START Command from QNX. "
                        "Sensor streaming RESUMED.\r\n"
                    ));
                }
                else if (cmd == 0x00U)
                {
                    g_sensorActive = false;

                    LOG_INFO((
                        ">>> [CMD RECEIVED] "
                        "STOP Command from QNX. "
                        "Sensor streaming PAUSED.\r\n"
                    ));
                }
            }
        }


        /* Clear RX flag */
        FLEXCAN_ClearMbStatusFlags(
            EXAMPLE_CAN,
            1U << RX_MB_NUM
        );
    }
}


/*******************************************************************************
 * Main
 ******************************************************************************/

int main(void)
{
    flexcan_config_t flexcanConfig;

    flexcan_rx_mb_config_t mbConfig;

    uint32_t cycle = 0U;


    /***************************************************************************
     * Board initialization
     ***************************************************************************/

    BOARD_InitHardware();


    /***************************************************************************
     * Ultrasonic initialization
     ***************************************************************************/

    Ultrasonic_TimerInit();

    Ultrasonic_Init();


    /***************************************************************************
     * IR initialization
     ***************************************************************************/

    IR_Init();


    /***************************************************************************
     * Startup messages
     ***************************************************************************/

    LOG_INFO((
        "\r\n"
        "====================================================\r\n"
    ));

    LOG_INFO((
        "   FRDM-MCXA156 CAN NODE %d\r\n",
        THIS_NODE_ID
    ));

    LOG_INFO((
        "   CAN + ULTRASONIC + IR SENSOR\r\n"
    ));

    LOG_INFO((
        "====================================================\r\n"
    ));

    LOG_INFO((
        "Bitrate: 500 kbps\r\n"
    ));

    LOG_INFO((
        "CAN RX: P1_12\r\n"
    ));

    LOG_INFO((
        "CAN TX: P1_13\r\n"
    ));

    LOG_INFO((
        "Ultrasonic: TRIG=P3_27 | ECHO=P3_28 | "
        "CAN ID=0x116\r\n"
    ));

    LOG_INFO((
        "IR Sensor: OUT=P1_14 | CAN ID=0x117\r\n"
    ));

    LOG_INFO((
        "IR logic: LOW = obstacle detected\r\n"
    ));

    LOG_INFO((
        "====================================================\r\n\r\n"
    ));


    /***************************************************************************
     * FlexCAN initialization
     ***************************************************************************/

    FLEXCAN_GetDefaultConfig(
        &flexcanConfig
    );


    flexcanConfig.bitRate =
        500000U;


    flexcanConfig.enableIndividMask =
        true;


#if defined(EXAMPLE_CAN_CLK_SOURCE)

    flexcanConfig.clkSrc =
        EXAMPLE_CAN_CLK_SOURCE;

#endif


    FLEXCAN_Init(
        EXAMPLE_CAN,
        &flexcanConfig,
        EXAMPLE_CAN_CLK_FREQ
    );


    /***************************************************************************
     * TX Message Buffer
     ***************************************************************************/

    FLEXCAN_SetTxMbConfig(
        EXAMPLE_CAN,
        TX_MB_NUM,
        true
    );


    /***************************************************************************
     * RX Message Buffer
     *
     * Accept:
     *
     *     0x200 broadcast
     *     0x201 Node 1
     *     0x202 Node 2
     *
     * Mask:
     *
     *     0x7F0
     ***************************************************************************/

    FLEXCAN_SetRxIndividualMask(
        EXAMPLE_CAN,
        RX_MB_NUM,
        FLEXCAN_RX_MB_STD_MASK(
            0x7F0U,
            0,
            0
        )
    );


    mbConfig.format =
        kFLEXCAN_FrameFormatStandard;


    mbConfig.type =
        kFLEXCAN_FrameTypeData;


    mbConfig.id =
        FLEXCAN_ID_STD(
            CAN_ID_NODE_CMD
        );


    FLEXCAN_SetRxMbConfig(
        EXAMPLE_CAN,
        RX_MB_NUM,
        &mbConfig,
        true
    );


    /***************************************************************************
     * Ready
     ***************************************************************************/

    LOG_INFO((
        "CAN ready. Starting transmission loop...\r\n"
    ));

    LOG_INFO((
        "Ultrasonic: TRIG=P3_27 | ECHO=P3_28 | "
        "CAN ID=0x116\r\n"
    ));

    LOG_INFO((
        "IR: OUT=P1_14 | CAN ID=0x117\r\n\r\n"
    ));


    /***************************************************************************
     * Main loop
     ***************************************************************************/

    while (true)
    {
        /***********************************************************************
         * Check commands from QNX
         ***********************************************************************/

        CheckIncomingCommands();


        /***********************************************************************
         * Sensor streaming active?
         ***********************************************************************/

        if (g_sensorActive)
        {
            float temp;
            float distanceCm;

            float hum;
            float spd;
            float volt;
            float curr;

            uint8_t irObstacle;


            cycle++;


            /*******************************************************************
             * Read existing sensors
             *******************************************************************/

            temp =
                ReadBuiltInTemperature();


            hum =
                ReadHumidity();


            spd =
                ReadSpeed();


            volt =
                ReadBatteryVoltage();


            curr =
                ReadBatteryCurrent();


            /*******************************************************************
             * Transmit existing sensor telemetry
             *******************************************************************/

            SendSensor(
                CAN_ID_TEMPERATURE,
                SENSOR_TYPE_TEMPERATURE,
                temp
            );


            SendSensor(
                CAN_ID_HUMIDITY,
                SENSOR_TYPE_HUMIDITY,
                hum
            );


            SendSensor(
                CAN_ID_SPEED,
                SENSOR_TYPE_SPEED,
                spd
            );


            SendSensor(
                CAN_ID_BATTERY_VOLTAGE,
                SENSOR_TYPE_BATTERY_VOLTAGE,
                volt
            );


            SendSensor(
                CAN_ID_BATTERY_CURRENT,
                SENSOR_TYPE_BATTERY_CURRENT,
                curr
            );


            /*******************************************************************
             * Ultrasonic measurement
             *******************************************************************/

            distanceCm =
                ReadUltrasonicDistance();


            SendSensor(
                CAN_ID_ULTRASONIC,
                SENSOR_TYPE_ULTRASONIC,
                distanceCm
            );


            /*******************************************************************
             * IR measurement
             *******************************************************************/

            irObstacle =
                GetIRObstacleStatus();


            /*******************************************************************
             * Send IR status over CAN
             *******************************************************************/

            SendIRSensor(
                irObstacle
            );


            /*******************************************************************
             * Convert values for UART display
             *******************************************************************/

            {
                uint32_t temp100;
                uint32_t hum10;
                uint32_t spd10;
                uint32_t volt100;
                uint32_t curr100;
                uint32_t dist100;


                temp100 =
                    (uint32_t)(
                        (temp * 100.0f) + 0.5f
                    );


                hum10 =
                    (uint32_t)(
                        (hum * 10.0f) + 0.5f
                    );


                spd10 =
                    (uint32_t)(
                        (spd * 10.0f) + 0.5f
                    );


                volt100 =
                    (uint32_t)(
                        (volt * 100.0f) + 0.5f
                    );


                curr100 =
                    (uint32_t)(
                        (curr * 100.0f) + 0.5f
                    );


                dist100 =
                    (uint32_t)(
                        (distanceCm * 100.0f) + 0.5f
                    );


                /****************************************************************
                 * Valid ultrasonic measurement
                 ****************************************************************/

                if (distanceCm > 0.0f)
                {
                    LOG_INFO((
                        "[Node %d #%u] "
                        "Temp: %lu.%02lu C | "
                        "Hum: %lu.%01lu %% | "
                        "Spd: %lu.%01lu km/h | "
                        "V: %lu.%02lu V | "
                        "I: %lu.%02lu A | "
                        "Distance: %lu.%02lu cm | "
                        "IR: %s\r\n",

                        THIS_NODE_ID,

                        cycle,

                        (unsigned long)
                            (temp100 / 100U),

                        (unsigned long)
                            (temp100 % 100U),

                        (unsigned long)
                            (hum10 / 10U),

                        (unsigned long)
                            (hum10 % 10U),

                        (unsigned long)
                            (spd10 / 10U),

                        (unsigned long)
                            (spd10 % 10U),

                        (unsigned long)
                            (volt100 / 100U),

                        (unsigned long)
                            (volt100 % 100U),

                        (unsigned long)
                            (curr100 / 100U),

                        (unsigned long)
                            (curr100 % 100U),

                        (unsigned long)
                            (dist100 / 100U),

                        (unsigned long)
                            (dist100 % 100U),

                        irObstacle
                            ? "OBSTACLE"
                            : "CLEAR"
                    ));
                }


                /****************************************************************
                 * Ultrasonic timeout
                 ****************************************************************/

                else
                {
                    LOG_INFO((
                        "[Node %d #%u] "
                        "Temp: %lu.%02lu C | "
                        "Hum: %lu.%01lu %% | "
                        "Spd: %lu.%01lu km/h | "
                        "V: %lu.%02lu V | "
                        "I: %lu.%02lu A | "
                        "Distance: TIMEOUT | "
                        "IR: %s\r\n",

                        THIS_NODE_ID,

                        cycle,

                        (unsigned long)
                            (temp100 / 100U),

                        (unsigned long)
                            (temp100 % 100U),

                        (unsigned long)
                            (hum10 / 10U),

                        (unsigned long)
                            (hum10 % 10U),

                        (unsigned long)
                            (spd10 / 10U),

                        (unsigned long)
                            (spd10 % 10U),

                        (unsigned long)
                            (volt100 / 100U),

                        (unsigned long)
                            (volt100 % 100U),

                        (unsigned long)
                            (curr100 / 100U),

                        (unsigned long)
                            (curr100 % 100U),

                        irObstacle
                            ? "OBSTACLE"
                            : "CLEAR"
                    ));
                }
            }
        }


        /***********************************************************************
         * Non-blocking delay
         *
         * Check QNX commands every 10 ms.
         ***********************************************************************/

        for (
            uint32_t elapsed = 0U;
            elapsed < SENSOR_INTERVAL_MS;
            elapsed += 10U
        )
        {
            CheckIncomingCommands();


            SDK_DelayAtLeastUs(
                10000U,
                SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY
            );
        }
    }
}