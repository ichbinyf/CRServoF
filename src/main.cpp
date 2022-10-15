#include <Arduino.h>
#include <CrsfSerial.h>
#include <median.h>
#include "target.h"

#define _DEBUG

#define NUM_OUTPUTS 8

// Configuration
// Map input CRSF channels (1-based, up to 16 for CRSF, 12 for ELRS) to outputs 1-8
// use a negative number to invert the signal (i.e. +100% becomes -100%)0
//constexpr int OUTPUT_MAP[NUM_OUTPUTS] = { 1, 2, 3, 4, 6, 7, 8, 12 };
constexpr int OUTPUT_MAP[NUM_OUTPUTS] = { 1, 2, 3, 4, 5, 6, 7, 8 };
// The failsafe action for each channel (fsaNoPulses, fsaHold, or microseconds)
constexpr int OUTPUT_FAILSAFE[NUM_OUTPUTS] = {
    1500, 1500, 988, 1500,                  // ch1-ch4
    fsaHold, fsaHold, fsaHold, fsaNoPulses  // ch5-ch8
    };
// Define the pins used to output servo PWM, must use hardware PWM,
// and change HardwareTimer targets below if the timers change
constexpr PinName OUTPUT_PINS[NUM_OUTPUTS] = { OUTPUT_PIN_MAP };

#define PWM_FREQ_HZ     50
#define VBAT_INTERVAL   500
#define VBAT_SMOOTH     5
// Scale used to calibrate or change to CRSF standard 0.1 scale
#define VBAT_SCALE      1.0

// Local Variables
static HardwareSerial CrsfSerialStream(USART_INPUT);
static CrsfSerial crsf(CrsfSerialStream);
static int g_OutputsUs[NUM_OUTPUTS];
static struct tagConnectionState {
    uint32_t lastVbatRead;
    MedianAvgFilter<unsigned int, VBAT_SMOOTH>vbatSmooth;
    unsigned int vbatValue;

    char serialInBuff[64];
    uint8_t serialInBuffLen;
    bool serialEcho;
} g_State;

static void crsfShiftyByte(uint8_t b)
{
    // A shifty byte is usually just log messages from ELRS
    //Serial.write(b);
}

static void servoSetUs(unsigned int servo, int usec)
{
    if (usec > 0)
    {
        // 0 means it was disabled previously, enable OUTPUT mode
        if (g_OutputsUs[servo] == 0)
            // vv pinMode(p, OUTPUT) vv
            pin_function(OUTPUT_PINS[servo], STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
        pwm_start(OUTPUT_PINS[servo], PWM_FREQ_HZ, usec, MICROSEC_COMPARE_FORMAT);

    }
    else
    {
        pwm_stop(OUTPUT_PINS[servo]);
        // vv pinMode(p, INPUT_PULLDOWN) vv
        pin_function(OUTPUT_PINS[servo], STM_PIN_DATA(STM_MODE_INPUT, GPIO_PULLDOWN, 0));

    }
    g_OutputsUs[servo] = usec;
}

static void packetChannels()
{
    for (unsigned int out=0; out<NUM_OUTPUTS; ++out)
    {
        const int chInput = OUTPUT_MAP[out];
        int usOutput; 
        if (chInput > 0)
            usOutput = crsf.getChannel(chInput);
        else
        {
            // if chInput is negative, invert the channel output
            usOutput = crsf.getChannel(-chInput);
            // (1500 - usOutput) + 1500
            usOutput = 3000 - usOutput;
        }
        servoSetUs(out, usOutput);
        
    }
#ifdef _DEBUG
     for (unsigned int ch=0; ch<4; ++ch)
     {
         Serial.print(ch);
         Serial.print('=');
         Serial.print(crsf.getChannel(ch));
         Serial.print('\t');
     }
     Serial.print("\r\n");
#endif
}

static void packetLinkStatistics(crsfLinkStatistics_t *link)
{
  //Serial.print(link->uplink_RSSI_1, DEC);
  //Serial.println("dBm");
}

static void crsfLinkUp()
{
  Serial.print("\r\nlink up\r\n");
    digitalWrite(DPIN_LED, HIGH ^ LED_INVERTED);
}

static void crsfLinkDown()
{
  Serial.print("\r\nlink down\r\n");
    digitalWrite(DPIN_LED, LOW ^ LED_INVERTED);

    // Perform the failsafe action
    for (unsigned int out=0; out<NUM_OUTPUTS; ++out)
    {
        if (OUTPUT_FAILSAFE[out] == fsaNoPulses)
            servoSetUs(out, 0);
        else if (OUTPUT_FAILSAFE[out] != fsaHold)
            servoSetUs(out, OUTPUT_FAILSAFE[out]);
        // else fsaHold does nothing, keep the same value
    }
 }

static void checkVbatt()
{
    if (millis() - g_State.lastVbatRead < (VBAT_INTERVAL / VBAT_SMOOTH))
        return;
    g_State.lastVbatRead = millis();

    unsigned int idx = g_State.vbatSmooth.add(analogRead(APIN_VBAT));
    if (idx != 0)
        return;

    unsigned int adc = g_State.vbatSmooth;
    g_State.vbatValue = 330U * adc * (VBAT_R1 + VBAT_R2) / VBAT_R2 / ((1 << 12) - 1);

    uint8_t crsfbatt[CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE] = { 0 };
    uint16_t scaledVoltage = g_State.vbatValue * VBAT_SCALE;
    crsfbatt[0] = scaledVoltage >> 8;
    crsfbatt[1] = scaledVoltage & 0xff;
    crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfbatt, sizeof(crsfbatt));

    //Serial.print("ADC="); Serial.print(adc, DEC);
    //Serial.print(" "); Serial.print(g_State.vbatValue, DEC); Serial.println("V");
}

static void setupCrsf()
{
    crsf.onLinkUp = &crsfLinkUp;
    crsf.onLinkDown = &crsfLinkDown;
    crsf.onShiftyByte = &crsfShiftyByte;
    crsf.onPacketChannels = &packetChannels;
    crsf.onPacketLinkStatistics = packetLinkStatistics;
    //crsf.setPassthroughMode(true);
}

static void setupGpio()
{
    pinMode(DPIN_LED, OUTPUT);
    digitalWrite(DPIN_LED, LOW ^ LED_INVERTED);
    analogReadResolution(12);

    // The servo outputs are initialized when the
    // first channels packet comes in and sets the PWM
    // output value, to prevent them from jerking around
    // on startup
}

void setup()
{
    Serial.begin(115200);
    setupGpio();
    setupCrsf();

}

void loop()
{
    crsf.loop();
    //checkVbatt();
}




