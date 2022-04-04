#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>

#include "pcm_audio.hpp"
#include <limits.h>
#include <event_groups.h>
#include <math.h>

using Sawtooth = Oscil<SAW2048_NUM_CELLS, SAMPLE_RATE>;
using SquareWv = Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, SAMPLE_RATE>;

#define PIN_SW1 2
#define PIN_SW2 3
#define PIN_RV1 A0
#define PIN_RV2 A1
#define PIN_RV3 A2
#define PIN_RV4 A3

// Variables
int potValue[4] = {0, 0, 0, 0};
EventGroupHandle_t eventGroupButton;

// Prototypes
void readPotentiometer(void *);
void ISR_button1(void);
void ISR_button2(void);
void waitButton(void *);
int8_t processVCF(int8_t vco);
void fillPCM(void *);

void task1(void *);
void task2(void *);

SquareWv squarewv_;
Sawtooth sawtooth_;

void setNoteHz(float note)
{
    squarewv_.setFreq(note);
    sawtooth_.setFreq(note);
}

int8_t nextSample()
{
    // VCO
    int8_t vco = sawtooth_.next() + squarewv_.next();

    // VCF (disabled)
    int8_t vcf = vco;

    // VCA (disabled)
    int8_t vca = vcf;

    int8_t output = vca;

    return output;
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_SW1, INPUT);

    Serial.begin(115200);

    // Oscillator.
    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);
    setNoteHz(440.0);

    pcmSetup();

    eventGroupButton = xEventGroupCreate();

    attachInterrupt(digitalPinToInterrupt(PIN_SW1), ISR_button1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SW2), ISR_button2, CHANGE);

    xTaskCreate(readPotentiometer,"readPot",128,NULL,1,NULL);
    xTaskCreate(waitButton,"waitButton",128,NULL,1,NULL);
    xTaskCreate(fillPCM,"fillPCM",500,NULL,3,NULL);

    // xTaskCreate(task1, "task1", 128, NULL, 1, NULL);
    // xTaskCreate(task2, "task2", 128, NULL, 1, NULL);
    
    Serial.println("Synth prototype ready");
}

void loop()
{
    // xEventGroupWaitBits(
    //                 eventGroupButton,
    //                 0x01,
    //                 pdTRUE,
    //                 pdFALSE,
    //                 portMAX_DELAY);
    //     // xTaskNotifyWait(0x00,ULONG_MAX,&ulNotifyValue,portMAX_DELAY);
    //     Serial.println("Waked up 2");
}

void task1(void *)
{
    for (;;)
    {
        Serial.println("Task 1");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void task2(void *)
{
    for (;;)
    {
        Serial.println("Task 2");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void readPotentiometer(void *)
{
    for (;;)
    {
        for (int i = 0; i < 4; i++)
        {
            potValue[i] = analogRead(i);
            // Serial.print(potValue[i]);
        }
        // Serial.print(potValue[0]);
        // Serial.println();
        vTaskDelay(100 / portTICK_PERIOD_MS); // 10Hz -> 100ms
        setNoteHz(map(potValue[0], 0, 1023, 0, 4000));
    }
}

void ISR_button1()
{
    if (digitalRead(PIN_SW1))
    {
        xEventGroupSetBitsFromISR(eventGroupButton, 0x01, pdFALSE);
    }
    else
    {
        xEventGroupClearBitsFromISR(eventGroupButton, 0x01);
    }
}

void ISR_button2()
{
    if (digitalRead(PIN_SW2))
    {
        xEventGroupSetBitsFromISR(eventGroupButton, 0x02, pdFALSE);
    }
    else
    {
        xEventGroupClearBitsFromISR(eventGroupButton, 0x02);
    }
}

void waitButton(void *)
{
    for (;;)
    {
        xEventGroupWaitBits(
            eventGroupButton,
            0x02,           // Check bit 0 and 1.
            pdTRUE,         // Clear bit on exit.
            pdTRUE,         // Wait all the bits
            portMAX_DELAY); // no Timeout
        Serial.println("Waked up 2");
        // processVCF(0);
    }
}

int8_t processVCF(int8_t vco) // partie 15.3 labo 3 a terminer...
{
    static int8_t y[3] = {0, 0, 0};
    int32_t result = 0;
    int8_t x = vco;

    for (int8_t i = 2; i > 0; i--)
    {
        if (i > 0)
        {
            y[i] = y[i - 1];
        }
    }

    float q_org = (potValue[1] * 1.0f) / 1023.0f;   // 0-1
    float f_org = (potValue[2] * (4000.0f * ((2.0f * PI) / 8000.0f))) / 1023.0f; // rad/sample
    int16_t q = q_org * 256;
    int16_t f = f_org * 256;
    int16_t fb = q + q / (1 + f);

    result = (f ^ 2) * x + (2 - ((2 * f) + (f * fb)) - ((f ^ 2) * fb)) * y[1] - (1 - ((2 * f) + (f * fb)) + (f ^ 2) - ((f ^ 2) * fb)) * y[2];

    y[0] = 0xFF & (result >> 8);
    return y[0];

    // Serial.print("  POT1: ");
    // // Serial.print(potValue[1]);
    // Serial.print(q);
    // Serial.print("  POT2: ");
    // // Serial.print(potValue[2]);
    // Serial.print(f);
    // Serial.print("  fb: ");
    // Serial.println(fb);
}

void fillPCM(void *)
{
    for (;;)
    {
        if(!pcmBufferFull())
        {
            pcmAddSample(nextSample());
            // Serial.println("test");
        }
        // while(!pcmBufferEmpty())
        // {

        // }
        // while (!pcmBufferFull())
        // {
        //     pcmAddSample(nextSample());
        // }
        // 
        // while(!pcmBufferEmpty())
        // {
        // taskYIELD();
        // vTaskDelay(2);
        // }
        vTaskDelay(1);
        // taskYIELD();
    }
}