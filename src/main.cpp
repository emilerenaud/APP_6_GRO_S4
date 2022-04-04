#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>

#include "pcm_audio.hpp"
#include <limits.h>
#include <event_groups.h>


using Sawtooth = Oscil<SAW2048_NUM_CELLS, SAMPLE_RATE>;
using SquareWv = Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, SAMPLE_RATE>;

#define PIN_SW1 2
#define PIN_SW2 3
#define PIN_RV1 A0
#define PIN_RV2 A1
#define PIN_RV3 A2
#define PIN_RV4 A3

// Variables
int potValue[4] = {0,0,0,0};
EventGroupHandle_t eventGroupButton;


// Prototypes
void readPotentiometer(void *);
void ISR_button1(void);
void ISR_button2(void);
void waitButton(void *);


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
    
    Serial.begin(9600);

    // Oscillator.
    squarewv_ = SquareWv(SQUARE_NO_ALIAS_2048_DATA);
    sawtooth_ = SquareWv(SAW2048_DATA);
    setNoteHz(110.0);

    pcmSetup();

    eventGroupButton = xEventGroupCreate();

    attachInterrupt(digitalPinToInterrupt(PIN_SW1),ISR_button1,CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_SW2),ISR_button2,CHANGE);

    xTaskCreate(readPotentiometer,"readPot",128,NULL,2,NULL);

    xTaskCreate(waitButton,"waitButton",128,NULL,2,NULL);


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

void readPotentiometer(void *)
{
    for(;;)
    {
        for(int i=0; i<4; i++)
        {
        potValue[i] = analogRead(i);
        // Serial.print(potValue[i]);
        }
        // Serial.println();
        vTaskDelay(100/portTICK_PERIOD_MS); // 10Hz -> 100ms
    }
}

void ISR_button1()
{
    if(digitalRead(PIN_SW1))
    {
        xEventGroupSetBitsFromISR(eventGroupButton,0x01,pdFALSE);
    }
    else
    {
        xEventGroupClearBitsFromISR(eventGroupButton,0x01);
    }
}

void ISR_button2()
{
    if(digitalRead(PIN_SW2))
    {
        xEventGroupSetBitsFromISR(eventGroupButton,0x02,pdFALSE);
    }
    else
    {
        xEventGroupClearBitsFromISR(eventGroupButton,0x02);
    }
}

void waitButton(void *)
{
    for(;;)
    {
        // xEventGroupWaitBits(
        //             eventGroupButton,
        //             0x03,    // Check bit 0 and 1.
        //             pdTRUE,  // Clear bit on exit.
        //             pdTRUE, // Wait all the bits
        //             portMAX_DELAY); // no Timeout
        // Serial.println("Waked up 2");
    }
}

void task_pot_RV1(void*)
{
    while(1)
    {
        int16_t valeur_pot = analogRead(PIN_RV1);
        vTaskDelay(6);
    }
    

}


int8_t processVCF_test(int8_t vco) // a verifier
{
    static int8_t x [3] = {0,0,0}; 
    static int8_t y [3] = {0,0,0}; 
    
    float b_org [3] ={0.0200833655642112, 0.0401667311284225, 0.0200833655642112};
    float a_org [3] ={1, -1.56101807580072, 0.641351538057563};
    int16_t b [3] = {b_org[1]*256, b_org[2]*256, b_org[3]*256};
    int16_t a [3] = {a_org[1]*256, a_org[2]*256, a_org[3]*256};

    for(int8_t i = 2; i > 0; i--)
    {
        if(i > 0)
        {
            x[i] = x[i-1];
            y[i] = y[i-1];
        }
    }
    x[0] = vco;
    int16_t yi = b[0]*x[0] + b[1]*x[1] + b[2]*x[2] + a[1]*y[1] + a[2]*y[2];
    y[0] = 0xFF & (yi >> 8);

    return y[0];
}

int8_t processVCF(int8_t vco) // partie 15.3 labo 3 a terminer...
{

}