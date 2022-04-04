#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>

#include "pcm_audio.hpp"

using Sawtooth = Oscil<SAW2048_NUM_CELLS, SAMPLE_RATE>;
using SquareWv = Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, SAMPLE_RATE>;

#define PIN_SW1 2
#define PIN_SW2 3
#define PIN_RV1 A0
#define PIN_RV2 A1
#define PIN_RV3 A2
#define PIN_RV4 A3

int potValue[4] = {0,0,0,0};
void readPotentiometer(void *);

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

    Serial.println("Synth prototype ready");

    xTaskCreate(readPotentiometer,"readPot",128,NULL,2,NULL);
}

void loop()
{    
    digitalWrite(LED_BUILTIN, 1);
    delay(500);
    digitalWrite(LED_BUILTIN, 0);
    delay(500);
}

void readPotentiometer(void *)
{
    for(;;)
    {
        for(int i=0; i<4; i++)
        {
        potValue[i] = analogRead(i);
        Serial.print(potValue[i]);
        }
        // Serial.println();
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}