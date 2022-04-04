#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define SAMPLE_RATE 8000

/* 8-bit, 8 kHz PCM audio playing through PWM. 
 * Adapted from "speaker_pcm" found on Arduino Playground:
 *   http://playground.arduino.cc/Code/PCMAudio
 */

#define SPEAKER_PIN 10 // Mega

#define PCM_BUFFER_SIZE 64

int8_t              pcm_buffer_[PCM_BUFFER_SIZE];
int                 pcm_buffer_in_;
int                 pcm_buffer_out_;

bool pcmBufferFull()
{
    // Check if by incrementing the input pointer by one, we will overlap the output pointer.
    if (((pcm_buffer_in_ + 1) % PCM_BUFFER_SIZE) == pcm_buffer_out_) {
        return true;
    } else {
        return false;
    }
}

bool pcmBufferEmpty()
{
    // Check if by reading a sample we would move ahead of the input pointer.
    if (pcm_buffer_out_ == pcm_buffer_in_) {
        return true;
    } else {
        return false;
    }
}
void pcmAddSample(const int8_t s)
{
    pcm_buffer_[pcm_buffer_in_] = s;

    pcm_buffer_in_++;
    if (pcm_buffer_in_ >= PCM_BUFFER_SIZE) {
        pcm_buffer_in_ = 0;
    }
}

int8_t pcmReadSample()
{
    int8_t out = pcm_buffer_[pcm_buffer_out_];

    pcm_buffer_out_++;
    if (pcm_buffer_out_ >= PCM_BUFFER_SIZE) {
        pcm_buffer_out_ = 0;
    }

    return out;
}

void pcmBufferInit()
{
    pcm_buffer_in_  = 0;
    pcm_buffer_out_ = 0;
    for (int i = 0; i < PCM_BUFFER_SIZE; ++i) {
        pcm_buffer_[i] = 0;
    }
}

volatile uint16_t pcm_sample;
uint8_t           pcm_last_sample;

void stopPlayback()
{
    // Disable playback per-sample interrupt.
    TIMSK1 &= ~_BV(OCIE1A);

    // Disable the per-sample timer completely.
    TCCR1B &= ~_BV(CS10);

    // Disable the PWM timer.
    TCCR2B &= ~_BV(CS10);

    digitalWrite(SPEAKER_PIN, LOW);
}

// This is called at 8000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {
    uint8_t sample = pcmReadSample() + 127;
    
    OCR2A = sample;
}

void setupTimers()
{
    pinMode(SPEAKER_PIN, OUTPUT);
    digitalWrite(SPEAKER_PIN, 0);

    // Set up Timer 2 to do pulse width modulation on the speaker
    // pin.

    // Use internal clock (datasheet p.160)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));

    // Set fast PWM mode  (p.157)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);

    if(SPEAKER_PIN == 10){
        // Do non-inverting PWM on pin OC2A (p.155)
        // On the Arduino MEGA this is pin 10.
        TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
        TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
        // No prescaler (p.158)
        TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

        // Set initial pulse width to the first sample.
        OCR2A = pcm_buffer_[0];
    } else {
        // Do non-inverting PWM on pin OC2B (p.155)
        // On the Arduino this is pin 3.
        // On the MEGA this is pin 9
        TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);
        TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));
        // No prescaler (p.158)
        TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

        // Set initial pulse width to the first sample.
        OCR2B = pcm_buffer_[0];
    }

    // Set up Timer 1 to send a sample every interrupt.

    cli();

    // Set CTC mode (Clear Timer on Compare Match) (p.133)
    // Have to set OCR1A *after*, otherwise it gets reset to 0!
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
    TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

    // No prescaler (p.134)
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set the compare register (OCR1A).
    // OCR1A is a 16-bit register, so we have to do this with
    // interrupts disabled to be safe.
    OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000

    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK1 |= _BV(OCIE1A);

    pcm_last_sample = 0;
    pcm_sample      = 0;

    sei();
}

void pcmSetup()
{
    pcmBufferInit();
    setupTimers();
}
