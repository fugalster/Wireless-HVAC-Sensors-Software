#include "rf_def.h"
#include <avr/sleep.h>
#include <avr/power.h>

volatile int f_wdt = 1;



uint8_t CE      = 8;
uint8_t CSN     = 9;
uint8_t clock   = 4;
uint8_t dataOut = 6;
uint8_t dataIn  = 5;
//uint8_t IRQ     = 8; // repurposed to CE

uint8_t boost   = 3;
uint8_t ds18b20 = 10;
uint8_t batt    = 7;
uint8_t sen_pow = 2;
uint8_t tmp36   = 1;
uint8_t hih5030 = 0;



void setup() {

    // watchdog timer stuff for sleep
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = 1<<WDP0 | 1<<WDP3;
    WDTCSR |= _BV(WDIE);

    analogReference(DEFAULT);

    pinMode(batt,    INPUT);
    pinMode(hih5030, INPUT);
    pinMode(tmp36,   INPUT);
    pinMode(sen_pow, OUTPUT);
    pinMode(boost,   OUTPUT);

    digitalWrite(boost,   HIGH);
    digitalWrite(sen_pow, HIGH);

    delay(70);              // settling time for hih5030
    delayMicroseconds(300); // settling time for tmp36
    
    //Serial.begin(9600);
    //Serial.println("Ready...");

    setup_rf24();
    setup_ds();

    delayMicroseconds(130);

}



void loop() {

    static long reading, reading_1, reading_2, reading_3;
    static uint8_t v_batt, humidity, temperature;

    // turn on voltage regulator
    digitalWrite(boost, HIGH);

    // turn on sensors
    digitalWrite(sen_pow, HIGH);
    delay(70); // settling time for hih5030
    //delayMicroseconds(300); // settling time for tmp36

    // read three sensors, 10 bits each, union into 30 bits + 2 padded bits = 4 bytes
    reading_1 = avgAnalogRead(batt,8);
    reading_2 = avgAnalogRead(hih5030,8);
    reading_3 = avgAnalogRead(tmp36,8);

    uint8_t byte_1, byte_2, byte_3, byte_4;
    byte_1 = reading_1 >> 4;
    byte_2 = (reading_1 << 4) | (reading_2 >> 6);
    byte_3 = (reading_2 << 6) | (reading_3 >> 8);
    byte_4 = reading_3;

    // turn off sensors
    // digitalWrite(sen_pow, LOW);

    // power up radio
    byte current_config = rf24_read(R_REGISTER | CONFIG);
    rf24_write((W_REGISTER | CONFIG), (current_config | 0x02));

    // max 4.5ms wake up time
    delay(5);

    // clear MAX_RT just in case last transmission failed
    byte current_status = rf24_read(R_REGISTER | STATUS);
    rf24_write((W_REGISTER | STATUS), (current_status | 0x10));

    // write payload
    delayMicroseconds(1);
    digitalWrite(CSN, LOW);
    shiftOut(dataOut, clock, MSBFIRST, W_TX_PAYLOAD);
    shiftOut(dataOut, clock, MSBFIRST, byte_1);
    shiftOut(dataOut, clock, MSBFIRST, byte_2);
    shiftOut(dataOut, clock, MSBFIRST, byte_3);
    shiftOut(dataOut, clock, MSBFIRST, byte_4);
    digitalWrite(CSN, HIGH);

    // start transmission
    digitalWrite(CE, HIGH);
    delayMicroseconds(10);
    digitalWrite(CE, LOW);

    // 500us delay between retries, 3 retires max = 2ms sending time max
    delay(2);

    // read status register to verify all went ok
    int foo = rf24_read(R_REGISTER | STATUS);
    //Serial.println(foo, BIN);
    //Serial.println(current_status, BIN);

    // power down
    current_config = rf24_read(R_REGISTER | CONFIG);
    rf24_write((W_REGISTER | CONFIG), (current_config & 0xFD));

    delay(1000);

    // if v_batt not too low, turn off boost
    // 589 ~= 1.9v
    ///if (reading_1 >= 589) {
    ///    digitalWrite(boost, LOW);
    ///}


    // enter sleep mode with watch dog timer wake
    // approx 8 seconds per wdt max out
    ///for (int i=0; i<7; i++) {
    ///    f_wdt = 0;
    ///    enter_sleep();
    ///}
}



ISR(WDT_vect) {
    if (f_wdt == 0) {
        f_wdt = 1;
    }
}



void enter_sleep() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    sleep_disable();
    power_all_enable();
}



int avgAnalogRead(int pin, int times) {
    unsigned long reads = 0;
    for (int i=0; i<times; i++) {
        reads += analogRead(pin);
    }
    return reads/times;
}
