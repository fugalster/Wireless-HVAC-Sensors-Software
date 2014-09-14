#define R_REGISTER           0b00000000
#define W_REGISTER           0b00100000
#define R_RX_PAYLOAD         0b01100001
#define W_TX_PAYLOAD         0b10100000
#define FLUSH_TX             0b11100001
#define FLUSH_RX             0b11100010
#define REUSE_TX_PL          0b11100011
#define R_RX_PL_WID          0b01100000
#define W_ACK_PAYLOAD        0b10101PPP
#define W_TX_PAYLOAD_NO_ACK  0b10110000
#define NOP                  0b11111111

#define CONFIG               0x00
#define EN_AA                0x01
#define EN_RXADDR            0x02
#define SETUP_AW             0x03
#define SETUP_RETR           0x04
#define RF_CH                0x05
#define RF_SETUP             0x06
#define STATUS               0x07
#define OBSERVE_TX           0x08
#define RPD                  0x09
#define RX_ADDR_P0           0x0A
#define RX_ADDR_P1           0x0B
#define RX_ADDR_P2           0x0C
#define RX_ADDR_P3           0x0D
#define RX_ADDR_P4           0x0E
#define RX_ADDR_P5           0x0F
#define TX_ADDR              0x10
#define RX_PW_P0             0x11
#define RX_PW_P1             0x12
#define RX_PW_P2             0x13
#define RX_PW_P3             0x14
#define RX_PW_P4             0x15
#define RX_PW_P5             0x16
#define FIFO_STATUS          0x17
#define DYNPD                0x1C
#define FEATURE              0x1D

uint8_t CE      = 12;
uint8_t CSN     = 9;
uint8_t clock   = 11;
uint8_t dataOut = 8;
uint8_t dataIn  = 10;
uint8_t IRQ     = 2;

volatile bool recieved = false;

void setup() {
    
    pinMode(CE, OUTPUT);
    pinMode(CSN, OUTPUT);
    pinMode(clock, OUTPUT);
    pinMode(dataOut, OUTPUT);
    pinMode(dataIn, INPUT);
    pinMode(IRQ, INPUT);

    pinMode(13, OUTPUT);

    attachInterrupt(0, IRQ_ISR, FALLING);

    Serial.begin(9600);
    Serial.print("PRX Status: ");
    Serial.println(rf24_read(STATUS), BIN);
    Serial.println();

    setup_rf24();

    // start active RX on PRX
    digitalWrite(CE, HIGH);
    delayMicroseconds(130);
}



void loop() {

    if (recieved) {

        unsigned long all_byte = 0;

        // read payload width recieved on PRX
        byte rx_pl_wid = rf24_read(R_RX_PL_WID);
        
        // read payload from PRX
        delayMicroseconds(1);
        digitalWrite(CSN, LOW);
        shiftOut(dataOut, clock, MSBFIRST, R_RX_PAYLOAD);
        for (int i=(rx_pl_wid-1); i>=0; i--) {
            unsigned long in_byte = shiftIn(dataIn, clock, MSBFIRST);
            all_byte |= (in_byte << (i*8));
        }

        unsigned int v_batt, humidity, temperature;
        v_batt =      (all_byte & 0b00111111111100000000000000000000) >> 20;
        humidity =    (all_byte & 0b00000000000011111111110000000000) >> 10;
        temperature = (all_byte & 0b00000000000000000000001111111111);

        Serial.print(float(v_batt)*3.3/1023.0);
        Serial.print(',');
        Serial.print(((float(humidity)/1023.0)-0.1515)/0.00636);
        Serial.print(',');
        Serial.print(float(temperature)*330/1023.0 - 50);

        Serial.println();
        digitalWrite(CSN, HIGH);

        delayMicroseconds(1);
        digitalWrite(CSN, LOW);
        shiftOut(dataOut, clock, MSBFIRST, FLUSH_RX);
        digitalWrite(CSN, HIGH);
    
        recieved = false;
    }

}



void IRQ_ISR() {
    byte current_status = rf24_read(STATUS);
    if ((current_status & 0x40) >> 6) {
        recieved = true;
    }
    rf24_write((W_REGISTER | STATUS), (current_status | 0x70));
}



byte rf24_write(byte address, byte content) {
    delayMicroseconds(1);
    digitalWrite(CSN, LOW);
    shiftOut(dataOut, clock, MSBFIRST, address);
    shiftOut(dataOut, clock, MSBFIRST, content);
    digitalWrite(CSN, HIGH);
}



byte rf24_read(byte address) {
    delayMicroseconds(1);
    digitalWrite(CSN, LOW);
    shiftOut(dataOut, clock, MSBFIRST, address);
    byte reg = shiftIn(dataIn, clock, MSBFIRST);
    digitalWrite(CSN, HIGH);
    return reg;
}



void setup_rf24() {

    // power on delay
    delay(100);

    // enter standby I mode
    digitalWrite(CE, LOW);

    // enagle RX_DR interrupt on IRQ
    // enable CRC
    // 2 byte CRC
    // enter powerup mode
    // PRX mode
    rf24_write((W_REGISTER | CONFIG), 0x3F);

    // enable auto ack on data pipe 0
    rf24_write((W_REGISTER | EN_AA), 0x01);
    
    // enable data pipe 0
    rf24_write((W_REGISTER | EN_RXADDR), 0x01);
    
    // setup address width to 5 bytes
    rf24_write((W_REGISTER | SETUP_AW), 0x03);
    
    // setup auto retransmit: 500us delay, 3 retries
    rf24_write((W_REGISTER | SETUP_RETR), 0x13);
    
    // setup rf channel: 2.402GHz
    rf24_write((W_REGISTER | RF_CH), 0x02);
    
    // configure rf:
    // no cont wave
    // no PLL lock
    // 1Mbps
    // 0dBm
    rf24_write((W_REGISTER | RF_SETUP), 0x06);

    // clear statuses
    byte current_status = rf24_read(R_REGISTER | STATUS);
    rf24_write((W_REGISTER | STATUS), (current_status | 0b01110000));

    // define pipe 0 address, LSByte first
    delayMicroseconds(1);
    digitalWrite(CSN, LOW);
    shiftOut(dataOut, clock, MSBFIRST, (W_REGISTER | RX_ADDR_P0));
    shiftOut(dataOut, clock, MSBFIRST, 0xE7);
    shiftOut(dataOut, clock, MSBFIRST, 0xE7);
    shiftOut(dataOut, clock, MSBFIRST, 0xE7);
    shiftOut(dataOut, clock, MSBFIRST, 0xE7);
    shiftOut(dataOut, clock, MSBFIRST, 0xE7);
    digitalWrite(CSN, HIGH);

    // enable dynamic payload length on pipe 0
    rf24_write((W_REGISTER | DYNPD), 0x01);

    // enable dynamic payload and payload ack
    rf24_write((W_REGISTER | FEATURE), 0x06);

    delayMicroseconds(1);
    digitalWrite(CSN, LOW);
    shiftOut(dataOut, clock, MSBFIRST, FLUSH_TX);
    digitalWrite(CSN, HIGH);

    delayMicroseconds(1);
    digitalWrite(CSN, LOW);
    shiftOut(dataOut, clock, MSBFIRST, FLUSH_RX);
    digitalWrite(CSN, HIGH);
}
