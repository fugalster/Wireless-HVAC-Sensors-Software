byte rf24_write(byte address, byte content) {
    delayMicroseconds(1);
    digitalWrite(CSN, LOW);
    shiftOut(dataOut, clock, MSBFIRST, address);
    shiftOut(dataOut, clock, MSBFIRST, content);
    digitalWrite(CSN, HIGH);
    digitalWrite(dataOut, LOW);
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

    pinMode(CE, OUTPUT);
    pinMode(CSN, OUTPUT);
    pinMode(clock, OUTPUT);
    pinMode(dataOut, OUTPUT);
    pinMode(dataIn, INPUT);
//  pinMode(IRQ, INPUT); // No big need for IRQ. Repurposed to CE pin

    // power on delay
    delay(100);

    // enter standby I mode
    digitalWrite(CE, LOW);

    // disable all interrupts since no access to IRQ
    // enable CRC
    // 2 byte CRC
    // enter powerdown mode
    // PTX mode
    rf24_write((W_REGISTER | CONFIG), 0x7C);

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
    // no cont carrier
    // no PLL lock
    // 1Mbps
    // 0dBm
    rf24_write((W_REGISTER | RF_SETUP), 0x06);

    // clear statuses
    byte current_status = rf24_read(R_REGISTER | STATUS);
    rf24_write((W_REGISTER | STATUS), (current_status | 0b01110000));
    //Serial.println(current_status, BIN);

    // define TX address, LSByte first
    delayMicroseconds(1);
    digitalWrite(CSN, LOW);
    shiftOut(dataOut, clock, MSBFIRST, (W_REGISTER | TX_ADDR));
    shiftOut(dataOut, clock, MSBFIRST, 0xE7);
    shiftOut(dataOut, clock, MSBFIRST, 0xE7);
    shiftOut(dataOut, clock, MSBFIRST, 0xE7);
    shiftOut(dataOut, clock, MSBFIRST, 0xE7);
    shiftOut(dataOut, clock, MSBFIRST, 0xE7);
    digitalWrite(CSN, HIGH);

    // define target RX pipe 0 address, LSByte first
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

    // flush TX and RX
    delayMicroseconds(1);
    digitalWrite(CSN, LOW);
    shiftOut(dataOut, clock, MSBFIRST, (W_REGISTER | FLUSH_TX));
    digitalWrite(CSN, HIGH);

    // power down
    byte current_config = rf24_read(R_REGISTER | CONFIG);
    rf24_write((W_REGISTER | CONFIG), (current_config & 0xFD));
    delay(5);
}
