void setup_ds() {

    digitalWrite(ds18b20,LOW);
    pinMode(ds18b20,INPUT);

}



bool init_ds() {

    digitalWrite(ds18b20,LOW);
    pinMode(ds18b20,OUTPUT);
    delayMicroseconds(480);
    pinMode(ds18b20,INPUT);
    delayMicroseconds(60);

    bool present = digitalRead(ds18b20);

    delayMicroseconds(420);

    return present;
}



void write_ds(bool val) {

    pinMode(ds18b20,OUTPUT);

    if (val) {
        delayMicroseconds(1);
        digitalWrite(ds18b20,HIGH);
        delayMicroseconds(59);
    } else {
        delayMicroseconds(60);
        digitalWrite(ds18b20,HIGH);
        delayMicroseconds(1);
    }

    digitalWrite(ds18b20,LOW);
    pinMode(ds18b20,INPUT);
}



uint8_t read_ds() {

    pinMode(ds18b20,OUTPUT);
    delayMicroseconds(1);
    pinMode(ds18b20,INPUT);
    delayMicroseconds(10);

    uint8_t val = digitalRead(ds18b20);

    delayMicroseconds(49);

    return val;
}



void rom_ds(byte command) {

    for (uint8_t i=0; i<8; i++) {
        write_ds(command >> i & 1); // write each bit of the command byte
    }
}



uint16_t getTemp() {

    init_ds();          // initialize
    rom_ds(0xcc);       // broadcast
    rom_ds(0x44);       // temperature convert
    
    init_ds();          // initialize
    rom_ds(0xcc);       // broadcast
    while(!digitalRead(ds18b20));
    rom_ds(0xbe);       // read scratchpad

    uint16_t temp = 0;
    for (int i=0; i<16; i++) {  // only need the first 2 bytes
        temp = temp | (read_ds() << i);
    }

    return temp;
}
