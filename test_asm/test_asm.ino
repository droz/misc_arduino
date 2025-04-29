void setup() {
  // put your setup code here, to run once:
  DDRB = 0x3F;
  while(true) {
    PORTB = 0x3F;
    for (uint16_t val = 365; val != 0; val--) {
      asm("nop");
    }
    PORTB = 0x00;
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
