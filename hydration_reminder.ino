#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

// D13 -> PB5

// States
const unsigned int HYDRATED = 0;
const unsigned int DEHYDRATED = 1;
const unsigned int STANDBY = 2;

const unsigned int BLINK_FREQ = 1; // x 8 seconds
// const unsigned int STANDBY_THRESH = 10800; // 1 day
const unsigned int STANDBY_THRESH = 3;
// const unsigned int DEHYDRATION_THRESH = 450; // 1h
const unsigned int DEHYDRATION_THRESH = 3;
const unsigned int EXT_INT_PAUSE_THRESH = 1;  // x8s How many cycles of 8s to pause external ints

volatile unsigned int wdt_counter = 0;
volatile int state = HYDRATED;
volatile bool tilted = false;
int i = 0;

void ultra_low_power_mode() {
  // Disable unused peripherals to save power
  PRR = (1 << PRADC)  // ADC off
        // | (1<<PRUSART0) // Serial off if not used
        | (1 << PRSPI)   // SPI off
        | (1 << PRTIM0)  // Timer0 off (affects millis() and delay())
        // | (1 << PRTIM1)  // Timer1 off
        | (1 << PRTIM2)  // Timer2 off
        | (1 << PRTWI);  // I2C off

  // Set unused pins as inputs or outputs to reduce leakage
  for (int pin = 0; pin <= 13; pin++) {
    pinMode(pin, OUTPUT);    // or INPUT_PULLUP if needed
    digitalWrite(pin, LOW);  // ensure LOW if output
  }
}

void printRegister8(const char* name, uint8_t reg) {
  Serial.print(name);
  Serial.print(" = 0b");
  for (int i = 7; i >= 0; i--) {
    Serial.print((reg >> i) & 1);
  }
  Serial.println();
}

void start_wdt(int wdp3, int wdp2, int wdp1, int wdp0) {
  wdt_reset();
  WDTCSR |= ((1 << WDCE) | (1 << WDE));
  WDTCSR = ((1 << WDIE) | (wdp0 << WDP0) | (wdp1 << WDP1) | (wdp2 << WDP2) | (wdp3 << WDP3));
  wdt_counter = 0;
}

void start_wdt_longest() {  //8s timeout
  start_wdt(1, 0, 0, 1);
}

void stop_wdt() {
  wdt_reset();
  WDTCSR |= ((1 << WDCE) | (1 << WDE));
  WDTCSR = 0;
  wdt_counter = 0;
}

void sleep() {
  SMCR |= (1 << SE);  //enable sleep
  asm("sleep");
  SMCR &= ~(1 << SE);  //disable sleep
}

void toggle_led() {
  PORTB ^= (1 << PB5);  // Pin 13 on Pro Mini is PB5
}

void turn_off_led() {
  PORTB &= ~(1 << PB5);  // Pin 13 on Pro Mini is PB5
}

void turn_on_led() {
  PORTB |= (1 << PB5);  // Pin 13 on Pro Mini is PB5
}

void start_dehydrated_state() {
  cli();
  stop_wdt();
  state = DEHYDRATED;
  sei();
}

void pause_external_ints() {
  // goes to sleep for a certain time before re-enabling external interrupts
  // avoid trigerring many sucessive external interrupts as the bottle stays
  // in tilted position
  start_wdt_longest();
  while (wdt_counter < EXT_INT_PAUSE_THRESH) {
    sleep();
  }
  stop_wdt();
  EIMSK = (1 << INT0);  //enable external interrupt for int0
}

void start_hydrated_state() {
  pause_external_ints();
  cli();
  start_wdt_longest();
  state = HYDRATED;
  sei();
}

void blink_nx(unsigned int n) {
  for (int j = 0; j < n; j++) {
    turn_on_led();
    start_wdt(0, 0, 1, 0);  //64ms
    while (wdt_counter == 0);
    turn_off_led();
    start_wdt(0, 1, 1, 0); //1s
    sleep();
  }
  stop_wdt();
}

void notify(){
    // by choice start_wdt_longest will zero wdt_counter, we use this variable to get around that
    unsigned int wdt_counter_cache;

    wdt_counter_cache = wdt_counter;
    blink_nx(3);
    start_wdt_longest();
    wdt_counter = wdt_counter_cache;
}

void hydrated_state() {
  Serial.println("hydrated");
  sleep();
  if (tilted) {
      tilted = false;
      start_hydrated_state();
    } else if (wdt_counter >= DEHYDRATION_THRESH) {
    start_dehydrated_state();
  }
}

void dehydrated_state() {

  Serial.println("Dehydrated");

  // drink event
  if (tilted) {
    turn_off_led();
    tilted = false;
    start_hydrated_state();
  // in dehydrated mode for too long event
  } else if (wdt_counter == STANDBY_THRESH) {
    start_standby_state();
  // notify event
  } else if ((wdt_counter % BLINK_FREQ) == 0) {
    Serial.println(wdt_counter);
    notify();
    Serial.println(wdt_counter);
  }
  sleep();

}

void start_standby_state() {
  state = STANDBY;
}

void standby_state() {
  // the standby state is triggered if in dehydrated state for too long, with the goal of conserving battery
  // only tilt will get out of standby state and into hydrated state
  stop_wdt();
  sleep();

  blink_nx(2);

  start_hydrated_state();
}


void setup() {
  cli();

  ultra_low_power_mode();

  // configuring led gpio
  DDRB |= (1 << PB5);  // set as output

  // watchdog timer config
  wdt_reset();
  WDTCSR |= ((1 << WDCE) | (1 << WDE));
  WDTCSR = ((1 << WDIE) | (1 << WDP3) | (1 << WDP0));  // enable wdt interrupt and choose largest prescaler

  //sleep mode config
  SMCR = (1 << SM1);  // power down mode

  // tilt sensor pin config
  DDRD &= ~(1 << PD2);  // set as input
  PORTD |= (1 << PD2);  //enable pullup resistor

  // external interrupt config (tilt sensor)
  EICRA = 0;            // interrupt at low level (0)
  EIMSK = (1 << INT0);  //enable external interrupt for int0

  Serial.begin(9600);

  sei();
}

void loop() {

  if (state == HYDRATED) {
    hydrated_state();
  } else if (state == DEHYDRATED) {
    dehydrated_state();
  } else if (state == STANDBY) {
    standby_state();
  }
}

ISR(WDT_vect) {
  wdt_counter++;
}

ISR(INT0_vect) {
  EIMSK = 0;  //disable external interrupt for int0 to avoid being stuck in this isr
  tilted = true;
}