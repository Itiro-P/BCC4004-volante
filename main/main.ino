#define ATUA_CW PB4
#define ATUA_CCW PB5 // HIGH aqui gira CCW
#define ATUA_STRENGTH PB3

#define ROTARY_ENC_A 6
#define ROTARY_ENC_B 7
#define ROTARY_ENC_PCINT_A PCINT22
#define ROTARY_ENC_PCINT_B PCINT23
#define ROTARY_ENC_PCINT_AB_IE PCIE2

#define MAX_STRENGTH 200

#define POS_SENSOR PB2 // switch (absolute position)
#define GP_BUTTON PB0 // general purpose button
#define SERVO PB1

#include "Rotary.h"
volatile long count = 0; // encoder_rotativo = posicao relativa depois de ligado
volatile bool absolute_sw = false; // chave de posicao do volante ativa?
volatile long centro = 0;
bool centralizado = 0;
int x = 0;

Rotary r = Rotary(ROTARY_ENC_A, ROTARY_ENC_B);

void setup() {
  Serial.begin(115200);
  r.begin(true);
  PCICR |= (1 << ROTARY_ENC_PCINT_AB_IE);
  PCMSK2 |= (1 << ROTARY_ENC_PCINT_A) | (1 << ROTARY_ENC_PCINT_B);

  DDRB &= ~((1<<GP_BUTTON)|(1<<POS_SENSOR));
  DDRB |= (1<<ATUA_CW)|(1<<ATUA_CCW)|(1<<ATUA_STRENGTH);

  PORTB |= (1<<POS_SENSOR);
  PORTB |= (1<<GP_BUTTON);
  PORTB &= ~(1<<ATUA_CW);
  PORTB &= ~(1<<ATUA_CCW);

  initPWM();
  sei();

  idle();
  encontrarCentro();
}

void initPWM() {
  OCR2A = 0;
  TCCR2A = (1<<WGM20);
  TCCR2B = (1<<CS21);
  TCCR2A |= (1<<COM2A1);
}

void setPWM(unsigned char val) {
  OCR2A = val < MAX_STRENGTH ? val: MAX_STRENGTH;
}

void stop() {
  PORTB |= (1<<ATUA_CW) | (1<<ATUA_CCW);
}

void idle() {
  setPWM(0);
  PORTB &= ~((1<<ATUA_CW) | (1<<ATUA_CCW));
}

void move(unsigned char power, bool cw = true) {
  if (power == 0)
    idle();
  else {
    if (cw) {
      PORTB |= (1<<ATUA_CW);
      PORTB &= ~(1<<ATUA_CCW);
    } else {
      PORTB &= ~(1<<ATUA_CW);
      PORTB |= (1<<ATUA_CCW);
    }
    setPWM(power);
  }
}

void encontrarCentro(){
  unsigned long tempoParaFrenagem = 0;
  absolute_sw = 0==(PINB&(1<<POS_SENSOR));
  // se o volante começar em cima da chave
  while(absolute_sw == 0) move(170, 0);

  stop();
  // enquanto nao estiver na chave, roda no sentido horario
  while(absolute_sw != 0) move(160, 1);
  stop();
  tempoParaFrenagem = millis();
  count = 0;
  while(millis()-tempoParaFrenagem < 1000) centro = 730-count;
  count = 0;
}

void centralizarVolante(){

  while(count < centro-5){ 
    move(155, 1);
  }
  stop();
  while(count > centro+5) {
    move(155, 0);
  }
  stop();
}

void loop() {
  centralizarVolante();

  // move(100, false); // move ccw
  // move(100); // move cw
  // debug only info
  if (millis()%300==0) {
    Serial.print(count);
    Serial.print(", ");
    Serial.println(absolute_sw==true?'1':'0');
    // Serial.print(", ");
    // Serial.println(distanciaRestante);
    Serial.println(centralizado);
  }
}

ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result == DIR_NONE) {
    // do nothing
  }
  else if (result == DIR_CW) count--;
  else if (result == DIR_CCW) count++;

  absolute_sw = 0==(PINB&(1<<POS_SENSOR));
}