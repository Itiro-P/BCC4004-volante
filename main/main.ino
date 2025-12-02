#define ATUA_CW PB4
#define ATUA_CCW PB5 // HIGH aqui gira CCW
#define ATUA_STRENGTH PB3

#define ROTARY_ENC_A 6
#define ROTARY_ENC_B 7
#define ROTARY_ENC_PCINT_A PCINT22
#define ROTARY_ENC_PCINT_B PCINT23
#define ROTARY_ENC_PCINT_AB_IE PCIE2

#define VOLTAS_ENCODER_DA_CHAVE_PARA_CENTRO 730
#define ATUA_CW PB4
#define ATUA_CCW PB5 // HIGH aqui gira CCW
#define ATUA_STRENGTH PB3

#define MAX_STRENGTH 160
#define OFFSET 5
#define POS_SENSOR PB2 // switch (absolute position)
#define GP_BUTTON PB0 // general purpose button
#define SERVO PB1
#define DEBOUNCE 125

#define MIN_PULSE 1050
#define MAX_PULSE 4700
#define PULSE_RANGE MAX_PULSE-MIN_PULSE
#define EDGE_COUNT 4500
#define ENCODER_RANGE (2 * EDGE_COUNT)
#define MAGIC_NUMBER 0xA5A5
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_CENTRO_ADDR 2

#include "Rotary.h"
#include <avr/eeprom.h>

volatile long count = 0; // encoder_rotativo = posicao relativa depois de ligado
volatile bool absolute_sw = false; // chave de posicao do volante ativa?
volatile long centro = -1; // ultima posicao do centro do volante
volatile bool centralizado = false;


bool gpLeitura = (PINB & (1<<GP_BUTTON)) ? 1 : 0, lastGpLeitura = gpLeitura;
unsigned long millisGp = 0;

Rotary r = Rotary(ROTARY_ENC_A, ROTARY_ENC_B);


void salvarCentro(long valor) {
    // Salva magic number (2 bytes)
    eeprom_update_word((uint16_t*)EEPROM_MAGIC_ADDR, MAGIC_NUMBER);

    // Salva o valor (4 bytes)
    eeprom_update_dword((uint32_t*)EEPROM_CENTRO_ADDR, (uint32_t)valor);

    Serial.println("Centro salvo na EEPROM");
}

long carregarCentro() {
    // Verifica número mágico
    uint16_t magic = eeprom_read_word((uint16_t*)EEPROM_MAGIC_ADDR);
    if (magic != MAGIC_NUMBER) {
        Serial.println("EEPROM corrompida (número mágico inválido), calibrando...");
        encontrarCentro();
        return 0;
    }

    // Lê o valor
    novoCentro = (long)eeprom_read_dword((uint32_t*)EEPROM_CENTRO_ADDR);

    Serial.print("Centro carregado: ");
    Serial.println(novoCentro);
    return novoCentro;
}

void move(unsigned char power, bool cw = true) {
    if (power == 0) idle();
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
    absolute_sw = (0==(PINB&(1<<POS_SENSOR)));
    // se o volante começar em cima da chave
    while(!absolute_sw){
      move(160, 1);
      tempoParaFrenagem = millis();
      while(tempoParaFrenagem + 1000 > millis());
    };
    stop();
    // enquanto nao estiver na chave, roda no sentido horario
    while(absolute_sw) move(160, 1);
    stop();
    tempoParaFrenagem = millis();
    count = 0;
    while(tempoParaFrenagem + 600 > millis()) centro = VOLTAS_ENCODER_DA_CHAVE_PARA_CENTRO-count;
    count = 0;
}

void setPWM(unsigned char val) {
    OCR2A = val < MAX_STRENGTH ? val: MAX_STRENGTH;
}

void stop(){
    PORTB |= (1<<ATUA_CW) | (1<<ATUA_CCW);
}

void idle() {
    setPWM(0);
    PORTB &= ~((1<<ATUA_CW) | (1<<ATUA_CCW));
}

void centralizarVolante(long alvo) {
    while(count < alvo-OFFSET) move(153, 1);
    stop();
    while(count > alvo+OFFSET) move(153, 0);
    stop();
    long long time = millis();
    while(time + 1000 > millis());

    if(count < alvo + OFFSET || count > alvo - OFFSET) centralizado = true;
    centro = alvo;
    idle();
    count = 0;
}

void servo() {
    OCR1A = map(pulse, -EDGE_COUNT, EDGE_COUNT, MAX_PULSE, MIN_PULSE);
}

void setup() {
    Serial.begin(115200);
    r.begin(true);
    PCICR |= (1 << ROTARY_ENC_PCINT_AB_IE);
    PCMSK2 |= (1 << ROTARY_ENC_PCINT_A) | (1 << ROTARY_ENC_PCINT_B);

    TCCR1A = (1<<WGM11);
    TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS11); // Prescaler 8
    TCCR1A |= (1<<COM1A1); // Ativa saída PWM no OC1A (PB1/SERVO)
    ICR1 = 39999; // 50 Hz (16MHz / 8 / 40000)
    OCR1A = (MAX_PULSE + MIN_PULSE) / 2; // Inicia no centro (90°)

    DDRB &= ~((1<<GP_BUTTON)|(1<<POS_SENSOR));
    DDRB |= (1<<ATUA_CW)|(1<<ATUA_CCW)|(1<<ATUA_STRENGTH)|(1<<SERVO);

    PORTB |= (1<<POS_SENSOR);
    PORTB |= (1<<GP_BUTTON);
    PORTB &= ~(1<<ATUA_CW);
    PORTB &= ~(1<<ATUA_CCW);

    // PWM do volante
    OCR2A = 0;
    TCCR2A = (1<<WGM20);
    TCCR2B = (1<<CS21);
    TCCR2A |= (1<<COM2A1);

    sei();
    idle();
    encontrarCentro();
}

void loop() {
    gpLeitura = (PINB & (1<<GP_BUTTON)) ? 1 : 0;
    if(!centralizado){
        centralizarVolante(centro);
        // Achamos o centro original. Então carregaremos a EEPROM e iremos ao centro definido nela.
        long novoCentro = carregarCentro();
        if(novoCentro != 0) centralizarVolante(novoCentro);
    } else {
        // Já está centralizado. Então podemos mexer no servo
        servo();
        // Botão de calibração
        if(!gpLeitura && lastGpLeitura && millis() - millisGp > DEBOUNCE) {
            salvarCentro(count - centro);
            millisGp = millis();
        }
    }
    if (millis()%300==0) {
        Serial.print(count);
        Serial.print(", ");
        Serial.println(absolute_sw==true?'1':'0');
        Serial.println(centralizado);
    }
    lastGpLeitura = gpLeitura;
}

ISR(PCINT2_vect) {
    unsigned char result = r.process();
    if(result == DIR_CW) count--;
    else if(result == DIR_CCW) count++;

    absolute_sw = 0==(PINB&(1<<POS_SENSOR));
}
