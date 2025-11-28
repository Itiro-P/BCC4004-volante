#define ATUA_CW PB4
#define ATUA_CCW PB5 // HIGH aqui gira CCW
#define ATUA_STRENGTH PB3

#define ROTARY_ENC_A 6
#define ROTARY_ENC_B 7
#define ROTARY_ENC_PCINT_A PCINT22
#define ROTARY_ENC_PCINT_B PCINT23
#define ROTARY_ENC_PCINT_AB_IE PCIE2

#define MAX_STRENGTH 160
#define OFFSET 5
#define POS_SENSOR PB2 // switch (absolute position)
#define GP_BUTTON PB0 // general purpose button
#define SERVO PB1
#define DEBOUNCE 200

#define VOLTAS_ENCODER_DA_CHAVE_PARA_CENTRO 730

#define MIN_PULSE 1050
#define MAX_PULSE 4950
#define PULSE_RANGE MAX_PULSE-MIN_PULSE
#define EDGE_COUNT 4500
#define ENCODER_RANGE (2 * EDGE_COUNT)
#define EEPROM_CENTRO_ADDR 0

#include "Rotary.h"
#include <avr/eeprom.h>

volatile long count = 0; // encoder_rotativo = posicao relativa depois de ligado
volatile bool absolute_sw = false; // chave de posicao do volante ativa?
volatile long centro = -1; // ultima posicao do centro do volante
volatile bool centralizado = false;

bool gpLeitura = (PINB & (1<<GP_BUTTON)) ? 1 : 0, lastGpLeitura = gpLeitura;
unsigned long millisGp = 0;

Rotary r = Rotary(ROTARY_ENC_A, ROTARY_ENC_B);

void carregarCentro() {
    // Lê 4 bytes (dword) do endereço de memória e armazena em 'centro'.
    centro = (long)eeprom_read_dword((uint32_t*)EEPROM_CENTRO_ADDR);

    // Verificação de segurança: Se o valor lido for um valor 'default'
    // ou absurdo (e.g., 0xFFFFFFFF, que pode indicar um valor nunca escrito),
    // inicie a rotina de encontrar o centro.
    if (centro == 0 || centro == -1) {
        // Se a EEPROM estiver "limpa" ou corrompida, execute a rotina de busca.
        encontrarCentro();
        salvarCentro(centro);
    }
}

void salvarCentro(long valor) {
    eeprom_update_dword((uint32_t*)EEPROM_CENTRO_ADDR, (uint32_t)valor);
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
    while(!absolute_sw) move(170, 0);
    stop();
    // enquanto nao estiver na chave, roda no sentido horario
    while(absolute_sw) move(170, 1);
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

void centralizarVolante(){
    while(count < centro-OFFSET) move(153, 1);
    stop();
    while(count > centro+OFFSET) move(153, 0);
    stop();
    long long time = millis();
    while(time + 500 > millis());
    if(count < centro + OFFSET || count > centro - OFFSET) centralizado = true;
}

void servo() {
    if(count > EDGE_COUNT) {
        OCR1A = MAX_PULSE;
        return;
    }
    if(count < -EDGE_COUNT) {
        OCR1A = MIN_PULSE;
        return;
    }

    // Pulso = (Normalizado / Encoder_Range) * Pulse_Range + MIN_PULSE
    unsigned short pulse = (unsigned short)((((count + EDGE_COUNT) * PULSE_RANGE) / ENCODER_RANGE) + MIN_PULSE);

    OCR1A = pulse;
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
    carregarCentro();
}

void loop() {
    gpLeitura = (PINB & (1<<GP_BUTTON)) ? 1 : 0;
    if(!centralizado){
        centralizarVolante();
        idle();
        count = 0;
    } else {
        // Já está centralizado. Então podemos mexer no servo
        servo();
        // Botão de calibração
        if(!gpLeitura && lastGpLeitura && millis() - millisGp > DEBOUNCE) {
            salvarCentro(count);
            Serial.println("Centro salvo na EEPROM");
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
