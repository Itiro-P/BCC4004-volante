#define ATUA_CW PB4
#define ATUA_CCW PB5 // HIGH aqui gira CCW
#define ATUA_STRENGTH PB3

#define ROTARY_ENC_A 6
#define ROTARY_ENC_B 7
#define ROTARY_ENC_PCINT_A PCINT22
#define ROTARY_ENC_PCINT_B PCINT23
#define ROTARY_ENC_PCINT_AB_IE PCIE2

#define ATUA_CW PB4
#define ATUA_CCW PB5 // HIGH aqui gira CCW
#define ATUA_STRENGTH PB3

#define POS_SENSOR PB2 // switch (absolute position)
#define GP_BUTTON PB0 // general purpose button
#define SERVO PB1
#define DEBOUNCE 125

/*
 * Valores testados com base no volante fornecido
 */

#define VOLTAS_ENCODER_DA_CHAVE_PARA_CENTRO 730
// Margem de erro da centralização
#define OFFSET 5

// Força máxima. Velocidade não chega a importar aqui, logo consideramos um valor mais baixo tempo para tenhamos maior precisão
#define MAX_STRENGTH 160

// Pulsos mínimo e máximo do servo utilizado (testado empiricamente)
#define MIN_PULSE 1050
#define MAX_PULSE 4700
#define PULSE_RANGE MAX_PULSE-MIN_PULSE

// Valores do contador de pulso: [-4500,4500] dá cerca de 2,5 voltas de um lado para o outro
#define EDGE_COUNT 4500
#define ENCODER_RANGE (2 * EDGE_COUNT)


/*
 * Variáveis para usar na EEPROM
 */

// Checagem simples para validar o número na EEPROM: se o nosso número mágico está guardado, então a distância guardada está correta
#define MAGIC_NUMBER 0xA5A5
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_DISTANCIA_ADDR 2

#include "Rotary.h"
#include <avr/eeprom.h>

volatile long distanciaChave = VOLTAS_ENCODER_DA_CHAVE_PARA_CENTRO;
volatile long count = 0; // encoder_rotativo = posicao relativa depois de ligado
volatile bool absolute_sw = false; // chave de posicao do volante ativa?
volatile long centro = -1; // ultima posicao do centro do volante
volatile bool centralizado = false;


bool gpLeitura = (PINB & (1<<GP_BUTTON)) ? 1 : 0, lastGpLeitura = gpLeitura;
unsigned long millisGp = 0;

Rotary r = Rotary(ROTARY_ENC_A, ROTARY_ENC_B);


void salvarDistancia(long valor) {
    // Salva número mágico (2 bytes)
    eeprom_update_word((uint16_t*)EEPROM_MAGIC_ADDR, MAGIC_NUMBER);

    // Salva o valor (4 bytes)
    eeprom_update_dword((uint32_t*)EEPROM_DISTANCIA_ADDR, (uint32_t)valor);

    Serial.println("Distancia ate a chave salvo na EEPROM");
}

bool carregarDistancia() {
    // carrega o número mágico
    // checamos se o número mágico é realmente o que colocamos previamente em algum momento
    if ((uint16_t)eeprom_read_word((uint16_t*)EEPROM_MAGIC_ADDR) != MAGIC_NUMBER) {
        Serial.println("EEPROM vazia/corrompida/estranha, usando distância padrão: 730");
        return false;
    }
    // Chegando aqui, o número mágico está correto. Logo, a distância até a chave guardada também.
    distanciaChave = (long)eeprom_read_dword((uint32_t*)EEPROM_DISTANCIA_ADDR);
    Serial.print("Distancia carregada da EEPROM: ");
    Serial.println(distanciaChave);
    return true;
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
    // se o volante começar em cima da chave, move um pouquinho para encontrar a posição exata na próxima volta
    while(!absolute_sw){
      move(160, 1);
      tempoParaFrenagem = millis();
      while(tempoParaFrenagem + 1000 > millis());
    };
    stop();
    // enquanto não estiver na chave, roda no sentido hoário
    while(absolute_sw) move(160, 1);
    stop();
    tempoParaFrenagem = millis();
    count = 0;
    while(tempoParaFrenagem + 600 > millis()) centro = distanciaChave-count;
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

void centralizarVolante() {
    // Procura centralizar o volante com um erro de OFFSET
    while(count < centro-OFFSET) move(153, 1);
    stop();
    while(count > centro+OFFSET) move(153, 0);
    stop();
    // Espera um pouco para ver se realmente estamos centralizados
    long long time = millis();
    while(time + 1000 > millis());
    if(count < centro + OFFSET || count > centro - OFFSET) centralizado = true;
    idle();
    count = 0;
}

void setup() {
    // Inicia serial para debug
    Serial.begin(115200);
    r.begin(true);
    // Configuração de portas (os mesmos do starter code exceto pelo botão e servo)
    PCICR |= (1 << ROTARY_ENC_PCINT_AB_IE);
    PCMSK2 |= (1 << ROTARY_ENC_PCINT_A) | (1 << ROTARY_ENC_PCINT_B);

    // Timer e pwm do servo
    TCCR1A |= ((1<<WGM11) | (1<<COM1A1)); // Ativa saída PWM no OC1A (PB1/SERVO)
    TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS11); // Prescaler 8
    ICR1 = 39999; // 50 Hz (16MHz / 8 / 40000)
    OCR1A = (MAX_PULSE + MIN_PULSE) / 2; // Inicia no centro (90°)

    DDRB &= ~((1<<GP_BUTTON) | (1<<POS_SENSOR));
    DDRB |= ((1<<ATUA_CW) | (1<<ATUA_CCW) | (1<<ATUA_STRENGTH) | (1<<SERVO));

    PORTB |= ((1<<POS_SENSOR) | (1<<GP_BUTTON));
    PORTB &= ~((1<<ATUA_CW)|(1<<ATUA_CCW));

    // PWM do volante
    OCR2A = 0;
    TCCR2A = ((1<<WGM20) | (1<<COM2A1));
    TCCR2B = (1<<CS21);

    sei();
    idle();
    carregarDistancia();
    encontrarCentro();
}

void loop() {
    gpLeitura = (PINB & (1<<GP_BUTTON)) ? 1 : 0;
    if(!centralizado){
        // Continuamos a centralizar o volante enquanto ele não estiver assim
        centralizarVolante();
    } else {
        // Já está centralizado. Então podemos mexer no servo
        // Aqui usamos a função map de forma inversa: min -> max e max -> min:
        OCR1A = map(count, -EDGE_COUNT, EDGE_COUNT, MAX_PULSE, MIN_PULSE);
        // Botão de calibração
        if(!gpLeitura && lastGpLeitura && millis() - millisGp > DEBOUNCE) {
            // Salvamos a distancia até a chave
            salvarDistancia(count+distanciaChave);
            millisGp = millis();
        }
    }
    lastGpLeitura = gpLeitura;
    // Informações de debug
    if (millis()%300==0) {
        Serial.print("Posicao do count: ");
        Serial.print(count);
        Serial.print("; Na chave? ");
        Serial.print(absolute_sw==true?"sim":"nao");
        Serial.println(centralizado==true?"; Centralizado? sim":"; Centralizado? nao");
    }
}

ISR(PCINT2_vect) {
    unsigned char result = r.process();
    if(result == DIR_CW) count--;
    else if(result == DIR_CCW) count++;

    absolute_sw = 0==(PINB&(1<<POS_SENSOR));
}
