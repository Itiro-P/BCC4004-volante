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
#define OFFSET 3

// Força máxima. Velocidade não chega a importar aqui, logo consideramos um valor mais baixo tempo para tenhamos maior precisão
#define MAX_STRENGTH 170

// Força a ser usada
#define NORMAL_STRENGTH 160
#define MENOR_STRENGTH 153

// Pulsos mínimo e máximo do servo utilizado (testado empiricamente)
#define MIN_PULSE 1050
#define MAX_PULSE 4700
#define PULSE_RANGE MAX_PULSE-MIN_PULSE

// Valores do contador de pulso: [-4500,4500] dá cerca de 2,5 voltas de um lado para o outro
#define EDGE_COUNT 4500
#define ENCODER_RANGE (2 * EDGE_COUNT)
#define PULSOS_POR_VOLTA (int)(ENCODER_RANGE / 2.5)

// define usado para forçar a centralização manual
#define MANUAL true

/*
 * Variáveis para usar na EEPROM
 */

// Checagem simples para validar o número na EEPROM: se o nosso número mágico está guardado, então a distância guardada está correta
#define MAGIC_NUMBER 0xA5A5
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_DISTANCIA_ADDR 2

#include "Rotary.h"
#include <avr/eeprom.h>

volatile long distanciaChave = VOLTAS_ENCODER_DA_CHAVE_PARA_CENTRO; // distância daté a chave
volatile long count = 0; // posicao relativa até a chave depois de ligado
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

/*
void encontrarCentro(){
    unsigned long tempoParaFrenagem = 0;
    unsigned long temp;
    absolute_sw = (0==(PINB&(1<<POS_SENSOR)));
    // se o volante começar em cima da chave, move um pouquinho para encontrar a posição exata na próxima volta
    do{
        while(!absolute_sw){
            move(NORMAL_STRENGTH, 1);
            tempoParaFrenagem = millis();
            while(tempoParaFrenagem + 200 > millis());
        };
        stop();
        // enquanto não estiver na chave, roda no sentido hoário
        unsigned long temp = millis();
        while(absolute_sw) move(NORMAL_STRENGTH, 1);
        if (temp+300 > millis()) move(MAX_STRENGTH, 1);
    } while(temp+300 > millis());
    stop();
    tempoParaFrenagem = millis();
    count = 0;
    while(tempoParaFrenagem + 600 > millis()) centro = distanciaChave-count;
    count = 0;
}
 */

 void encontrarCentro(){
     absolute_sw = (0 == (PINB & (1 << POS_SENSOR)));

     // Se ligou em cima da chave, sai dela
     while(!absolute_sw) move(NORMAL_STRENGTH, 1);
     stop();
     // Delay de 1 segundo para estabilizar
     unsigned long time = millis();
     while(time + 1000 > millis());

     // Procura a chave
     while(absolute_sw) move(NORMAL_STRENGTH, 1);

     // Achou! Zera a referência absoluta
     stop();
     count = 0;

     // Define o alvo baseado na memória (ou fallback de 730)
     centro = distanciaChave;
 }

void setPWM(unsigned char val) {
    OCR2A = val < MAX_STRENGTH ? val: MAX_STRENGTH;
}

void stop(){
    PORTB |= ((1<<ATUA_CW) | (1<<ATUA_CCW));
}

void idle() {
    setPWM(0);
    PORTB &= ~((1<<ATUA_CW) | (1<<ATUA_CCW));
}

/*
void centralizarVolante() {
    // Procura centralizar o volante com um erro de OFFSET
    while(count < centro-OFFSET) move(MENOR_STRENGTH, 1);
    stop();
    while(count > centro+OFFSET) move(MENOR_STRENGTH, 0);
    stop();
    // Espera um pouco para estabilizar e ver se realmente estamos centralizados
    long long time = millis();
    while(time + 1000 > millis());
    if(count >= centro - OFFSET && count <= centro + OFFSET) centralizado = true;
    idle();
    count = 0;
}
 */

 void centralizarVolante() {
     // Procura centralizar o volante com um erro de OFFSET
     // Talvez um `while` não faria muito sentido já que este código pode ser chamado mais de uma vez no loop() tranquilamente
     if(count < centro-OFFSET) move(MENOR_STRENGTH, 1);
     else if(count > centro+OFFSET) move(MENOR_STRENGTH, 0);
     else {
         // Espera um pouco para estabilizar e ver se realmente estamos centralizados
         // Chamar isso no loop() pode dar problema. Então confiaremos no sistema físico
         // e não utilizaremos delay
         //unsigned long time = millis();
         //while(time + 1000 > millis());
         stop();
         if(count >= centro - OFFSET && count <= centro + OFFSET) {
             idle();
             centralizado = true;
         }
     }
 }

/*
void servo() {
    if(count < -EDGE_COUNT || count > EDGE_COUNT) return;
    // Aqui usamos a função map de forma inversa: min -> max e max -> min:
    OCR1A = map(count, -EDGE_COUNT, EDGE_COUNT, MAX_PULSE, MIN_PULSE);
}
 */

void servo() {
    long pos_relativa = count-centro;
    if(pos_relativa < -EDGE_COUNT || pos_relativa > EDGE_COUNT) return;
    // Aqui usamos a função map de forma inversa: min -> max e max -> min:
    OCR1A = map(pos_relativa, -EDGE_COUNT, EDGE_COUNT, MAX_PULSE, MIN_PULSE);
}

void setup() {
    // Inicia serial para debug
    Serial.begin(115200);
    r.begin(true);
    // Configuração de portas (os mesmos do starter code exceto pelo botão e servo)
    PCICR |= (1 << ROTARY_ENC_PCINT_AB_IE);
    PCMSK2 |= (1 << ROTARY_ENC_PCINT_A) | (1 << ROTARY_ENC_PCINT_B);

    DDRB |= ((1<<ATUA_CW) | (1<<ATUA_CCW) | (1<<ATUA_STRENGTH) | (1<<SERVO));
    DDRB &= ~((1<<GP_BUTTON) | (1<<POS_SENSOR));

    PORTB |= ((1<<POS_SENSOR) | (1<<GP_BUTTON));
    PORTB &= ~((1<<ATUA_CW)|(1<<ATUA_CCW));

    // PWM do servo
    // Ativa saída PWM no pino OC1A (PB1) - modo não invertido
    // WGM11 = 1
    TCCR1A = ((1<<WGM11) | (1<<COM1A1));
    TCCR1B = ((1<<WGM13) | (1<<WGM12) | (1<<CS11)); // WGM13=1, WGM12=1, Prescaler 8
    ICR1 = 39999; // ICR1 = (16MHz / (8 * 50Hz)) - 1 = 40000 - 1
    OCR1A = (MAX_PULSE + MIN_PULSE) / 2; // Inicia no centro (90°)

    // PWM do volante
    OCR2A = 0;
    TCCR2A = ((1<<WGM20) | (1<<COM2A1));
    TCCR2B = (1<<CS21);

    sei();
    idle();
    if(!MANUAL) carregarDistancia();
    encontrarCentro();
}

void loop() {
    gpLeitura = (PINB & (1<<GP_BUTTON)) ? 1 : 0;
    if(!centralizado){
        // Continuamos a centralizar o volante enquanto ele não estiver assim
        centralizarVolante();
    } else {
        // Já está centralizado. Então podemos mexer no servo
        servo();
        // Botão de calibração
        if(!gpLeitura && lastGpLeitura && millis() - millisGp > DEBOUNCE) {
            // Salvamos a distancia até a chave
            //salvarDistancia(count+distanciaChave);
            // Tentamos reduzir o número de voltas que o volante daria até 'distanciaChave'
            // ENCODER_RANGE é basicamente 2,5 voltas. Então temos PULSOS_POR_VOLTA (ENCODER_RANGE/2,5) para uma volta
            long novaDistancia = count % PULSOS_POR_VOLTA;
            // Garante que o resultado seja positivo (caso tenha calibrado girando para a esquerda)
            if(novaDistancia < 0) novaDistancia += PULSOS_POR_VOLTA;
            salvarDistancia(novaDistancia);
            distanciaChave = novaDistancia;
            centro = distanciaChave;
            millisGp = millis();
        }
    }
    lastGpLeitura = gpLeitura;
    // Informações de debug
    if (millis()%300==0) {
        Serial.print("Posicao do count: ");
        Serial.print(count);
        Serial.print("; Na chave? ");
        Serial.print(absolute_sw==true?"nao":"sim");
        Serial.println(centralizado==true?"; Centralizado? sim":"; Centralizado? nao");
    }
}

ISR(PCINT2_vect) {
    unsigned char result = r.process();
    if(result == DIR_CW) count--;
    else if(result == DIR_CCW) count++;

    absolute_sw = 0==(PINB&(1<<POS_SENSOR));
}
