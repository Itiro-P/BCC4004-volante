### Projeto Volante.

Aqui contém o código `main.ino` para um projeto de **volante elétrico (drive‑by‑wire)**. O objetivo é **medir a posição do volante**, **centralizá‑lo na inicialização** e **atuar nas rodas** por meio de um servo quando o volante for manipulado. O projeto usa um encoder ótico, um motor DC com controle PWM e um servo (sinal PWM gerado pelo Timer1). Abaixo está a explicação da estrutura do código, entradas, processamento, saídas, funções e cálculos para garantir **2,5 voltas** do volante entre os limites de esterçamento.

---

### Resumo rápido

-   **Propósito**: centralizar automaticamente o volante na inicialização e controlar o servo das rodas conforme a posição do volante.
-   **Entrada**: encoder incremental, chave absoluta, botão de calibração.
-   **Processamento**: ISR atualiza `count`; `encontrarCentro()` e `centralizarVolante()` tratam referência absoluta; `servo()` mapeia posição para pulso.
-   **Saída**: motor DC via ponte H (direção + PWM) e servo via Timer1 PWM.
-   **2,5 voltas** lock‑to‑lock garantidas por `ENCODER_RANGE = 9000` e `PULSOS_POR_VOLTA = 3600`.

---

### Visão geral.

-   **Entrada**
    -   Encoder ótico (Rotary) → gera pulsos; variável global `count` armazena posição relativa desde a energização.
    -   Chave mecânica de referência absoluta → `POS_SENSOR` (PB2) indica quando o volante passa pela posição da chave.
    -   Botão de configuração → `GP_BUTTON` (PB0) para salvar calibração na EEPROM.
-   **Processamento**
    -   Interrupção do encoder (`ISR(PCINT2_vect)`) atualiza `count` e lê `absolute_sw`.
    -   Rotinas de inicialização detectam a chave e calculam a posição central (`encontrarCentro`).
    -   Loop principal decide entre centralizar o volante (`centralizarVolante`) ou controlar o servo (`servo`) e tratar o botão de calibração.
-   **Saída**
    -   Ponte H (pinos PB4, PB5) para girar o motor CW/CCW; PWM em PB3 (OCR2A) controla força.
    -   Servo nas rodas via PWM do Timer1 (OC1A / PB1) com `OCR1A` ajustado conforme posição relativa.

---

### Arquivos auxiliares

-   **Rotary.h / Rotary.cpp**  
    Biblioteca simples para leitura do encoder incremental.
    -   `Rotary(char, char)` construtor com pinos A e B.
    -   `begin(bool pullup=true)` configura entradas.
    -   `unsigned char process()` processa estado e retorna `DIR_CW`, `DIR_CCW` ou `DIR_NONE`.

---

### Hardware e constantes relevantes do código

```c
#define VOLTAS_ENCODER_DA_CHAVE_PARA_CENTRO 730
#define OFFSET 3
#define MAX_STRENGTH 170
#define NORMAL_STRENGTH 160
#define MENOR_STRENGTH 153
#define MIN_PULSE 1050
#define MAX_PULSE 4700
#define EDGE_COUNT 4500
#define ENCODER_RANGE (2 * EDGE_COUNT) // 9000
#define PULSOS_POR_VOLTA (int)(ENCODER_RANGE / 2.5) // 3600
#define MANUAL true
```

-   **EDGE_COUNT = 4500** → metade do alcance do encoder usado pelo sistema (centro → limite).
-   **ENCODER_RANGE = 9000** → total de pulsos considerado para 2,5 voltas (lock‑to‑lock).
-   **PULSOS_POR_VOLTA = 3600** → pulsos por volta do volante (usado para reduzir a distância armazenada).

**Relação com o hardware fornecido**

-   Encoder: **600 ppr** montado em engrenagem de **20 dentes**.
-   Volante: engrenagem de **120 dentes**.
-   Razão mecânica entre volante e encoder: \(120 / 20 = 6\).
-   Pulsos por volta do volante: \(600 \times 6 = 3600\). Isso coincide com `PULSOS_POR_VOLTA` no código.

---

### Cálculo para 2,5 voltas lock‑to‑lock

-   **Desejo**: 2,5 voltas do volante correspondem ao curso total de esterçamento.
-   **Encoder total considerado**: `ENCODER_RANGE = 9000` pulsos → isso representa 2,5 voltas.
-   **Pulsos por volta**: \(9000 / 2.5 = 3600\) pulsos/volta.
-   **Metade do curso (centro → limite)**: `EDGE_COUNT = 4500` pulsos → 1,25 voltas de cada lado do centro.
-   **Mapeamento para servo**: `servo()` mapeia `pos_relativa = count - centro` no intervalo \([-4500, 4500]\) para o pulso do servo \([MAX\_PULSE, MIN\_PULSE]\).

Trecho relevante do código:

```c
long pos_relativa = count - centro;
if(pos_relativa < -EDGE_COUNT || pos_relativa > EDGE_COUNT) return;
OCR1A = map(pos_relativa, -EDGE_COUNT, EDGE_COUNT, MAX_PULSE, MIN_PULSE);
```

---

### Estrutura do código main.ino

#### Principais variáveis globais

-   `volatile long distanciaChave` — distância (em pulsos) entre a chave e o centro (carregada da EEPROM ou valor padrão).
-   `volatile long count` — contador de pulsos do encoder desde a energização.
-   `volatile bool absolute_sw` — estado atual da chave de referência (true = ativa).
-   `volatile long centro` — posição alvo do centro (em pulsos).
-   `volatile bool centralizado` — flag indicando se o volante está centralizado.
-   `Rotary r` — instância da biblioteca do encoder.

---

### Descrição das funções em main.ino

> Para cada função abaixo: **parâmetros** e **retorno** são indicados.

#### `void salvarDistancia(long valor)`

-   **Parâmetros**: `valor` — distância (em pulsos) a salvar na EEPROM.
-   **Retorno**: `void`.
-   **O que faz**: grava um _magic number_ (2 bytes) e o valor (4 bytes) na EEPROM usando `eeprom_update_word` e `eeprom_update_dword`. Emite mensagem via `Serial`.

#### `bool carregarDistancia()`

-   **Parâmetros**: nenhum.
-   **Retorno**: `bool` — `true` se a EEPROM continha o _magic number_ e a distância foi carregada; `false` caso contrário.
-   **O que faz**: verifica o _magic number_ na EEPROM; se válido, lê `distanciaChave` e retorna `true`. Caso contrário, mantém o valor padrão e retorna `false`.

#### `void move(unsigned char power, bool cw = true)`

-   **Parâmetros**:
    -   `power` — valor PWM desejado (0..255, mas o código limita a `MAX_STRENGTH`).
    -   `cw` — direção: `true` = clockwise, `false` = counterclockwise.
-   **Retorno**: `void`.
-   **O que faz**: se `power == 0` chama `idle()`. Caso contrário, seta os pinos de direção (PB4/PB5) conforme `cw` e chama `setPWM(power)` para ajustar `OCR2A`.

#### `void encontrarCentro()`

-   **Parâmetros**: nenhum.
-   **Retorno**: `void`.
-   **O que faz**: rotina de inicialização que:
    -   Lê `absolute_sw`.
    -   Se o sistema ligou em cima da chave, sai dela movendo o motor.
    -   Aguarda estabilização.
    -   Procura a chave e, ao encontrá‑la, zera `count`.
    -   Define `centro = distanciaChave` (valor carregado da EEPROM ou padrão).
-   **Observação**: implementa comportamento robusto para garantir referência absoluta.

#### `void setPWM(unsigned char val)`

-   **Parâmetros**: `val` — valor de PWM.
-   **Retorno**: `void`.
-   **O que faz**: limita `val` a `MAX_STRENGTH` e escreve em `OCR2A` (Timer2 PWM para motor).

#### `void stop()`

-   **Parâmetros**: nenhum.
-   **Retorno**: `void`.
-   **O que faz**: ativa ambos os pinos de direção para frear o motor (ponte H em modo frenagem).

#### `void idle()`

-   **Parâmetros**: nenhum.
-   **Retorno**: `void`.
-   **O que faz**: desliga PWM (`setPWM(0)`) e coloca pinos de direção em 0 (motor em alta impedância / livre).

#### `void centralizarVolante()`

-   **Parâmetros**: nenhum.
-   **Retorno**: `void`.
-   **O que faz**: função chamada repetidamente no `loop()` até `centralizado == true`.
    -   Se `count < centro - OFFSET` → chama `move(MENOR_STRENGTH, 1)` (gira CW).
    -   Se `count > centro + OFFSET` → chama `move(MENOR_STRENGTH, 0)` (gira CCW).
    -   Caso esteja dentro do `OFFSET`, chama `stop()` e marca `centralizado = true` e `idle()`.

#### `void servo()`

-   **Parâmetros**: nenhum.
-   **Retorno**: `void`.
-   **O que faz**: calcula `pos_relativa = count - centro`. Se estiver dentro de `[-EDGE_COUNT, EDGE_COUNT]`, mapeia essa posição para o pulso do servo usando `map()` e escreve em `OCR1A` (Timer1, saída OC1A / PB1). Isso gera o sinal PWM para o servo sem usar a biblioteca `<Servo.h>`.

#### `void setup()`

-   **Parâmetros**: nenhum.
-   **Retorno**: `void`.
-   **O que faz**:
    -   Inicializa `Serial`.
    -   Inicializa o encoder `r.begin(true)`.
    -   Configura interrupções PCINT para os pinos do encoder.
    -   Configura DDRB e pullups para botões e sensores.
    -   Configura Timer1 para PWM do servo (modo de top ICR1, prescaler 8, frequência ~50 Hz).
    -   Configura Timer2 para PWM do motor.
    -   Habilita interrupções globais (`sei()`).
    -   Se `MANUAL` for `false`, tenta `carregarDistancia()` da EEPROM.
    -   Chama `encontrarCentro()`.

#### `void loop()`

-   **Parâmetros**: nenhum.
-   **Retorno**: `void`.
-   **O que faz**:
    -   Lê botão `GP_BUTTON` com debounce.
    -   Se não estiver `centralizado`, chama `centralizarVolante()`.
    -   Se já estiver `centralizado`, chama `servo()` para controlar o servo conforme `count`.
    -   Trata o botão de calibração: ao pressionar, calcula `novaDistancia = count % PULSOS_POR_VOLTA`, ajusta para positivo e chama `salvarDistancia(novaDistancia)`, atualizando `distanciaChave` e `centro`.
    -   Emite mensagens de debug via `Serial` periodicamente.

#### `ISR(PCINT2_vect)`

-   **Parâmetros**: nenhum (rotina de interrupção).
-   **Retorno**: nenhum.
-   **O que faz**:
    -   Chama `r.process()` da biblioteca `Rotary`.
    -   Se `DIR_CW` decrementa `count`; se `DIR_CCW` incrementa `count`.
    -   Atualiza `absolute_sw` lendo `POS_SENSOR`.

---

### Testes e validação.

-   [ ] **Conferir ligações**: PB4/PB5 direção motor; PB3 PWM motor; PB1 PWM servo; PB2 chave; PB0 botão; pinos encoder conforme `ROTARY_ENC_A/B`.
-   [ ] **Verificar Timer1**: ICR1 = 39999 e prescaler 8 → ~50 Hz para servo.
-   [ ] **Limitar PWM do motor**: não usar valores > 170 em testes.
-   [ ] **Testar centralização**:
    -   Energizar com volante em posição aleatória.
    -   Observar se `encontrarCentro()` encontra a chave e define `centro`.
    -   Verificar `centralizarVolante()` aproxima o `count` de `centro` dentro de `OFFSET`.
-   [ ] **Testar mapeamento servo**:
    -   Girar volante de um extremo ao outro; verificar `OCR1A` varia entre `MAX_PULSE` e `MIN_PULSE`.
    -   Confirmar que 2,5 voltas do volante correspondem ao curso total do servo (lock‑to‑lock).
-   [ ] **Salvar calibração**:
    -   Pressionar botão de calibração; verificar EEPROM escrita e mensagem `Serial`.

---

### Trechos do código fundamentais.

**Mapeamento servo (centro → pulso):**

```c
long pos_relativa = count - centro;
if(pos_relativa < -EDGE_COUNT || pos_relativa > EDGE_COUNT) return;
OCR1A = map(pos_relativa, -EDGE_COUNT, EDGE_COUNT, MAX_PULSE, MIN_PULSE);
```

**Salvar distância na EEPROM:**

```c
eeprom_update_word((uint16_t*)EEPROM_MAGIC_ADDR, MAGIC_NUMBER);
eeprom_update_dword((uint32_t*)EEPROM_DISTANCIA_ADDR, (uint32_t)valor);
```

**Interrupção do encoder:**

```c
ISR(PCINT2_vect) {
    unsigned char result = r.process();
    if(result == DIR_CW) count--;
    else if(result == DIR_CCW) count++;
    absolute_sw = 0==(PINB&(1<<POS_SENSOR));
}
```

---

### Dicas práticas e observações

> **Dica**: mantenha `MANUAL` em `true` durante desenvolvimento para evitar sobrescrever EEPROM até que a rotina de calibração esteja validada.

-   O projeto assume que **9000 pulsos** representam 2,5 voltas do volante; isso foi escolhido para coincidir com a relação mecânica e o encoder disponível (600 ppr × razão 6 = 3600 pulsos/volta, e 3600 × 2.5 = 9000).
-   `OFFSET` pequeno (3 pulsos) é usado para evitar oscilações por ruído; ajuste conforme comportamento físico.
-   O servo é controlado diretamente via Timer1 para evitar dependência de bibliotecas e permitir controle fino do pulso.

---
