# Sequência de Verificações para Acionamento dos Pulsos de Ativação do Motor

## 🎯 Visão Geral

Quando o sistema reinicia, a sequência de acionamento dos pulsos do motor segue uma máquina de estados bem definida com múltiplas verificações de segurança e validação. O processo é assíncrono e não-bloqueante, garantindo que o sistema continue responsivo durante a restauração do estado do motor.

---

## 📋 Fases de Inicialização (Função `setup()`)

### **1. Inicialização de Hardware Base** (linhas 1024-1180)
```
✓ DAC travado em 0 (válvula fechada)
✓ Serial iniciado (115200 baud)
✓ FreeRTOS: Mutex (I2C e Data) + Fila de Logs criados
✓ Estruturas de dados zeradas (gTime, gInputs, gSchedule, gCmd, gModbus, gRun)
✓ I2C e Display OLED inicializados
✓ PCF8574 (Entrada) testado e validado
✓ PCF8574 (Saída) testado e validado
✓ RTC DS1307 detectado
✓ NTP sincronizado (relógio do sistema)
✓ Sensor de Nível calibrado (limites carregados da NVS)
✓ Hardware Watchdog iniciado (3 pulsos de boot handshake)
✓ Serial Modbus/Hardware Secundário iniciado (9600 baud)
```

### **2. Criação das Tarefas FreeRTOS** (linhas 1200-1215)
As tarefas são criadas em 2 cores paralelos:
- **Core 0 (Hardware)**: TaskSupervisor, TaskRTC, TaskModbus, TaskDisplay, TaskIOControl
- **Core 1 (Comunicação)**: TaskBlynk

---

## 🔄 Máquina de Estados dos Pulsos de Motor

A execução dos pulsos segue 3 estados principais:

### **Estado 1: `PULSE_IDLE` (Ocioso)**
- Controlador está pronto para receber novos comandos
- Nenhum pulso em execução
- Pulsos pendentes podem ser enfileirados

### **Estado 2: `PULSE_VALIDATING` (Validando - 2500ms timeout)**

Este é o estado crítico onde todas as verificações de segurança ocorrem:

#### **2.1 Esperar 2 segundos (linha 2832)**
```
- Aguarda estabilização da comunicação I2C (2000ms)
- Reduz interferências eletromagnéticas
- Garante que o barramento I2C esteja estável
```

#### **2.2 Verificação do Barramento I2C (linhas 2809-2818)**
```
✓ Verificar se barramento responde (tentativa de leitura no endereço 0x68)
✓ Se FALHA → Pulso ABORTADO ("BARRAMENTO TRAVADO")
✓ Se TIMEOUT no mutex → Pulso ABORTADO (timeout de 2500ms geral)
```

#### **2.3 Verificação Modo de Operação (linhas 2834-2843)**
Se o comando for do APP (appCommand == true):
```
✓ Obter mutex de dados (timeout 50ms)
✓ Verificar se NÃO está em modo LOCAL
✓ Verificar se está em modo MANUAL APP (gSchedule.remotoOuAgenda == 0)
✓ Se FALHA → Pulso é cancelado, não salva em NVS
✓ Se SUCESSO → Será salvo em NVS (linha 2847)
```

#### **2.4 Envio do Pulso (linhas 2845-2857)**
```
✓ Preparar bit de pulso:
  - PULSE_LIGA   → Bit 0 (relé de acionamento "LIGAR")
  - PULSE_DESLIGA → Bit 1 (relé de acionamento "DESLIGAR")

✓ Limpar o bit no output_PLC: output_PLC &= ~(1 << pulseBit)
  (Lógica invertida: 0 = Ligado, 1 = Desligado)

✓ Enviar para o PCF8574 de saída via I2C (writeOutputPLC)
  - Se FALHA → Pulso ABORTADO, bit é restaurado
  - Se SUCESSO → Passa para o estado PULSE_ACTIVE
```

#### **2.5 Salvar em NVS (linhas 2857-2871)** (apenas para APP)
```
✓ Se foi comando do APP:
  - Abrir namespace NVS "my-app"
  - Salvar: preferências.putBool("MemMotorState", motorOff)
  - Fechar NVS
  - Atualizar variável global: ultimoComandoMotorOff = motorOff
```

### **Estado 3: `PULSE_ACTIVE` (Ativo - 1000ms duração)**

O pulso permanece ativo por 1000ms:

#### **3.1 Período Ativo (linha 2878)**
```
- Bit permanece em nível lógico 0 (ligado no PCF8574)
- Circuito externo decodifica o pulso
- Relé de acionamento do motor é energizado
```

#### **3.2 Finalização do Pulso (linhas 2878-2887)**
```
✓ Após 1000ms, retornar bit ao nível 1 (desligado)
✓ Enviar para o PCF8574 de saída (writeOutputPLC)
✓ Registrar log de sucesso
✓ Voltar para PULSE_IDLE
✓ Executar pulsos pendentes (tryStartPendingPulse)
```

---

## 🚀 Fluxo de Inicialização do Motor (TaskIOControl)

Após `motorInputsInitialized = false` ser lido pela primeira vez:

### **Leitura Inicial de Entradas (linhas 1899-1932)**
```
✓ Ler PCF8574 de entrada (IN1=Motor Status, IN2=Modo Local)
✓ Validar leitura I2C (timeout 200ms)
✓ Decodificar bits:
  - IN1 (Bit 0) → gInputs.motorStatus (true=DESLIGADO, false=LIGADO)
  - IN2 (Bit 1) → gInputs.modoLocal
✓ Ler sensor de nível (ADC em GPIO)
✓ Processar leitura com calibração salva em NVS
```

### **Verificação de Reconciliação (linhas 2001-2037)**
Se `startupMotorReconcilePending == true`:

#### **Caso 1: Modo LOCAL (linha 2003)**
```
✓ Verificar: gInputs.modoLocal == true
  → Ignorar qualquer restauração remota
  → Motor controlado exclusivamente pelo painel físico
  → Marcar reconciliação como completa
```

#### **Caso 2: Modo MANUAL APP (linhas 2007-2020)**
```
✓ Verificar: gSchedule.remotoOuAgenda == 0 (MODO MANUAL)
✓ Carregar estado salvo em NVS: ultimoComandoMotorOff
✓ Comparar com estado físico atual: gInputs.motorStatus
✓ Se DIVERGENTES:
  - Se deve estar DESLIGADO (startupMotorTargetOff=true)
    → Enviar PULSO_DESLIGA ("memoria manual")
  - Se deve estar LIGADO (startupMotorTargetOff=false)
    → Enviar PULSO_LIGA ("memoria manual")
  → Aguardar confirmação de feedback
✓ Registrar status no log detalhado
```

#### **Caso 3: Modo AGENDA (linhas 2021-2032)**
```
✓ Verificar: gSchedule.remotoOuAgenda == 1 (MODO AGENDA)
✓ Marcar reconciliação como completa
✓ Nota: Pulsos de agenda são disparados em ciclo separado
  (não na fase de inicialização)
```

#### **Caso 4: Modo Inválido (linhas 2033-2036)**
```
✗ Modo desconhecido
  → Nenhum pulso de inicialização enviado
  → Log de erro crítico
```

### **Aguardar Feedback (linhas 2039-2043)**
```
Se startupMotorAwaitingFeedback == true:
  ✓ Monitorar gInputs.motorStatus
  ✓ Se estado == estado_esperado (startupMotorTargetOff)
    → Confirmação recebida
    → Marcar como completo
    → Desabilitar aguardo
```

---

## ⚙️ Verificações em Paralelo

### **Blynk Cloud (TaskBlynk)** - Core 1
```
✓ Conectar ao servidor Blynk
✓ Manter conexão Wi-Fi ativa
✓ Receber comandos via app (Botões V-pins)
✓ Atualizar display do app com estado real
```

### **Supervisor de Corrente (TaskIOControl)**
```
✓ Monitorar corrente trifásica (R, S, T)
✓ Calcular média: (iR + iS + iT) / 3
✓ Se corrente < setCorrente AND motor desligado:
  → Ligar relés 3 e 4 (válvula proporcional fechada)
  → Proteger bomba contra cavitação
✓ Se corrente adequada:
  → Desligar relés 3 e 4 (válvula aberta)
```

### **Controle de Nível (TaskIOControl)**
```
✓ Se Float_Level_CNTRL_Status_ON_OFF == true E motor autorizado:
  - Se nível < nível_mínimo AND motor DESLIGADO
    → Solicitar PULSO_LIGA ("nivel minimo")
  - Se nível > nível_máximo AND motor LIGADO
    → Solicitar PULSO_DESLIGA ("nivel maximo")
```

---

## 🛡️ Mecanismos de Segurança

### **1. Dupla Verificação de I2C**
- Teste durante `setup()` (linhas 1070-1080)
- Verificação antes de cada pulso (linha 2810)

### **2. Timeout de Estabilização (2500ms)**
- Aguarda barramento I2C se recuperar de colisões
- Se ultrapassado → Pulso abortado

### **3. Validação de Modo**
- Pulsos de APP só são salvos em MODO MANUAL
- Outros modos ignoram comandos remotos se apropriado

### **4. Fila de Pulsos Limitada**
- Máximo 2 pulsos em fila (1 ativo + 1 pendente)
- Pulsos extras são rejeitados com log

### **5. Mutex de Dados**
- Acesso seguro a estruturas compartilhadas
- Timeout de 50ms para operações rápidas

### **6. Hardware Watchdog**
- Pulsos de heartbeat a cada ciclo
- Reinicia ESP32 se WDT não receber pulso por ~100ms (configurable)

---

## 📊 Diagrama de Estados

```
                    ┌─────────────────┐
                    │   PULSE_IDLE    │
                    └────────┬────────┘
                             │
                 requestPulse() / startPulseNow()
                             │
                             ▼
                 ┌─────────────────────────────┐
                 │   PULSE_VALIDATING (2500ms) │
                 ├─────────────────────────────┤
                 │ 1. Esperar 2000ms           │
                 │ 2. Verificar I2C            │
                 │ 3. Validar modo operação    │
                 │ 4. Enviar pulso ao PCF8574  │
                 │ 5. Salvar em NVS (APP)      │
                 └────────────┬────────────────┘
                              │ OK
                              ▼
                 ┌─────────────────────────────┐
                 │   PULSE_ACTIVE (1000ms)     │
                 ├─────────────────────────────┤
                 │ 1. Manter pulso ativo       │
                 │ 2. Aguardar 1000ms          │
                 │ 3. Retirar pulso do PCF8574 │
                 │ 4. Registrar sucesso        │
                 └────────────┬────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │   PULSE_IDLE    │
                    │ tryStartPending │
                    │   Pulse()       │
                    └─────────────────┘
```

---

## 🔧 Variáveis Globais Importantes

```c
// Estrutura de Controle de Pulsos
typedef struct {
  PulseType step;              // IDLE, VALIDATING, ACTIVE
  PulseType type;              // LIGA ou DESLIGA
  TickType_t startedAt;        // Timestamp de início
  TickType_t pulseAt;          // Timestamp quando pulso foi enviado
  bool appCommand;             // Veio do APP?
  bool floatLevelCommand;      // Veio do controle de nível?
  char reason[24];             // Motivo do pulso
  
  // Pulso pendente (fila com 1 slot)
  PulseType pendingType;
  bool pendingAppCommand;
  char pendingReason[24];
} PulseController;

// Estado do motor (salvo em NVS)
bool ultimoComandoMotorOff;    // Último comando enviado (true=DESLIGAR)

// Entrada física do PCF8574
struct InputData {
  uint8_t raw;                 // Byte bruto lido
  bool motorStatus;            // true = DESLIGADO, false = LIGADO
  bool modoLocal;              // true = LOCAL, false = APP/AGENDA
};

// Modo de operação
gSchedule.remotoOuAgenda:      // 0=MANUAL APP, 1=AGENDA
```

---

## 📝 Resumo da Sequência

1. **Setup:** Inicializar hardware, criar tarefas FreeRTOS
2. **TaskIOControl (1ª execução):**
   - Ler PCF8574 de entrada → Detectar estado do motor físico
   - Se MODO MANUAL: Comparar com NVS
     - Se divergente → Enviar PULSO apropriado
3. **ProcessPulseController:**
   - **VALIDATING (2000-2500ms):** Verificações de segurança
   - **ACTIVE (1000ms):** Manter pulso ativo
   - **IDLE:** Pronto para próximo pulso
4. **TaskIOControl (ciclos posteriores):**
   - Executar controle por nível (se habilitado)
   - Executar controle por agenda (se em AGENDA)
   - Processar comandos do APP (se em MANUAL)

---

## ⚠️ Casos de Falha Conhecidos

| Falha | Verificação | Ação |
|-------|------------|------|
| I2C Travado | `barramentoOk == false` | Abort pulso |
| Timeout I2C | `(nowTick - startedAt) >= 2500ms` | Abort pulso |
| Falha ao enviar | `writeOutputPLC() == false` | Abort pulso |
| Falha NVS | `preferences.putBool() == 0` | Log erro (pulso já enviado) |
| PCF8574 desconectado | Leitura no setup | Exibir "FALHA OUTPUT"/"FALHA INPUT" |
| RTC ausente | Leitura no setup | Exibir "FALHA RTC" |
| Sensor de Corrente offline | `mbCopy.status != 0x00` | Desabilitar relés 3/4 (segurança) |

