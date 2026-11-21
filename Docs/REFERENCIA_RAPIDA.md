# Referência Rápida: Verificações para Acionamento dos Pulsos

## 🎯 10 Verificações Principais (Em Ordem)

### 1️⃣ **Hardware OK no Setup**
- PCF8574 Entrada detectado
- PCF8574 Saída detectado  
- RTC DS1307 iniciado
- **Ação se falhar:** Congelar no failMSG() com mensagem de erro

---

### 2️⃣ **Leitura do PCF8574 (200ms timeout)**
```cpp
Wire.requestFrom(PCF_INPUT_ADDR, 1);
if (!Wire.available()) → Falha silenciosa, próximo ciclo
```
- Detectar estado físico do motor (IN1)
- Detectar modo de operação (IN2)
- **Ação se falhar:** Pular ciclo, tentar novamente em 200ms

---

### 3️⃣ **Comparar com Estado Salvo (NVS)**
```
Modo MANUAL APP:
  ultimoComandoMotorOff (salvo) vs gInputs.motorStatus (físico)
  
  Se iguais → Nenhum pulso
  Se divergentes → Enviar pulso apropriado
```

---

### 4️⃣ **Validar Modo de Operação**
```
✓ Local → Ignorar restauração remota
✓ Manual (0) → Usar estado da NVS
✓ Agenda (1) → Validar RTC + horário
✗ Inválido → Nenhum pulso
```

---

### 5️⃣ **Iniciar Pulso - Máquina de Estados**

#### **Estado 1: PULSE_VALIDATING**

**5a. Aguardar 2 segundos**
```
if (now - startedAt) < 2000ms → Aguardar
```

**5b. Verificar Barramento I2C**
```cpp
Wire.requestFrom(0x68, 1);  // RTC
if (bytesRecebidos == 0) {
  → ERROR: "BARRAMENTO TRAVADO"
  → Pulso ABORTADO
}
```

**5c. Validar Modo (se APP)**
```cpp
if (gPulseCtrl.appCommand) {
  if (!gInputs.modoLocal && 
      gSchedule.remotoOuAgenda == 0) {
    → OK para salvar em NVS
  } else {
    → Pulso cancelado (não salva)
  }
}
```

**5d. Enviar Pulso ao PCF8574**
```cpp
pulseBit = (type == LIGA) ? 0 : 1;
output_PLC &= ~(1 << pulseBit);  // Bit = 0 (ativo)
if (!writeOutputPLC()) {
  → ERROR: Falha ao enviar
  → Restaurar bit e abortar
}
```

**5e. Salvar em NVS (apenas APP)**
```cpp
preferences.putBool("MemMotorState", motorOff);
// motorOff = (type == DESLIGA)
```

#### **Estado 2: PULSE_ACTIVE (1000ms)**

**5f. Manter Pulso Ativo**
```
Bit permanece em 0 (ligado no PCF)
Aguardar 1000ms
```

**5g. Retirar Pulso**
```cpp
pulseBit = (type == LIGA) ? 0 : 1;
output_PLC |= (1 << pulseBit);  // Bit = 1 (inativo)
writeOutputPLC();
```

---

### 6️⃣ **Registrar Sucesso**
```
queueLogf("Pulso LIGAR/DESLIGAR por [razão]");
LOG: "Pulso [TIPO] finalizado com sucesso"
```

---

### 7️⃣ **Executar Pulso Pendente**
```
if (gPulseCtrl.pendingType != PULSE_NONE)
  → Iniciar máquina de estados novamente
```

---

### 8️⃣ **Aguardar Confirmação de Feedback**
```
Comparar estado físico com esperado:
  gInputs.motorStatus == startupMotorTargetOff
  
✓ Igual → Confirmação recebida
✗ Diferente → Continuar monitorando
```

---

### 9️⃣ **Marcar Inicialização Completa**
```
motorInputsInitialized = true
startupMotorReconcilePending = false
startupMotorAwaitingFeedback = false
```

---

### 🔟 **Operação Normal**
```
TaskIOControl executa ciclos de 50ms:
✓ Controle por nível (se ativo)
✓ Controle por agenda (se em AGENDA)
✓ Processamento de comandos APP
✓ Processamento de pulsos (máquina de estados)
```

---

## ⏱️ Prazos e Timeouts

| Operação | Timeout | Ação se Expirar |
|----------|---------|-----------------|
| Leitura PCF8574 | 200ms | Pular ciclo |
| Obter mutex I2C | 20-50ms | Log de aviso / Cancelar |
| PULSE_VALIDATING | 2500ms | Abort pulso |
| Aguardar estabilização I2C | 2000ms | Prosseguir |
| Pulso ativo (PULSE_ACTIVE) | 1000ms | Retirar pulso |
| Sincronização NTP | Sistema | Log de aviso |

---

## 🛡️ Falhas Críticas (Abort Pulso)

```
1. Barramento I2C não responde
   → Pulso ABORTADO
   
2. Timeout PULSE_VALIDATING > 2500ms
   → Pulso ABORTADO
   
3. Falha ao enviar para PCF8574
   → Pulso ABORTADO
   
4. Pulso APP em modo LOCAL ou AGENDA
   → Pulso CANCELADO (não salva em NVS)
```

---

## 💾 Dados Salvos em NVS

| Chave | Tipo | Propósito | Restaurado em |
|-------|------|----------|----------------|
| `MemMotorState` | bool | Último comando APP (LIGA/DESLIGA) | Boot MANUAL |
| `counterRST` | uint | Contador de resets | Display/Blynk |
| `sensorMin` | int | Calibração sensor nível min | Leitura ADC |
| `sensorMax` | int | Calibração sensor nível max | Leitura ADC |

---

## 📊 Modos de Operação

### **LOCAL (Painel Físico)**
```
Lógica de pulsos desabilitada
Motor controlado por botões do painel
Nenhuma restauração remota
```

### **MANUAL APP**
```
✓ Pulsos enviados por comando do APP
✓ Estado salvo em NVS
✓ Estado restaurado no próximo boot
✓ Comparação: NVS vs Físico
```

### **AGENDA**
```
✓ Pulsos enviados por programa horário
✗ Estado NÃO é salvo em NVS
✓ RTC deve estar operacional
✓ Horários: gSchedule.horaLigaSec / horaDesligaSec
```

---

## 🔄 Fluxo de Acionamento (Resumido)

```
1. App envia comando LIGAR/DESLIGAR
   ↓
2. TaskIOControl detecta gCmd.forcaLiga / forcaDesliga
   ↓
3. requestPulse(PULSE_LIGA/DESLIGA, "APP", true)
   ↓
4. startPulseNow() → gPulseCtrl.step = PULSE_VALIDATING
   ↓
5. processPulseController() executa máquina de estados:
   - Aguardar 2000ms
   - Verificar I2C
   - Validar modo
   - Enviar pulso (1 ciclo)
   ↓
6. Pulso permanece ativo por 1000ms (PULSE_ACTIVE)
   ↓
7. Retirar pulso
   ↓
8. Voltar para IDLE
   ↓
9. Monitorar feedback da entrada física
   ↓
10. Próximo comando (ou pulso pendente)
```

---

## 🚨 Checklist de Verificação

Ao investigar problema com acionamento do motor:

- [ ] PCF8574 entrada conectado (endereço 0x20)
- [ ] PCF8574 saída conectado (endereço 0x24)
- [ ] RTC DS1307 presente (endereço 0x68)
- [ ] Barramento I2C sem colisões
- [ ] Modo de operação correto (LOCAL/MANUAL/AGENDA)
- [ ] NVS contém estado anterior (se MANUAL)
- [ ] Entrada física do motor responde
- [ ] Relés de acionamento funcionam
- [ ] Sensor de corrente ativo (Modbus)
- [ ] Blynk conectado ao servidor (se APP)

---

## 🔗 Referências de Código

| Função | Localização | Propósito |
|--------|-------------|----------|
| `requestPulse()` | Main.cpp:456 | Enfileirar pulso |
| `startPulseNow()` | Main.cpp:418 | Iniciar máquina de estados |
| `processPulseController()` | Main.cpp:2791 | Executar máquina de estados |
| `TaskIOControl()` | Main.cpp:1833 | Loop principal de I/O |
| `pulseLiga()` | Main.cpp:2894 | Alias para PULSE_LIGA |
| `pulseDesliga()` | Main.cpp:2898 | Alias para PULSE_DESLIGA |
| `writeOutputPLC()` | Main.cpp:~2740 | Enviar bits para PCF8574 saída |

---

## 📝 Logs Importantes

### Sucesso
```
"Pulso LIGAR solicitado. Motivo=memoria manual"
"Pulso LIGAR finalizado com sucesso. Motivo=memoria manual"
"Restauracao MANUAL: salvo=LIGADO atual=DESLIGADO"
"Estado do motor confirmado apos pulso de inicializacao"
```

### Aviso
```
"Pulso enfileirado: controlador ocupado"
"Timeout ao acessar I2C para leitura PCF entrada"
"Falha ao sincronizar NTP"
```

### Erro
```
"### FALHA CRÍTICA: BARRAMENTO TRAVADO. PULSO ABORTADO! ###"
"Falha ao enviar pulso LIGAR; comando nao salvo"
"Nao foi possivel encontrar o RTC DS1307"
"Falha ao salvar comando do motor na NVS"
```

