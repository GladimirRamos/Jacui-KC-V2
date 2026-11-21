# Sumário Executivo: Verificações para Acionamento dos Pulsos de Ativação do Motor

## 🎯 Resposta Direta

Quando o sistema reinicia, a sequência de verificações para acionamento dos pulsos de ativação do motor segue **10 etapas principais**:

### **Fase 1: Verificação Hardware (Setup)**
1. ✓ Testar PCF8574 entrada (endereço 0x20)
2. ✓ Testar PCF8574 saída (endereço 0x24)
3. ✓ Detectar RTC DS1307
4. ✓ Carregar estado anterior da NVS

### **Fase 2: Leitura Inicial de Entradas**
5. ✓ Ler PCF8574 entrada (1ª detecção do estado físico do motor)
6. ✓ Decodificar bits (IN1=Motor, IN2=Modo)

### **Fase 3: Verificação de Reconciliação**
7. ✓ Comparar estado salvo (NVS) com estado físico (apenas MANUAL)
8. ✓ Se divergentes → Enfileirar pulso apropriado

### **Fase 4: Máquina de Estados do Pulso**
9. ✓ Estado VALIDATING (2500ms timeout):
   - Aguardar 2000ms (estabilização)
   - Verificar barramento I2C (0x68/RTC)
   - Validar modo (se APP)
   - Enviar pulso ao PCF8574
   - Salvar em NVS (se APP em MANUAL)
   
10. ✓ Estado ACTIVE (1000ms):
    - Manter bit ativo
    - Depois retirar bit

---

## ⏱️ Prazos Críticos

| Etapa | Timeout | Consequência |
|-------|---------|--------------|
| Leitura PCF | 200ms | Pular ciclo |
| PULSE_VALIDATING | 2500ms | Abort pulso |
| Estabilização I2C | 2000ms | Prosseguir se OK |
| Pulso ativo | 1000ms | Retirar |

---

## 🛡️ 5 Verificações de Segurança

```
1. BARRAMENTO I2C OK
   if (Wire.requestFrom(0x68, 1) > 0) ✓
   
2. MODO VALIDADO (apenas APP)
   if (!modoLocal && remotoOuAgenda == 0) ✓
   
3. PCF8574 RESPONDE
   if (writeOutputPLC()) ✓
   
4. NVS ESCREVE (apenas APP)
   if (preferences.putBool(...) > 0) ✓
   
5. FILA LIMITADA
   máx 2 pulsos (1 ativo + 1 pendente)
```

---

## 🔄 Fluxo Simplificado

```
Boot
  ↓
Testar hardware (setup)
  ↓
Ler estado físico (TaskIOControl)
  ↓
Modo LOCAL?
  → SIM: Parar (sem pulso)
  → NÃO: Prosseguir
  ↓
Modo MANUAL?
  → SIM: Comparar NVS vs Físico
  → NÃO (AGENDA): Validar RTC + horário
  ↓
Estados divergentes?
  → SIM: requestPulse()
  → NÃO: Parar
  ↓
processPulseController() - Máquina de Estados:
  PULSE_VALIDATING (2500ms) → PULSE_ACTIVE (1000ms) → IDLE
  ↓
Próximo ciclo/comando
```

---

## 📊 Modos de Operação

### **LOCAL**
- Motor controlado por painel físico
- NÃO há pulsos de restauração
- Sistema apenas monitora

### **MANUAL APP** (gSchedule.remotoOuAgenda == 0)
- Pulsos enviados por comando Blynk
- Estado salvo em NVS
- Restaurado no próximo boot se divergente

### **AGENDA** (gSchedule.remotoOuAgenda == 1)
- Pulsos enviados por horário programado
- Estado NÃO é salvo em NVS
- RTC deve estar operacional

---

## 🚨 Situações de Falha

| Falha | Verificação | Resultado |
|-------|-------------|-----------|
| I2C Travado | `Wire.available() == 0` | Pulso ABORTADO |
| Timeout | `2500ms` | Pulso ABORTADO |
| PCF desconectado | `writeOutputPLC() == false` | Pulso ABORTADO |
| NVS falha | `putBool() == 0` | Pulso enviado, não salvo |
| Fila cheia | `pendingType != NONE` | Pulso REJEITADO |

---

## 📁 Documentação Gerada

Este repositório contém 4 documentos detalhados:

1. **SEQUENCIA_PULSOS_MOTOR.md** (11KB)
   - Explicação completa de cada etapa
   - Código comentado
   - Estruturas de dados
   - Mecanismos de segurança

2. **FLUXOGRAMA_PULSOS.txt** (23KB)
   - Diagrama ASCII visual de toda sequência
   - Estados e transições
   - Verificações em cada passo
   - Fluxo de boot até operação normal

3. **REFERENCIA_RAPIDA.md** (7KB)
   - 10 verificações principais (checklist)
   - Prazos e timeouts (tabela)
   - Modos de operação
   - Logs importantes
   - Referências de código

4. **CENARIOS_PRATICOS.md** (15KB)
   - 7 cenários reais com sequência completa
   - Restart normal, falhas, fila de pulsos, etc.
   - Incluindo resultados e consequências
   - Para debug e teste

---

## 🔧 Código Principal (Referências)

```cpp
// Funções Chaves
requestPulse()           // Main.cpp:456  → Enfileirar pulso
startPulseNow()         // Main.cpp:418  → Iniciar máquina
processPulseController()// Main.cpp:2791 → Executar máquina
TaskIOControl()         // Main.cpp:1833 → Loop I/O + reconciliação

// Estrutura
gPulseCtrl              // Estado da máquina de pulsos
gInputs                 // Estado físico do PCF (entrada)
gCmd                    // Comandos do APP/agenda
gSchedule               // Configurações de operação
ultimoComandoMotorOff   // Salvo em NVS (MANUAL)
```

---

## 📝 Logs a Monitorar

**Inicialização OK:**
```
"PCF8574 entrada OK"
"PCF8574 saida OK"
"RTC DS1307 identificado"
"Restauracao MANUAL: salvo=LIGADO atual=DESLIGADO"
"Pulso LIGAR solicitado. Motivo=memoria manual"
"Estado do motor confirmado apos pulso de inicializacao"
```

**Problemas:**
```
"### FALHA CRÍTICA: BARRAMENTO TRAVADO. PULSO ABORTADO! ###"
"Pulso enfileirado: controlador ocupado"
"Falha ao salvar comando do motor na NVS"
"Timeout ao acessar I2C para leitura PCF entrada"
```

---

## ✅ Checklist de Compreensão

- [ ] Entendo os 3 modos de operação (LOCAL, MANUAL, AGENDA)
- [ ] Conheço os 3 estados do pulso (IDLE, VALIDATING, ACTIVE)
- [ ] Sei que NVS é salvo APENAS em modo MANUAL APP
- [ ] Entendo o timeout de 2500ms com verificação I2C aos 2000ms
- [ ] Sei que fila de pulsos é limitada a 2 (1 ativo + 1 pendente)
- [ ] Conheço os 5 mecanismos de segurança
- [ ] Consigo rastrear um problema usando os logs
- [ ] Entendo as consequências de cada falha

---

## 📞 Próximos Passos

Para aprofundar em caso de debug:

1. **Verificar logs** → Usar FLUXOGRAMA_PULSOS para rastrear
2. **Verificar hardware** → Testar PCF8574 entrada/saída com I2C scanner
3. **Verificar NVS** → Usar `preferences.begin()` para validar
4. **Verificar RTC** → Checar se `Wire.requestFrom(0x68, 1)` retorna bytes
5. **Validar barramento** → Usar osciloscópio em SDA/SCL se houver colisão

