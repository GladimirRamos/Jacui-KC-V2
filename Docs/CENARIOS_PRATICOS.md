# Cenários Práticos: Acionamento do Motor no Boot

## 📋 Cenário 1: Restart após Comando LIGAR pelo APP (MANUAL)

**Situação:**
- Sistema estava em modo MANUAL APP
- Usuário clicou "LIGAR" no aplicativo Blynk
- Motor ligou com sucesso
- ESP32 reinicia (watchdog, software, ou alimentação)

**Sequência:**

```
╔════════════════════════════════════════════════════╗
║ BOOT #1: Sistema estava LIGADO                    ║
╚════════════════════════════════════════════════════╝

1. setup() → Inicializar hardware
   ✓ PCF entrada: OK
   ✓ PCF saída: OK
   ✓ RTC: OK
   
2. Carregar NVS
   ultimoComandoMotorOff = false  (salvo como LIGADO)
   gSchedule.remotoOuAgenda = 0   (MANUAL)

3. TaskIOControl - 1ª leitura (200ms)
   Wire.requestFrom(0x20, 1)
   inputPCF = 0xFE (bit 0 = 0 → motor LIGADO)
   motorInputsInitialized = true
   
4. Verificação de Reconciliação
   ✓ Não está em LOCAL
   ✓ Está em MANUAL (remotoOuAgenda == 0)
   ✓ Comparar:
     - ultimoComandoMotorOff = false (LIGADO)
     - gInputs.motorStatus = false (LIGADO)
   ✓ ESTADOS IGUAIS → Nenhum pulso enviado
   
   LOG: "Restauracao MANUAL: salvo=LIGADO atual=LIGADO"
   
5. Sistema pronto para operação normal
   ✓ Motor continua ligado
   ✓ Nenhum comando enviado
```

---

## 📋 Cenário 2: Restart após Retirada de Alimentação (Motor Desligou Naturalmente)

**Situação:**
- Sistema estava em modo MANUAL APP
- Motor estava LIGADO (ultimoComandoMotorOff = false)
- Falha de alimentação por alguns segundos
- Motor desligou (relé perdeu energia)
- Alimentação retorna, ESP32 reinicia

**Sequência:**

```
╔════════════════════════════════════════════════════╗
║ BOOT #2: Motor desligou mas NVS diz LIGADO         ║
╚════════════════════════════════════════════════════╝

1. setup() → Inicializar hardware
   ✓ Todos os testes passam

2. Carregar NVS
   ultimoComandoMotorOff = false  (salvo como LIGADO)
   gSchedule.remotoOuAgenda = 0   (MANUAL)

3. TaskIOControl - 1ª leitura (200ms)
   Wire.requestFrom(0x20, 1)
   inputPCF = 0xFF (bit 0 = 1 → motor DESLIGADO)
   gInputs.motorStatus = true  (DESLIGADO)
   
4. Verificação de Reconciliação 
   startupMotorReconcilePending = true
   
   ✓ Não está em LOCAL
   ✓ Está em MANUAL (remotoOuAgenda == 0)
   ✓ Comparar:
     - ultimoComandoMotorOff = false (esperado LIGADO)
     - gInputs.motorStatus = true (físico DESLIGADO)
   ✗ DIVERGENTES → ENVIAR PULSO!
   
   startupMotorTargetOff = false (esperado LIGADO)
   pulseLiga("memoria manual")  ← Enfileirar PULSO LIGAR
   startupMotorAwaitingFeedback = true
   
   LOG: "Restauracao MANUAL: salvo=LIGADO atual=DESLIGADO"

5. processPulseController() - Estado: PULSE_VALIDATING
   
   5a. Aguardar 2000ms
       if (now - startedAt) < 2000ms { return; }
       
   5b. Verificar I2C (após 2000ms)
       Wire.requestFrom(0x68, 1)  // RTC
       if (bytesRecebidos > 0) {
           ✓ Barramento OK
       } else {
           ✗ ERRO: "BARRAMENTO TRAVADO"
           Pulso ABORTADO
           → Ir para Cenário 7
       }
       
   5c. Aguardar total 2000ms (já agüardou 2000ms, faltam 0ms)
       if (now - startedAt) >= 2000ms {
           Prosseguir para envio
       }
       
   5d. Validar modo (appCommand = true)
       xSemaphoreTake(mtxData, 50ms)
       if (!gInputs.modoLocal && 
           gSchedule.remotoOuAgenda == 0) {
           ✓ OK para salvar em NVS
       }
       
   5e. Enviar pulso ao PCF8574
       pulseBit = 0  (PULSE_LIGA)
       output_PLC &= ~(1 << 0)   // output_PLC &= 0xFE
       if (writeOutputPLC()) {
           ✓ Bit enviado com sucesso
           gPulseCtrl.step = PULSE_ACTIVE
       } else {
           ✗ ERRO ao enviar
           Pulso ABORTADO
       }
       
   5f. Salvar em NVS
       motorOff = false  (tipo == PULSE_LIGA)
       preferences.putBool("MemMotorState", false)
       ultimoComandoMotorOff = false
       
       LOG: "Comando do motor gravado na NVS: LIGAR"

6. processPulseController() - Estado: PULSE_ACTIVE
   Aguardar 1000ms com bit = 0 (ativo)
   
   Circuito externo:
   - Detecta bit = 0 no relé 1 (GPIO1 do PCF)
   - Aciona decodificador / relé de comando
   - Envia sinal para contactora do motor
   - Motor inicia

7. Após 1000ms:
   output_PLC |= (1 << 0)   // output_PLC |= 0x01 (bit volta a 1)
   writeOutputPLC()
   gPulseCtrl.step = PULSE_IDLE
   gPulseCtrl.type = PULSE_NONE
   
   LOG: "Pulso LIGAR finalizado com sucesso. Motivo=memoria manual"
   queueLogf("Pulso LIGAR por memoria manual")

8. TaskIOControl aguarda feedback
   startupMotorAwaitingFeedback = true
   
   Próxima leitura do PCF (200ms depois):
   inputPCF = 0xFE (bit 0 = 0 → motor LIGADO)
   gInputs.motorStatus = false  (LIGADO)
   
   if (gInputs.motorStatus == startupMotorTargetOff) {
       // false == false? NÃO, espera false (LIGADO)
       // mas recebeu false → IGUAL
       startupMotorAwaitingFeedback = false
       
       LOG: "Estado do motor confirmado apos pulso de inicializacao"
   }

9. Sistema pronto
   ✓ Motor ligado
   ✓ Estado restaurado com sucesso
   ✓ NVS sincronizado com realidade
```

---

## 📋 Cenário 3: Modo LOCAL (Botão Físico do Painel)

**Situação:**
- Sistema estava em modo LOCAL
- Motor controlado exclusivamente por botões do painel
- Alguma razão causou restart

**Sequência:**

```
╔════════════════════════════════════════════════════╗
║ BOOT #3: Modo LOCAL - Sem restauração remota      ║
╚════════════════════════════════════════════════════╝

1. setup() → Inicializar hardware OK

2. Carregar NVS
   ultimoComandoMotorOff = false
   gSchedule.remotoOuAgenda = 0 ou 1 (irrelevante)

3. TaskIOControl - 1ª leitura
   Wire.requestFrom(0x20, 1)
   inputPCF = 0xFD (bit 1 = 1 → LOCAL)
   gInputs.modoLocal = true
   gInputs.motorStatus = true (DESLIGADO)

4. Verificação de Reconciliação
   startupMotorReconcilePending = true
   
   if (gInputs.modoLocal == true) {
       startupMotorReconcilePending = false
       
       LOG: "Inicializacao em modo LOCAL; restauracao remota do motor ignorada"
   }
   
   ✗ NENHUM PULSO ENVIADO
   
   Motor permanece em seu estado físico
   (Sem nenhuma interferência do sistema)

5. Sistema aguarda comando do painel físico
   Botões no painel controlam o motor
   Sistema monitora e atualiza display/Blynk com status real
```

---

## 📋 Cenário 4: Modo AGENDA

**Situação:**
- Sistema estava em modo AGENDA
- Horário programado: LIGA 06:00, DESLIGA 18:00
- Restart ocorre às 10:00

**Sequência:**

```
╔════════════════════════════════════════════════════╗
║ BOOT #4: Modo AGENDA - Sincronizar com horário    ║
╚════════════════════════════════════════════════════╝

1. setup() → Inicializar hardware OK
   RTC sincronizado com NTP

2. Carregar NVS
   ultimoComandoMotorOff = X (irrelevante)
   gSchedule.remotoOuAgenda = 1 (AGENDA)
   gSchedule.horaLigaSec = 06:00:00 = 21600s
   gSchedule.horaDesligaSec = 18:00:00 = 64800s

3. TaskIOControl - 1ª leitura
   gInputs = {motorStatus: ?, modoLocal: false}

4. Verificação de Reconciliação
   startupMotorReconcilePending = true
   
   if (gInputs.modoLocal == false) {
       if (gSchedule.remotoOuAgenda == 0) {
           // MANUAL - não entra aqui
       } else if (gSchedule.remotoOuAgenda == 1) {
           startupMotorReconcilePending = false
           
           LOG: "Entrada em AGENDA: esperado=? atual=?"
           ✗ NÃO há pulso imediato no boot
       }
   }
   
   Nota: No ciclo seguinte da TaskIOControl,
   verificará o horário e será enviado pulso
   apenas se necessário

5. TaskIOControl - Ciclos normais (50ms)
   Calcular estado esperado por agenda:
   
   Hora atual: 10:00:00 = 36000s
   horaLiga: 21600s (06:00)
   horaDesliga: 64800s (18:00)
   
   if (36000 >= 21600 && 36000 <= 64800) {
       motorOffEsperado = false  (LIGADO)
   }
   
   Comparar com físico:
   if (gInputs.motorStatus != motorOffEsperado) {
       // Disparidade → Enviar pulso apropriado
   } else {
       // OK, motor está no estado esperado
   }

6. Próximo horário de desligamento (18:00)
   TaskIOControl contínuo monitora hora
   Quando atingir 18:00:00 = 64800s:
   
   motorOffEsperado = true  (DESLIGADO)
   if (gInputs.motorStatus != true) {
       pulseDesliga("agendamento")
   }

7. ✓ Sistema funciona conforme agenda
```

---

## 📋 Cenário 5: Falha de I2C (Barramento Travado)

**Situação:**
- Algum dispositivo I2C está puxando a linha SDA ou SCL
- RTC não responde
- Sistema tenta acionar motor
- Teste de barramento falha

**Sequência:**

```
╔════════════════════════════════════════════════════╗
║ BOOT #5: I2C Travado - Abort Pulso                ║
╚════════════════════════════════════════════════════╝

1. setup() → Inicializar hardware
   
   Teste PCF entrada:
   Wire.beginTransmission(0x20)
   if (errorCode != 0) {
       ✗ ERRO → failMSG("FALHA INPUT")
       ESP_LOGE(TAG_I2C, "Falha no PCF8574 de entrada")
       → Congelar no loop infinito

2. Se conseguir passar do setup() mas I2C falhar depois:

   TaskIOControl tenta ler PCF:
   if (xSemaphoreTake(mtxI2C, 200ms) == pdTRUE) {
       Wire.requestFrom(0x20, 1)
       if (!Wire.available()) {
           readOk = false
           motorInputsInitialized = false
       }
   }
   
   → Próximo ciclo tenta novamente

3. Se sistema conseguir enfileirar pulso antes da falha:

   processPulseController() - PULSE_VALIDATING
   
   Após 2000ms:
   if (xSemaphoreTake(mtxI2C, 20ms) == pdTRUE) {
       Wire.requestFrom(0x68, 1)  // RTC
       bytesRecebidos = Wire.available()
       if (bytesRecebidos == 0) {
           ✗ barramentoOk = false
       }
   }
   
   if (!barramentoOk) {
       ESP_LOGE(TAG_IO,
           "### FALHA CRÍTICA: BARRAMENTO TRAVADO. 
            PULSO LIGAR ABORTADO! ###");
       
       gPulseCtrl.type = PULSE_NONE
       gPulseCtrl.step = PULSE_IDLE
       tryStartPendingPulse()
       return;
   }

4. Motor NÃO recebe pulso
   Estado não é alterado
   NVS não é atualizado
   LOG mostra erro crítico

5. Usuário vê no Blynk:
   "Estado anterior: DESLIGADO" (do último ciclo bem-sucedido)
   "ERRO: Barramento I2C travado"

6. Ação corretiva:
   - Verificar conexões I2C
   - Verificar se RTC está respondendo
   - Se problema persiste → Reiniciar ESP32
```

---

## 📋 Cenário 6: Fila de Pulsos (2 comandos rápidos)

**Situação:**
- Usuário clica "LIGAR" no APP
- Pulso está sendo processado (PULSE_VALIDATING)
- Usuário clica "DESLIGAR" antes de terminar (< 2.5s depois)

**Sequência:**

```
╔════════════════════════════════════════════════════╗
║ BOOT #6: Fila de Pulsos - Máximo 2 pulsos         ║
╚════════════════════════════════════════════════════╝

1. t=0ms: Usuário clica "LIGAR"
   
   gCmd.forcaLiga = true (atualizado no Blynk)
   TaskIOControl recebe comando:
   if (cmd.forcaLiga) {
       if (requestPulse(PULSE_LIGA, "APP", true)) {
           manualMotorSolicitadoLigado = true
       }
   }
   
   requestPulse():
   gPulseCtrl.step == PULSE_IDLE
   → startPulseNow(PULSE_LIGA, ...)
   → gPulseCtrl.step = PULSE_VALIDATING
   
   LOG: "Pulso LIGAR solicitado. Motivo=APP"

2. t=50ms a t=1500ms:
   processPulseController() executando PULSE_VALIDATING
   
   Aguardando 2000ms total
   I2C sendo testado a cada ciclo
   
3. t=1200ms: Usuário clica "DESLIGAR"
   
   gCmd.forcaDesliga = true
   TaskIOControl recebe:
   if (cmd.forcaDesliga) {
       manualMotorSolicitadoLigado = false
       pulseDesliga("APP", true)  → requestPulse()
   }
   
   requestPulse(PULSE_DESLIGA, ...):
   
   if (gPulseCtrl.step != PULSE_IDLE) {
       // Sim, está em VALIDATING
       if (gPulseCtrl.pendingType == PULSE_NONE) {
           gPulseCtrl.pendingType = PULSE_DESLIGA
           gPulseCtrl.pendingAppCommand = true
           strncpy(gPulseCtrl.pendingReason, "APP", ...)
           
           ESP_LOGW(TAG_IO,
               "Pulso enfileirado: controlador ocupado (LIGAR).
                Pendente=DESLIGAR");
           
           return true;
       } else {
           // Já existe pendente → Rejeitar
           ESP_LOGW(TAG_IO,
               "Pulso ignorado: fila pendente cheia.
                Atual=LIGAR Pendente=DESLIGAR Novo=DESLIGAR");
           return false;
       }
   }
   
   LOG: "Pulso DESLIGAR enfileirado"

4. t=2000ms: Pulso LIGAR entra em PULSE_VALIDATING
   I2C testado, barramento OK
   Enviar bit = 0 ao PCF (Relé LIGAR)
   
   gPulseCtrl.step = PULSE_ACTIVE
   gPulseCtrl.pulseAt = now

5. t=3000ms: Pulso LIGAR finalizado
   Retirar bit (volta a 1)
   
   tryStartPendingPulse():
   gPulseCtrl.step = PULSE_IDLE
   gPulseCtrl.pendingType = PULSE_DESLIGA
   
   → startPulseNow(PULSE_DESLIGA, ...)
   → gPulseCtrl.step = PULSE_VALIDATING (novamente)
   
   LOG: "Executando pulso pendente: DESLIGAR (APP)"

6. t=5000ms: Pulso DESLIGAR finalizado
   
   gPulseCtrl.step = PULSE_IDLE
   gPulseCtrl.pendingType = PULSE_NONE
   
   tryStartPendingPulse():
   Nenhum pendente → Retornar

7. ✓ Ambos os pulsos executados sequencialmente
   Motor: DESLIGADO → LIGADO → DESLIGADO
   (Em ~5 segundos no total)
```

---

## 📋 Cenário 7: Falha ao Salvar NVS (Mas Pulso Enviado)

**Situação:**
- Pulso APP foi enviado com sucesso
- NVS falha ao salvar (memória flash corrompida ou cheia)

**Sequência:**

```
╔════════════════════════════════════════════════════╗
║ BOOT #7: Falha NVS - Pulso Enviado mas Não Salvo  ║
╚════════════════════════════════════════════════════╝

1. processPulseController() - PULSE_VALIDATING
   
   Verificações OK:
   - I2C barramento OK
   - Modo validado (MANUAL APP)
   
2. Enviar pulso ao PCF8574
   output_PLC &= ~(1 << 0)
   if (writeOutputPLC()) {
       ✓ Bit enviado com sucesso
       gPulseCtrl.step = PULSE_ACTIVE
   }

3. Tentar salvar em NVS (Pulso APP)
   
   if (salvarComandoApp) {
       if (!preferences.begin("my-app", false)) {
           ✗ ERRO: "Falha ao abrir NVS"
           ESP_LOGE(TAG_NVS, "Falha ao abrir NVS...")
           
           Pulso JÁ FOI ENVIADO
           Mas não será salvo
       }
   }
   
   OU se abrir mas falhar a escrita:
   
   size_t bytesWritten = preferences.putBool(...);
   if (bytesWritten == 0) {
       ✗ ERRO: "Falha ao salvar comando do motor"
       ESP_LOGE(TAG_NVS, "Falha ao salvar...")
       
       ultimoComandoMotorOff NÃO é atualizado
   }

4. RESULTADO:
   
   Pulso: ✓ Enviado (motor atuou)
   NVS: ✗ Não atualizado
   
   ultimoComandoMotorOff = false (ainda salvo antigo)

5. Próximo reinício:
   
   NVS carrega: ultimoComandoMotorOff = false (LIGADO)
   Físico: gInputs.motorStatus = false (LIGADO)
   
   ESTADOS IGUAIS → Nenhum pulso
   
   Motor continua LIGADO (correto por acaso)

6. Se tivesse sido DESLIGAR:
   
   Pulso: ✓ Motor desligou
   NVS: ✗ ultimoComandoMotorOff = false (ainda diz LIGADO)
   
   Próximo reinício:
   esperado: LIGADO (NVS antigo)
   físico: DESLIGADO (pulso funcionou mas NVS falhou)
   
   → Sistema envia PULSO LIGAR novamente
   Motor liga (inesperado)

7. ⚠️ Consequência:
   
   Comportamento imprevisível no restart
   LOG mostra erro de NVS
   Motor pode atuar diferente a cada restart
   
   AÇÃO: Verificar espaço livre em NVS / Flash corrompida
```

---

## 🎯 Resumo dos Cenários

| Cenário | Situação                             | Resultado                | Pulso               | Válvula |
|---------|--------------------------------------|--------------------------|---------------------|---------|
| 1       | Restart com estado OK                | Nenhuma ação             | Não                 | Fechada |
| 2       | Alimentação perdida, motor desligou  | Restauração automática   | Sim                 |
| 3       | Modo LOCAL                           | Sem interferência remota | Não                 |
| 4       | Modo AGENDA                          | Sincronizar com horário  | Sim (se necessário) |
| 5       | I2C Travado                          | Abort com log crítico    | Não                 |
| 6       | 2 comandos rápidos                   | Fila de 1 pulso          | Sim + Sim           |
| 7       | Falha NVS                            | Pulso enviado, não salvo | Sim                 |

Válvula proporcional fecha a cada restart pelo tempo ajustado!