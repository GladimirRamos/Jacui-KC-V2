#include <Arduino.h>

/*************************************************************
  Blynk library is licensed under MIT license
 *************************************************************
  Projeto refatorado:
  - FreeRTOS
  - BlynkEdgent no Core 1
  - I/O, RTC, Display, Modbus e Supervisor no Core 0
  - Logs via esp_log

  - Comentada a linha 50 em Setings.h para ignorar o warning do LED não configurado
  - Comentada a linha 125 em Indicator.h para ignorar o warning do LED não configurado

 *************************************************************/

#define BLYNK_TEMPLATE_ID      "TMPL2WSHP95Ku"
#define BLYNK_TEMPLATE_NAME    "Jacui KC V2"
#define BLYNK_FIRMWARE_VERSION "0.1.0"

#define USE_ESP32_DEV_MODULE
#define HEARTBEAT_PIN 23

#include <WiFi.h>
#include "time.h"
#include "BlynkEdgent.h"
#include "HardwareSerial.h"
#include "SPI.h"
#include "RTClib.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Wire.h"
#include <Preferences.h>
#include "ModbusClientRTU.h"
#include "soc/rtc_wdt.h"
#include "esp_log.h"
#include <stdarg.h>

// =====================================================
// TAGs esp_log
// =====================================================

static const char *TAG_MAIN       = "MAIN";
static const char *TAG_BLYNK      = "BLYNK";
static const char *TAG_IO         = "IO";
static const char *TAG_RTC        = "RTC";
static const char *TAG_DISPLAY    = "DISPLAY";
static const char *TAG_MODBUS     = "MODBUS";
static const char *TAG_WDT        = "WDT";
static const char *TAG_NVS        = "NVS";
static const char *TAG_I2C        = "I2C";
static const char *TAG_SUPERVISOR = "SUPERVISOR";

#define LOG_TASK_START(TAG) \
  ESP_LOGI(TAG, "Task iniciada | core=%d | task=%s", xPortGetCoreID(), pcTaskGetName(NULL))

#define LOG_HEAP(TAG) \
  ESP_LOGI(TAG, "Heap livre: %lu bytes", (unsigned long)ESP.getFreeHeap())

// =====================================================
// Hardware
// =====================================================

#define I2C_SCL 15
#define I2C_SDA 4

#define PCF_INPUT_ADDR   0x22
#define PCF_OUTPUT_ADDR  0x24
#define OLED_ADDR        0x3C

#define WDT_TIMEOUT      30000UL // 2 minutos = 120.000 ms (UL)

Adafruit_SSD1306 display(128, 64, &Wire, -1);
RTC_DS1307 RTC;
Preferences preferences;
//ModbusMaster node1
// Instanciação do cliente eModbus associado à Serial1
ModbusClientRTU mbClient(Serial1);

unsigned char output_PLC = 0b11111111;

// =====================================================
// NTP
// =====================================================

const char* ntpServer = "br.pool.ntp.org";
const long  gmtOffset_sec = -10800;
const int   daylightOffset_sec = 0;

// =====================================================
// Sensor interno temperatura ESP32
// =====================================================

#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

// =====================================================
// FreeRTOS
// =====================================================

TaskHandle_t taskBlynkHandle      = NULL;
TaskHandle_t taskIOHandle         = NULL;
TaskHandle_t taskRTCHandle        = NULL;
TaskHandle_t taskDisplayHandle    = NULL;
TaskHandle_t taskModbusHandle     = NULL;
TaskHandle_t taskSupervisorHandle = NULL;

// Tamanho de pilha para cada tarefa (em bytes)
#define STACK_BLYNK        8192
#define STACK_IO_CONTROL   3072
#define STACK_RTC          4096 //2048 - CRASH RESET - 
//Guru Meditation Error: Core  0 panic'ed (Unhandled debug exception). 
//Debug exception reason: Stack canary watchpoint triggered (TaskRTC) 
#define STACK_DISPLAY      3584
#define STACK_MODBUS       4096
#define STACK_SUPERVISOR   2048

SemaphoreHandle_t mtxI2C  = NULL;
SemaphoreHandle_t mtxData = NULL;

typedef struct {
  char text[128];
} LogMessage;

QueueHandle_t qLog = NULL;

// =====================================================
// Estruturas de dados compartilhados
// =====================================================

struct SystemTimeData {
  int sec;
  int min;
  int hour;
  int day;
  int month;
  int year;
  int wdayBlynk;       // 1 segunda ... 7 domingo
  uint32_t secDay;
  char rtcText[64];
};

struct InputData {
  uint8_t raw;
  bool motorOff;       // true = motor desligado
  bool modoLocal;      // true = modo local
};

struct ScheduleData {
  uint32_t horaLigaSec;
  uint32_t horaDesligaSec;
  int remotoOuAgenda;  // 0 manual APP, 1 agenda
  char diasSemana[32]; // Exemplo Blynk: "1,2,3"
};

struct CommandData {
  bool forcaLiga;
  bool forcaDesliga;
  bool requestSetRTC;
  bool requestRestart;
  int rele5;
};

struct ModbusData {
  double vR;
  double vS;
  double vT;

  double iR;
  double iS;
  double iT;

  double pR;
  double pS;
  double pT;

  uint16_t status;
};

struct RuntimeData {
  long rssi;
  int temp;
  uint32_t counterRST;
  uint32_t blynkState;
  char blynkStateText[24];
  char modoText[16];
  bool sendResetLog;
  int valorAnalogico;
  bool rtcError;  // Sinaliza erro crítico do RTC
};

SystemTimeData gTime;
InputData      gInputs;
ScheduleData   gSchedule;
CommandData    gCmd;
ModbusData     gModbus;
RuntimeData    gRun;

bool oldMotorOff = true;

int cicloON  = 0;
int cicloOFF = 0;

uint32_t lastAliveBlynk   = 0;
uint32_t lastAliveIO      = 0;
uint32_t lastAliveRTC     = 0;
uint32_t lastAliveDisplay = 0;
uint32_t lastAliveModbus  = 0;

// Variáveis globais para armazenar o tempo gasto (em microssegundos)
volatile uint32_t tempoTaskBlynk   = 0;
volatile uint32_t tempoTaskIO      = 0;
volatile uint32_t tempoTaskRTC     = 0;
volatile uint32_t tempoTaskDisplay = 0;
volatile uint32_t tempoTaskModbus  = 0;

// =====================================================
// Protótipos
// =====================================================

void TaskBlynk(void *pv);
void TaskIOControl(void *pv);
void TaskRTC(void *pv);
void TaskDisplay(void *pv);
void TaskModbus(void *pv);
void TaskSupervisor(void *pv);

void initRtcWdt();
void failMSG(String HW_status);

void writeOutputPLC();
void pulseLiga();
void pulseDesliga();

void queueLogf(const char *fmt, ...);
bool setRTCFromNTP();

void loadCounterAndMotorState(bool &memMotorState);
void loadSettingsFromNVS();
void saveScheduleToNVS();
void saveModoToNVS();
void restoreMotorState(bool memMotorState);

const char *resetReasonName(esp_reset_reason_t r);
void updateBlynkStateText(uint32_t state, char *buffer, size_t len);

// =====================================================
// Blynk callbacks
// =====================================================

// Enumeração para identificar os botões controlados
enum TipoBotao { BTN_MANUAL, BTN_AGENDA, BTN_RESET, BTN_LIGA, BTN_DESLIGA, BTN_COUNT };

// Estrutura para gerenciar o temporizador de cada botão
struct ControleBotao {
    uint8_t virtualPin;
    unsigned long tempoInicio;
    bool aguardando;
    int valorAtual;
};

// Inicializa a matriz com os pinos virtuais correspondentes
ControleBotao botoes[BTN_COUNT] = {
    {27, 0, false, 0}, // BTN_MANUAL
    {28, 0, false, 0}, // BTN_AGENDA
    {39, 0, false, 0}, // BTN_RESET
    {41, 0, false, 0}, // BTN_LIGA
    {42, 0, false, 0}  // BTN_DESLIGA
};

// Função auxiliar para iniciar a contagem do botão
void gerenciarCallbackBotao(TipoBotao tipo, int valor) {
    botoes[tipo].valorAtual = valor;
    if (valor == 1) {
        botoes[tipo].tempoInicio = millis();
        botoes[tipo].aguardando = true;
        //queueLogf("Botao V%d ativo. Aguardando 5 segundos para confirmar...", botoes[tipo].virtualPin);
    } else {
        if (botoes[tipo].aguardando) {
            //queueLogf("Comando V%d cancelado (botao solto antes dos 5s)", botoes[tipo].virtualPin);
        }
        botoes[tipo].aguardando = false;
    }
}

//int BotaoRESET = 0; // Mantido caso use em outro local do escopo

BLYNK_WRITE(V27) { gerenciarCallbackBotao(BTN_MANUAL, param.asInt()); }
BLYNK_WRITE(V28) { gerenciarCallbackBotao(BTN_AGENDA, param.asInt()); }
BLYNK_WRITE(V39) { 
    //BotaoRESET = param.asInt();
    gerenciarCallbackBotao(BTN_RESET, param.asInt()); 
}
BLYNK_WRITE(V41) { gerenciarCallbackBotao(BTN_LIGA, param.asInt()); }
BLYNK_WRITE(V42) { gerenciarCallbackBotao(BTN_DESLIGA, param.asInt()); }


BLYNK_WRITE(V40)
{
  uint32_t ligaSec    = param[0].asInt();
  uint32_t desligaSec = param[1].asInt();
  String dias         = param[3].asStr();

  if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(100)) == pdTRUE) {
    gSchedule.horaLigaSec    = ligaSec;
    gSchedule.horaDesligaSec = desligaSec;

    strncpy(gSchedule.diasSemana, dias.c_str(), sizeof(gSchedule.diasSemana) - 1);
    gSchedule.diasSemana[sizeof(gSchedule.diasSemana) - 1] = '\0';

    xSemaphoreGive(mtxData);
  }

  saveScheduleToNVS();
  // completa "Agenda APP recebida: liga=%lu desliga=%lu dias=%s",
  queueLogf("Agenda recebida do APP",
            (unsigned long)ligaSec,
            (unsigned long)desligaSec,
            dias.c_str());
}

BLYNK_WRITE(V55)
{
  int rele = param.asInt();

  if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
    gCmd.rele5 = rele;
    xSemaphoreGive(mtxData);
  }

  queueLogf("Comando Rele 5: %s", rele == 1 ? "Ligado" : "Desligado");
}

// =====================================================
// Função de estatisticas DIAGNÓSTICO DE SAÚDE 
// =====================================================

void imprimirDiagnosticoSistema() {
    Serial.println(F("\n==================================================="));
    Serial.println(F("         DIAGNÓSTICO DE SAÚDE DO ESP32       "));
    Serial.println(F("==================================================="));

    // 1. Tempo de atividade (Uptime)
    uint64_t uptime_us = esp_timer_get_time();
    uint32_t uptime_s = (uint32_t)(uptime_us / 1000000ULL);
    Serial.printf("Tempo ligado (Uptime): %u segundos (%u minutos)\n", uptime_s, uptime_s / 60);

    // 2. Memória RAM (Heap) - Crítico para sistemas com WiFi/Blynk
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_free_heap = esp_get_minimum_free_heap_size(); // Menor nível de RAM que o chip já atingiu
    Serial.printf("Memória RAM Livre Atual: %u bytes\n", free_heap);
    Serial.printf("Menor RAM Livre Histórica: %u bytes\n", min_free_heap);

    if (min_free_heap < 10000) {
        Serial.println(F("[ALERTA]: Memória RAM perigosamente baixa! Risco de crash."));
    }

    // 3. Frequência do Processador
    Serial.printf("Frequência da CPU: %u MHz\n", getCpuFrequencyMhz());

    // 4. Detalhes de Chip e Modelo
    Serial.printf("Revisão do Chip ESP32: %d\n", ESP.getChipRevision());
    Serial.println(F("---------------------------------------------------"));

}
// =====================================================
// Setup
// =====================================================

void vTaskDisplayInit(void *pvParameters) {
  
  vTaskDelay(pdMS_TO_TICKS(100)); //tempo para o setup() configurar os perifericos
  int           tempoStart = 60;
  uint32_t ultimoDisplayMs = 0;
  uint32_t ultimoCoracaoMs = 0;
  bool        exibeCoracao = false; // Controla se o coração aparece ou não

  while (tempoStart > 0) {
    // Bloqueia a tarefa por 500ms, liberando a CPU
    vTaskDelay(pdMS_TO_TICKS(500)); 

    // Gera pulso para Hardware Watchdog externo (10ms)
    digitalWrite(HEARTBEAT_PIN, HIGH);
    delayMicroseconds(10000);  // 10ms de pulso
    digitalWrite(HEARTBEAT_PIN, LOW);

    // Atualiza o estado do pisca (ocorre a cada 500ms garantidos)
    exibeCoracao = !exibeCoracao;

    // A cada duas iterações de 500ms, decrementa 1 segundo do tempo de início
    static bool alternaSegundo = false;
    alternaSegundo = !alternaSegundo;

    if (alternaSegundo) {
        ESP_LOGI(TAG_MAIN, "Inicio em %d segundos", tempoStart);
        rtc_wdt_feed();
        ESP_LOGD(TAG_WDT, "Watchdog alimentado");
        tempoStart--;
    }

    if (xSemaphoreTake(mtxI2C, portMAX_DELAY) == pdTRUE) {
            display.clearDisplay();
            
            // Cabeçalho da Empresa
            display.setTextSize(2);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(45, 10);
            display.println("R&M");
            display.setTextSize(1);
            display.setCursor(42, 30);
            display.println("Company");
            
            // Informações de Diagnóstico
            display.setCursor(5, 55);
            display.print("RST:");
            display.println(gRun.counterRST);
            display.setCursor(70, 55);
            display.print("FW:");
            display.println(BLYNK_FIRMWARE_VERSION);

            // Contador de Inicialização
            display.setTextSize(2);
            display.setCursor(96, 24);
            display.print(tempoStart);

            if (exibeCoracao) {
            // Batida alta: Desenha o coração cheio na posição original
            display.setCursor(96, 3);
            display.write(3); 
            } else {
            // Batida baixa: Você pode deixar vazio ou desenhar um caractere menor 
            display.setCursor(96, 3);
            display.print(" ");
            }
            
            display.display();
            xSemaphoreGive(mtxI2C);
        }
  }
    //  Ao finalizar os 60 segundos, deleta a task para liberar memória
    //  removeer o comentário abaixo se quiser que a task seja deletada 
    //  mas ele precisa ser criada no setup() com xTaskCreate() e não chamado diretamente como está sendo feito
    //vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  esp_log_level_set("*", ESP_LOG_INFO);
  ESP_LOGI(TAG_MAIN, "Inicializando firmware %s", BLYNK_FIRMWARE_VERSION);
  ESP_LOGI(TAG_MAIN, "Setup rodando no core %d", xPortGetCoreID());

  initRtcWdt();

  // Criação dos recursos do FreeRTOS
  mtxI2C  = xSemaphoreCreateMutex();
  mtxData = xSemaphoreCreateMutex();
  qLog    = xQueueCreate(20, sizeof(LogMessage));

  if (!mtxI2C || !mtxData || !qLog) {
    ESP_LOGE(TAG_MAIN, "Falha ao criar mutex ou fila");
    while (1) { delay(1000); }
  }

  // Inicialização das estruturas de dados
  memset(&gTime, 0, sizeof(gTime));
  memset(&gInputs, 0, sizeof(gInputs));
  memset(&gSchedule, 0, sizeof(gSchedule));
  memset(&gCmd, 0, sizeof(gCmd));
  memset(&gModbus, 0, sizeof(gModbus));
  memset(&gRun, 0, sizeof(gRun));

  strncpy(gRun.blynkStateText, "START", sizeof(gRun.blynkStateText) - 1);
  strncpy(gRun.modoText, "START", sizeof(gRun.modoText) - 1);

  // Inicializa Barramento I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  ESP_LOGI(TAG_I2C, "I2C iniciado SDA=%d SCL=%d", I2C_SDA, I2C_SCL);

  // Inicializa Display OLED
  if (xSemaphoreTake(mtxI2C, portMAX_DELAY) == pdTRUE) {
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.clearDisplay();
    display.display();
    xSemaphoreGive(mtxI2C);
  }

  // Teste PCF8574 saída
  if (xSemaphoreTake(mtxI2C, portMAX_DELAY) == pdTRUE) {
    Wire.beginTransmission(PCF_OUTPUT_ADDR);
    Wire.write(output_PLC);
    int errorCode_OUTPUT = Wire.endTransmission();
    xSemaphoreGive(mtxI2C);

    if (errorCode_OUTPUT != 0) {
      ESP_LOGE(TAG_I2C, "Falha no PCF8574 de saida. Endereco=0x%02X erro=%d", PCF_OUTPUT_ADDR, errorCode_OUTPUT);
      failMSG("FALHA OUTPUT");
    } else {
      ESP_LOGI(TAG_I2C, "PCF8574 saida OK. Endereco=0x%02X", PCF_OUTPUT_ADDR);
    }
  }
  delay(50);

  // Teste PCF8574 entrada
  if (xSemaphoreTake(mtxI2C, portMAX_DELAY) == pdTRUE) {
    Wire.beginTransmission(PCF_INPUT_ADDR);
    int errorCode_INPUT = Wire.endTransmission();
    xSemaphoreGive(mtxI2C);

    if (errorCode_INPUT != 0) {
      ESP_LOGE(TAG_I2C, "Falha no PCF8574 de entrada. Endereco=0x%02X erro=%d", PCF_INPUT_ADDR, errorCode_INPUT);
      failMSG("FALHA INPUT");
    } else {
      ESP_LOGI(TAG_I2C, "PCF8574 entrada OK. Endereco=0x%02X", PCF_INPUT_ADDR);
    }
  }
  delay(50);

  bool memMotorState = true;
  loadCounterAndMotorState(memMotorState);
  loadSettingsFromNVS();

  ESP_LOGI(TAG_NVS, "Quantidade de RESETs: %lu", (unsigned long)gRun.counterRST);
  esp_reset_reason_t reason = esp_reset_reason();
  ESP_LOGI(TAG_WDT, "Reset reason: %d - %s", reason, resetReasonName(reason));

  // =====================================================
  // Inicializa Hardware Watchdog (Heartbeat Pin)
  // =====================================================
  pinMode(HEARTBEAT_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_PIN, LOW);
  
  // Gera 3 pulsos de confirmação (boot handshake)
  ESP_LOGI(TAG_WDT, "Iniciando Hardware Watchdog - Gerando pulsos de boot");
  for (int i = 0; i < 3; i++) {
    digitalWrite(HEARTBEAT_PIN, HIGH);
    delayMicroseconds(10000);  // 10ms de pulso
    digitalWrite(HEARTBEAT_PIN, LOW);
    delay(200);
  }
  ESP_LOGI(TAG_WDT, "Hardware Watchdog iniciado - Pulsos de boot OK");

  // --- NOVA LÓGICA DE TEMPORIZAÇÃO ASSÍNCRONA DE INICIALIZAÇÃO (60s) ---
  ESP_LOGI(TAG_MAIN, "Temporizando inicio do sistema");

  // Executa o display init de forma síncrona (bloqueando pelo tempoStart)
  vTaskDisplayInit(NULL);

  restoreMotorState(memMotorState);

  // Inicializa Serial Modbus/Hardware Secundário
  Serial1.begin(9600, SERIAL_8N1, 14, 27);

// Inicializa RTC Externo
  // Define um timeout de 100 milissegundos em vez de esperar para sempre
  if (xSemaphoreTake(mtxI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
    bool rtcOk = RTC.begin();
    xSemaphoreGive(mtxI2C); // Libera o semáforo imediatamente após o uso
    
    if (!rtcOk) {
      ESP_LOGE(TAG_RTC, "Nao foi possivel encontrar o RTC DS1307");
      failMSG("FALHA RTC");
    } else {
      ESP_LOGI(TAG_RTC, "RTC DS1307 identificado com sucesso");
    }
  } 
  else {
    // TRATAMENTO DE ERRO: O que acontece se o display prender o I2C por mais de 500ms
    ESP_LOGE(TAG_RTC, "Timeout: Nao foi possivel obter o semaforo mtxI2C para inicializar o RTC");
    failMSG("TIMEOUT I2C");
  }

  // Inicializa Sincronismo NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  ESP_LOGI(TAG_RTC, "NTP configurado. Servidor=%s GMT offset=%ld daylight=%d", ntpServer, gmtOffset_sec, daylightOffset_sec);

  // =====================================================
  // Criação e Inicialização das Tarefas do FreeRTOS
  // =====================================================
  
  // TaskBlynk: Roda no Core 1 (Core padrão do ecossistema Arduino/Wi-Fi)
  xTaskCreatePinnedToCore(TaskBlynk, "TaskBlynk", STACK_BLYNK, NULL, 3, &taskBlynkHandle, 1);
  
  // Tasks de Hardware/Controle: Rodam no Core 0 para não sofrerem interferência do Wi-Fi
  xTaskCreatePinnedToCore(TaskIOControl, "TaskIOControl", STACK_IO_CONTROL, NULL, 4, &taskIOHandle, 0);
  xTaskCreatePinnedToCore(TaskRTC, "TaskRTC", STACK_RTC, NULL, 3, &taskRTCHandle, 0);
  xTaskCreatePinnedToCore(TaskDisplay, "TaskDisplay", STACK_DISPLAY, NULL, 1, &taskDisplayHandle, 0);
  xTaskCreatePinnedToCore(TaskModbus, "TaskModbus", STACK_MODBUS, NULL, 2, &taskModbusHandle, 0);
  xTaskCreatePinnedToCore(TaskSupervisor, "TaskSupervisor", STACK_SUPERVISOR, NULL, 5, &taskSupervisorHandle, 0);

  ESP_LOGI(TAG_MAIN, "----------------------- SETUP OK ---------------------------");
  LOG_HEAP(TAG_MAIN);
}

void loop()
{
  // Tarefa que mostra um diagnóstico de saude do ESP32 a cada 10 segundos, mas está comentada para não poluir o log
  static uint32_t temporizadorDiagnostico = 0;
  if (millis() - temporizadorDiagnostico > 60000) { temporizadorDiagnostico = millis(); imprimirDiagnosticoSistema(); }
  vTaskDelay(pdMS_TO_TICKS(1)); 
  
  //vTaskDelay(portMAX_DELAY); //coloca a tarefa do loop() em estado de bloqueio permanente 
}

// =====================================================
// Task Blynk - Core 1
// =====================================================

void TaskBlynk(void *pv)
{
  LOG_TASK_START(TAG_BLYNK);

  BlynkEdgent.begin();

  uint32_t lastSendFast = 0;
  uint32_t lastSendSlow = 0;

  LogMessage logMsg;

  for (;;) {
    BlynkEdgent.run();
    lastAliveBlynk = millis();

    uint32_t now = millis();

// --- VERIFICAÇÃO DOS BOTÕES POR 5 SEGUNDOS ---
        for (int i = 0; i < BTN_COUNT; i++) {
            if (botoes[i].aguardando && (now - botoes[i].tempoInicio >= 5000)) {
                botoes[i].aguardando = false; // Reseta o estado para não repetir
                
                // Tenta pegar o Mutex para salvar as alterações com segurança
                if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
                    switch (i) {
                        case BTN_MANUAL:
                            gSchedule.remotoOuAgenda = 0;
                            cicloON = 0; cicloOFF = 0;
                            xSemaphoreGive(mtxData);
                            saveModoToNVS();
                            queueLogf("Modo MANUAL APP");
                            break;

                        case BTN_AGENDA:
                            gSchedule.remotoOuAgenda = 1;
                            cicloON = 0; cicloOFF = 0;
                            xSemaphoreGive(mtxData);
                            saveModoToNVS();
                            queueLogf("Modo AGENDA");
                            break;

                        case BTN_RESET:
                            gCmd.requestSetRTC = true;
                            
                            // 1. Zera o contador na estrutura global de execução
                            //gRun.counterRST = 0;
                            
                            // 2. Grava o valor zero diretamente na NVS para persistir no boot
                            preferences.begin("my-app", false); // Ajuste o nome do namespace se for diferente de "my-app"
                            preferences.putUInt("counterRST", -1); // -1 porque já soma 1 ao reiniciar, então vai ficar 0
                            preferences.end();
                            
                            // 3. Solicita o restart do sistema (WDT vai atuar na TaskRTC)
                            gCmd.requestRestart = true; 
                            
                            xSemaphoreGive(mtxData);
                            //queueLogf("Reiniciando e zerando RST...");
                            break;

                        case BTN_LIGA:
                            gCmd.forcaLiga = true;
                            xSemaphoreGive(mtxData);
                            //queueLogf("Comando LIGAR");
                            break;

                        case BTN_DESLIGA:
                            gCmd.forcaDesliga = true;
                            xSemaphoreGive(mtxData);
                            //queueLogf("Comando DESLIGAR");
                            break;
                    }
                } else {
                    ESP_LOGW(TAG_BLYNK, "Timeout ao tentar aplicar comando do botao V%d", botoes[i].virtualPin);
                }
            }
        }

    // Envia logs da fila para o Blynk adicionando Data e Hora
    while (xQueueReceive(qLog, &logMsg, 0) == pdTRUE) {
            char logComTimestamp[150]; // Buffer para armazenar o log final com o tempo
            
            // Captura o tempo atual do sistema (usando a struct tm padrão do ESP32)
            struct tm timeinfo;
            if (getLocalTime(&timeinfo)) {
                // Formata a data e hora idêntico ao resetMsg (ex: [24/06/2026 17:55:01] )
                strftime(logComTimestamp, sizeof(logComTimestamp), "%d.%m.%Y - %H:%M:%S ", &timeinfo);
            } else {
                // Caso o RTC falhe por algum motivo, põe um indicador genérico
                strcpy(logComTimestamp, "00.00.0000 - 00:00:00 ");
            }
            
            // Concatena o texto original do log que veio da fila
            strncat(logComTimestamp, logMsg.text, sizeof(logComTimestamp) - strlen(logComTimestamp) - 1);
            
            // Envia para o pino virtual do Terminal/Log no Blynk
            Blynk.virtualWrite(V45, logComTimestamp);
        }

    // Envio rápido a cada 5 segundos
    if (now - lastSendFast >= 1000) {
      lastSendFast = now;

      SystemTimeData timeCopy;
      InputData inputCopy;
      RuntimeData runCopy;
      ScheduleData scheduleCopy;

      if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
         timeCopy      = gTime;
         inputCopy     = gInputs;
         runCopy       = gRun;       // runCopy já vai trazer o valorAnalogico atualizado
        scheduleCopy = gSchedule;
        xSemaphoreGive(mtxData);
        }

      if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
        timeCopy     = gTime;
        inputCopy    = gInputs;
        runCopy      = gRun;
        scheduleCopy = gSchedule;
        xSemaphoreGive(mtxData);
      } else {
        ESP_LOGW(TAG_BLYNK, "Timeout ao copiar dados compartilhados");
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      uint32_t state = BlynkState::get();
      char stateText[24];
      updateBlynkStateText(state, stateText, sizeof(stateText));

      Blynk.virtualWrite(V46, timeCopy.rtcText);
      Blynk.virtualWrite(V54, runCopy.rssi);
      Blynk.virtualWrite(V52, runCopy.temp);
      Blynk.virtualWrite(V53, runCopy.counterRST);

      Blynk.virtualWrite(V44, inputCopy.motorOff ? 0 : 1);
      Blynk.virtualWrite(V43, inputCopy.motorOff ? "MOTOR DESLIGADO" : "MOTOR LIGADO");

      Blynk.virtualWrite(V48, inputCopy.modoLocal ? 1 : 0);
      Blynk.virtualWrite(V56, runCopy.valorAnalogico); // <- Publica no pino virtual V56 analogico escalonado de 0 a 100

      if (inputCopy.modoLocal) {
        Blynk.virtualWrite(V49, 0);
      } else {
        Blynk.virtualWrite(V49, scheduleCopy.remotoOuAgenda == 0 ? 1 : 2);
      }

      ESP_LOGD(TAG_BLYNK,
               "Dados enviados. Estado=%lu %s RSSI=%ld Temp=%d",
               (unsigned long)state,
               stateText,
               runCopy.rssi,
               runCopy.temp);

      // Log do reset uma única vez quando estiver RUNNING
      if (runCopy.sendResetLog && state == 4) {
        char resetMsg[128];

        snprintf(resetMsg,
                 sizeof(resetMsg),
                 "%s %s RST=%lu",
                 timeCopy.rtcText,
                 resetReasonName(esp_reset_reason()),
                 (unsigned long)runCopy.counterRST);

        Blynk.virtualWrite(V45, resetMsg);
        ESP_LOGI(TAG_BLYNK, "Log reset enviado: %s", resetMsg);

        if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
          gRun.sendResetLog = false;
          xSemaphoreGive(mtxData);
        }
      }
    }

    // Envio Modbus a cada 10 segundos
    if (now - lastSendSlow >= 10000) {
      lastSendSlow = now;

      ModbusData mb;

      if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
        mb = gModbus;
        xSemaphoreGive(mtxData);
      } else {
        ESP_LOGW(TAG_BLYNK, "Timeout ao copiar dados Modbus");
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      Blynk.virtualWrite(V30, mb.vR);
      Blynk.virtualWrite(V31, mb.vS);
      Blynk.virtualWrite(V32, mb.vT);

      Blynk.virtualWrite(V33, mb.iR);
      Blynk.virtualWrite(V34, mb.iS);
      Blynk.virtualWrite(V35, mb.iT);

      Blynk.virtualWrite(V36, mb.pR);
      Blynk.virtualWrite(V37, mb.pS);
      Blynk.virtualWrite(V38, mb.pT);

      ESP_LOGD(TAG_BLYNK, "Dados Modbus enviados ao Blynk");
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =====================================================
// Task RTC - Core 0
// =====================================================

void TaskRTC(void *pv)
{
  LOG_TASK_START(TAG_RTC);
  
  // Mantemos a flag aqui, mas controlamos estritamente suas mudanças
  bool setRTCToday = false; 
  bool logBateriaEnviado = false; // Nova flag para evitar travar o ESP com logs infinitos

  for (;;) {
    lastAliveRTC = millis();

    DateTime now;

    if (xSemaphoreTake(mtxI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
      now = RTC.now();
      xSemaphoreGive(mtxI2C);
      // Limpa flag de erro se conseguiu ler o RTC
      if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
        gRun.rtcError = false;
        xSemaphoreGive(mtxData);
      }
    } else {
      ESP_LOGW(TAG_RTC, "Timeout ao acessar I2C para leitura do RTC");
      // Sinaliza erro
      if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
        gRun.rtcError = true;
        xSemaphoreGive(mtxData);
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    SystemTimeData t;

    t.sec   = now.second();
    t.min   = now.minute();
    t.hour  = now.hour();
    t.day   = now.day();
    t.month = now.month();
    t.year  = now.year();

    int rtcWday = now.dayOfTheWeek(); 
    t.wdayBlynk = rtcWday == 0 ? 7 : rtcWday;

    t.secDay = (t.hour * 3600UL) + (t.min * 60UL) + t.sec;

    snprintf(t.rtcText,
             sizeof(t.rtcText),
             "%02d.%02d.%04d - %02d:%02d:%02d",
             t.day,
             t.month,
             t.year,
             t.hour,
             t.min,
             t.sec);

    if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
      gTime = t;
      xSemaphoreGive(mtxData);
    }

    ESP_LOGD(TAG_RTC, "RTC: %s Wday=%d SecDay=%lu",
             t.rtcText,
             t.wdayBlynk,
             (unsigned long)t.secDay);

    // Evita que o log de bateria fraca rode em loop infinito a cada 1 segundo
    // se ano menor e calibração NTP falhar, apenas loga uma vez e espera o ano voltar ao normal para resetar a flag
    if (t.year < 2026 && setRTCFromNTP()) {
      setRTCFromNTP();
      if (!logBateriaEnviado) {
        queueLogf("Relógio calibrado, ver bateria!");
        logBateriaEnviado = true;
      }
    } else {
      logBateriaEnviado = false; // Reseta se o ano voltar ao normal
    }

     // Executa a calibração automática apenas se a flag for falsa
    if (t.hour == 18 && t.min == 0 && !setRTCToday) {
      // Só marca como feito se a função retornar 'true' (sucesso)
      if (setRTCFromNTP()) {
        queueLogf("Relógio calibrado automaticamente");
        setRTCToday = true;  
        } else {
          // vai ficar tentando a cada segundo até , mas não vai travar o ESP por 1 minuto, apenas loga a falha 
          ESP_LOGE(TAG_RTC, "Tentativa de calibração falhou. Tentando novamente no próximo segundo...");
        }
    }

    // Só reseta a flag se ela estiver marcada como true. Evita processamento inútil no minuto 5:01.
    if (t.hour == 18 && t.min == 1 && setRTCToday) {
      setRTCToday = false; 
    }

    bool doSetRTC  = false;
    bool doRestart = false;

    if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
      doSetRTC  = gCmd.requestSetRTC;
      doRestart = gCmd.requestRestart;

      gCmd.requestSetRTC  = false;
      gCmd.requestRestart = false;

      xSemaphoreGive(mtxData);
    }

    if (doSetRTC) {
      setRTCFromNTP();
      queueLogf("Relógio calibrado por comando APP");
    }

    if (doRestart) {
      queueLogf("Reiniciando por comando APP");
      ESP_LOGW(TAG_RTC, "Reiniciando por comando APP");
      while (true)
      {
        ESP_LOGI(TAG_WDT, "RTC Watchdog vai atuar para reiniciar o sistema...");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Evita que o Core do ESP trave sem alimentar o IDLE do RTOS
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =====================================================
// Task IO - Core 0
// =====================================================

void TaskIOControl(void *pv)
{
  LOG_TASK_START(TAG_IO);

  uint32_t lastRead = 0;

  for (;;) {
    lastAliveIO = millis();

    uint32_t nowMs = millis();

    if (nowMs - lastRead >= 200) {
      lastRead = nowMs;

      uint8_t inputPCF = 0xFF;
      bool readOk = false;

      if (xSemaphoreTake(mtxI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
        Wire.requestFrom(PCF_INPUT_ADDR, 1);

        if (Wire.available()) {
          inputPCF = Wire.read();
          readOk = true;
        }

        xSemaphoreGive(mtxI2C);
      } else {
        ESP_LOGW(TAG_IO, "Timeout ao acessar I2C para leitura PCF entrada");
      }

      if (readOk) {
        InputData in;

        in.raw       = inputPCF;
        in.motorOff  = inputPCF & (1 << 0); // IN1
        in.modoLocal = inputPCF & (1 << 1); // IN2

        // Leitura analógica do GPIO36 (Faixa nativa de 0 a 4095)
        int leituraRaw = analogRead(36);
  
        // Mapeia o valor de 0-4095 para a escala de 0-100
        int valorEscalonado = map(leituraRaw, 0, 4095, 0, 100);

        if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
           gInputs = in;
           gRun.valorAnalogico = valorEscalonado; // <- Salva o valor escalonado com segurança
           xSemaphoreGive(mtxData);
           }

        if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
          gInputs = in;

          if (in.modoLocal) {
            strncpy(gRun.modoText, "LOCAL", sizeof(gRun.modoText) - 1);
          } else {
            if (gSchedule.remotoOuAgenda == 0) {
              strncpy(gRun.modoText, "MANUAL", sizeof(gRun.modoText) - 1);
            } else {
              strncpy(gRun.modoText, "AGENDA", sizeof(gRun.modoText) - 1);
            }
          }

          gRun.modoText[sizeof(gRun.modoText) - 1] = '\0';

          xSemaphoreGive(mtxData);
        }

        ESP_LOGD(TAG_IO,
                 "Entradas PCF=0x%02X Motor=%s Modo=%s",
                 inputPCF,
                 in.motorOff ? "DESLIGADO" : "LIGADO",
                 in.modoLocal ? "LOCAL" : "APP");

        // Grava estado do motor somente quando muda
        if (in.motorOff != oldMotorOff) {
          preferences.begin("my-app", false);
          preferences.putBool("MemMotorState", in.motorOff);
          preferences.end();

          oldMotorOff = in.motorOff;

          ESP_LOGI(TAG_NVS,
                   "Estado motor gravado na NVS: %s",
                   in.motorOff ? "DESLIGADO" : "LIGADO");
        }
      } else {
        ESP_LOGW(TAG_IO, "Falha ao ler PCF8574 de entrada");
      }
    }

    CommandData cmd;
    ScheduleData sch;
    SystemTimeData timeCopy;
    InputData inputCopy;
    ModbusData mbCopy; // Adicionado para receber os dados do sensor

    if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
      cmd       = gCmd;
      sch       = gSchedule;
      timeCopy  = gTime;
      inputCopy = gInputs;
      mbCopy    = gModbus; // Copia os dados do Modbus de forma segura

      gCmd.forcaLiga    = false;
      gCmd.forcaDesliga = false;

      xSemaphoreGive(mtxData);
    } else {
      ESP_LOGW(TAG_IO, "Timeout ao copiar dados compartilhados");
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // --- CONTROLE DOS RELÉS POR CORRENTE (SUPERVISOR INTEGRADO) ---
    if (mbCopy.status == 0x00) { // Sensor OK
      double mediaCorrente = (mbCopy.iR + mbCopy.iS + mbCopy.iT) / 3.0;

      if (mediaCorrente > 20.0) {
        // Liga Relé 3 (Bit 2) e Relé 4 (Bit 3) em nível lógico BAIXO (0 = Ligado)
        output_PLC &= ~(1 << 2); 
        output_PLC &= ~(1 << 3); 
        ESP_LOGD(TAG_IO, "Supervisor: Corrente %.2fA > 20A. Relés 3 e 4 LIGADOS.", mediaCorrente);
      } else {
        // Desliga Relé 3 e Relé 4 colocando em nível lógico ALTO (1 = Desligado)
        output_PLC |= (1 << 2);
        output_PLC |= (1 << 3);
      }
    } else {
      // Caso o sensor caia ou dê erro, por segurança desliga os relés 3 e 4
      output_PLC |= (1 << 2);
      output_PLC |= (1 << 3);
    }

    // --- CONTROLE DO RELÉ 5 ---
    if (cmd.rele5 == 1) {
      output_PLC &= ~(1 << 4);
    } else {
      output_PLC |= (1 << 4);
    }

    // --- SINALIZAÇÃO DO ESTADO DO MOTOR ---
    if (inputCopy.motorOff) {
      output_PLC |= (1 << 5);
    } else {
      output_PLC &= ~(1 << 5);
    }

    // Envia todas as alterações de bits feitas acima para o PCF8574 físico
    writeOutputPLC();

    // Se está em modo local, não comanda via APP/agenda abaixo
    if (inputCopy.modoLocal) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Modo manual APP
    if (sch.remotoOuAgenda == 0) {
      if (cmd.forcaLiga) {
        pulseLiga();
        cicloOFF = 0;
        queueLogf("Ligar pelo APP");
        ESP_LOGI(TAG_IO, "Pulso LIGAR executado pelo APP");
      }

      if (cmd.forcaDesliga) {
        pulseDesliga();
        cicloON = 0;
        queueLogf("Desligar pelo APP");
        ESP_LOGI(TAG_IO, "Pulso DESLIGAR executado pelo APP");
      }
    }

    // Modo agenda
    if (sch.remotoOuAgenda == 1) {
      bool diaAtivo = false;

      char diaAtual[4];
      snprintf(diaAtual, sizeof(diaAtual), "%d", timeCopy.wdayBlynk);

      if (strstr(sch.diasSemana, diaAtual) != NULL) {
        diaAtivo = true;
      }

      bool dentroHorario = false;

      if (sch.horaLigaSec <= sch.horaDesligaSec) {
        dentroHorario =
          timeCopy.secDay >= sch.horaLigaSec &&
          timeCopy.secDay <= sch.horaDesligaSec;
      } else {
        // Agenda atravessando meia-noite
        dentroHorario =
          timeCopy.secDay >= sch.horaLigaSec ||
          timeCopy.secDay <= sch.horaDesligaSec;
      }

      ESP_LOGD(TAG_IO,
               "Agenda: diaAtivo=%d dentroHorario=%d secDay=%lu liga=%lu desliga=%lu dias=%s",
               diaAtivo,
               dentroHorario,
               (unsigned long)timeCopy.secDay,
               (unsigned long)sch.horaLigaSec,
               (unsigned long)sch.horaDesligaSec,
               sch.diasSemana);

      if (diaAtivo && dentroHorario) {
        // Dentro do horário: DESLIGA
        for (; cicloOFF < 1; cicloOFF++) {
          pulseDesliga();
          cicloON = 0;
          queueLogf("Desligar por agendamento");
          ESP_LOGI(TAG_IO, "Pulso DESLIGAR por agendamento");
        }
      } else {
        // Fora do horário: LIGA
        for (; cicloON < 1; cicloON++) {
          pulseLiga();
          cicloOFF = 0;
          queueLogf("Ligar por agendamento");
          ESP_LOGI(TAG_IO, "Pulso LIGAR por agendamento");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =====================================================
// Task Display - Core 0
// =====================================================

void TaskDisplay(void *pv)
{
  LOG_TASK_START(TAG_DISPLAY);

  // Variável estática para guardar o segundo da última atualização válida
  static int ultimoSegundo = -1;
  // Tornar a variável static para manter o estado entre os loops
  static bool heartBeat = false;
  // Variável para controlar o tempo da última batida (em milissegundos)
  static uint32_t ultimoHeartbeatMs = 0;

  for (;;) {
    lastAliveDisplay = millis();

    SystemTimeData t;
    RuntimeData r;

    if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
      t = gTime;
      r = gRun;
      xSemaphoreGive(mtxData);
    } else {
      ESP_LOGW(TAG_DISPLAY, "Timeout ao copiar dados compartilhados");
      vTaskDelay(pdMS_TO_TICKS(500)); // Reduzido para tentar novamente mais rápido em caso de falha
      continue;
    }
    
    // Variáveis de controle para decidir se precisamos atualizar o display
    bool segundoMudou = (t.sec != ultimoSegundo);
    bool meioSegundoPassou = (millis() - ultimoHeartbeatMs >= 500);

    // Condição: Atualiza se o segundo mudou OU se já passou 500ms desde o último pulso do coração
    if (segundoMudou || meioSegundoPassou) {

      if (xSemaphoreTake(mtxI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);

        // Hora
        display.setTextSize(2);
        display.setCursor(15, 0);

        if (t.hour < 10) display.print(' ');
        display.print(t.hour);
        display.print(":");

        if (t.min < 10) display.print('0');
        display.print(t.min);
        // Descomente abaixo se quiser exibir os segundos na tela também:
        // display.print(":");
        // if (t.sec < 10) display.print('0');
        // display.print(t.sec);

        // Se a atualização foi motivada pelo tempo (500ms), invertemos o coração
        if (meioSegundoPassou) {
          heartBeat = !heartBeat; // Inverte o estado do heartBeat
          ultimoHeartbeatMs = millis();             // Reseta o cronômetro do heartbeat
        }

        if (heartBeat) {
          // Batida alta: Desenha o coração cheio na posição original
          display.setCursor(96, 3);
          display.write(3); 
        } else {
          // Batida baixa: Você pode deixar vazio ou desenhar um caractere menor 
          display.setCursor(96, 3);
          display.print(" "); 
        }

        // Temperatura
        display.setCursor(0, 50);
        display.cp437(true);
        display.print(r.temp);
        display.write(0xF8);

        // Modo
        display.setCursor(55, 28);
        display.print(r.modoText);

        // RSSI e estado Blynk
        display.setTextSize(1);
        display.setCursor(44, 57);
        display.print(r.rssi);
        display.print(" ");
        display.print(r.blynkStateText);

        // Barras RSSI
        long rssi = r.rssi;
        if (rssi > -55 && rssi < -3) {
          display.fillRect(40, 25, 4, 17, WHITE);
          display.fillRect(33, 29, 4, 13, WHITE);
          display.fillRect(26, 33, 4, 9, WHITE);
          display.fillRect(19, 37, 4, 5, WHITE);
        } else if (rssi < -55 && rssi > -70) {
          display.drawRect(40, 25, 4, 17, WHITE);
          display.fillRect(33, 29, 4, 13, WHITE);
          display.fillRect(26, 33, 4, 9, WHITE);
          display.fillRect(19, 37, 4, 5, WHITE);
        } else if (rssi < -70 && rssi > -78) {
          display.drawRect(40, 25, 4, 17, WHITE);
          display.drawRect(33, 29, 4, 13, WHITE);
          display.fillRect(26, 33, 4, 9, WHITE);
          display.fillRect(19, 37, 4, 5, WHITE);
        } else if (rssi < -78 && rssi > -82) {
          display.drawRect(40, 25, 4, 17, WHITE);
          display.drawRect(33, 29, 4, 13, WHITE);
          display.drawRect(26, 33, 4, 9, WHITE);
          display.fillRect(19, 37, 4, 5, WHITE);
        } else {
          display.drawRect(40, 25, 4, 17, WHITE);
          display.drawRect(33, 29, 4, 13, WHITE);
          display.drawRect(26, 33, 4, 9, WHITE);
          display.drawRect(19, 37, 4, 5, WHITE);
        }

        display.display();

        xSemaphoreGive(mtxI2C);

        // Salva o segundo atual para a próxima comparação
        ultimoSegundo = t.sec; 

      } else {
        ESP_LOGW(TAG_DISPLAY, "Timeout ao acessar I2C para display");
      }
    }

    // IMPORTANTE: Reduzido de 1000ms para 50ms para que a Task verifique 
    // a mudança de segundo de forma responsiva sem prender o processador.
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =====================================================
// Task Modbus - Core 0
// =====================================================

void TaskModbus(void *pv) {
  LOG_TASK_START(TAG_MODBUS);

  // Instanciação do cliente eModbus associado à Serial1
  //ModbusClientRTU mbClient(Serial1);
  // Inicializa a porta serial com os pinos e baudrate definidos (ajuste se necessário)
  //Serial1.begin(9600, SERIAL_8N1, 14, 27);
  
  // Inicializa o cliente eModbus
  mbClient.begin(Serial1);

  ESP_LOGI(TAG_MODBUS, "RS485 iniciado em Serial1 RX=14 TX=27 baud=9600 slave=1");

  static bool lastModbusError = false;  // Rastreia o estado anterior do erro

  for (;;) {
    lastAliveModbus = millis();

    // Executa a requisição síncrona (bloqueia apenas esta Task até responder ou dar timeout)
    // Parâmetros: token (millis), serverID (1), functionCode (READ_HOLD_REG), endereço (0x100), quantidade (9)
    //ModbusMessage response = mbClient.syncRequest(millis(), 1, READ_HOLD_REG, 0x100, 9);
    ModbusMessage response = mbClient.syncRequest(millis(), 1, 0x03, 0x100, 9);

    ModbusData mb;
    memset(&mb, 0, sizeof(mb));
    
    // Obtém o código de erro retornado pela eModbus
    Error err = response.getError();
    mb.status = err;

    if (err == SUCCESS) {
      uint16_t reg[9];

      // O eModbus armazena a resposta bruta em formato de vetor.
      // No frame RTU de resposta do código 0x03, os dados úteis começam no offset 3:
      // [0]=Slave ID, [1]=Função, [2]=Contagem de Bytes, [3...]=Registradores (High/Low)
      // O método .get() converte automaticamente de Big-Endian (Modbus) para Little-Endian (ESP32)
      for (int i = 0; i < 9; i++) {
        response.get(3 + (i * 2), reg[i]);
      }

      // Aplica as suas equações matemáticas originais com base nos registradores lidos
      mb.vR   = reg[0] / 100.00;
      mb.vS   = reg[1] / 100.00;
      mb.vT   = reg[2] / 100.00;
      mb.iR   = (reg[3] / 100.00) - 10;
      mb.iS   = (reg[4] / 100.00) - 5;
      mb.iT   = (reg[5] / 100.00) - 7;
      mb.pR   = reg[6] / 1000.00;
      mb.pS   = reg[7] / 1000.00;
      mb.pT   = reg[8] / 1000.00;

      // imprime os valores lidos no log para monitoramento
      ESP_LOGD(TAG_MODBUS,
               "RS485 OK | V %.2f %.2f %.2f | I %.2f %.2f %.2f | P %.2f %.2f %.2f",
               mb.vR, mb.vS, mb.vT,
               mb.iR, mb.iS, mb.iT,
               mb.pR, mb.pS, mb.pT);
      
      // Se estava em erro e agora voltou ao normal, envia log uma vez
      if (lastModbusError) {
        queueLogf("RS485 restaurado com sucesso");
        ESP_LOGI(TAG_MODBUS, "RS485 restaurado");
        lastModbusError = false;
      }
    } else {
      // O ModbusError(err).toStr() converte o código hexadecimal em uma string legível (ex: "TIMEOUT")
      ESP_LOGW(TAG_MODBUS, "Falha leitura RS485. Erro: %s", (const char *)ModbusError(err));
      
      // Envia log apenas na primeira vez que o erro ocorre
      if (!lastModbusError) {
        queueLogf("RS485 em falha: %s", (const char *)ModbusError(err));
        lastModbusError = true;
      }
    }

    // Gravação segura dos dados tratados na struct global utilizando seu Semáforo
    if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
      gModbus = mb;
      xSemaphoreGive(mtxData);
    } else {
      ESP_LOGW(TAG_MODBUS, "Timeout ao gravar dados Modbus compartilhados");
    }

    // Intervalo de tempo entre as leituras (essencial para evitar estouro de Watchdog no Core 0)
    vTaskDelay(pdMS_TO_TICKS(1000)); 
  }
}

// =====================================================
// Task Supervisor - Core 0
// =====================================================

void TaskSupervisor(void *pv) {
   LOG_TASK_START(TAG_SUPERVISOR);

   // Configura o GPIO do LED Heartbeat
   pinMode(HEARTBEAT_PIN, OUTPUT);

   // Contador para controlar a checagem do Watchdog a cada 1 segundo
   uint8_t ciclosWatchdog = 0; 

   for (;;) {
      // Roda a cada 250ms - só faz a checagem a cada 4 ciclos (4 x 250ms = 1000ms = 1s)
      ciclosWatchdog++;
      if (ciclosWatchdog >= 4) {
         ciclosWatchdog = 0; // Reseta o contador

         uint32_t now = millis();

         // Verifica status do RTC
         bool rtcError = false;
         if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
            rtcError = gRun.rtcError;
            xSemaphoreGive(mtxData);
         }
         
         if (rtcError) {
            ESP_LOGE(TAG_WDT, "ERRO CRÍTICO: RTC não respondendo. WDT nao sera alimentado.");
            queueLogf("Erro critico no RTC!");
         }

         bool ok = 
            !rtcError &&
            (now - lastAliveBlynk   < 120000UL) &&
            (now - lastAliveIO      < 10000UL) &&
            (now - lastAliveRTC     < 10000UL) &&
            (now - lastAliveDisplay < 15000UL) &&
            (now - lastAliveModbus  < 30000UL);

         // Gera pulso de heartbeat (10ms) apenas quando sistema está saudável
         if (ok) {
            // Pulso curto (10ms) para hardware watchdog externo
            digitalWrite(HEARTBEAT_PIN, HIGH);
            delayMicroseconds(10000);  // 10ms de pulso
            digitalWrite(HEARTBEAT_PIN, LOW);
            
            rtc_wdt_feed();
            ESP_LOGD(TAG_WDT, "Watchdog alimentado - Pulso OK (10ms)");
         } else {
            // Mantém LOW se houver erro
            digitalWrite(HEARTBEAT_PIN, LOW);
            
            ESP_LOGE(TAG_WDT,
               "ERRO: Task timeout! Blynk:%lums IO:%lums RTC:%lums Display:%lums Modbus:%lums | RtcError=%d",
               (unsigned long)(now - lastAliveBlynk),
               (unsigned long)(now - lastAliveIO),
               (unsigned long)(now - lastAliveRTC),
               (unsigned long)(now - lastAliveDisplay),
               (unsigned long)(now - lastAliveModbus),
               rtcError);

            queueLogf("Erro critico - Task Supervisor!");
         }

         // A sua lógica do mtxData (BlynkState, Temp, RSSI) que está na linha 1398 
         // também continuará rodando aqui de forma segura a cada 1 segundo.
         if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(50)) == pdTRUE) {
            gRun.rssi = WiFi.RSSI();
            gRun.temp = ((temprature_sens_read() - 32) / 1.8) - 30;
            gRun.blynkState = BlynkState::get();
            updateBlynkStateText(gRun.blynkState, gRun.blynkStateText, sizeof(gRun.blynkStateText));
            xSemaphoreGive(mtxData);
         }
		 
			// Print de diagnóstico (temporizado ou esporádico).  USAR PARA DEBUG, NÃO DEIXAR ATIVO EM PRODUÇÃO

			// --- Executa também o relatório de monitoramento de memória ---
			/*
			ESP_LOGI("SUPERVISOR", "--- MEMÓRIA LIVRE POR TASK ---");
			if (taskBlynkHandle != NULL)      ESP_LOGI("SUPERVISOR", "TaskBlynk: %d B", uxTaskGetStackHighWaterMark(taskBlynkHandle));
			if (taskIOHandle != NULL)         ESP_LOGI("SUPERVISOR", "TaskIOControl: %d B", uxTaskGetStackHighWaterMark(taskIOHandle));
			if (taskRTCHandle != NULL)        ESP_LOGI("SUPERVISOR", "TaskRTC: %d B", uxTaskGetStackHighWaterMark(taskRTCHandle));
			if (taskDisplayHandle != NULL)    ESP_LOGI("SUPERVISOR", "TaskDisplay: %d B", uxTaskGetStackHighWaterMark(taskDisplayHandle));
			if (taskModbusHandle != NULL)     ESP_LOGI("SUPERVISOR", "TaskModbus: %d B", uxTaskGetStackHighWaterMark(taskModbusHandle));
			ESP_LOGI("SUPERVISOR", "Heap Global Livre: %u bytes", esp_get_free_heap_size());
			ESP_LOGI("SUPERVISOR", "--------------------------------------------------");
			*/
			//Se o log apontar valores muito baixos (ex: menores que 150-200 bytes): Aumente o valor do respectivo #define STACK_XXX
    
			//Se o log apontar valores muito altos constantemente (ex: sobrando 1500 bytes ou mais): Significa que a tarefa foi superdimensionada. 
			//Você pode diminuir com segurança o respectivo #define STACK_XXX para liberar mais memória RAM (Heap) global para o sistema.
    
      }

      // 3. Bloqueia a task por apenas 250ms em vez de 1000ms
      vTaskDelay(pdMS_TO_TICKS(250)); 
   }
}

// =====================================================
// Saídas PCF8574
// =====================================================

void writeOutputPLC()
{
  if (xSemaphoreTake(mtxI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
    Wire.beginTransmission(PCF_OUTPUT_ADDR);
    Wire.write(output_PLC);
    int err = Wire.endTransmission();

    xSemaphoreGive(mtxI2C);

    if (err != 0) {
      ESP_LOGW(TAG_IO, "Falha escrita PCF saida. Erro=%d output=0x%02X", err, output_PLC);
    } else {
      ESP_LOGD(TAG_IO, "PCF saida atualizado: 0x%02X", output_PLC);
    }
  } else {
    ESP_LOGW(TAG_IO, "Timeout ao acessar I2C para escrita PCF saida");
  }
}

void pulseLiga()
{
  ESP_LOGI(TAG_IO, "Pulso LIGAR iniciado");

  output_PLC &= ~(1 << 0);
  writeOutputPLC();

  vTaskDelay(pdMS_TO_TICKS(1000));

  output_PLC |= (1 << 0);
  writeOutputPLC();

  ESP_LOGI(TAG_IO, "Pulso LIGAR finalizado");
}

void pulseDesliga()
{
  ESP_LOGI(TAG_IO, "Pulso DESLIGAR iniciado");

  output_PLC &= ~(1 << 1);
  writeOutputPLC();

  vTaskDelay(pdMS_TO_TICKS(1000));

  output_PLC |= (1 << 1);
  writeOutputPLC();

  ESP_LOGI(TAG_IO, "Pulso DESLIGAR finalizado");
}

// =====================================================
// Logs para Serial via esp_log e para Blynk via fila
// =====================================================

void queueLogf(const char *fmt, ...)
{
  if (!qLog) return;

  LogMessage msg;
  memset(&msg, 0, sizeof(msg));

  va_list args;
  va_start(args, fmt);
  vsnprintf(msg.text, sizeof(msg.text), fmt, args);
  va_end(args);

  ESP_LOGI(TAG_BLYNK, "%s", msg.text);

  if (xQueueSend(qLog, &msg, 0) != pdTRUE) {
    ESP_LOGW(TAG_BLYNK, "Fila de logs cheia. Mensagem descartada: %s", msg.text);
  }
}

// =====================================================
// RTC via NTP
// =====================================================

bool setRTCFromNTP() // Alterado de void para bool
{
  struct tm timeinfo;

  ESP_LOGI(TAG_RTC, "Solicitando data/hora via NTP");

  if (!getLocalTime(&timeinfo)) {
    ESP_LOGW(TAG_RTC, "Falha ao sincronizar NTP");
    queueLogf("Falha sincronismo NTP");
    return false; // Retorna falso se falhar a internet/NTP
  }

  int ye = timeinfo.tm_year + 1900;
  int mo = timeinfo.tm_mon + 1;
  int da = timeinfo.tm_mday;

  int ho = timeinfo.tm_hour;
  int mi = timeinfo.tm_min;
  int se = timeinfo.tm_sec + 1;

  ESP_LOGI(TAG_RTC,
           "Hora NTP recebida: %02d/%02d/%04d %02d:%02d:%02d",
           da, mo, ye, ho, mi, se);

  if (xSemaphoreTake(mtxI2C, pdMS_TO_TICKS(1000)) == pdTRUE) {
    RTC.adjust(DateTime(ye, mo, da, ho, mi, se));
    xSemaphoreGive(mtxI2C);

    ESP_LOGI(TAG_RTC, "RTC DS1307 ajustado com sucesso");
  } else {
    ESP_LOGE(TAG_RTC, "Timeout ao tentar ajustar RTC via I2C");
    return false; // Retorna falso se falhar o barramento I2C
  }

  return true; // Retorna verdadeiro se tudo deu certo!
}

// =====================================================
// NVS
// =====================================================

void loadCounterAndMotorState(bool &memMotorState)
{
  preferences.begin("my-app", false);

  uint32_t counterRST = preferences.getUInt("counterRST", 0);
  counterRST++;

  preferences.putUInt("counterRST", counterRST);

  memMotorState = preferences.getBool("MemMotorState", true);

  preferences.end();

  if (xSemaphoreTake(mtxData, portMAX_DELAY) == pdTRUE) {
    gRun.counterRST = counterRST;
    gRun.sendResetLog = true;
    xSemaphoreGive(mtxData);
  }

  oldMotorOff = memMotorState;

  ESP_LOGI(TAG_NVS,
           "NVS carregada: counterRST=%lu MemMotorState=%s",
           (unsigned long)counterRST,
           memMotorState ? "DESLIGADO" : "LIGADO");
}

void loadSettingsFromNVS()
{
  preferences.begin("my-app", false);

  uint32_t liga    = preferences.getUInt("HoraLigaPGM", 0);
  uint32_t desliga = preferences.getUInt("HoraDESLigaPGM", 0);
  int modo         = preferences.getUInt("RemotoOuAgenda", 0);
  String dias      = preferences.getString("DiaSemPGM", "NULL");

  preferences.end();

  if (xSemaphoreTake(mtxData, portMAX_DELAY) == pdTRUE) {
    gSchedule.horaLigaSec    = liga;
    gSchedule.horaDesligaSec = desliga;
    gSchedule.remotoOuAgenda = modo;

    strncpy(gSchedule.diasSemana, dias.c_str(), sizeof(gSchedule.diasSemana) - 1);
    gSchedule.diasSemana[sizeof(gSchedule.diasSemana) - 1] = '\0';

    strncpy(gRun.modoText, "START", sizeof(gRun.modoText) - 1);
    gRun.modoText[sizeof(gRun.modoText) - 1] = '\0';

    xSemaphoreGive(mtxData);
  }

  ESP_LOGI(TAG_NVS,
           "Agenda NVS: liga=%lu desliga=%lu modo=%d dias=%s",
           (unsigned long)liga,
           (unsigned long)desliga,
           modo,
           dias.c_str());
}

void saveScheduleToNVS()
{
  ScheduleData s;

  if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(100)) == pdTRUE) {
    s = gSchedule;
    xSemaphoreGive(mtxData);
  } else {
    ESP_LOGW(TAG_NVS, "Timeout ao copiar agenda para salvar NVS");
    return;
  }

  preferences.begin("my-app", false);

  preferences.putUInt("HoraLigaPGM", s.horaLigaSec);
  preferences.putUInt("HoraDESLigaPGM", s.horaDesligaSec);
  preferences.putString("DiaSemPGM", s.diasSemana);

  preferences.end();

  ESP_LOGI(TAG_NVS,
           "Agenda salva NVS: liga=%lu desliga=%lu dias=%s",
           (unsigned long)s.horaLigaSec,
           (unsigned long)s.horaDesligaSec,
           s.diasSemana);
}

void saveModoToNVS()
{
  int modo = 0;

  if (xSemaphoreTake(mtxData, pdMS_TO_TICKS(100)) == pdTRUE) {
    modo = gSchedule.remotoOuAgenda;
    xSemaphoreGive(mtxData);
  } else {
    ESP_LOGW(TAG_NVS, "Timeout ao copiar modo para salvar NVS");
    return;
  }

  preferences.begin("my-app", false);
  preferences.putUInt("RemotoOuAgenda", modo);
  preferences.end();

  ESP_LOGI(TAG_NVS, "Modo salvo NVS: %d", modo);
}

// =====================================================
// Restauração do estado do motor
// =====================================================

void restoreMotorState(bool memMotorState)
{
  ESP_LOGI(TAG_IO,
           "Restaurando ultimo estado do motor. 1=desligado: %d",
           memMotorState);

  if (memMotorState == false) {
    pulseLiga();
    cicloOFF = 0;
    ESP_LOGI(TAG_IO, "Motor ativado pela memoria");
  } else {
    pulseDesliga();
    cicloON = 0;
    ESP_LOGI(TAG_IO, "Motor desativado pela memoria");
  }
}

// =====================================================
// Watchdog
// =====================================================

void initRtcWdt()
{
  ESP_LOGI(TAG_WDT, "Configurando RTC Watchdog timeout=%lu ms", (unsigned long)WDT_TIMEOUT);

  rtc_wdt_protect_off();
  rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_RTC);
  rtc_wdt_set_time(RTC_WDT_STAGE0, WDT_TIMEOUT);
  rtc_wdt_enable();
  rtc_wdt_protect_on();

  ESP_LOGI(TAG_WDT, "RTC Watchdog habilitado");
}

const char *resetReasonName(esp_reset_reason_t r)
{
  switch (r) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN RESET";
    case ESP_RST_POWERON:   return "POWER ON RESET";
    case ESP_RST_EXT:       return "EXTERN PIN RESET";
    case ESP_RST_SW:        return "SOFTWARE REBOOT";
    case ESP_RST_PANIC:     return "CRASH RESET";
    case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG";
    case ESP_RST_TASK_WDT:  return "TASK WATCHDOG";
    case ESP_RST_WDT:       return "RTC WATCHDOG";
    case ESP_RST_DEEPSLEEP: return "SLEEP RESET";
    case ESP_RST_BROWNOUT:  return "BROWNOUT RESET";
    case ESP_RST_SDIO:      return "RESET OVER SDIO";
    default:                return "";
  }
}

// =====================================================
// Blynk state text
// =====================================================

void updateBlynkStateText(uint32_t state, char *buffer, size_t len)
{
  const char *txt = "UNKNOWN";

  switch (state) {
    case 0: txt = "WAIT CONFIG"; break;
    case 1: txt = "CONFIG";      break;
    case 2: txt = "NET Conn";    break;
    case 3: txt = "CLOUD Conn";  break;
    case 4: txt = "RUNNING";     break;
    case 5: txt = "OTA";         break;
    case 6: txt = "STATION";     break;
    case 7: txt = "RESET CFG";   break;
    case 8: txt = "ERROR";       break;
    default: txt = "UNKNOWN";    break;
  }

  strncpy(buffer, txt, len - 1);
  buffer[len - 1] = '\0';
}

// =====================================================
// Mensagem de falha crítica
// =====================================================

void failMSG(String HW_status)
{
  ESP_LOGE(TAG_MAIN, "Falha critica de hardware: %s", HW_status.c_str());

  if (mtxI2C && xSemaphoreTake(mtxI2C, pdMS_TO_TICKS(500)) == pdTRUE) {
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(45, 7);
    display.println("R&M");

    display.setTextSize(1);
    display.setCursor(42, 27);
    display.println("Company");

    display.setCursor(15, 50);
    display.print("STOP: ");
    display.println(HW_status);

    display.display();

    xSemaphoreGive(mtxI2C);
  }

  // Para a execução. O RTC WDT fará o reset.
  while (1) {
    ESP_LOGE(TAG_MAIN, "Sistema parado por seguranca: %s", HW_status.c_str());
    delay(5000);
  }
}