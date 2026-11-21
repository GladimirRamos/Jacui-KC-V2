# Jacuí KC V2

Firmware para ESP32 e controlador KinCony KC868-A6, desenvolvido para monitorar e comandar um motor por pulsos, com operação local, pelo aplicativo Blynk ou por agenda.

## Funcionalidades

- Comandos de ligar e desligar o motor e leitura do seu estado físico.
- Controle automático por nível mínimo e máximo, com calibração do sensor.
- Controle de válvula proporcional por saída DAC, em modo manual ou automático.
- Programação de agenda pelo Blynk e sincronização de horário.
- Monitoramento de dados via Modbus RTU, display OLED e logs.
- Armazenamento de configurações e do último comando manual na memória NVS.
- Supervisão das tarefas com watchdog.

## Modos de operação

| Modo | Funcionamento |
| --- | --- |
| **Local** | O painel controla o motor; novos comandos pelo app e pela agenda não são processados. |
| **Manual APP** | O aplicativo solicita pulsos de ligar ou desligar. Na inicialização, o estado físico é comparado ao último comando manual salvo. |
| **Agenda** | O estado esperado é calculado a partir dos dias e horários programados. No intervalo ativo, a agenda espera o motor desligado; fora dele, espera o motor ligado. |

O controle de nível, quando habilitado e autorizado pelo modo, solicita o desligamento abaixo do mínimo e a ligação acima do máximo. Ao desativá-lo com a agenda ativa, o firmware reconcilia o motor com o estado esperado pela agenda: **o motor pode ligar mesmo abaixo do nível mínimo**.

> **Atenção:** este firmware comanda equipamentos físicos. Antes de carregar ou testar, confirme as ligações, os limites de nível, a programação e os dispositivos de proteção independentes. A troca para modo local não cancela necessariamente um pulso já iniciado.

Consulte o [fluxograma de pulsos](FLUXOGRAMA_PULSOS.txt) para detalhes da inicialização, reconciliação, controle de nível e sequência de pulsos.

## Hardware

O firmware utiliza ESP32, expansores PCF8574 para entradas e saídas, RTC DS1307, display OLED, sensor analógico de nível e comunicação Modbus RTU. A configuração de pinos e endereços está em [`src/Main.cpp`](src/Main.cpp).

Referências da placa:

- [Pinagem KC868-A6](KC868-A6-ESP_pinout.png)
- [Esquema elétrico](KC868-A6-schematic.pdf)
- [Diagrama do controlador](KinCony-KC868-A6-smart-controller-diagram.jpg)

## Compilação e gravação

### Requisitos

- [Visual Studio Code](https://code.visualstudio.com/) com a extensão [PlatformIO IDE](https://platformio.org/platformio-ide), ou PlatformIO Core.
- Placa ESP32 e conexão USB adequadas para a gravação.
- Acesso à rede Wi-Fi e configuração do dispositivo no Blynk para as funções do aplicativo.

Os ambientes estão definidos em [`platformio.ini`](platformio.ini):

| Ambiente | Configuração |
| --- | --- |
| `jacui` | Jacuí |
| `levante` | Levante |
| `coqueiro` | Coqueiro |

Os ambientes possuem ajustes próprios, incluindo pino do sensor, limites de corrente e configuração Blynk. Confira a configuração correspondente em [`src/Main.cpp`](src/Main.cpp) antes de gravar.

Na raiz do projeto, compile o ambiente desejado:

```sh
pio run -e jacui
```

Para gravar na placa conectada e abrir o monitor serial:

```sh
pio run -e jacui -t upload
pio device monitor -b 115200
```

Substitua `jacui` por `levante` ou `coqueiro` conforme a instalação. O PlatformIO obtém as dependências declaradas em `platformio.ini` na primeira compilação. O firmware compilado fica em `.pio/build/<ambiente>/firmware.bin`.

## Estrutura do projeto

```text
.
├── src/Main.cpp              # Aplicação e tarefas FreeRTOS
├── include/                  # Configuração e componentes Blynk Edgent
├── lib/                      # Bibliotecas locais
├── test/                     # Diretório de testes PlatformIO
├── platformio.ini            # Ambientes e dependências
└── FLUXOGRAMA_PULSOS.txt     # Fluxo de acionamento do motor
```
