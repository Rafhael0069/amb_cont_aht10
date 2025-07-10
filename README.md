# Sistema de Monitoramento Ambiental com AHT10

Este projeto implementa um sistema de monitoramento ambiental utilizando o microcontrolador RP2040 (Raspberry Pi Pico), sensor AHT10 para medição de temperatura e umidade, e um display OLED para visualização dos dados. O sistema inclui alertas visuais quando os valores medidos ultrapassam limites pré-definidos.

##  Funcionalidades Principais

- Leitura precisa de temperatura e umidade ambiente
- Exibição em tempo real no display OLED
- Sistema de alertas para condições anormais:
  - Alerta de umidade alta (>70%)
  - Alerta de temperatura baixa (<20°C)
- Interface limpa e informativa
- Auto-recuperação em caso de falha de comunicação

## Componentes Necessários

- 1 × Raspberry Pi Pico (ou qualquer placa com RP2040)
- 1 × Sensor AHT10 (Temperatura e Umidade)
- 1 × Display OLED SSD1306 (128x64 pixels, I2C)

## Configuração do Hardware

| Componente | Pino RP2040 |
|------------|-------------|
| AHT10 SDA  | GP0         |
| AHT10 SCL  | GP1         |
| OLED SDA   | GP14        |
| OLED SCL   | GP15        |

## Dependências

- [Pico SDK](https://github.com/raspberrypi/pico-sdk)
- Biblioteca `hardware_i2c` (incluída no Pico SDK)
- Biblioteca `ssd1306` (incluída no projeto)

## Como Compilar e Flashear

1. Clone este repositório:
   ```bash
   git clone https://github.com/Rafhael0069/amb_cont_aht10
   cd amb_cont_aht10
   ```

2. Configure o ambiente Pico SDK:
   ```bash
   export PICO_SDK_PATH=/caminho/para/pico-sdk
   ```

3. Crie a pasta de build e compile:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

4. Conecte o RP2040 no modo bootloader (segure BOOT enquanto conecta USB)

5. Copie o arquivo `.uf2` para a unidade montada:
   ```bash
   cp monitoramento_ambiental.uf2 /mnt/RPI-RP2/
   ```

## Estrutura de Arquivos

```
monitoramento-ambiental-rp2040/
├── inc/
│   └── ssd1306.h            # Biblioteca SSD1306
|── include/                 # Include de cabeçalhos
|    └── oled_display.h      # Header OLED
|── src/                     # Include de implementações
|    └── oled_display.c      # Driver OLED
|   amb_cont_aht10.c         # Código principal 
├── CMakeLists.txt           # Configuração de build
└── README.md
```

## Saída Esperada

- **Monitor Serial**:
  ```
  Iniciando sistema com AHT10 e OLED...
  AHT10 pronto
  Umidade: 45.50%, Temperatura: 23.20C
  ```

- **Display OLED**:
  ```
  Temp: 23.2 C
  Umid: 45.5 %
  Ambiente OK
  ```

  Ou em caso de alerta:
  ```
  Temp: 18.7 C
  Umid: 72.3 %
  
  BAIXA: Temp!
  ```

## Licença

Este projeto é de código aberto e pode ser usado livremente para fins educacionais e pessoais.
