# 📡 Leitura de Vibração e Frequência com ADXL345 + ESP32

Projeto acadêmico de monitoramento de vibração em equipamentos industriais/mecânicos utilizando o acelerômetro ADXL345 conectado a um ESP32, com interface web em tempo real acessível via Wi-Fi.

---

## 🎯 Objetivo

Medir a vibração de um equipamento e convertê-la em frequência (Hz), permitindo uma análise simplificada do estado do equipamento:

| Condição | Indicação |
|---|---|
| Frequência estável e baixa amplitude | ✅ Equipamento OK |
| Frequência elevada com amplitude alta | ⚠️ Possível desbalanceamento |
| Anomalia detectada pelo sistema | ❌ Verificar equipamento |

> **Nota:** Este projeto utiliza o ADXL345, um acelerômetro de baixo custo para fins acadêmicos. Para aplicações industriais reais, recomenda-se sensores de maior precisão.

---

## 🔧 Hardware necessário

- ESP32 (qualquer variante com Wi-Fi)
- Sensor ADXL345
- Jumpers para ligação
- Fonte de alimentação (USB ou externa 3.3V)

---

## 🔌 Ligações (modo I2C)

```
ESP32        →      ADXL345
─────────────────────────────
GND          →      GND
3.3V         →      VCC
3.3V         →      CS      (força modo I2C)
GND          →      SDO     (define endereço 0x53)
G21 (SDA)    →      SDA
G22 (SCL)    →      SCL
```

> ⚠️ **Atenção:** Não conecte ao 5V — o ADXL345 opera em 3.3V.

---

## 📚 Bibliotecas necessárias (Arduino IDE)

Instale pelo **Gerenciador de Bibliotecas** (`Sketch → Incluir Biblioteca → Gerenciar Bibliotecas`):

| `Adafruit ADXL345`        | Adafruit        |
| `Adafruit Unified Sensor` | Adafruit        |
| `arduinoFFT`              | kosme           |
| `WebSockets`              | Markus Sattler  |
| `ArduinoJson`             | Benoit Blanchon |  

---

## ⚙️ Configuração antes de usar

No início do código, altere as credenciais Wi-Fi para a sua rede:

```cpp
const char* ssid     = "NOME_DA_SUA_REDE";
const char* password = "SENHA_DA_SUA_REDE";
```

---

## 🚀 Como usar

### 1. Gravar o código
- Abra o arquivo `.ino` no Arduino IDE
- Selecione a placa: `ESP32 Dev Module`
- Grave o código no ESP32

### 2. Conectar ao Wi-Fi
- Abra o **Monitor Serial** (115200 baud)
- Aguarde a mensagem `Conectado!` e anote o IP exibido

### 3. Acessar a interface web
- No navegador, acesse: `http://IP_DO_ESP32`
- A interface exibe em tempo real:
  - Gráfico de vibração nos eixos X, Y e Z
  - Frequência dominante (Hz)
  - RMS e pico a pico da vibração
  - Espectro FFT
  - Log de anomalias

### 4. Calibrar o sensor
- Deixe o equipamento **parado e estável**
- Clique em **Calibrar** na interface
- Aguarde a calibração completar (~800 amostras)
- Após calibrado, o sistema passa a detectar anomalias automaticamente

### 5. Exportar dados
- Clique em **Baixar CSV** para exportar o histórico de leituras
- O arquivo contém: data/hora, operador, X, Y, Z, frequência, RMS e pico a pico

---

## 📊 Como interpretar os resultados

### Frequência dominante (Hz)
A FFT analisa o sinal de vibração e identifica a frequência com maior energia:

```
Frequência baixa  (<10 Hz)   → vibração lenta, estrutural
Frequência média  (10–50 Hz) → típico de motores elétricos comuns
Frequência alta   (>50 Hz)   → rotação elevada ou harmônicos
```

### RMS (Root Mean Square)
Representa a intensidade média da vibração:
- Valor baixo e estável → equipamento OK
- Valor crescente ao longo do tempo → desgaste ou desbalanceamento

### Detecção de anomalias (Z-Score)
O sistema aprende o comportamento normal do equipamento após a calibração e sinaliza quando o RMS se desvia mais de **3.5 desvios padrão** da média — indicando uma possível anomalia.

---

## 🧠 Funcionamento interno (resumo técnico)

```
Leitura bruta (800 Hz)
        ↓
Filtro anti-aliasing
        ↓
Filtro passa-alta  → remove gravidade/DC
        ↓
Filtro passa-baixa → suaviza ruído de alta frequência
        ↓
FFT (256 amostras) → identifica frequência dominante
        ↓
Interface Web (WebSocket) → atualização em tempo real
```

Os filtros se ajustam automaticamente com base na frequência dominante detectada, otimizando a leitura para o tipo de equipamento monitorado.

---

## 📁 Estrutura do repositório

```
📦 vibration-monitor-esp32
 ┣ 📄 adxl-esp32.ino        → firmware principal
 ┣ 📄 README.md             → este arquivo
 ┃ 📄 esquema.png        → diagrama de ligações
 | 📄 Fluxograma.png     → Fçuxograma
 ┃ 📄 caixa.stl          → modelo 3D da caixa
 ┃ 📄 artigo.pdf         → artigo completo

```

## 👥 Autores

Projeto acadêmico desenvolvido no curso de Engenharia Eletrônica — UNIENSINO.

- Clayton Duarte
- Fabiano Moor
- Mateus Alves
- Peterson Andrade

---

## ⚠️ Limitações

- O ADXL345 tem resolução de 10 bits e alcance máximo de ±16g — suficiente para análise acadêmica, mas limitado para diagnósticos industriais precisos
- A taxa de amostragem de 800 Hz permite detectar frequências de até 400 Hz (limite de Nyquist)
- A precisão da FFT depende da fixação física do sensor no equipamento — quanto melhor fixado, mais confiável a leitura
- O sistema requer Wi-Fi ativo para funcionamento da interface
