// =============================================================================
// SISTEMA DE MONITORAMENTO DE VIBRACAO - VERSAO REVISADA
// Hardware: ESP32 / ESP32-C3 + ADXL345 por I2C
// Projeto : Engenharia Eletronica - UNIENSINO
// Autores : Clayton Duarte, Fabiano Moor, Mateus Alves, Peterson Andrade
// =============================================================================
//
// BIBLIOTECAS NECESSARIAS (Gerenciador de Bibliotecas da Arduino IDE):
//   - Adafruit ADXL345
//   - Adafruit Unified Sensor
//   - WebSockets by Markus Sattler
//   - arduinoFFT by Enrique Condes
//
// PINOS I2C:
//   ESP32 tradicional: SDA 21 / SCL 22
//   ESP32-C3: por padrao deste codigo SDA 8 / SCL 9
//   Altere I2C_SDA e I2C_SCL abaixo se sua placa usar outros pinos.
// =============================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <arduinoFFT.h>
#include <time.h>

// =============================================================================
// CONFIGURACAO GERAL
// =============================================================================

const char* WIFI_SSID = "Fabiano D. Moor";
const char* WIFI_SENHA = "123456789";

#if defined(CONFIG_IDF_TARGET_ESP32C3)
constexpr int I2C_SDA = 8;
constexpr int I2C_SCL = 9;
#else
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
#endif

constexpr uint16_t TAM_BUFFER = 512;
constexpr uint16_t FFT_SAMPLES = 256;
constexpr uint16_t SAMPLING_FREQ = 400;
constexpr uint32_t PERIODO_AMOSTRA_US = 1000000UL / SAMPLING_FREQ;
constexpr uint16_t INTERVALO_ENVIO_MS = 50;   // 20 atualizacoes/s
constexpr uint16_t INTERVALO_FFT_MS = 500;    // FFT 2 vezes/s
constexpr uint8_t MAX_AMOSTRAS_PACOTE = 28;

constexpr float FREQUENCIA_MINIMA_HZ = 2.0f;
constexpr float FREQUENCIA_MAXIMA_HZ = 180.0f;
constexpr float FATOR_PICO_RUIDO = 5.0f;
constexpr float FC_PASSA_ALTA_HZ = 1.0f;
constexpr float GANHO_HAMMING = 0.54f;

static_assert((FFT_SAMPLES & (FFT_SAMPLES - 1)) == 0,
              "FFT_SAMPLES precisa ser potencia de 2");
static_assert(TAM_BUFFER >= FFT_SAMPLES,
              "TAM_BUFFER precisa ser maior ou igual a FFT_SAMPLES");

// =============================================================================
// OBJETOS PRINCIPAIS
// =============================================================================

WebServer server(80);
WebSocketsServer webSocket(81);
Adafruit_ADXL345_Unified accel(12345);

// =============================================================================
// BUFFER CIRCULAR E AQUISICAO
// =============================================================================

float bufferX[TAM_BUFFER];
float bufferY[TAM_BUFFER];
float bufferZ[TAM_BUFFER];
uint32_t bufferTempoUs[TAM_BUFFER];

volatile uint16_t indiceBuffer = 0;
volatile uint16_t amostrasValidas = 0;
volatile uint32_t totalAmostras = 0;
uint32_t totalAmostrasEnviadas = 0;

uint32_t proximaAmostraUs = 0;
uint32_t instanteInicialUs = 0;

// Estados do passa-alta digital fixo.
float hpX = 0.0f, hpY = 0.0f, hpZ = 0.0f;
float anteriorX = 0.0f, anteriorY = 0.0f, anteriorZ = 0.0f;
float alphaHP = 0.0f;

// =============================================================================
// CALIBRACAO DE REPOUSO
// =============================================================================

constexpr uint16_t AMOSTRAS_CALIBRACAO = 800; // 2 segundos em 400 Hz

bool calibrando = false;
bool calibrado = false;
uint16_t indiceCalibracao = 0;
uint32_t proximaCalibracaoUs = 0;

double somaCalX = 0.0, somaCalY = 0.0, somaCalZ = 0.0;
double somaQuadCalX = 0.0, somaQuadCalY = 0.0, somaQuadCalZ = 0.0;

float offsetX = 0.0f, offsetY = 0.0f, offsetZ = 0.0f;
float ruidoX = 0.0f, ruidoY = 0.0f, ruidoZ = 0.0f;
float limiarRmsRepouso = 0.09f;
float limiarPicoPicoRepouso = 0.35f;

// =============================================================================
// FFT E METRICAS
// =============================================================================

double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
float espectroDb[FFT_SAMPLES / 2];
float espectroLinear[FFT_SAMPLES / 2];

ArduinoFFT<double> FFT(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQ);

constexpr float RESOLUCAO_FFT = (float)SAMPLING_FREQ / FFT_SAMPLES;
constexpr float FREQUENCIA_NYQUIST = (float)SAMPLING_FREQ / 2.0f;

float rmsX = 0.0f, rmsY = 0.0f, rmsZ = 0.0f, rmsTotal = 0.0f;
float picoPicoX = 0.0f, picoPicoY = 0.0f, picoPicoZ = 0.0f;
float rmsEixo = 0.0f, picoPicoEixo = 0.0f;
float frequenciaDominante = 0.0f;
float magnitudeDominante = 0.0f;
float pisoRuidoEspectral = 0.0f;
char eixoAnalise = 'X';
char eixoManual = 'A'; // A = automatico; X/Y/Z = manual
bool sistemaEmRepouso = true;
bool fftAtualizada = false;

// =============================================================================
// BASELINE E EVENTOS
// =============================================================================

struct Evento {
  char dataHora[20];
  char operador[32];
  char tipo[20];
  float rms;
  float frequencia;
  float picoPico;
};

constexpr uint8_t MAX_EVENTOS = 30;
Evento eventos[MAX_EVENTOS];
uint32_t totalEventos = 0;
uint32_t ultimoEventoMs = 0;

char operadorAtual[32] = "Desconhecido";

float baselineRms = 0.0f;
float baselineVar = 0.0f;
uint16_t baselineAmostras = 0;
bool baselinePronto = false;
constexpr uint16_t BASELINE_WARMUP = 40;
constexpr float BASELINE_ALPHA = 0.025f;
constexpr float LIMIAR_ZSCORE = 3.5f;
constexpr uint32_t COOLDOWN_EVENTO_MS = 10000UL;
char diagnostico[20] = "REPOUSO";

// =============================================================================
// TEMPORIZACAO
// =============================================================================

uint32_t ultimoEnvioMs = 0;
uint32_t ultimaFftMs = 0;

// =============================================================================
// FUNCOES AUXILIARES
// =============================================================================

float quadrado(float valor) {
  return valor * valor;
}

void copiarTextoSeguro(char* destino, size_t tamanho, const String& origem) {
  if (tamanho == 0) return;
  origem.substring(0, tamanho - 1).toCharArray(destino, tamanho);
  destino[tamanho - 1] = '\0';
}

String timestampAtual() {
  struct tm infoTempo;
  if (!getLocalTime(&infoTempo, 20)) {
    return "--/--/---- --:--:--";
  }

  char texto[20];
  strftime(texto, sizeof(texto), "%d/%m/%Y %H:%M:%S", &infoTempo);
  return String(texto);
}

void zerarProcessamento() {
  hpX = hpY = hpZ = 0.0f;
  anteriorX = anteriorY = anteriorZ = 0.0f;

  indiceBuffer = 0;
  amostrasValidas = 0;
  totalAmostras = 0;
  totalAmostrasEnviadas = 0;

  for (uint16_t i = 0; i < TAM_BUFFER; i++) {
    bufferX[i] = 0.0f;
    bufferY[i] = 0.0f;
    bufferZ[i] = 0.0f;
    bufferTempoUs[i] = 0;
  }

  for (uint16_t i = 0; i < FFT_SAMPLES / 2; i++) {
    espectroDb[i] = -100.0f;
    espectroLinear[i] = 0.0f;
  }

  rmsX = rmsY = rmsZ = rmsTotal = 0.0f;
  picoPicoX = picoPicoY = picoPicoZ = 0.0f;
  rmsEixo = picoPicoEixo = 0.0f;
  frequenciaDominante = magnitudeDominante = 0.0f;
  pisoRuidoEspectral = 0.0f;
  sistemaEmRepouso = true;
  eixoAnalise = 'X';
  fftAtualizada = true;

  baselineRms = 0.0f;
  baselineVar = 0.0f;
  baselineAmostras = 0;
  baselinePronto = false;
  strncpy(diagnostico, "REPOUSO", sizeof(diagnostico));
  diagnostico[sizeof(diagnostico) - 1] = '\0';

  instanteInicialUs = micros();
  proximaAmostraUs = instanteInicialUs + PERIODO_AMOSTRA_US;
}

// =============================================================================
// CALIBRACAO
// =============================================================================

void iniciarCalibracao() {
  calibrando = true;
  calibrado = false;
  indiceCalibracao = 0;

  somaCalX = somaCalY = somaCalZ = 0.0;
  somaQuadCalX = somaQuadCalY = somaQuadCalZ = 0.0;

  proximaCalibracaoUs = micros();
  strncpy(diagnostico, "CALIBRANDO", sizeof(diagnostico));
  diagnostico[sizeof(diagnostico) - 1] = '\0';

  Serial.println("Calibracao iniciada. Mantenha o sensor totalmente parado.");
}

void finalizarCalibracao() {
  offsetX = somaCalX / AMOSTRAS_CALIBRACAO;
  offsetY = somaCalY / AMOSTRAS_CALIBRACAO;
  offsetZ = somaCalZ / AMOSTRAS_CALIBRACAO;

  const double varX = max(0.0, somaQuadCalX / AMOSTRAS_CALIBRACAO - quadrado(offsetX));
  const double varY = max(0.0, somaQuadCalY / AMOSTRAS_CALIBRACAO - quadrado(offsetY));
  const double varZ = max(0.0, somaQuadCalZ / AMOSTRAS_CALIBRACAO - quadrado(offsetZ));

  ruidoX = sqrt(varX);
  ruidoY = sqrt(varY);
  ruidoZ = sqrt(varZ);

  const float maiorRuido = max(ruidoX, max(ruidoY, ruidoZ));

  // Limiar de RMS e pico a pico calculados a partir do ruido medido.
  // Os limites inferiores evitam ficar no limite de um unico LSB do ADXL345.
  limiarRmsRepouso = constrain(maiorRuido * 3.5f, 0.07f, 0.30f);
  limiarPicoPicoRepouso = constrain(maiorRuido * 10.0f, 0.25f, 1.20f);

  calibrado = true;
  calibrando = false;
  zerarProcessamento();

  Serial.println("Calibracao concluida.");
  Serial.printf("Offsets: X=%.4f Y=%.4f Z=%.4f m/s2\n", offsetX, offsetY, offsetZ);
  Serial.printf("Ruido:   X=%.4f Y=%.4f Z=%.4f m/s2\n", ruidoX, ruidoY, ruidoZ);
  Serial.printf("Limiares: RMS=%.4f PP=%.4f m/s2\n",
                limiarRmsRepouso, limiarPicoPicoRepouso);
}

void tickCalibracao() {
  if (!calibrando) return;

  const uint32_t agoraUs = micros();
  if ((int32_t)(agoraUs - proximaCalibracaoUs) < 0) return;

  proximaCalibracaoUs += PERIODO_AMOSTRA_US;
  if ((int32_t)(agoraUs - proximaCalibracaoUs) > (int32_t)PERIODO_AMOSTRA_US) {
    proximaCalibracaoUs = agoraUs + PERIODO_AMOSTRA_US;
  }

  sensors_event_t evento;
  accel.getEvent(&evento);

  const float x = evento.acceleration.x;
  const float y = evento.acceleration.y;
  const float z = evento.acceleration.z;

  somaCalX += x;
  somaCalY += y;
  somaCalZ += z;
  somaQuadCalX += x * x;
  somaQuadCalY += y * y;
  somaQuadCalZ += z * z;
  indiceCalibracao++;

  if (indiceCalibracao >= AMOSTRAS_CALIBRACAO) {
    finalizarCalibracao();
  }
}

// =============================================================================
// AQUISICAO DO SENSOR
// =============================================================================

void adquirirAmostra() {
  sensors_event_t evento;
  accel.getEvent(&evento);

  // A calibracao remove a gravidade e os offsets para a posicao de montagem.
  const float xCorrigido = evento.acceleration.x - offsetX;
  const float yCorrigido = evento.acceleration.y - offsetY;
  const float zCorrigido = evento.acceleration.z - offsetZ;

  // Passa-alta fixo: remove deriva lenta sem alterar o filtro conforme a FFT.
  hpX = alphaHP * (hpX + xCorrigido - anteriorX);
  hpY = alphaHP * (hpY + yCorrigido - anteriorY);
  hpZ = alphaHP * (hpZ + zCorrigido - anteriorZ);

  anteriorX = xCorrigido;
  anteriorY = yCorrigido;
  anteriorZ = zCorrigido;

  const uint16_t posicao = indiceBuffer;
  bufferX[posicao] = hpX;
  bufferY[posicao] = hpY;
  bufferZ[posicao] = hpZ;
  bufferTempoUs[posicao] = micros() - instanteInicialUs;

  indiceBuffer = (posicao + 1) % TAM_BUFFER;
  if (amostrasValidas < TAM_BUFFER) amostrasValidas++;
  totalAmostras++;
}

void tickAquisicao() {
  if (calibrando || !calibrado) return;

  const uint32_t agoraUs = micros();
  if ((int32_t)(agoraUs - proximaAmostraUs) < 0) return;

  adquirirAmostra();
  proximaAmostraUs += PERIODO_AMOSTRA_US;

  // Nao le varias vezes em sequencia para "recuperar" atraso causado pela rede.
  if ((int32_t)(agoraUs - proximaAmostraUs) > (int32_t)PERIODO_AMOSTRA_US) {
    proximaAmostraUs = agoraUs + PERIODO_AMOSTRA_US;
  }
}

// =============================================================================
// EVENTOS E BASELINE
// =============================================================================

void registrarEvento(const char* tipo) {
  const uint32_t agora = millis();
  if (agora - ultimoEventoMs < COOLDOWN_EVENTO_MS) return;
  ultimoEventoMs = agora;

  const uint8_t indice = totalEventos % MAX_EVENTOS;
  copiarTextoSeguro(eventos[indice].dataHora,
                    sizeof(eventos[indice].dataHora),
                    timestampAtual());
  strncpy(eventos[indice].operador, operadorAtual,
          sizeof(eventos[indice].operador));
  eventos[indice].operador[sizeof(eventos[indice].operador) - 1] = '\0';
  strncpy(eventos[indice].tipo, tipo, sizeof(eventos[indice].tipo));
  eventos[indice].tipo[sizeof(eventos[indice].tipo) - 1] = '\0';
  eventos[indice].rms = rmsTotal;
  eventos[indice].frequencia = frequenciaDominante;
  eventos[indice].picoPico = picoPicoEixo;
  totalEventos++;
}

void atualizarBaseline() {
  if (!calibrado || sistemaEmRepouso) {
    strncpy(diagnostico, "REPOUSO", sizeof(diagnostico));
    diagnostico[sizeof(diagnostico) - 1] = '\0';
    return;
  }

  if (!baselinePronto) {
    baselineAmostras++;
    baselineRms += (rmsTotal - baselineRms) / baselineAmostras;

    strncpy(diagnostico, "APRENDENDO", sizeof(diagnostico));
    diagnostico[sizeof(diagnostico) - 1] = '\0';

    if (baselineAmostras >= BASELINE_WARMUP) {
      baselineVar = max(quadrado(baselineRms) * 0.0025f, 0.0001f);
      baselinePronto = true;
    }
    return;
  }

  const float diferenca = rmsTotal - baselineRms;
  const float sigma = sqrt(max(baselineVar, 0.000001f));
  const float zscore = fabs(diferenca) / sigma;

  if (zscore > LIMIAR_ZSCORE && diferenca > 0.0f) {
    strncpy(diagnostico, "ALERTA", sizeof(diagnostico));
    diagnostico[sizeof(diagnostico) - 1] = '\0';
    registrarEvento("ANOMALIA_RMS");

    // Uma anomalia nao e incorporada ao comportamento considerado normal.
    return;
  }

  strncpy(diagnostico, "NORMAL", sizeof(diagnostico));
  diagnostico[sizeof(diagnostico) - 1] = '\0';

  baselineVar = (1.0f - BASELINE_ALPHA) *
                (baselineVar + BASELINE_ALPHA * diferenca * diferenca);
  baselineRms = (1.0f - BASELINE_ALPHA) * baselineRms +
                BASELINE_ALPHA * rmsTotal;
}

// =============================================================================
// PROCESSAMENTO FFT
// =============================================================================

void zerarResultadoFft() {
  frequenciaDominante = 0.0f;
  magnitudeDominante = 0.0f;
  pisoRuidoEspectral = 0.0f;

  for (uint16_t i = 0; i < FFT_SAMPLES / 2; i++) {
    espectroLinear[i] = 0.0f;
    espectroDb[i] = -100.0f;
  }

  fftAtualizada = true;
}

void processarFFT() {
  if (amostrasValidas < FFT_SAMPLES || calibrando || !calibrado) {
    zerarResultadoFft();
    return;
  }

  double somaX = 0.0, somaY = 0.0, somaZ = 0.0;
  double somaQuadX = 0.0, somaQuadY = 0.0, somaQuadZ = 0.0;
  float minimoX = 1.0e9f, minimoY = 1.0e9f, minimoZ = 1.0e9f;
  float maximoX = -1.0e9f, maximoY = -1.0e9f, maximoZ = -1.0e9f;

  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    const int indice = (indiceBuffer - FFT_SAMPLES + i + TAM_BUFFER) % TAM_BUFFER;
    const float x = bufferX[indice];
    const float y = bufferY[indice];
    const float z = bufferZ[indice];

    somaX += x;
    somaY += y;
    somaZ += z;
    somaQuadX += x * x;
    somaQuadY += y * y;
    somaQuadZ += z * z;

    if (x < minimoX) minimoX = x;
    if (x > maximoX) maximoX = x;
    if (y < minimoY) minimoY = y;
    if (y > maximoY) maximoY = y;
    if (z < minimoZ) minimoZ = z;
    if (z > maximoZ) maximoZ = z;
  }

  const float mediaX = somaX / FFT_SAMPLES;
  const float mediaY = somaY / FFT_SAMPLES;
  const float mediaZ = somaZ / FFT_SAMPLES;

  // RMS AC: a media residual e retirada matematicamente.
  rmsX = sqrt(max(0.0, somaQuadX / FFT_SAMPLES - mediaX * mediaX));
  rmsY = sqrt(max(0.0, somaQuadY / FFT_SAMPLES - mediaY * mediaY));
  rmsZ = sqrt(max(0.0, somaQuadZ / FFT_SAMPLES - mediaZ * mediaZ));
  rmsTotal = sqrt(quadrado(rmsX) + quadrado(rmsY) + quadrado(rmsZ));

  picoPicoX = maximoX - minimoX;
  picoPicoY = maximoY - minimoY;
  picoPicoZ = maximoZ - minimoZ;

  const float maiorRms = max(rmsX, max(rmsY, rmsZ));
  const float maiorPicoPico = max(picoPicoX, max(picoPicoY, picoPicoZ));

  // Histerese evita alternancia rapida entre repouso e vibracao.
  if (sistemaEmRepouso) {
    if (maiorRms > limiarRmsRepouso * 1.50f ||
        maiorPicoPico > limiarPicoPicoRepouso * 1.35f) {
      sistemaEmRepouso = false;
    }
  } else {
    if (maiorRms < limiarRmsRepouso &&
        maiorPicoPico < limiarPicoPicoRepouso) {
      sistemaEmRepouso = true;
    }
  }

  if (eixoManual == 'X' || eixoManual == 'Y' || eixoManual == 'Z') {
    eixoAnalise = eixoManual;
  } else if (rmsY > rmsX && rmsY >= rmsZ) {
    eixoAnalise = 'Y';
  } else if (rmsZ > rmsX && rmsZ > rmsY) {
    eixoAnalise = 'Z';
  } else {
    eixoAnalise = 'X';
  }

  if (eixoAnalise == 'X') {
    rmsEixo = rmsX;
    picoPicoEixo = picoPicoX;
  } else if (eixoAnalise == 'Y') {
    rmsEixo = rmsY;
    picoPicoEixo = picoPicoY;
  } else {
    rmsEixo = rmsZ;
    picoPicoEixo = picoPicoZ;
  }

  if (sistemaEmRepouso) {
    zerarResultadoFft();
    atualizarBaseline();
    return;
  }

  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    const int indice = (indiceBuffer - FFT_SAMPLES + i + TAM_BUFFER) % TAM_BUFFER;

    if (eixoAnalise == 'X') {
      vReal[i] = bufferX[indice] - mediaX;
    } else if (eixoAnalise == 'Y') {
      vReal[i] = bufferY[indice] - mediaY;
    } else {
      vReal[i] = bufferZ[indice] - mediaZ;
    }
    vImag[i] = 0.0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  const uint16_t primeiroBin = max((int)1,
      (int)ceil(FREQUENCIA_MINIMA_HZ / RESOLUCAO_FFT));
  const uint16_t ultimoBin = min((int)(FFT_SAMPLES / 2 - 1),
      (int)floor(FREQUENCIA_MAXIMA_HZ / RESOLUCAO_FFT));

  float maiorMagnitude = 0.0f;
  uint16_t binDominante = 0;
  double somaSemMaior = 0.0;
  uint16_t quantidadeBins = 0;

  for (uint16_t i = 0; i < FFT_SAMPLES / 2; i++) {
    float magnitude = 0.0f;

    if (i > 0) {
      magnitude = (2.0f * (float)vReal[i]) /
                  (FFT_SAMPLES * GANHO_HAMMING);
    }

    espectroLinear[i] = magnitude;
    espectroDb[i] = 20.0f * log10(max(magnitude, 0.00001f));

    if (i >= primeiroBin && i <= ultimoBin) {
      somaSemMaior += magnitude;
      quantidadeBins++;

      if (magnitude > maiorMagnitude) {
        maiorMagnitude = magnitude;
        binDominante = i;
      }
    }
  }

  if (quantidadeBins > 1) {
    somaSemMaior -= maiorMagnitude;
    pisoRuidoEspectral = somaSemMaior / (quantidadeBins - 1);
  } else {
    pisoRuidoEspectral = 0.0f;
  }

  const bool picoConfiavel =
      maiorMagnitude > 0.0f &&
      maiorMagnitude >= max(0.005f, pisoRuidoEspectral * FATOR_PICO_RUIDO) &&
      rmsEixo >= limiarRmsRepouso;

  if (picoConfiavel) {
    float deslocamentoBin = 0.0f;

    // Interpolacao parabolica melhora a frequencia entre dois bins.
    if (binDominante > 0 && binDominante < FFT_SAMPLES / 2 - 1) {
      const float esquerda = espectroLinear[binDominante - 1];
      const float centro = espectroLinear[binDominante];
      const float direita = espectroLinear[binDominante + 1];
      const float denominador = esquerda - 2.0f * centro + direita;

      if (fabs(denominador) > 1.0e-9f) {
        deslocamentoBin = 0.5f * (esquerda - direita) / denominador;
        deslocamentoBin = constrain(deslocamentoBin, -0.5f, 0.5f);
      }
    }

    frequenciaDominante = (binDominante + deslocamentoBin) * RESOLUCAO_FFT;
    magnitudeDominante = maiorMagnitude;
  } else {
    frequenciaDominante = 0.0f;
    magnitudeDominante = 0.0f;
  }

  atualizarBaseline();
  fftAtualizada = true;
}

// =============================================================================
// JSON E WEBSOCKET
// =============================================================================

void adicionarArrayBuffer(String& json,
                          const char* nome,
                          const float* buffer,
                          uint32_t primeiraSequencia,
                          uint8_t quantidade,
                          bool virgulaDepois) {
  json += '\"';
  json += nome;
  json += "\":[";

  for (uint8_t i = 0; i < quantidade; i++) {
    const uint32_t sequencia = primeiraSequencia + i;
    const uint32_t idade = totalAmostras - sequencia;
    const int indice = (indiceBuffer - (int)idade + TAM_BUFFER) % TAM_BUFFER;

    json += String(buffer[indice], 3);
    if (i + 1 < quantidade) json += ',';
  }

  json += ']';
  if (virgulaDepois) json += ',';
}

void adicionarMetricasJson(String& json, bool incluirFft) {
  json += "\"freq\":" + String(frequenciaDominante, 3) + ',';
  json += "\"mag\":" + String(magnitudeDominante, 4) + ',';
  json += "\"rmsX\":" + String(rmsX, 4) + ',';
  json += "\"rmsY\":" + String(rmsY, 4) + ',';
  json += "\"rmsZ\":" + String(rmsZ, 4) + ',';
  json += "\"rmsTotal\":" + String(rmsTotal, 4) + ',';
  json += "\"ppX\":" + String(picoPicoX, 4) + ',';
  json += "\"ppY\":" + String(picoPicoY, 4) + ',';
  json += "\"ppZ\":" + String(picoPicoZ, 4) + ',';
  json += "\"rmsEixo\":" + String(rmsEixo, 4) + ',';
  json += "\"ppEixo\":" + String(picoPicoEixo, 4) + ',';
  json += "\"eixo\":\"" + String(eixoAnalise) + "\",";
  json += "\"modoEixo\":\"" + String(eixoManual) + "\",";
  json += "\"repouso\":" + String(sistemaEmRepouso ? "true" : "false") + ',';
  json += "\"calibrando\":" + String(calibrando ? "true" : "false") + ',';
  json += "\"calibrado\":" + String(calibrado ? "true" : "false") + ',';
  json += "\"progressoCal\":" + String(
      calibrando ? (100.0f * indiceCalibracao / AMOSTRAS_CALIBRACAO) : 100.0f, 1) + ',';
  json += "\"limiarRms\":" + String(limiarRmsRepouso, 4) + ',';
  json += "\"limiarPP\":" + String(limiarPicoPicoRepouso, 4) + ',';
  json += "\"resolucao\":" + String(RESOLUCAO_FFT, 4) + ',';
  json += "\"nyquist\":" + String(FREQUENCIA_NYQUIST, 1) + ',';
  json += "\"fs\":" + String(SAMPLING_FREQ) + ',';
  json += "\"diagnostico\":\"" + String(diagnostico) + "\",";
  json += "\"operador\":\"" + String(operadorAtual) + "\",";
  json += "\"eventos\":" + String(totalEventos) + ',';
  json += "\"fftNova\":" + String(incluirFft ? "true" : "false");

  if (incluirFft) {
    json += ",\"fft\":[";
    for (uint16_t i = 0; i < FFT_SAMPLES / 2; i++) {
      json += String(espectroDb[i], 1);
      if (i + 1 < FFT_SAMPLES / 2) json += ',';
    }
    json += ']';
  }
}

void enviarIncremental() {
  if (webSocket.connectedClients() == 0) return;

  uint32_t pendentes = totalAmostras - totalAmostrasEnviadas;

  if (pendentes > TAM_BUFFER) {
    totalAmostrasEnviadas = totalAmostras - TAM_BUFFER;
    pendentes = TAM_BUFFER;
  }

  const uint8_t quantidade = min((uint32_t)MAX_AMOSTRAS_PACOTE, pendentes);
  const uint32_t primeiraSequencia = totalAmostrasEnviadas;
  const bool incluirFft = fftAtualizada;

  String json;
  json.reserve(5200);
  json = '{';

  adicionarArrayBuffer(json, "x", bufferX, primeiraSequencia, quantidade, true);
  adicionarArrayBuffer(json, "y", bufferY, primeiraSequencia, quantidade, true);
  adicionarArrayBuffer(json, "z", bufferZ, primeiraSequencia, quantidade, true);
  adicionarMetricasJson(json, incluirFft);
  json += '}';

  webSocket.broadcastTXT(json);
  totalAmostrasEnviadas += quantidade;
  if (incluirFft) fftAtualizada = false;
}

void enviarSnapshot(uint8_t cliente) {
  const uint16_t amostrasDisponiveis = amostrasValidas;
  const uint16_t quantidade = (amostrasDisponiveis < 256U) ? amostrasDisponiveis : 256U;

  String json;
  json.reserve(10500);
  json = "{\"snapshot\":true,";

  const uint32_t primeiraSequencia = totalAmostras - quantidade;
  adicionarArrayBuffer(json, "x", bufferX, primeiraSequencia, quantidade, true);
  adicionarArrayBuffer(json, "y", bufferY, primeiraSequencia, quantidade, true);
  adicionarArrayBuffer(json, "z", bufferZ, primeiraSequencia, quantidade, true);
  adicionarMetricasJson(json, true);
  json += '}';

  webSocket.sendTXT(cliente, json);
}

void processarComandoWebSocket(const String& comandoOriginal) {
  String comando = comandoOriginal;
  comando.trim();

  if (comando == "CALIBRAR") {
    iniciarCalibracao();
    return;
  }

  if (comando.startsWith("EIXO:")) {
    String valor = comando.substring(5);
    valor.toUpperCase();

    if (valor == "AUTO") eixoManual = 'A';
    else if (valor == "X") eixoManual = 'X';
    else if (valor == "Y") eixoManual = 'Y';
    else if (valor == "Z") eixoManual = 'Z';
    return;
  }

  if (comando.startsWith("OPERADOR:")) {
    String nome = comando.substring(9);
    nome.trim();
    nome.replace("\"", "");
    nome.replace("\\", "");
    if (nome.length() == 0) nome = "Desconhecido";
    copiarTextoSeguro(operadorAtual, sizeof(operadorAtual), nome);
    return;
  }

  if (comando == "ZERAR_BASELINE") {
    baselineRms = 0.0f;
    baselineVar = 0.0f;
    baselineAmostras = 0;
    baselinePronto = false;
    strncpy(diagnostico, sistemaEmRepouso ? "REPOUSO" : "APRENDENDO",
            sizeof(diagnostico));
    diagnostico[sizeof(diagnostico) - 1] = '\0';
  }
}

void eventoWebSocket(uint8_t cliente,
                     WStype_t tipo,
                     uint8_t* payload,
                     size_t tamanho) {
  switch (tipo) {
    case WStype_CONNECTED:
      enviarSnapshot(cliente);
      break;

    case WStype_TEXT: {
      String comando;
      comando.reserve(tamanho + 1);
      for (size_t i = 0; i < tamanho; i++) {
        comando += (char)payload[i];
      }
      processarComandoWebSocket(comando);
      break;
    }

    default:
      break;
  }
}

// =============================================================================
// INTERFACE WEB
// =============================================================================

static const char PAGINA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>Monitoramento de Vibração</title>
<style>
:root{
  --bg:#eef2f7;--card:rgba(255,255,255,.82);--border:rgba(255,255,255,.92);
  --text:#111827;--muted:#64748b;--accent:#0071e3;--grid:rgba(15,23,42,.10);
  --x:#ff453a;--y:#0a84ff;--z:#30d158;--fft:#7c3aed;--ok:#16a34a;
  --warn:#f59e0b;--danger:#dc2626;--shadow:0 20px 55px rgba(15,23,42,.12)
}
body.dark{
  --bg:#0b1220;--card:rgba(17,24,39,.88);--border:rgba(255,255,255,.08);
  --text:#f8fafc;--muted:#9ca3af;--accent:#60a5fa;--grid:rgba(255,255,255,.10);
  --shadow:0 20px 55px rgba(0,0,0,.35)
}
*{box-sizing:border-box}
body{margin:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;color:var(--text);
background:radial-gradient(circle at 10% 0,rgba(0,113,227,.13),transparent 28%),var(--bg);min-height:100vh}
.wrapper{max-width:1380px;margin:auto;padding:22px}
.header,.card{background:var(--card);border:1px solid var(--border);box-shadow:var(--shadow);backdrop-filter:blur(22px)}
.header{border-radius:28px;padding:22px 24px;margin-bottom:18px;display:flex;justify-content:space-between;gap:16px;align-items:center;flex-wrap:wrap}
.header h1{font-size:1.8rem;margin:0;letter-spacing:-.035em}.header p{margin:7px 0 0;color:var(--muted)}
.status{padding:10px 15px;border-radius:999px;background:rgba(22,163,74,.12);font-weight:750;color:var(--ok)}
.toolbar{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:18px}
button,input,select{font:inherit}
button,.select,.input{border:1px solid var(--border);background:var(--card);color:var(--text);border-radius:14px;padding:10px 13px;box-shadow:0 8px 24px rgba(15,23,42,.08)}
button{cursor:pointer;font-weight:700}button:hover{transform:translateY(-1px)}
button.primary{background:var(--accent);color:#fff;border-color:transparent}
button.active{outline:2px solid var(--accent);background:rgba(0,113,227,.10)}
.input{min-width:180px}.select{min-width:130px}
.layout{display:grid;grid-template-columns:minmax(0,1.65fr) minmax(300px,.75fr);gap:18px}
.stack{display:grid;gap:18px}.card{border-radius:25px;overflow:hidden}.card-head{padding:19px 20px 8px}.card-head h2{margin:0;font-size:1.08rem}.card-head p{margin:6px 0;color:var(--muted);font-size:.9rem}
.controls{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:10px 20px 16px}.control{padding:12px;border-radius:16px;background:rgba(127,127,127,.06)}
.control label{display:flex;justify-content:space-between;color:var(--muted);font-size:.87rem;margin-bottom:8px}.control span{color:var(--accent);font-weight:750}
input[type=range]{width:100%}.chips{padding:0 20px 15px;display:flex;gap:8px;flex-wrap:wrap}.chip{padding:8px 13px;border-radius:999px}
.canvas-wrap{padding:0 16px 16px}canvas{display:block;width:100%;height:360px;border-radius:19px;background:rgba(127,127,127,.045);border:1px solid var(--border)}
#fft{height:285px}.metrics{display:grid;grid-template-columns:1fr 1fr;gap:11px;padding:16px}.metric{padding:15px;border-radius:18px;background:rgba(127,127,127,.065);border:1px solid var(--border)}
.metric .label{color:var(--muted);font-size:.79rem}.metric .value{font-size:1.35rem;font-weight:780;margin-top:6px;letter-spacing:-.025em}.wide{grid-column:1/-1}
.details{padding:0 16px 16px}.details pre{white-space:pre-wrap;margin:0;padding:15px;border-radius:18px;background:rgba(127,127,127,.065);color:var(--text);line-height:1.65;font-family:ui-monospace,Consolas,monospace;font-size:.88rem}
.progress{height:8px;background:rgba(127,127,127,.15);border-radius:99px;overflow:hidden;margin-top:10px}.progress div{height:100%;width:0;background:var(--accent);transition:width .15s}
.note{font-size:.82rem;color:var(--muted);padding:0 16px 16px;line-height:1.5}.theme{position:fixed;right:16px;bottom:16px;width:48px;height:48px;border-radius:50%;z-index:10}
@media(max-width:970px){.layout{grid-template-columns:1fr}.controls{grid-template-columns:1fr}}
@media(max-width:600px){.wrapper{padding:12px}.header{padding:18px}.header h1{font-size:1.45rem}.metrics{grid-template-columns:1fr}canvas{height:300px}}
</style>
</head>
<body>
<div class="wrapper">
  <header class="header">
    <div><h1>Sistema de Monitoramento de Vibração</h1><p>ESP32 + ADXL345 • UNIENSINO • aquisição e FFT em tempo real</p></div>
    <div id="status" class="status">Conectando...</div>
  </header>

  <div class="toolbar">
    <input id="operador" class="input" maxlength="30" placeholder="Nome do operador">
    <button id="salvarOperador">Definir operador</button>
    <select id="eixoFft" class="select">
      <option value="AUTO">FFT automática</option><option value="X">FFT eixo X</option>
      <option value="Y">FFT eixo Y</option><option value="Z">FFT eixo Z</option>
    </select>
    <button id="calibrar" class="primary">Calibrar repouso</button>
    <button id="zerarBaseline">Reaprender normal</button>
    <button onclick="location.href='/csv'">Exportar CSV</button>
    <button onclick="location.href='/eventos.csv'">Log de eventos</button>
  </div>

  <main class="layout">
    <div class="stack">
      <section class="card">
        <div class="card-head"><h2>Osciloscópio</h2><p>Sinal dinâmico real, sem zeragem artificial das amostras pequenas.</p></div>
        <div class="controls">
          <div class="control"><label>Janela visível <span id="janelaLabel">240</span></label><input id="janela" type="range" min="60" max="512" value="240"></div>
          <div class="control"><label>Ganho visual <span id="ganhoLabel">1.0×</span></label><input id="ganho" type="range" min="0.5" max="8" step="0.1" value="1"></div>
        </div>
        <div class="chips"><button id="tx" class="chip active">X</button><button id="ty" class="chip active">Y</button><button id="tz" class="chip active">Z</button><button id="pausa" class="chip">Pausar</button></div>
        <div class="canvas-wrap"><canvas id="tempo"></canvas></div>
      </section>

      <section class="card">
        <div class="card-head"><h2>FFT</h2><p id="fftDescricao">Espectro em dB.</p></div>
        <div class="canvas-wrap"><canvas id="fft"></canvas></div>
      </section>
    </div>

    <aside class="stack">
      <section class="card">
        <div class="card-head"><h2>Diagnóstico</h2><p>Métricas calculadas no ESP32.</p></div>
        <div class="metrics">
          <div class="metric wide"><div class="label">Estado</div><div id="estado" class="value">Calibrando</div><div class="progress"><div id="barraCal"></div></div></div>
          <div class="metric"><div class="label">Frequência dominante</div><div id="freq" class="value">0.00 Hz</div></div>
          <div class="metric"><div class="label">Eixo analisado</div><div id="eixo" class="value">X</div></div>
          <div class="metric"><div class="label">RMS total</div><div id="rmsTotal" class="value">0.000</div></div>
          <div class="metric"><div class="label">RMS do eixo</div><div id="rmsEixo" class="value">0.000</div></div>
          <div class="metric"><div class="label">Pico a pico</div><div id="pp" class="value">0.000</div></div>
          <div class="metric"><div class="label">Magnitude FFT</div><div id="mag" class="value">0.000</div></div>
          <div class="metric"><div class="label">Resolução FFT</div><div id="resolucao" class="value">1.563 Hz</div></div>
          <div class="metric"><div class="label">Eventos</div><div id="eventos" class="value">0</div></div>
        </div>
        <div class="details"><pre id="detalhes">Aguardando dados...</pre></div>
        <div class="note">Calibre com o sensor parado e já fixado na posição de uso. O gráfico preserva o ruído real; o limiar é usado somente para classificar repouso.</div>
      </section>
    </aside>
  </main>
</div>
<button id="tema" class="theme">◐</button>
<script>
const MAX_VISUAL=512;
let ws=null,x=[],y=[],z=[],fft=[];
let pausado=false,mostrarX=true,mostrarY=true,mostrarZ=true;
let janela=240,ganho=1,renderPendente=false,ultimoPacote=0;
let dados={fs:400,nyquist:200,resolucao:1.5625,eixo:'X',freq:0};
const $=id=>document.getElementById(id);

function status(texto,cor){$('status').textContent=texto;$('status').style.color=cor}
function adicionar(destino,novos){if(!Array.isArray(novos))return;for(const v of novos)destino.push(Number(v));if(destino.length>MAX_VISUAL)destino.splice(0,destino.length-MAX_VISUAL)}
function render(){if(renderPendente)return;renderPendente=true;requestAnimationFrame(()=>{renderPendente=false;desenharTempo();desenharFft()})}
function conectar(){
  ws=new WebSocket(`ws://${location.hostname}:81/`);
  ws.onopen=()=>{status('Conectado','#16a34a');ultimoPacote=Date.now()};
  ws.onclose=()=>{status('Reconectando...','#f59e0b');setTimeout(conectar,1200)};
  ws.onerror=()=>{try{ws.close()}catch(e){}};
  ws.onmessage=ev=>{
    ultimoPacote=Date.now();let d;try{d=JSON.parse(ev.data)}catch(e){return}
    if(pausado)return;
    if(d.snapshot){x=[];y=[];z=[]}
    adicionar(x,d.x);adicionar(y,d.y);adicionar(z,d.z);
    if(d.fftNova&&Array.isArray(d.fft))fft=d.fft.map(Number);
    dados={...dados,...d};atualizarPainel();render();
  };
}
function enviar(cmd){if(ws&&ws.readyState===1)ws.send(cmd)}
function ultimo(arr){return arr.length?arr[arr.length-1]:0}
function atualizarPainel(){
  const calibrando=!!dados.calibrando,repouso=!!dados.repouso;
  let estado=calibrando?`CALIBRANDO ${Number(dados.progressoCal||0).toFixed(0)}%`:(repouso?'REPOUSO':String(dados.diagnostico||'VIBRAÇÃO'));
  $('estado').textContent=estado;$('estado').style.color=estado.includes('ALERTA')?'#dc2626':(repouso?'#64748b':'#16a34a');
  $('barraCal').style.width=calibrando?`${dados.progressoCal||0}%`:'100%';
  $('freq').textContent=`${Number(dados.freq||0).toFixed(2)} Hz`;$('eixo').textContent=dados.eixo||'X';
  $('rmsTotal').textContent=Number(dados.rmsTotal||0).toFixed(4);$('rmsEixo').textContent=Number(dados.rmsEixo||0).toFixed(4);
  $('pp').textContent=Number(dados.ppEixo||0).toFixed(4);$('mag').textContent=Number(dados.mag||0).toFixed(4);
  $('resolucao').textContent=`${Number(dados.resolucao||0).toFixed(3)} Hz`;$('eventos').textContent=dados.eventos||0;
  $('fftDescricao').textContent=`Eixo ${dados.eixo||'X'} • Fs ${dados.fs||0} Hz • Nyquist ${Number(dados.nyquist||0).toFixed(0)} Hz`;
  $('detalhes').textContent=`Operador: ${dados.operador||'Desconhecido'}\nX: ${ultimo(x).toFixed(4)} m/s²   RMS: ${Number(dados.rmsX||0).toFixed(4)}   PP: ${Number(dados.ppX||0).toFixed(4)}\nY: ${ultimo(y).toFixed(4)} m/s²   RMS: ${Number(dados.rmsY||0).toFixed(4)}   PP: ${Number(dados.ppY||0).toFixed(4)}\nZ: ${ultimo(z).toFixed(4)} m/s²   RMS: ${Number(dados.rmsZ||0).toFixed(4)}   PP: ${Number(dados.ppZ||0).toFixed(4)}\nLimiar RMS: ${Number(dados.limiarRms||0).toFixed(4)}\nLimiar PP: ${Number(dados.limiarPP||0).toFixed(4)}`;
}
function prepararCanvas(c){const dpr=devicePixelRatio||1,r=c.getBoundingClientRect();if(c.width!==Math.round(r.width*dpr)||c.height!==Math.round(r.height*dpr)){c.width=Math.round(r.width*dpr);c.height=Math.round(r.height*dpr)}const ctx=c.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);return{ctx,w:r.width,h:r.height}}
function cor(css){return getComputedStyle(document.body).getPropertyValue(css).trim()}
function grade(ctx,l,t,w,h,v=10,hor=6){ctx.lineWidth=1;ctx.strokeStyle=cor('--grid');for(let i=0;i<=v;i++){let px=l+i*w/v;ctx.beginPath();ctx.moveTo(px,t);ctx.lineTo(px,t+h);ctx.stroke()}for(let i=0;i<=hor;i++){let py=t+i*h/hor;ctx.beginPath();ctx.moveTo(l,py);ctx.lineTo(l+w,py);ctx.stroke()}}
function traco(ctx,a,c,l,t,w,h,lim){if(a.length<2)return;ctx.beginPath();ctx.strokeStyle=c;ctx.lineWidth=1.8;for(let i=0;i<a.length;i++){let px=l+i*w/(a.length-1),py=t+h/2-(a[i]/lim)*(h/2);i?ctx.lineTo(px,py):ctx.moveTo(px,py)}ctx.stroke()}
function desenharTempo(){
  const c=$('tempo'),{ctx,w,h}=prepararCanvas(c);ctx.clearRect(0,0,w,h);const l=56,t=15,r=15,b=25,pw=w-l-r,ph=h-t-b;grade(ctx,l,t,pw,ph);
  const ax=x.slice(-janela),ay=y.slice(-janela),az=z.slice(-janela),todos=[];if(mostrarX)todos.push(...ax);if(mostrarY)todos.push(...ay);if(mostrarZ)todos.push(...az);
  let pico=.1;for(const v of todos)pico=Math.max(pico,Math.abs(v));let lim=Math.max(.08,pico*1.18)/ganho;
  ctx.fillStyle=cor('--muted');ctx.font='11px sans-serif';ctx.textAlign='right';for(let i=0;i<=6;i++){let val=lim-(2*lim*i/6);ctx.fillText(val.toFixed(2),l-7,t+ph*i/6+4)}
  if(mostrarX)traco(ctx,ax,cor('--x'),l,t,pw,ph,lim);if(mostrarY)traco(ctx,ay,cor('--y'),l,t,pw,ph,lim);if(mostrarZ)traco(ctx,az,cor('--z'),l,t,pw,ph,lim);
}
function desenharFft(){
  const c=$('fft'),{ctx,w,h}=prepararCanvas(c);ctx.clearRect(0,0,w,h);const l=54,t=15,r=15,b=35,pw=w-l-r,ph=h-t-b;grade(ctx,l,t,pw,ph,8,5);if(!fft.length)return;
  let min=-90,max=-10;for(const v of fft)if(isFinite(v)){min=Math.min(min,v);max=Math.max(max,v)}max=Math.ceil(max/10)*10;min=Math.min(-50,Math.floor(min/10)*10);if(max-min<30)min=max-30;
  const bw=pw/fft.length;ctx.fillStyle=cor('--fft');for(let i=0;i<fft.length;i++){let n=(fft[i]-min)/(max-min),bh=Math.max(0,Math.min(1,n))*ph;ctx.fillRect(l+i*bw,t+ph-bh,Math.max(1,bw-1),bh)}
  ctx.fillStyle=cor('--muted');ctx.font='11px sans-serif';ctx.textAlign='center';for(let i=0;i<=8;i++){let hz=(dados.nyquist||200)*i/8;ctx.fillText(`${hz.toFixed(0)} Hz`,l+pw*i/8,t+ph+17)}
  if(dados.freq>0){let px=l+(dados.freq/(dados.nyquist||200))*pw;ctx.strokeStyle=cor('--x');ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(px,t);ctx.lineTo(px,t+ph);ctx.stroke()}
}
$('janela').oninput=e=>{janela=Number(e.target.value);$('janelaLabel').textContent=janela;render()};
$('ganho').oninput=e=>{ganho=Number(e.target.value);$('ganhoLabel').textContent=`${ganho.toFixed(1)}×`;render()};
for(const [id,nome] of [['tx','X'],['ty','Y'],['tz','Z']])$(id).onclick=()=>{if(nome==='X')mostrarX=!mostrarX;if(nome==='Y')mostrarY=!mostrarY;if(nome==='Z')mostrarZ=!mostrarZ;$(id).classList.toggle('active',nome==='X'?mostrarX:nome==='Y'?mostrarY:mostrarZ);render()};
$('pausa').onclick=()=>{pausado=!pausado;$('pausa').textContent=pausado?'Retomar':'Pausar';$('pausa').classList.toggle('active',pausado)};
$('calibrar').onclick=()=>{if(confirm('Mantenha o sensor totalmente parado durante a calibração.'))enviar('CALIBRAR')};
$('zerarBaseline').onclick=()=>enviar('ZERAR_BASELINE');
$('eixoFft').onchange=e=>enviar(`EIXO:${e.target.value}`);
$('salvarOperador').onclick=()=>enviar(`OPERADOR:${$('operador').value}`);
$('tema').onclick=()=>{document.body.classList.toggle('dark');localStorage.setItem('tema-vib',document.body.classList.contains('dark')?'1':'0');render()};
if(localStorage.getItem('tema-vib')==='1')document.body.classList.add('dark');
setInterval(()=>{if(Date.now()-ultimoPacote>2500)status('Sem dados','#dc2626')},1000);
window.onresize=render;conectar();render();
</script>
</body>
</html>
)rawliteral";

void paginaPrincipal() {
  server.send_P(200, "text/html; charset=utf-8", PAGINA_HTML);
}

// =============================================================================
// CSV
// =============================================================================

void enviarCSV() {
  server.sendHeader("Content-Disposition",
                    "attachment; filename=medicao_vibracao.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv; charset=utf-8", "");
  server.sendContent(
      "tempo_s;x_ms2;y_ms2;z_ms2;operador;freq_hz;rms_total;eixo_fft\r\n");

  const uint16_t quantidade = amostrasValidas;
  const int primeiro = (indiceBuffer - quantidade + TAM_BUFFER) % TAM_BUFFER;

  for (uint16_t i = 0; i < quantidade; i++) {
    const int indice = (primeiro + i) % TAM_BUFFER;
    String linha;
    linha.reserve(120);
    linha += String(bufferTempoUs[indice] / 1000000.0f, 6) + ';';
    linha += String(bufferX[indice], 5) + ';';
    linha += String(bufferY[indice], 5) + ';';
    linha += String(bufferZ[indice], 5) + ';';
    linha += String(operadorAtual) + ';';
    linha += String(frequenciaDominante, 3) + ';';
    linha += String(rmsTotal, 5) + ';';
    linha += String(eixoAnalise) + "\r\n";
    server.sendContent(linha);
  }

  server.sendContent("");
}

void enviarEventosCSV() {
  server.sendHeader("Content-Disposition",
                    "attachment; filename=eventos_vibracao.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv; charset=utf-8", "");
  server.sendContent("data_hora;operador;tipo;rms;frequencia_hz;pico_pico\r\n");

  const uint32_t quantidade = min(totalEventos, (uint32_t)MAX_EVENTOS);
  const uint32_t primeiroEvento = totalEventos > MAX_EVENTOS
      ? totalEventos - MAX_EVENTOS
      : 0;

  for (uint32_t i = 0; i < quantidade; i++) {
    const uint8_t indice = (primeiroEvento + i) % MAX_EVENTOS;
    String linha;
    linha.reserve(130);
    linha += String(eventos[indice].dataHora) + ';';
    linha += String(eventos[indice].operador) + ';';
    linha += String(eventos[indice].tipo) + ';';
    linha += String(eventos[indice].rms, 5) + ';';
    linha += String(eventos[indice].frequencia, 3) + ';';
    linha += String(eventos[indice].picoPico, 5) + "\r\n";
    server.sendContent(linha);
  }

  server.sendContent("");
}

// =============================================================================
// SETUP E LOOP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(150);

  const float dt = 1.0f / SAMPLING_FREQ;
  const float rcHP = 1.0f / (2.0f * PI * FC_PASSA_ALTA_HZ);
  alphaHP = rcHP / (rcHP + dt);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!accel.begin()) {
    Serial.println("ERRO: ADXL345 nao encontrado no barramento I2C.");
    while (true) delay(1000);
  }

  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_400_HZ);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_SENHA);

  Serial.print("Conectando ao Wi-Fi");
  const uint32_t inicioWifi = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicioWifi < 20000UL) {
    delay(250);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWi-Fi nao conectado. Reinicie apos conferir SSID e senha.");
  }

  // Fuso horario de Sao Paulo atualmente UTC-3, sem horario de verao.
  configTzTime("BRT3", "pool.ntp.org", "time.google.com");

  server.on("/", HTTP_GET, paginaPrincipal);
  server.on("/csv", HTTP_GET, enviarCSV);
  server.on("/eventos.csv", HTTP_GET, enviarEventosCSV);
  server.onNotFound([]() {
    server.send(404, "text/plain; charset=utf-8", "Rota nao encontrada");
  });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(eventoWebSocket);

  zerarProcessamento();
  iniciarCalibracao();
}

void loop() {
  // A aquisicao/calibracao vem antes das tarefas de rede.
  if (calibrando) tickCalibracao();
  else tickAquisicao();

  webSocket.loop();
  server.handleClient();

  const uint32_t agoraMs = millis();

  if (!calibrando && calibrado &&
      agoraMs - ultimaFftMs >= INTERVALO_FFT_MS) {
    ultimaFftMs = agoraMs;
    processarFFT();
  }

  if (agoraMs - ultimoEnvioMs >= INTERVALO_ENVIO_MS) {
    ultimoEnvioMs = agoraMs;
    enviarIncremental();
  }
}
