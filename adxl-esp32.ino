// =============================================================================
//  SISTEMA DE MONITORAMENTO DE VIBRAÇÃO
//  Hardware : ESP32-C3 Super Mini + ADXL345 (modo I2C)
//  Projeto  : Engenharia Eletrônica — UNIENSINO
//  Autores  : Clayton Duarte, Fabiano Moor, Mateus Alves, Peterson Andrade
// =============================================================================

// =============================================================================
// SEÇÃO 1 — LIGAÇÕES FÍSICAS (DIAGRAMA DE PINOS)
// =============================================================================
//
//  ESP32                ──────►  ADXL345
//  GND                  ──────►  GND
//  3.3V                 ──────►  VCC
//  3.3V                 ──────►   CS     (força modo I2C)
//  GND                  ──────►  SDO     (endereço I2C = 0x53)
//  G21                   ──────►  SDA
//  G22                   ──────►  SCL
//
// =============================================================================


// =============================================================================
// SEÇÃO 2 — BIBLIOTECAS
// =============================================================================

#include <WiFi.h>               // Conexão Wi-Fi
#include <WebServer.h>          // Servidor HTTP (porta 80)
#include <WebSocketsServer.h>   // WebSocket em tempo real (porta 81)
#include <Wire.h>               // Comunicação I2C
#include <Adafruit_Sensor.h>    // Abstração de sensores Adafruit
#include <Adafruit_ADXL345_U.h> // Driver do acelerômetro ADXL345
#include <arduinoFFT.h>         // Transformada Rápida de Fourier
#include <ArduinoJson.h>        // Serialização JSON para WebSocket
#include <time.h>               // Sincronização de horário via NTP


// =============================================================================
// SEÇÃO 3 — CONFIGURAÇÃO WI-FI
// =============================================================================

const char* ssid     = "Fabiano D. Moor";
const char* password = "123456789";
// Alternativa: use #include "secrets.h" para não expor credenciais no código


// =============================================================================
// SEÇÃO 4 — SERVIDORES (HTTP + WEBSOCKET)
// =============================================================================

WebServer server(80);                    // Serve a página HTML
WebSocketsServer webSocket = WebSocketsServer(81); // Envia dados em tempo real


// =============================================================================
// SEÇÃO 5 — SENSOR ADXL345
// =============================================================================

// O número (12345) é apenas um ID interno da biblioteca Adafruit
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


// =============================================================================
// SEÇÃO 6 — BUFFER CIRCULAR DE AMOSTRAS
// =============================================================================
//
//  Armazena as últimas 256 leituras de cada eixo + vetor de aceleração total.
//  O índice avança em círculo: quando chega em 255, volta para 0.
//  Isso garante que sempre temos os dados mais recentes sem alocar memória nova.

#define TAM 256

float bufferX[TAM];       // Eixo X filtrado
float bufferY[TAM];       // Eixo Y filtrado
float bufferZ[TAM];       // Eixo Z filtrado
float bufferA[TAM];       // Magnitude total (√X²+Y²+Z²)
unsigned long bufferT[TAM]; // Timestamp de cada amostra (millis)
char bufferOp[TAM][32];   // Nome do operador em cada amostra

int indexBuffer = 0;      // Posição atual no buffer circular

#define FFT_SAMPLES   256
#define SAMPLING_FREQ 800

// =============================================================================
// SEÇÃO 7 — VARIÁVEIS DE FILTRAGEM DO SINAL
// =============================================================================

// ── Saídas filtradas (usadas no buffer e na FFT) ──────────────────────────
float xFiltrado = 0, yFiltrado = 0, zFiltrado = 0;
float aFiltrado = 0;

// ── Filtro Passa-Alta (remove gravidade / componente DC) ──────────────────
//  Equação: HP[n] = α × (HP[n-1] + X[n] − X[n-1])
//  Quanto mais próximo de 1, menor a frequência de corte (remove menos)
float alphaHP = 0.95;
float xHP = 0, yHP = 0, zHP = 0;
float xPrev = 0, yPrev = 0, zPrev = 0;

// ── Filtro Passa-Baixa / EMA (suaviza ruído de alta frequência) ───────────
//  α = 0.50 → corte ≈ 88 Hz  → ideal para motores 1800–3600 RPM
//  α = 0.70 → corte ≈ 148 Hz → ideal para motores acima de 3600 RPM
//  α = 0.30 → corte ≈ 44 Hz  → ideal para motores abaixo de 1800 RPM
float alphaLPF = 0.50;
float freqMotorEma = 0; // EMA da frequência dominante (para ajuste automático)

float ultimoX = 0, ultimoY = 0, ultimoZ = 0, ultimoA = 0;

// ── Filtro Anti-Aliasing (aplicado antes de tudo, no sinal bruto) ─────────
//  Evita que frequências acima de Nyquist contaminem o espectro da FFT
const float fcAntiAliasing = 200.0; // Hz
float aaX = 0, aaY = 0, aaZ = 0;

// Coeficiente pré-calculado para o anti-aliasing (EMA de 1ª ordem)
const float alphaAA = (1.0f / SAMPLING_FREQ) /
  (1.0f / (2.0f * PI * fcAntiAliasing) + 1.0f / SAMPLING_FREQ);


// =============================================================================
// SEÇÃO 8 — CONFIGURAÇÃO DA FFT
// =============================================================================
//
//  FFT_SAMPLES : número de pontos por janela (potência de 2, máx. 256 no ESP32-C3)
//  SAMPLING_FREQ: taxa de amostragem do sensor (configurada no ADXL345)
//  Resolução   : SAMPLING_FREQ / FFT_SAMPLES = 800/256 ≈ 3.125 Hz/bin
//  Nyquist     : SAMPLING_FREQ / 2 = 400 Hz (frequência máxima detectável)

double vReal[FFT_SAMPLES];          // Parte real da FFT (recebe as amostras)
double vImag[FFT_SAMPLES];          // Parte imaginária (zerada antes de cada FFT)
double espectro[FFT_SAMPLES / 2];   // Magnitudes do espectro (128 bins)

ArduinoFFT<double> FFT(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQ);

// ── Resultados da FFT ─────────────────────────────────────────────────────
float freqDominante    = 0;     // Frequência do pico mais alto (Hz)
float magnitudeDominante = 0;   // Magnitude desse pico
String eixoFFTDominante = "Y";  // Eixo usado na última FFT ("X","Y","Z","Repouso")
String nomeEixoAnalise  = "Y";  // Mesmo que acima (exibido na interface)

float rmsY = 0, picoPicoY = 0;        // RMS e pico-a-pico do eixo Y
float rmsEixo = 0, picoPicoEixo = 0;  // RMS e pico-a-pico do eixo dominante
float rmsA = 0, picoPicoA = 0;        // RMS e pico-a-pico da aceleração total

float resolucaoFFT = (float)SAMPLING_FREQ / FFT_SAMPLES; // Hz por bin
float freqNyquist  = (float)SAMPLING_FREQ / 2.0;        // Frequência máxima

// ── Seleção de eixo para FFT ──────────────────────────────────────────────
bool modoFFTManual = false;  // false = automático (eixo com maior vibração)
char eixoFFTManual = 'X';
bool usarFFT_X = true, usarFFT_Y = true, usarFFT_Z = true;


// =============================================================================
// SEÇÃO 9 — CALIBRAÇÃO DE REPOUSO
// =============================================================================
//
//  Coleta 800 amostras com o sensor parado para calcular:
//   - Offsets (médias de cada eixo) → usados para zerar a leitura
//   - Ruído (desvio padrão) → usado para definir o limiar de repouso
//  Durante a calibração, o buffer principal não é alimentado.

const float LIMIAR_REPOUSO_FFT = 0.20; // Limiar padrão antes de calibrar

bool  calibrado = false;
float offsetX = 0, offsetY = 0, offsetZ = 0;
float ruidoX  = 0, ruidoY  = 0, ruidoZ  = 0;
float limiarRepousoCalibrado = 0.20;

// Variáveis internas da calibração assíncrona
bool  calibrando      = false;
int   calibracaoIndex = 0;
unsigned long tCalibracao = 0;
const int AMOSTRAS_CAL = 800;
double _somaX, _somaY, _somaZ;
double _somaQuadX, _somaQuadY, _somaQuadZ;


// =============================================================================
// SEÇÃO 10 — DETECÇÃO DE ANOMALIAS (BASELINE + Z-SCORE)
// =============================================================================
//
//  Após a calibração, o sistema aprende o comportamento normal do motor:
//   - EMA do RMS → média exponencial móvel (α = 0.02)
//   - EMA da variância → idem
//  Se o RMS atual se afastar mais de ZSCORE_LIM desvios padrões da média,
//  o evento é registrado no log de anomalias.

struct Evento {
  char  dataHora[20];  // "DD/MM/AAAA HH:MM:SS"
  char  operador[32];
  float rms;
  float freq;
  float picoPico;
  char  tipo[20];      // Ex: "ANOMALIA_RMS"
};

#define MAX_EVENTOS 50
Evento logEventos[MAX_EVENTOS];
int    numEventos = 0;

// Parâmetros do modelo de baseline
float       emaRMS     = 0;
float       emaVar     = 0;
const float alphaML    = 0.02;
const float ZSCORE_LIM = 3.5;
bool        mlPronto   = false;
int         mlAmostras = 0;
const int   ML_WARMUP  = 150; // amostras antes de ativar a detecção


// =============================================================================
// SEÇÃO 11 — CONTROLE DE TEMPORIZAÇÃO
// =============================================================================

unsigned long tLeitura = 0; // Controla a taxa de leitura do sensor (800 Hz)
unsigned long tFFT     = 0; // Controla o processamento da FFT (1×/s)
unsigned long tEnvio   = 0; // Controla o envio de dados via WebSocket (~12,5×/s)

String timestampInicioAquisicao = ""; // Horário do início da última FFT
String operadorAtual = "Desconhecido"; // Nome do operador ativo


// =============================================================================
// SEÇÃO 12 — FUNÇÕES DE FILTRAGEM
// =============================================================================

// Filtro Passa-Baixa (EMA): suaviza ruído de alta frequência
float filtroPassaBaixa(float entrada, float valorAnterior) {
  return alphaLPF * entrada + (1 - alphaLPF) * valorAnterior;
}

// Filtro Anti-Aliasing (EMA de 1ª ordem): aplicado antes de tudo
float filtroAntiAliasing(float entrada, float anterior) {
  return anterior + alphaAA * (entrada - anterior);
}

// Ajuste automático dos filtros com base na frequência dominante detectada
void ajustarFiltrosAuto() {
  if (freqDominante < 5.0) return; // ignora se não há vibração real (< 300 RPM)

  // Suaviza a frequência para evitar mudanças bruscas nos filtros
  freqMotorEma = 0.90 * freqMotorEma + 0.10 * freqDominante;

  // Passa-Baixa: corte em 3× a freq do motor (captura fundamental + 2 harmônicos)
  float fc_lpf = constrain(freqMotorEma * 3.0, 10.0, 380.0);
  alphaLPF = constrain(1.0 - exp(-2.0 * PI * fc_lpf / (float)SAMPLING_FREQ), 0.05, 0.95);

  // Passa-Alta: corte em 30% da freq do motor (remove DC, mantém vibração)
  float fc_hp = constrain(freqMotorEma * 0.3, 0.5, 20.0);
  alphaHP = constrain(1.0 / (1.0 + 2.0 * PI * fc_hp / (float)SAMPLING_FREQ), 0.90, 0.999);
}


// =============================================================================
// SEÇÃO 13 — FUNÇÕES DE TIMESTAMP E LOG
// =============================================================================

// Retorna a data/hora atual formatada (requer NTP sincronizado)
String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "??/??/???? ??:??:??";
  char buf[20];
  strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &timeinfo);
  return String(buf);
}

// Salva um evento no log circular de anomalias
void registrarEvento(float rms, float freq, float picoPico, const char* tipo) {
  int idx = numEventos % MAX_EVENTOS;
  String ts = getTimestamp();
  ts.toCharArray(logEventos[idx].dataHora, 20);
  operadorAtual.toCharArray(logEventos[idx].operador, 32);
  logEventos[idx].rms      = rms;
  logEventos[idx].freq     = freq;
  logEventos[idx].picoPico = picoPico;
  strncpy(logEventos[idx].tipo, tipo, 20);
  numEventos++;
}

// Atualiza o modelo de baseline e verifica anomalias por Z-score
void atualizarML(float rms, float freq, float picoPico) {
  if (!calibrado) return;
  mlAmostras++;

  // Fase de aquecimento: calcula apenas a média
  if (!mlPronto) {
    emaRMS = emaRMS + (rms - emaRMS) * (1.0f / mlAmostras);
    if (mlAmostras >= ML_WARMUP) {
      emaVar = emaRMS * emaRMS * 0.01f; // variância inicial estimada em 1% da média²
      mlPronto = true;
    }
    return;
  }

  // Fase ativa: atualiza média e variância, verifica Z-score
  float diff   = rms - emaRMS;
  emaVar       = (1 - alphaML) * (emaVar + alphaML * diff * diff);
  float sigma  = sqrt(emaVar);
  float zscore = (sigma > 0.001f) ? abs(diff) / sigma : 0;

  if (zscore > ZSCORE_LIM) {
    registrarEvento(rms, freq, picoPico, "ANOMALIA_RMS");
  }

  emaRMS = (1 - alphaML) * emaRMS + alphaML * rms;
}


// =============================================================================
// SEÇÃO 14 — LEITURA E FILTRAGEM DO SENSOR (800 Hz)
// =============================================================================
//
//  Cadeia de processamento aplicada a cada amostra:
//   1. Leitura bruta do ADXL345
//   2. Subtração do offset de calibração
//   3. Filtro anti-aliasing (EMA suave)
//   4. Filtro passa-alta (remove gravidade / DC)
//   5. Filtro passa-baixa (suaviza ruído de alta frequência)
//   6. Zeragem abaixo do limiar de repouso (supressão de ruído)
//   7. Cálculo da magnitude total e armazenamento no buffer

void atualizarBuffer() {
  sensors_event_t event;
  accel.getEvent(&event);

  // 1–2. Bruto corrigido pelo offset
  float xBruto = event.acceleration.x - offsetX;
  float yBruto = event.acceleration.y - offsetY;
  float zBruto = event.acceleration.z - offsetZ;

  // 3. Anti-aliasing
  aaX = filtroAntiAliasing(xBruto, aaX);
  aaY = filtroAntiAliasing(yBruto, aaY);
  aaZ = filtroAntiAliasing(zBruto, aaZ);

  // 4. Passa-alta (remove componente DC / gravidade)
  xHP = alphaHP * (xHP + aaX - xPrev);
  yHP = alphaHP * (yHP + aaY - yPrev);
  zHP = alphaHP * (zHP + aaZ - zPrev);
  xPrev = aaX; yPrev = aaY; zPrev = aaZ;

  // 5. Passa-baixa (suaviza)
  xFiltrado = filtroPassaBaixa(xHP, ultimoX);
  yFiltrado = filtroPassaBaixa(yHP, ultimoY);
  zFiltrado = filtroPassaBaixa(zHP, ultimoZ);
  ultimoX = xFiltrado; ultimoY = yFiltrado; ultimoZ = zFiltrado;

  // 6. Supressão de ruído abaixo do limiar
  float limiarGrafico = calibrado ? limiarRepousoCalibrado * 0.5 : 0.03;
  if (abs(xFiltrado) < limiarGrafico) xFiltrado = 0;
  if (abs(yFiltrado) < limiarGrafico) yFiltrado = 0;
  if (abs(zFiltrado) < limiarGrafico) zFiltrado = 0;

  // 7. Magnitude total e gravação no buffer circular
  float aVibracao = sqrt(xFiltrado*xFiltrado + yFiltrado*yFiltrado + zFiltrado*zFiltrado);

  bufferX[indexBuffer] = xFiltrado;
  bufferY[indexBuffer] = yFiltrado;
  bufferZ[indexBuffer] = zFiltrado;
  bufferA[indexBuffer] = aVibracao;
  bufferT[indexBuffer] = millis();
  operadorAtual.toCharArray(bufferOp[indexBuffer], 32);

  indexBuffer = (indexBuffer + 1) % TAM;
}


// =============================================================================
// SEÇÃO 15 — CALIBRAÇÃO ASSÍNCRONA
// =============================================================================
//
//  A calibração é feita em duas funções para não bloquear o loop():
//   - iniciarCalibracao(): prepara as variáveis e ativa a flag
//   - tickCalibracao()   : coletada uma amostra por chamada (não usa delay)
//  Ao final das 800 amostras, calcula offsets e limiar de repouso.

void iniciarCalibracao() {
  calibrando      = true;
  calibracaoIndex = 0;
  _somaX = _somaY = _somaZ = 0;
  _somaQuadX = _somaQuadY = _somaQuadZ = 0;
  tCalibracao = micros();
}

void tickCalibracao() {
  if (!calibrando) return;
  if (micros() - tCalibracao < (1000000UL / SAMPLING_FREQ)) return;
  tCalibracao = micros();

  sensors_event_t event;
  accel.getEvent(&event);

  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;

  _somaX += x;    _somaY += y;    _somaZ += z;
  _somaQuadX += x*x; _somaQuadY += y*y; _somaQuadZ += z*z;
  calibracaoIndex++;

  if (calibracaoIndex >= AMOSTRAS_CAL) {
    // Calcula offsets (médias)
    offsetX = _somaX / AMOSTRAS_CAL;
    offsetY = _somaY / AMOSTRAS_CAL;
    offsetZ = _somaZ / AMOSTRAS_CAL;

    // Calcula ruído (desvio padrão)
    ruidoX = sqrt((_somaQuadX / AMOSTRAS_CAL) - offsetX * offsetX);
    ruidoY = sqrt((_somaQuadY / AMOSTRAS_CAL) - offsetY * offsetY);
    ruidoZ = sqrt((_somaQuadZ / AMOSTRAS_CAL) - offsetZ * offsetZ);

    // Limiar = 6× o maior ruído (entre 0.10 e 0.50 m/s²)
    float maiorRuido = max(ruidoX, max(ruidoY, ruidoZ));
    limiarRepousoCalibrado = constrain(maiorRuido * 6.0, 0.10, 0.50);

    // Zera todos os estados de filtragem e buffer
    aaX = aaY = aaZ = 0;
    xHP = yHP = zHP = 0;
    xPrev = yPrev = zPrev = 0;
    ultimoX = ultimoY = ultimoZ = ultimoA = 0;
    xFiltrado = yFiltrado = zFiltrado = aFiltrado = 0;

    for (int i = 0; i < TAM; i++)
      bufferX[i] = bufferY[i] = bufferZ[i] = bufferA[i] = 0;
    for (int i = 0; i < FFT_SAMPLES / 2; i++)
      espectro[i] = 0;

    freqDominante    = magnitudeDominante = 0;
    eixoFFTDominante = "Repouso";

    calibrado  = true;
    calibrando = false;
  }
}


// =============================================================================
// SEÇÃO 16 — PROCESSAMENTO FFT (1×/segundo)
// =============================================================================
//
//  Usa os últimos FFT_SAMPLES (256) valores do buffer para:
//   1. Calcular RMS e pico-a-pico de cada eixo
//   2. Detectar repouso (todos os picos abaixo do limiar)
//   3. Escolher o eixo dominante (manual ou automático)
//   4. Calcular a FFT com janela de Hamming
//   5. Extrair a frequência dominante
//   6. Ajustar filtros automaticamente
//   7. Atualizar o modelo de anomalias (ML baseline)

void processarFFT() {
  timestampInicioAquisicao = getTimestamp();

  double somaQuadraticaX = 0, somaQuadraticaY = 0;
  double somaQuadraticaZ = 0, somaQuadraticaA = 0;
  float  minX = 999999, maxX = -999999;
  float  minY = 999999, maxY = -999999;
  float  minZ = 999999, maxZ = -999999;
  float  minA = 999999, maxA = -999999;
  float  mediaX = 0, mediaY = 0, mediaZ = 0;

  // ── 1. Percorre o buffer e acumula estatísticas ──────────────────────────
  for (int i = 0; i < FFT_SAMPLES; i++) {
    int idx = (indexBuffer - FFT_SAMPLES + i + TAM) % TAM;

    float aX = bufferX[idx], aY = bufferY[idx];
    float aZ = bufferZ[idx], aA = bufferA[idx];

    mediaX += aX; mediaY += aY; mediaZ += aZ;
    somaQuadraticaX += aX * aX; somaQuadraticaY += aY * aY;
    somaQuadraticaZ += aZ * aZ; somaQuadraticaA += aA * aA;

    if (aX < minX) minX = aX; if (aX > maxX) maxX = aX;
    if (aY < minY) minY = aY; if (aY > maxY) maxY = aY;
    if (aZ < minZ) minZ = aZ; if (aZ > maxZ) maxZ = aZ;
    if (aA < minA) minA = aA; if (aA > maxA) maxA = aA;
  }

  mediaX /= FFT_SAMPLES; mediaY /= FFT_SAMPLES; mediaZ /= FFT_SAMPLES;

  float rmsX = sqrt(somaQuadraticaX / FFT_SAMPLES);
  rmsY       = sqrt(somaQuadraticaY / FFT_SAMPLES);
  float rmsZ = sqrt(somaQuadraticaZ / FFT_SAMPLES);
  rmsA       = sqrt(somaQuadraticaA / FFT_SAMPLES);

  float picoPicoX = maxX - minX;
  picoPicoY       = maxY - minY;
  float picoPicoZ = maxZ - minZ;
  picoPicoA       = maxA - minA;

  // ── 2. Detecção de repouso ───────────────────────────────────────────────
  float limiarFFT = calibrado ? limiarRepousoCalibrado : LIMIAR_REPOUSO_FFT;

  if (picoPicoX < limiarFFT && picoPicoY < limiarFFT && picoPicoZ < limiarFFT) {
    freqDominante = magnitudeDominante = 0;
    eixoFFTDominante = nomeEixoAnalise = "Repouso";
    rmsY = rmsEixo = picoPicoY = picoPicoEixo = rmsA = picoPicoA = 0;
    for (int i = 0; i < FFT_SAMPLES / 2; i++) espectro[i] = 0;
    return;
  }

  // ── 3. Seleção do eixo dominante ─────────────────────────────────────────
  char eixoEscolhido = 'X';

  if (modoFFTManual) {
    float maiorPP = -1;
    if (usarFFT_X && picoPicoX > maiorPP) { maiorPP = picoPicoX; eixoEscolhido = 'X'; }
    if (usarFFT_Y && picoPicoY > maiorPP) { maiorPP = picoPicoY; eixoEscolhido = 'Y'; }
    if (usarFFT_Z && picoPicoZ > maiorPP) {                       eixoEscolhido = 'Z'; }
  } else {
    if      (picoPicoY > picoPicoX && picoPicoY > picoPicoZ) eixoEscolhido = 'Y';
    else if (picoPicoZ > picoPicoX && picoPicoZ > picoPicoY) eixoEscolhido = 'Z';
    else                                                      eixoEscolhido = 'X';
  }

  eixoFFTDominante = nomeEixoAnalise = String(eixoEscolhido);

  if      (eixoEscolhido == 'X') { rmsEixo = rmsX; picoPicoEixo = picoPicoX; }
  else if (eixoEscolhido == 'Y') { rmsEixo = rmsY; picoPicoEixo = picoPicoY; }
  else                           { rmsEixo = rmsZ; picoPicoEixo = picoPicoZ; }

  // ── 4. Monta o vetor para a FFT (remove média para evitar pico em DC) ───
  for (int i = 0; i < FFT_SAMPLES; i++) {
    int idx = (indexBuffer - FFT_SAMPLES + i + TAM) % TAM;

    if      (eixoEscolhido == 'X') vReal[i] = bufferX[idx] - mediaX;
    else if (eixoEscolhido == 'Y') vReal[i] = bufferY[idx] - mediaY;
    else                           vReal[i] = bufferZ[idx] - mediaZ;

    vImag[i] = 0;
  }

  // ── 5. Calcula a FFT e extrai o espectro ─────────────────────────────────
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  double maior = 0;
  freqDominante = magnitudeDominante = 0;
  espectro[0] = 0;

  for (int i = 1; i < FFT_SAMPLES / 2; i++) {
    espectro[i] = vReal[i];
    if (vReal[i] > maior) {
      maior            = vReal[i];
      magnitudeDominante = vReal[i];
      freqDominante      = (i * SAMPLING_FREQ) / (float)FFT_SAMPLES;
    }
  }

  // ── 6–7. Ajusta filtros e atualiza modelo ────────────────────────────────
  ajustarFiltrosAuto();
  atualizarML(rmsA, freqDominante, picoPicoA);
}


// =============================================================================
// SEÇÃO 17 — ENVIO DE DADOS VIA WEBSOCKET (~12,5×/segundo)
// =============================================================================
//
//  Serializa os buffers X/Y/Z, o espectro FFT e todas as métricas em JSON
//  e transmite para todos os clientes conectados via broadcast.

void enviarTempoReal() {
  JsonDocument doc;

  JsonArray jx = doc["x"].to<JsonArray>();
  JsonArray jy = doc["y"].to<JsonArray>();
  JsonArray jz = doc["z"].to<JsonArray>();
  JsonArray jf = doc["fft"].to<JsonArray>();

  for (int i = 0; i < TAM; i++) {
    int idx = (indexBuffer + i) % TAM;
    jx.add(serialized(String(bufferX[idx], 2)));
    jy.add(serialized(String(bufferY[idx], 2)));
    jz.add(serialized(String(bufferZ[idx], 2)));
  }

  for (int i = 0; i < FFT_SAMPLES / 2; i++) {
    jf.add(serialized(String(espectro[i], 2)));
  }

  doc["freq"]         = freqDominante;
  doc["eixoFFT"]      = eixoFFTDominante;
  doc["rmsY"]         = rmsY;
  doc["picoPicoY"]    = picoPicoY;
  doc["rmsEixo"]      = rmsEixo;
  doc["picoPicoEixo"] = picoPicoEixo;
  doc["eixoAnalise"]  = nomeEixoAnalise;
  doc["rmsA"]         = rmsA;
  doc["picoPicoA"]    = picoPicoA;
  doc["magDominante"] = magnitudeDominante;
  doc["resolucaoFFT"] = resolucaoFFT;
  doc["nyquist"]      = freqNyquist;
  doc["samplingFreq"] = (float)SAMPLING_FREQ;

  String json;
  json.reserve(9500);
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
}


// =============================================================================
// SEÇÃO 18 — INTERFACE WEB (HTML + CSS + JavaScript)
// =============================================================================
//
//  A página completa é servida pelo ESP32 via HTTP GET em "/".
//  Recursos:
//   • Osciloscópio em tempo real (Canvas 2D)
//   • Espectro FFT em tempo real
//   • Painel de métricas (RMS, pico-a-pico, frequência dominante)
//   • Diagnóstico automático (OK / ALERTA / CRÍTICO / REPOUSO)
//   • Modo automático (ajusta janela e eixo sozinho)
//   • Modo manual (usuário controla janela, ganho e eixos)
//   • Calibração de repouso
//   • Identificação de operador
//   • Aba de aquisição (grava dados e exporta CSV + PNG)
//   • Tema claro / escuro com memória no localStorage
//   • Reconexão automática ao WebSocket

void paginaPrincipal()
{
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Monitoramento de Vibração</title>

<style>
:root{
  --bg1:#f8fafc;
  --bg2:#eef2f7;
  --card:rgba(255,255,255,0.72);
  --card-border:rgba(255,255,255,0.8);
  --text:#111827;
  --muted:#6b7280;
  --accent:#0071e3;
  --accent-soft:rgba(0,113,227,0.12);
  --x:#ff453a;
  --y:#0a84ff;
  --z:#30d158;
  --fft:#7c3aed;
  --shadow:0 20px 60px rgba(15,23,42,0.12);
  --radius:26px;
  --status-bg:rgba(255,255,255,0.92);
  --status-border:rgba(255,255,255,0.96);
  --control-bg:rgba(255,255,255,0.70);
  --control-border:rgba(255,255,255,0.9);
  --chip-bg:rgba(255,255,255,0.74);
  --chip-text:#374151;
  --chip-border:rgba(255,255,255,0.95);
  --canvas-bg1:rgba(255,255,255,0.50);
  --canvas-bg2:rgba(255,255,255,0.82);
  --canvas-border:rgba(255,255,255,0.85);
  --metric-bg:rgba(255,255,255,0.72);
  --metric-border:rgba(255,255,255,0.88);
  --header-bg1:rgba(255,255,255,0.82);
  --header-bg2:rgba(255,255,255,0.68);
  --header-border:rgba(255,255,255,0.92);
  --header-badge-bg:rgba(255,255,255,0.72);
  --header-badge-border:rgba(255,255,255,0.95);
  --header-text:#111827;
  --subtext:#374151;
  --grid-strong:rgba(17,24,39,0.18);
  --grid-mid:rgba(17,24,39,0.11);
  --grid-soft:rgba(17,24,39,0.08);
  --grid-softer:rgba(17,24,39,0.06);
  --panel-fill:rgba(255,255,255,0.18);
  --label-text:#6b7280;
  --canvas-text:#4b5563;
  --fft-tag-bg:rgba(255,255,255,0.96);
  --theme-btn-bg:rgba(255,255,255,0.92);
  --theme-btn-border:rgba(255,255,255,0.96);
  --theme-btn-shadow:0 16px 40px rgba(15,23,42,0.16);
  --slider-track:linear-gradient(90deg,#cbd5e1,#94a3b8);
}

body.dark-theme{
  --bg1:#0b1220;
  --bg2:#111827;
  --card:rgba(17,24,39,0.78);
  --card-border:rgba(255,255,255,0.08);
  --text:#f3f4f6;
  --muted:#9ca3af;
  --accent:#60a5fa;
  --accent-soft:rgba(96,165,250,0.15);
  --shadow:0 20px 60px rgba(0,0,0,0.35);
  --status-bg:rgba(17,24,39,0.92);
  --status-border:rgba(255,255,255,0.08);
  --control-bg:rgba(17,24,39,0.78);
  --control-border:rgba(255,255,255,0.08);
  --chip-bg:rgba(17,24,39,0.84);
  --chip-text:#e5e7eb;
  --chip-border:rgba(255,255,255,0.08);
  --canvas-bg1:rgba(17,24,39,0.72);
  --canvas-bg2:rgba(2,6,23,0.90);
  --canvas-border:rgba(255,255,255,0.08);
  --metric-bg:rgba(17,24,39,0.82);
  --metric-border:rgba(255,255,255,0.08);
  --header-bg1:rgba(17,24,39,0.88);
  --header-bg2:rgba(2,6,23,0.88);
  --header-border:rgba(255,255,255,0.08);
  --header-badge-bg:rgba(17,24,39,0.80);
  --header-badge-border:rgba(255,255,255,0.08);
  --header-text:#f9fafb;
  --subtext:#d1d5db;
  --grid-strong:rgba(255,255,255,0.18);
  --grid-mid:rgba(255,255,255,0.12);
  --grid-soft:rgba(255,255,255,0.09);
  --grid-softer:rgba(255,255,255,0.06);
  --panel-fill:rgba(255,255,255,0.04);
  --label-text:#9ca3af;
  --canvas-text:#d1d5db;
  --fft-tag-bg:rgba(17,24,39,0.96);
  --theme-btn-bg:rgba(17,24,39,0.95);
  --theme-btn-border:rgba(255,255,255,0.10);
  --theme-btn-shadow:0 16px 40px rgba(0,0,0,0.40);
  --slider-track:linear-gradient(90deg,#334155,#64748b);
}

*{ box-sizing:border-box; }

body{
  margin:0;
  font-family:-apple-system,BlinkMacSystemFont,"SF Pro Display","Segoe UI",sans-serif;
  color:var(--text);
  background:
    radial-gradient(circle at top left, rgba(0,113,227,0.10), transparent 24%),
    radial-gradient(circle at top right, rgba(124,58,237,0.08), transparent 20%),
    linear-gradient(180deg,var(--bg1) 0%, var(--bg2) 100%);
  transition:background 0.25s ease,color 0.25s ease;
}

.wrapper{
  max-width:1280px;
  margin:0 auto;
  padding:24px 18px 40px;
}

.theme-toggle{
  position:fixed;
  top:16px;
  left:16px;
  width:52px;
  height:52px;
  border:none;
  border-radius:50%;
  cursor:pointer;
  z-index:9999;
  background:var(--theme-btn-bg);
  border:1px solid var(--theme-btn-border);
  box-shadow:var(--theme-btn-shadow);
  backdrop-filter:blur(18px);
  -webkit-backdrop-filter:blur(18px);
  display:flex;
  align-items:center;
  justify-content:center;
  transition:transform 0.2s ease, background 0.25s ease, box-shadow 0.25s ease;
}
.theme-toggle:hover{ transform:scale(1.05); }
.theme-toggle:active{ transform:scale(0.96); }

.theme-icon{
  position:relative;
  display:block;
  width:24px;
  height:24px;
}

.sun-icon{
  border-radius:50%;
  background:#facc15;
  box-shadow:
    0 0 0 3px rgba(250,204,21,0.18),
    0 0 18px rgba(250,204,21,0.45);
}
.sun-icon::before{
  content:"";
  position:absolute;
  inset:-8px;
  border-radius:50%;
  background:
    conic-gradient(
      from 0deg,
      #f59e0b 0deg 10deg, transparent 10deg 35deg,
      #f59e0b 35deg 45deg, transparent 45deg 80deg,
      #f59e0b 80deg 90deg, transparent 90deg 125deg,
      #f59e0b 125deg 135deg, transparent 135deg 170deg,
      #f59e0b 170deg 180deg, transparent 180deg 215deg,
      #f59e0b 215deg 225deg, transparent 225deg 260deg,
      #f59e0b 260deg 270deg, transparent 270deg 305deg,
      #f59e0b 305deg 315deg, transparent 315deg 350deg,
      #f59e0b 350deg 360deg
    );
  -webkit-mask:radial-gradient(circle, transparent 0 58%, #000 60%);
  mask:radial-gradient(circle, transparent 0 58%, #000 60%);
}

.moon-icon{
  border-radius:50%;
  background:#e5e7eb;
  box-shadow:0 0 14px rgba(255,255,255,0.18);
}
.moon-icon::before{
  content:"";
  position:absolute;
  width:24px;
  height:24px;
  border-radius:50%;
  background:var(--theme-btn-bg);
  top:-2px;
  left:8px;
}

.header{
  position:relative;
  overflow:hidden;
  text-align:center;
  padding:26px 24px 22px;
  margin-bottom:18px;
  background:linear-gradient(180deg, var(--header-bg1), var(--header-bg2));
  border:1px solid var(--header-border);
  border-radius:28px;
  box-shadow:0 24px 60px rgba(15,23,42,0.10);
  backdrop-filter:blur(28px);
  -webkit-backdrop-filter:blur(28px);
}
.header::before{
  content:"";
  position:absolute;
  inset:-30% auto auto -10%;
  width:220px;
  height:220px;
  background:radial-gradient(circle, rgba(0,113,227,0.10), transparent 68%);
  pointer-events:none;
}
.header::after{
  content:"";
  position:absolute;
  inset:auto -8% -60% auto;
  width:240px;
  height:240px;
  background:radial-gradient(circle, rgba(124,58,237,0.08), transparent 70%);
  pointer-events:none;
}
.header-badge{
  position:relative;
  z-index:1;
  display:inline-flex;
  align-items:center;
  gap:8px;
  padding:8px 14px;
  border-radius:999px;
  background:var(--header-badge-bg);
  border:1px solid var(--header-badge-border);
  color:var(--muted);
  font-size:0.82rem;
  font-weight:700;
  letter-spacing:0.04em;
  margin-bottom:14px;
  box-shadow:0 10px 24px rgba(15,23,42,0.06);
}
.header .uni{
  position:relative;
  z-index:1;
  font-size:1.85rem;
  font-weight:800;
  letter-spacing:-0.04em;
  color:var(--header-text);
  margin-bottom:6px;
  line-height:1.1;
}
.header .curso{
  position:relative;
  z-index:1;
  font-size:1.04rem;
  font-weight:600;
  color:var(--subtext);
  margin-bottom:10px;
  line-height:1.4;
}
.header .nomes{
  position:relative;
  z-index:1;
  max-width:760px;
  margin:0 auto;
  font-size:0.95rem;
  color:var(--muted);
  line-height:1.7;
}

.tabs{
  display:flex;
  gap:10px;
  flex-wrap:wrap;
  margin-bottom:18px;
}
.tab-btn{
  border:none;
  border-radius:999px;
  padding:11px 16px;
  cursor:pointer;
  font-weight:700;
  font-size:0.93rem;
  background:var(--chip-bg);
  color:var(--chip-text);
  border:1px solid var(--chip-border);
  box-shadow:var(--shadow);
}
.tab-btn.active{
  background:var(--accent-soft);
  color:var(--accent);
  border-color:rgba(0,113,227,0.24);
}

.tab-page{ display:none; }
.tab-page.active{ display:block; }

.topbar{
  display:flex;
  justify-content:space-between;
  align-items:flex-start;
  gap:18px;
  flex-wrap:wrap;
  margin-bottom:18px;
}
.hero h1{
  margin:0;
  font-size:2rem;
  letter-spacing:-0.04em;
  font-weight:750;
}
.hero p{
  margin:8px 0 0;
  color:var(--muted);
  font-size:0.98rem;
}

.status{
  padding:12px 18px;
  border-radius:999px;
  background:var(--status-bg);
  border:1px solid var(--status-border);
  box-shadow:var(--shadow);
  backdrop-filter:blur(20px);
  -webkit-backdrop-filter:blur(20px);
  font-size:0.95rem;
  color:#166534;
  font-weight:700;
}

.layout{
  display:grid;
  grid-template-columns:1.55fr 1fr;
  gap:18px;
}

.card{
  background:var(--card);
  border:1px solid var(--card-border);
  border-radius:26px;
  box-shadow:var(--shadow);
  backdrop-filter:blur(24px);
  -webkit-backdrop-filter:blur(24px);
  overflow:hidden;
}
.card-header{ padding:22px 22px 10px; }
.card-title{
  margin:0;
  font-size:1.08rem;
  font-weight:700;
  letter-spacing:-0.02em;
}
.card-subtitle{
  margin:6px 0 0;
  font-size:0.92rem;
  color:var(--muted);
}
.card-body{ padding:16px 22px 22px; }

.legend{
  display:flex;
  gap:18px;
  flex-wrap:wrap;
  margin-top:12px;
}
.legend-item{
  display:flex;
  align-items:center;
  gap:8px;
  font-size:0.92rem;
  color:var(--muted);
}
.dot{
  width:12px;
  height:12px;
  border-radius:999px;
}
.dot.x{ background:var(--x); }
.dot.y{ background:var(--y); }
.dot.z{ background:var(--z); }

.controls{
  display:grid;
  grid-template-columns:repeat(2,1fr);
  gap:14px;
}
.control{
  padding:14px 16px;
  border-radius:18px;
  background:var(--control-bg);
  border:1px solid var(--control-border);
}
.control label{
  display:flex;
  justify-content:space-between;
  gap:10px;
  align-items:center;
  font-size:0.92rem;
  color:var(--subtext);
  margin-bottom:10px;
  font-weight:600;
}
.control label span{
  color:var(--accent);
  font-weight:700;
}

input[type="range"]{
  width:100%;
  appearance:none;
  height:6px;
  border-radius:999px;
  background:var(--slider-track);
  outline:none;
}
input[type="range"]::-webkit-slider-thumb{
  appearance:none;
  width:20px;
  height:20px;
  border-radius:50%;
  background:#fff;
  border:2px solid var(--accent);
  box-shadow:0 4px 14px rgba(0,113,227,0.20);
  cursor:pointer;
}

.mode-btn{
  padding:10px 18px;
  font-size:14px;
  border-radius:16px;
  max-width:220px;
}

.switches{
  display:flex;
  gap:10px;
  flex-wrap:wrap;
  margin-top:14px;
}
.chip{
  border:none;
  border-radius:999px;
  padding:10px 14px;
  cursor:pointer;
  font-weight:600;
  font-size:0.9rem;
  transition:0.2s ease;
  background:var(--chip-bg);
  color:var(--chip-text);
  border:1px solid var(--chip-border);
}
.chip.active{
  background:var(--accent-soft);
  color:var(--accent);
  border-color:rgba(0,113,227,0.24);
}

.scope{ padding:0 18px 18px; }

canvas{
  width:100%;
  height:380px;
  display:block;
  border-radius:22px;
  background:linear-gradient(180deg, var(--canvas-bg1), var(--canvas-bg2));
  border:1px solid var(--canvas-border);
}
.small canvas{ height:380px; }

.stack{
  display:grid;
  gap:18px;
}

.metrics{
  display:grid;
  grid-template-columns:repeat(2,1fr);
  gap:14px;
}
.metric{
  padding:18px;
  border-radius:20px;
  background:var(--metric-bg);
  border:1px solid var(--metric-border);
}
.metric-label{
  font-size:0.86rem;
  color:var(--muted);
  margin-bottom:8px;
}
.metric-value{
  font-size:1.7rem;
  font-weight:750;
  letter-spacing:-0.03em;
}

.axis-box{
  padding:18px;
  border-radius:20px;
  background:var(--metric-bg);
  border:1px solid var(--metric-border);
}
.axis-title{
  font-size:0.86rem;
  color:var(--muted);
  margin-bottom:10px;
}
.axis-values{
  font-family:ui-monospace,SFMono-Regular,Menlo,monospace;
  font-size:1rem;
  line-height:1.9;
  color:var(--subtext);
}

.pause-btn{
  width:100%;
  border:none;
  border-radius:18px;
  padding:14px 18px;
  cursor:pointer;
  background:linear-gradient(135deg,#111827,#374151);
  color:#fff;
  font-size:0.96rem;
  font-weight:700;
  box-shadow:0 16px 32px rgba(17,24,39,0.18);
}
.pause-btn.paused{
  background:linear-gradient(135deg,#9a3412,#f97316);
}

.mode-btn{
  max-width:280px;
  padding:12px 18px;
  margin-left:auto;
  display:block;
}
.mode-btn.manual{
  background:linear-gradient(135deg,#065f46,#10b981);
}

.action-btn{
  width:100%;
  border:none;
  border-radius:18px;
  padding:14px 18px;
  cursor:pointer;
  color:#fff;
  font-size:0.96rem;
  font-weight:700;
  background:linear-gradient(135deg,#065f46,#10b981);
  box-shadow:0 16px 32px rgba(16,185,129,0.18);
}
.action-btn.recording{
  background:linear-gradient(135deg,#991b1b,#ef4444);
  box-shadow:0 16px 32px rgba(239,68,68,0.22);
}

.footer-note{
  margin-top:10px;
  font-size:0.85rem;
  color:var(--muted);
}

.aquisicao-grid{
  display:grid;
  grid-template-columns:380px 1fr;
  gap:18px;
  align-items:stretch;
}
.aq-card-body{
  padding:20px 22px 22px;
}
.aq-status{
  padding:14px 16px;
  border-radius:18px;
  background:var(--control-bg);
  border:1px solid var(--control-border);
  color:var(--subtext);
  font-size:0.92rem;
  line-height:1.7;
}
.aq-status strong{
  color:var(--text);
}
.aq-mini-grid{
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:12px;
  margin-top:14px;
}
.aq-mini{
  padding:16px;
  border-radius:18px;
  background:var(--metric-bg);
  border:1px solid var(--metric-border);
}
.aq-mini .label{
  font-size:0.84rem;
  color:var(--muted);
  margin-bottom:8px;
}
.aq-mini .value{
  font-size:1.2rem;
  font-weight:800;
  letter-spacing:-0.03em;
}
.aq-graph-wrap{
  padding:18px;
}
#aquisicaoChart{
  height:420px;
}

@media(max-width:1100px){
  .aquisicao-grid{
    grid-template-columns:1fr;
  }
}
@media(max-width:980px){
  .layout{ grid-template-columns:1fr; }
  .controls, .metrics, .aq-mini-grid{ grid-template-columns:1fr; }
}
@media(max-width:700px){
  .header{
    padding:22px 16px 18px;
    border-radius:24px;
  }
  .header-badge{
    font-size:0.76rem;
    padding:7px 12px;
    margin-bottom:12px;
  }
  .header .uni{ font-size:1.5rem; }
  .header .curso{ font-size:0.95rem; }
  .header .nomes{
    font-size:0.88rem;
    line-height:1.6;
  }
  .theme-toggle{
    top:12px;
    left:12px;
    width:46px;
    height:46px;
  }
  .theme-icon{
    width:21px;
    height:21px;
  }
  .moon-icon::before{
    width:21px;
    height:21px;
    left:7px;
    top:-2px;
  }
}
</style>
</head>
<body>

<button id="themeToggle" class="theme-toggle" aria-label="Alternar tema" title="Alternar tema">
  <span id="themeIcon" class="theme-icon sun-icon"></span>
</button>

<div class="wrapper">

  <div class="header">
    <div class="header-badge">Projeto Acadêmico • Monitoramento Inteligente</div>
    <div class="uni">Clayton Duarte • Fabiano Moor • Mateus Alves • Peterson Andrade</div>
    <div class="curso">Engenharia Eletrônica • Análise de Vibração</div>
    <div class="nomes">UNIENSINO</div>
  </div>

  <div class="tabs">
    <button class="tab-btn active" data-tab="monitor">Monitoramento</button>
    <button class="tab-btn" data-tab="aquisicao">Aquisição de Dados</button>
  </div>

  <!-- ABA MONITORAMENTO -->
  <div class="tab-page active" id="tab-monitor">
    <div class="topbar">
      <div class="hero">
        <h1>Sistema de Monitoramento de Vibração</h1>
        <p>ESP32 + ADXL345 • visual em tempo real com comportamento de osciloscópio</p>
      </div>
      <div id="status" class="status">Conectado</div>
    </div>

    <div class="layout">
      <div class="stack">
        <div class="card">
          <div class="card-header">
            <h2 class="card-title">Osciloscópio</h2>
            <p class="card-subtitle">Ajuste o tempo horizontal, ganho vertical e escolha os eixos visíveis</p>
            <div class="legend">
              <div class="legend-item"><span class="dot x"></span>Eixo X</div>
              <div class="legend-item"><span class="dot y"></span>Eixo Y</div>
              <div class="legend-item"><span class="dot z"></span>Eixo Z</div>
            </div>
          </div>
          <div class="card-body">
            <div class="controls">
              <div class="control">
                <label>Janela de tempo <span id="timeWindowLabel">120</span></label>
                <input type="range" id="timeWindow" min="30" max="256" step="1" value="120">
              </div>
              <div class="control">
                <label>Ganho vertical <span id="verticalGainLabel">1.0x</span></label>
                <input type="range" id="verticalGain" min="0.5" max="4" step="0.1" value="1.0">
              </div>
            </div>
            <div class="switches" style="display:flex; justify-content:space-between; align-items:center;">
              <div>
                <button class="chip active" id="toggleX">X</button>
                <button class="chip active" id="toggleY">Y</button>
                <button class="chip active" id="toggleZ">Z</button>
              </div>
              <button id="modoBtn" class="pause-btn mode-btn">Modo automático</button>
            </div>
          </div>
          <div class="scope">
            <canvas id="tempo"></canvas>
          </div>
        </div>

        <div class="card small">
          <div class="card-header">
            <h2 class="card-title">FFT</h2>
            <p id="fftSubtitle" class="card-subtitle">Espectro</p>
          </div>
          <div class="scope">
            <canvas id="fft"></canvas>
          </div>
        </div>
      </div>

      <div class="stack">
        <div class="card">
          <div class="card-header">
            <h2 class="card-title">Painel</h2>
            <p class="card-subtitle">Métricas em tempo real</p>
          </div>
          <div class="card-body">
            <div class="metrics">
              <div class="metric">
                <div class="metric-label">Frequência dominante</div>
                <div class="metric-value" id="freqValue">0.00 Hz</div>
              </div>
              <div class="metric">
                <div class="metric-label">Amostras visíveis</div>
                <div class="metric-value" id="samplesValue">120</div>
              </div>
            </div>

            <div style="height:14px;"></div>

            <button id="pauseBtn" class="pause-btn">Pausar visualização</button>
            <div class="footer-note">A pausa congela apenas a interface. O ESP32 continua adquirindo e processando os dados.</div>

            <div style="height:12px;"></div>

            <button id="calibrarBtn" class="pause-btn">Calibrar repouso</button>
            <div class="footer-note">Deixe o sensor parado antes de calibrar.</div>

            <div style="height:12px;"></div>

            <div style="display:flex;align-items:center;gap:8px;">
              <input type="text" id="operadorInput" placeholder="Nome do operador"
                style="flex:1;padding:10px 12px;border-radius:14px;
                       border:1px solid var(--control-border);
                       background:var(--control-bg);color:var(--text);font-size:0.9rem">
              <button onclick="definirOperador()"
                style="padding:10px 16px;border-radius:14px;border:1px solid var(--control-border);
                       background:var(--control-bg);color:var(--text);font-size:0.9rem;cursor:pointer;
                       white-space:nowrap">
                Confirmar
              </button>
            </div>
            <div class="footer-note" id="operadorAtivo">Nenhum operador definido.</div>

            <div style="height:16px;"></div>

            <div class="axis-box">
              <div class="axis-title">Diagnóstico automático</div>
              <div style="margin-top:10px; display:flex; align-items:center; gap:10px;">
                <div id="diagnosticoLed" style="width:14px;height:14px;border-radius:50%;background:#9ca3af;"></div>
                <div id="diagnosticoStatus" style="font-weight:800;">AGUARDANDO</div>
              </div>
              <div id="diagnosticoMensagem" style="margin-top:8px; font-size:0.9rem; color:#6b7280;">Sem diagnóstico ainda.</div>
            </div>

            <div style="height:16px;"></div>

            <div class="axis-box">
              <div class="axis-title">Últimos valores dos eixos</div>
              <div class="axis-values" id="valores">X: 0.00<br>Y: 0.00<br>Z: 0.00</div>
            </div>

            <div style="height:16px;"></div>

            <button onclick="baixarCSV()" class="pause-btn">Exportar CSV</button>
            <div class="footer-note">Exportar dados para excel.</div>

            <div style="height:10px;"></div>

            <button onclick="window.open('/eventos')" class="pause-btn">Exportar anomalias</button>
            <div class="footer-note">Histórico de picos detectados com hora e operador.</div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <!-- ABA AQUISIÇÃO -->
  <div class="tab-page" id="tab-aquisicao">
    <div class="topbar">
      <div class="hero">
        <h1>Aquisição de Dados</h1>
        <p>Inicie a coleta, visualize X/Y/Z juntos e gere CSV + imagem do gráfico ao finalizar</p>
      </div>
      <div id="aqLiveBadge" class="status">Pronto para adquirir</div>
    </div>

    <div class="aquisicao-grid">
      <div class="card">
        <div class="card-header">
          <h2 class="card-title">Controle de aquisição</h2>
          <p class="card-subtitle">O mesmo botão inicia e encerra a coleta dos dados</p>
        </div>
        <div class="aq-card-body">
          <button id="aqBtn" class="action-btn">Adquirir dados</button>
          <div style="height:14px;"></div>
          <div class="aq-status" id="aqStatusText">
            <strong>Estado:</strong> aguardando início<br>
            <strong>Arquivo:</strong> CSV + imagem PNG ao terminar<br>
            <strong>Formato:</strong> compatível com Excel
          </div>
          <div class="aq-mini-grid">
            <div class="aq-mini"><div class="label">Amostras coletadas</div><div class="value" id="aqSamples">0</div></div>
            <div class="aq-mini"><div class="label">Duração</div><div class="value" id="aqDuration">0.0 s</div></div>
            <div class="aq-mini"><div class="label">Último eixo Y</div><div class="value" id="aqLastY">0.00</div></div>
            <div class="aq-mini"><div class="label">Freq. dominante</div><div class="value" id="aqLastFreq">0.00 Hz</div></div>
          </div>
          <div style="height:14px;"></div>
          <div class="footer-note">Ao finalizar, o sistema baixa automaticamente um CSV com os dados e uma imagem PNG do gráfico.</div>
        </div>
      </div>

      <div class="card">
        <div class="card-header">
          <h2 class="card-title">Gráfico da aquisição</h2>
          <p class="card-subtitle">Exibição em tempo real dos eixos X, Y e Z</p>
        </div>
        <div class="aq-graph-wrap">
          <canvas id="aquisicaoChart"></canvas>
        </div>
      </div>
    </div>
  </div>

</div>

<script>
// =====================================================================
// JAVASCRIPT — CLIENTE (roda no navegador)
// =====================================================================

// ----- Estado global -----
let ws = null;
let operadorNome = "";
let aquisicaoDataHora = "";
let tentandoReconectar = false;
let reconectarTimer = null;
let estavaConectadoAntes = false;
let houveQuedaDeConexao = false;

// Dados recebidos via WebSocket
let x = [], y = [], z = [], f = [];
let xFiltrado = [], yFiltrado = [], zFiltrado = [];

// Métricas do servidor
let freqDominante = 0, eixoFFT = "Y";
let rmsY = 0, picoPicoY = 0;
let rmsEixo = 0, picoPicoEixo = 0;
let eixoAnalise = "Y";
let rmsA = 0, picoPicoA = 0;
let magDominante = 0, resolucaoFFT = 0, nyquist = 0, samplingFreq = 0;

// Controles do osciloscópio
let paused = false;
let timeWindow = 120;
let verticalGain = 1.0;
let showX = true, showY = true, showZ = true;

// Modo automático
let autoMode = true;
let eixoDominanteAuto = "Repouso";
let rmsXAuto = 0, rmsYAuto = 0, rmsZAuto = 0;
let picoPicoAuto = 0;
let sistemaEmRepouso = true;

let ultimoPacote = Date.now();

// Aquisição de dados
let adquirindo = false;
let aquisicaoInicio = 0;
let aquisicaoDados = [];
let aquisicaoPlotX = [], aquisicaoPlotY = [], aquisicaoPlotZ = [];
const AQ_MAX_PLOT = 120;

// Limiares para detecção de repouso no cliente
const LIMIAR_RMS_REPOUSO = 0.10;
const LIMIAR_PP_REPOUSO  = 0.35;
const SUAVIZACAO_FILTRO  = 0.35;

// ----- Referências DOM -----
const timeWindowSlider  = document.getElementById("timeWindow");
const verticalGainSlider = document.getElementById("verticalGain");
const toggleX    = document.getElementById("toggleX");
const toggleY    = document.getElementById("toggleY");
const toggleZ    = document.getElementById("toggleZ");
const pauseBtn   = document.getElementById("pauseBtn");
const calibrarBtn = document.getElementById("calibrarBtn");
const modoBtn    = document.getElementById("modoBtn");
const timeWindowLabel  = document.getElementById("timeWindowLabel");
const verticalGainLabel = document.getElementById("verticalGainLabel");
const samplesValue = document.getElementById("samplesValue");
const freqValue    = document.getElementById("freqValue");
const valoresEl    = document.getElementById("valores");
const themeToggle  = document.getElementById("themeToggle");
const themeIcon    = document.getElementById("themeIcon");
const aqBtn        = document.getElementById("aqBtn");
const aqStatusText = document.getElementById("aqStatusText");
const aqSamples    = document.getElementById("aqSamples");
const aqDuration   = document.getElementById("aqDuration");
const aqLastY      = document.getElementById("aqLastY");
const aqLastFreq   = document.getElementById("aqLastFreq");
const aqLiveBadge  = document.getElementById("aqLiveBadge");

// ----- Navegação por abas -----
document.querySelectorAll(".tab-btn").forEach(btn => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".tab-btn").forEach(b => b.classList.remove("active"));
    document.querySelectorAll(".tab-page").forEach(p => p.classList.remove("active"));
    btn.classList.add("active");
    document.getElementById("tab-" + btn.dataset.tab).classList.add("active");
    ajustarCanvas();
  });
});

// ----- Tema claro/escuro -----
function atualizarIconeTema(){
  const isDark = document.body.classList.contains("dark-theme");
  themeIcon.className = isDark ? "theme-icon moon-icon" : "theme-icon sun-icon";
  themeToggle.title = isDark ? "Tema escuro" : "Tema claro";
  themeToggle.setAttribute("aria-label", isDark ? "Tema escuro" : "Tema claro");
}

function aplicarTemaSalvo(){
  if(localStorage.getItem("tema-site-vibracao") === "dark")
    document.body.classList.add("dark-theme");
  atualizarIconeTema();
}

function alternarTema(){
  document.body.classList.toggle("dark-theme");
  const isDark = document.body.classList.contains("dark-theme");
  localStorage.setItem("tema-site-vibracao", isDark ? "dark" : "light");
  atualizarIconeTema();
  drawTempo(); drawFFT(); drawAquisicao();
}
themeToggle.addEventListener("click", alternarTema);

function corVar(nome){
  return getComputedStyle(document.body).getPropertyValue(nome).trim();
}

// ----- Utilitários de sinal -----
function calcularRMS(arr){
  if(!arr.length) return 0;
  return Math.sqrt(arr.reduce((s,v) => s + v*v, 0) / arr.length);
}

function calcularPicoPico(arr){
  if(!arr.length) return 0;
  return Math.max(...arr) - Math.min(...arr);
}

function removerDC(arr){
  if(!arr.length) return [];
  const media = arr.reduce((s,v) => s+v, 0) / arr.length;
  return arr.map(v => v - media);
}

function suavizarArray(arr, alpha = 0.35){
  if(!arr.length) return [];
  const out = [arr[0]];
  for(let i = 1; i < arr.length; i++)
    out.push(alpha * arr[i] + (1 - alpha) * out[i-1]);
  return out;
}

// Remove DC e suaviza — prepara o sinal para exibição no osciloscópio
function prepararSinalVibracao(arr){
  return suavizarArray(removerDC(arr), SUAVIZACAO_FILTRO);
}

// ----- Status e badge -----
function setStatus(texto, cor){
  const el = document.getElementById("status");
  el.innerText = texto; el.style.color = cor;
}

function setAqBadge(texto, cor){
  aqLiveBadge.innerText = texto; aqLiveBadge.style.color = cor;
}

// ----- Controles do osciloscópio -----
function atualizarBotoesEixo(){
  toggleX.classList.toggle("active", showX);
  toggleY.classList.toggle("active", showY);
  toggleZ.classList.toggle("active", showZ);
}

function enviarModoFFT(){
  if(!ws || ws.readyState !== WebSocket.OPEN) return;
  if(autoMode){ ws.send("FFT_AUTO"); return; }
  if(showX && !showY && !showZ) ws.send("FFT_X");
  else if(showY && !showX && !showZ) ws.send("FFT_Y");
  else if(showZ && !showX && !showY) ws.send("FFT_Z");
  else if(showX && showY && !showZ) ws.send("FFT_XY");
  else if(showX && !showY && showZ) ws.send("FFT_XZ");
  else if(!showX && showY && showZ) ws.send("FFT_YZ");
  else ws.send("FFT_XYZ");
}

function atualizarModoControle(){
  if(autoMode){
    modoBtn.innerText = "Modo automático";
    modoBtn.classList.remove("manual");
    timeWindowSlider.disabled = verticalGainSlider.disabled = true;
    timeWindowSlider.style.opacity = verticalGainSlider.style.opacity = "0.55";
  } else {
    modoBtn.innerText = "Modo manual";
    modoBtn.classList.add("manual");
    timeWindowSlider.disabled = verticalGainSlider.disabled = false;
    timeWindowSlider.style.opacity = verticalGainSlider.style.opacity = "1";
    timeWindowLabel.innerText = timeWindow;
    verticalGainLabel.innerText = verticalGain.toFixed(1) + "x";
  }
  atualizarBotoesEixo();
}

// ----- Modo automático (ajusta janela e eixo com base no sinal) -----
function aplicarModoInteligente(){
  if(!autoMode) return;
  const n = Math.min(128, xFiltrado.length, yFiltrado.length, zFiltrado.length);
  if(n < 16) return;

  const xA = xFiltrado.slice(-n), yA = yFiltrado.slice(-n), zA = zFiltrado.slice(-n);
  rmsXAuto = calcularRMS(xA); rmsYAuto = calcularRMS(yA); rmsZAuto = calcularRMS(zA);
  const ppX = calcularPicoPico(xA), ppY = calcularPicoPico(yA), ppZ = calcularPicoPico(zA);

  let maiorRMS = rmsXAuto; eixoDominanteAuto = "X"; picoPicoAuto = ppX;
  if(rmsYAuto > maiorRMS){ maiorRMS = rmsYAuto; eixoDominanteAuto = "Y"; picoPicoAuto = ppY; }
  if(rmsZAuto > maiorRMS){ maiorRMS = rmsZAuto; eixoDominanteAuto = "Z"; picoPicoAuto = ppZ; }

  sistemaEmRepouso = (maiorRMS < LIMIAR_RMS_REPOUSO) && (picoPicoAuto < LIMIAR_PP_REPOUSO);

  if(sistemaEmRepouso){
    eixoDominanteAuto = "Repouso";
    showX = showY = showZ = true;
    picoPicoAuto = Math.max(ppX, ppY, ppZ);
    timeWindow = Math.round(timeWindow * 0.8 + 180 * 0.2);
  } else {
    showX = eixoDominanteAuto === "X";
    showY = eixoDominanteAuto === "Y";
    showZ = eixoDominanteAuto === "Z";
    let novaJanela = freqDominante > 0.5 && samplingFreq > 0
      ? Math.round((samplingFreq * 4.0) / freqDominante) : 160;
    novaJanela = Math.min(Math.max(novaJanela, 40), 240);
    timeWindow = Math.round(timeWindow * 0.7 + novaJanela * 0.3);
  }

  atualizarBotoesEixo();
  timeWindowSlider.value = timeWindow;
  timeWindowLabel.innerText = "AUTO " + timeWindow;
  samplesValue.innerText = timeWindow;
  verticalGainLabel.innerText = "AUTO";
  verticalGainSlider.value = 1.0;
  timeWindowSlider.disabled = verticalGainSlider.disabled = true;
  timeWindowSlider.style.opacity = verticalGainSlider.style.opacity = "0.55";
}

// ----- Limpa dados da tela (ao desconectar) -----
function limparDadosTela(){
  x = []; y = []; z = []; f = [];
  xFiltrado = []; yFiltrado = []; zFiltrado = [];
  freqDominante = rmsY = picoPicoY = magDominante = resolucaoFFT = nyquist = samplingFreq = 0;
  rmsXAuto = rmsYAuto = rmsZAuto = picoPicoAuto = 0;
  eixoDominanteAuto = "Repouso"; sistemaEmRepouso = true;
  valoresEl.innerHTML = "X: 0.00<br>Y: 0.00<br>Z: 0.00<br><br>Modo: " + (autoMode ? "AUTO" : "MANUAL") + "<br>Estado: Repouso<br>RMS vibração: 0.000<br>Pico a Pico: 0.000";
  freqValue.innerText = "0.00 Hz";
  drawTempo(); drawFFT();
}

// ----- WebSocket — conexão e reconexão automática -----
function agendarReconexao(){
  if(tentandoReconectar) return;
  tentandoReconectar = true;
  setStatus("Reconectando...", "#f59e0b");
  setAqBadge("Reconectando...", "#f59e0b");
  reconectarTimer = setTimeout(() => { tentandoReconectar = false; conectarWebSocket(); }, 1500);
}

function conectarWebSocket(){
  if(ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return;

  try {
    ws = new WebSocket("ws://" + location.hostname + ":81/");

    ws.onopen = () => {
      tentandoReconectar = false;
      if(reconectarTimer){ clearTimeout(reconectarTimer); reconectarTimer = null; }
      ultimoPacote = Date.now();
      setStatus("Conectado", "#16a34a");
      setAqBadge(adquirindo ? "Adquirindo dados" : "Pronto para adquirir", adquirindo ? "#dc2626" : "#16a34a");
      if(houveQuedaDeConexao){ setTimeout(() => location.reload(), 500); return; }
      estavaConectadoAntes = true;
    };

    ws.onclose = () => {
      setStatus("Desconectado", "#dc2626");
      setAqBadge("Desconectado", "#dc2626");
      if(estavaConectadoAntes) houveQuedaDeConexao = true;
      limparDadosTela();
      agendarReconexao();
    };

    ws.onerror = () => {
      setStatus("Erro na conexão", "#f59e0b");
      setAqBadge("Erro na conexão", "#f59e0b");
      try { ws.close(); } catch(e) {}
    };

    // ----- Recepção de dados -----
    ws.onmessage = (event) => {
      ultimoPacote = Date.now();
      setStatus("Conectado", "#16a34a");
      setAqBadge(adquirindo ? "Adquirindo dados" : "Pronto para adquirir", adquirindo ? "#dc2626" : "#16a34a");

      const d = JSON.parse(event.data);
      if(paused) return;

      // Atualiza arrays de dados
      x = d.x || []; y = d.y || []; z = d.z || []; f = d.fft || [];
      xFiltrado = prepararSinalVibracao(x);
      yFiltrado = prepararSinalVibracao(y);
      zFiltrado = prepararSinalVibracao(z);

      // Atualiza métricas
      freqDominante = Number(d.freq || 0);
      eixoFFT       = d.eixoFFT || "Y";
      rmsY          = Number(d.rmsY || 0);
      picoPicoY     = Number(d.picoPicoY || 0);
      rmsEixo       = Number(d.rmsEixo || 0);
      picoPicoEixo  = Number(d.picoPicoEixo || 0);
      eixoAnalise   = d.eixoAnalise || eixoFFT || "Y";
      rmsA          = Number(d.rmsA || 0);
      picoPicoA     = Number(d.picoPicoA || 0);
      magDominante  = Number(d.magDominante || 0);
      resolucaoFFT  = Number(d.resolucaoFFT || 0);
      nyquist       = Number(d.nyquist || 0);
      samplingFreq  = Number(d.samplingFreq || 0);

      // Atualiza diagnóstico
      const freq = freqDominante, rms = rmsEixo, pico = picoPicoEixo;
      let diagStatus, diagMsg, diagCor;

      if(eixoFFT === "Repouso" || freq === 0){
        diagStatus = "REPOUSO"; diagMsg = "Sem vibração significativa detectada."; diagCor = "#6b7280";
      } else if(rms > 5.0 || pico > 15.0 || (freq > 220 && (rms > 0.8 || pico > 2.5))){
        diagStatus = "CRÍTICO"; diagMsg = "Alta frequência de vibração detectada. Possível falha em rolamento, folga mecânica ou vibração severa."; diagCor = "#dc2626";
      } else if(rms > 2.0 || pico > 6.0 || (freq > 120 && (rms > 0.3 || pico > 1.0))){
        diagStatus = "ALERTA"; diagMsg = "Frequência elevada detectada. Monitorar rolamento, fixação, eixo ou acoplamento."; diagCor = "#f59e0b";
      } else {
        diagStatus = "OK"; diagMsg = "Motor dentro do padrão normal. Frequência: " + freq.toFixed(2) + " Hz | RMS: " + rms.toFixed(3) + " | Pico: " + pico.toFixed(3); diagCor = "#16a34a";
      }

      document.getElementById("diagnosticoStatus").innerText = diagStatus;
      document.getElementById("diagnosticoStatus").style.color = diagCor;
      document.getElementById("diagnosticoMensagem").innerText = diagMsg;
      document.getElementById("diagnosticoLed").style.background = diagCor;
      document.getElementById("diagnosticoLed").style.boxShadow = "0 0 12px " + diagCor;

      aplicarModoInteligente();

      // Atualiza painel de valores
      const lastX = x.length ? x[x.length-1] : 0;
      const lastY = y.length ? y[y.length-1] : 0;
      const lastZ = z.length ? z[z.length-1] : 0;

      valoresEl.innerHTML =
        "X: " + lastX.toFixed(2) + "<br>Y: " + lastY.toFixed(2) + "<br>Z: " + lastZ.toFixed(2) + "<br><br>" +
        "Modo: " + (autoMode ? "AUTO" : "MANUAL") + "<br>" +
        "Estado: " + (eixoFFT === "Repouso" ? "Repouso" : "Vibração") + "<br>" +
        "Eixo analisado: " + (eixoFFT === "Repouso" ? "Repouso" : eixoAnalise) + "<br>" +
        "RMS " + eixoAnalise + ": " + rmsEixo.toFixed(3) + "<br>" +
        "RMS Global A: " + rmsA.toFixed(3) + "<br>" +
        "Pico a Pico " + eixoAnalise + ": " + picoPicoEixo.toFixed(3) + "<br>" +
        "Pico a Pico Global A: " + picoPicoA.toFixed(3);

      freqValue.innerText = eixoFFT === "Repouso" ? "--" : freqDominante.toFixed(2) + " Hz (" + eixoFFT + ")";

      const fftSubtitle = document.getElementById("fftSubtitle");
      if(eixoFFT === "Repouso") fftSubtitle.innerText = "Sem vibração detectada";
      else if(autoMode)         fftSubtitle.innerText = "Espectro do eixo dominante (" + eixoFFT + ")";
      else                      fftSubtitle.innerText = "Espectro do eixo selecionado (" + eixoFFT + ")";

      drawTempo(); drawFFT();

      // Registra amostras durante aquisição
      if(adquirindo){
        const agora = Date.now();
        const tempoMs = agora - aquisicaoInicio;

        aquisicaoDados.push({ tempo_ms: tempoMs, x: lastX, y: lastY, z: lastZ,
          freq: freqDominante, eixoFFT, rmsY, picoPicoY, rmsA, picoPicoA });

        const lxv = xFiltrado.length ? xFiltrado[xFiltrado.length-1] : 0;
        const lyv = yFiltrado.length ? yFiltrado[yFiltrado.length-1] : 0;
        const lzv = zFiltrado.length ? zFiltrado[zFiltrado.length-1] : 0;

        aquisicaoPlotX.push(lxv); aquisicaoPlotY.push(lyv); aquisicaoPlotZ.push(lzv);
        if(aquisicaoPlotX.length > AQ_MAX_PLOT) aquisicaoPlotX.shift();
        if(aquisicaoPlotY.length > AQ_MAX_PLOT) aquisicaoPlotY.shift();
        if(aquisicaoPlotZ.length > AQ_MAX_PLOT) aquisicaoPlotZ.shift();

        aqSamples.innerText  = aquisicaoDados.length;
        aqDuration.innerText = (tempoMs / 1000).toFixed(1) + " s";
        aqLastY.innerText    = lastY.toFixed(2);
        aqLastFreq.innerText = eixoFFT === "Repouso" ? "--" : freqDominante.toFixed(2) + " Hz (" + eixoFFT + ")";
        aqStatusText.innerHTML = "<strong>Estado:</strong> aquisição em andamento<br><strong>Amostras:</strong> " + aquisicaoDados.length + "<br><strong>Exportação:</strong> CSV + PNG ao finalizar";

        drawAquisicao();
      }
    };

  } catch(e) {
    setStatus("Falha ao conectar", "#dc2626");
    setAqBadge("Falha ao conectar", "#dc2626");
    agendarReconexao();
  }
}

// ----- Eventos dos controles -----
timeWindowSlider.addEventListener("input", function(){
  if(autoMode) return;
  timeWindow = parseInt(this.value);
  timeWindowLabel.innerText = timeWindow;
  samplesValue.innerText = timeWindow;
  drawTempo();
});

verticalGainSlider.addEventListener("input", function(){
  if(autoMode) return;
  verticalGain = parseFloat(this.value);
  verticalGainLabel.innerText = verticalGain.toFixed(1) + "x";
  drawTempo();
});

toggleX.addEventListener("click", function(){ if(autoMode) return; showX = !showX; this.classList.toggle("active", showX); enviarModoFFT(); drawTempo(); });
toggleY.addEventListener("click", function(){ if(autoMode) return; showY = !showY; this.classList.toggle("active", showY); enviarModoFFT(); drawTempo(); });
toggleZ.addEventListener("click", function(){ if(autoMode) return; showZ = !showZ; this.classList.toggle("active", showZ); enviarModoFFT(); drawTempo(); });

modoBtn.addEventListener("click", function(){
  autoMode = !autoMode;
  if(!autoMode){ showX = showY = showZ = true; }
  atualizarModoControle(); enviarModoFFT(); drawTempo();
});

pauseBtn.addEventListener("click", function(){
  paused = !paused;
  pauseBtn.innerText = paused ? "Retomar visualização" : "Pausar visualização";
  pauseBtn.classList.toggle("paused", paused);
});

calibrarBtn.addEventListener("click", function(){
  if(!ws || ws.readyState !== WebSocket.OPEN) return;
  calibrarBtn.innerText = "Calibrando...";
  calibrarBtn.disabled = true;
  calibrarBtn.style.opacity = "0.6";
  ws.send("CALIBRAR");
  setTimeout(() => { calibrarBtn.innerText = "Calibrar repouso"; calibrarBtn.disabled = false; calibrarBtn.style.opacity = "1"; }, 1800);
});

aqBtn.addEventListener("click", () => { adquirindo ? terminarAquisicao() : iniciarAquisicao(); });

// ----- Aquisição de dados -----
function iniciarAquisicao(){
  adquirindo = true; aquisicaoInicio = Date.now();
  aquisicaoDataHora = new Date().toLocaleString('pt-BR');
  aquisicaoDados = []; aquisicaoPlotX = []; aquisicaoPlotY = []; aquisicaoPlotZ = [];
  aqBtn.innerText = "Terminar aquisição"; aqBtn.classList.add("recording");
  aqSamples.innerText = "0"; aqDuration.innerText = "0.0 s"; aqLastY.innerText = "0.00"; aqLastFreq.innerText = "0.00 Hz";
  aqStatusText.innerHTML = "<strong>Estado:</strong> aquisição em andamento<br><strong>Arquivo:</strong> CSV + PNG serão gerados ao finalizar<br><strong>Formato:</strong> compatível com Excel";
  setAqBadge("Adquirindo dados", "#dc2626");
  drawAquisicao();
}

function terminarAquisicao(){
  adquirindo = false;
  aqBtn.innerText = "Adquirir dados"; aqBtn.classList.remove("recording");
  aqStatusText.innerHTML = "<strong>Estado:</strong> aquisição finalizada (última coleta exibida)<br><strong>Amostras:</strong> " + aquisicaoDados.length + "<br><strong>Gráfico:</strong> último sinal capturado<br><strong>Arquivo:</strong> download iniciado";
  setAqBadge("Aquisição finalizada", "#16a34a");
  exportarCSVAdquirido();
  exportarImagemGraficoAquisicao();
}

function exportarCSVAdquirido(){
  let csv = "data_hora;operador;tempo_ms;x;y;z;freq;eixoFFT;rmsY;picoPicoY;rmsA;picoPicoA\n";
  for(let i = 0; i < aquisicaoDados.length; i++){
    const d = aquisicaoDados[i];
    csv += aquisicaoDataHora + ";" + (operadorNome||"Desconhecido") + ";" + d.tempo_ms + ";" +
      Number(d.x).toFixed(3) + ";" + Number(d.y).toFixed(3) + ";" + Number(d.z).toFixed(3) + ";" +
      Number(d.freq).toFixed(2) + ";" + d.eixoFFT + ";" +
      Number(d.rmsY).toFixed(3) + ";" + Number(d.picoPicoY).toFixed(3) + ";" +
      Number(d.rmsA).toFixed(3) + ";" + Number(d.picoPicoA).toFixed(3) + "\n";
  }
  const blob = new Blob(["\uFEFF" + csv], { type: "text/csv;charset=utf-8;" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = gerarNomeBaseArquivo() + ".csv";
  document.body.appendChild(a); a.click(); document.body.removeChild(a);
}

function exportarImagemGraficoAquisicao(){
  const link = document.createElement("a");
  link.href = document.getElementById("aquisicaoChart").toDataURL("image/png");
  link.download = gerarNomeBaseArquivo() + ".png";
  document.body.appendChild(link); link.click(); document.body.removeChild(link);
}

function gerarNomeBaseArquivo(){
  const d = new Date();
  return "aquisicao_vibracao_" + d.getFullYear() +
    String(d.getMonth()+1).padStart(2,"0") + String(d.getDate()).padStart(2,"0") + "_" +
    String(d.getHours()).padStart(2,"0") + String(d.getMinutes()).padStart(2,"0") + String(d.getSeconds()).padStart(2,"0");
}

// ----- Watchdog de conexão (verifica a cada 1s) -----
setInterval(() => {
  const agora = Date.now();
  if(agora - ultimoPacote > 2000){
    setStatus("Sem dados", "#dc2626");
    if(!adquirindo) setAqBadge("Sem dados", "#dc2626");
    if(ws && ws.readyState === WebSocket.OPEN){ try{ ws.close(); }catch(e){} }
    else if(!ws || ws.readyState === WebSocket.CLOSED) agendarReconexao();
  }
  if(adquirindo) aqDuration.innerText = ((agora - aquisicaoInicio)/1000).toFixed(1) + " s";
}, 1000);

// ----- Canvas: ajuste de resolução (DPI) -----
function ajustarCanvas(){
  document.querySelectorAll("canvas").forEach(c => {
    const dpr = window.devicePixelRatio || 1;
    const rect = c.getBoundingClientRect();
    c.width  = rect.width  * dpr;
    c.height = rect.height * dpr;
    c.getContext("2d").setTransform(dpr, 0, 0, dpr, 0, 0);
  });
  drawTempo(); drawFFT(); drawAquisicao();
}

// ----- Primitivas de desenho -----
function drawTrace(ctx, data, color, left, top, plotW, plotH, minVal, maxVal){
  if(data.length < 2) return;
  const denom = (maxVal - minVal) === 0 ? 1 : (maxVal - minVal);
  ctx.beginPath();
  ctx.strokeStyle = color; ctx.lineWidth = 2.4;
  ctx.lineJoin = "round"; ctx.lineCap = "round";
  for(let i = 0; i < data.length; i++){
    const px = left + (i / (data.length - 1)) * plotW;
    const py = top + plotH - ((data[i] - minVal) / denom) * plotH;
    i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
  }
  ctx.stroke();
}

// ----- Osciloscópio -----
function drawTempo(){
  const c = document.getElementById("tempo");
  const ctx = c.getContext("2d");
  const w = c.clientWidth, h = c.clientHeight;
  ctx.clearRect(0, 0, w, h);

  const left = 62, right = 18, top = 18, bottom = 42;
  const plotW = w - left - right, plotH = h - top - bottom;

  const visX = xFiltrado.slice(-timeWindow);
  const visY = yFiltrado.slice(-timeWindow);
  const visZ = zFiltrado.slice(-timeWindow);

  let selected = [];
  if(showX) selected = selected.concat(visX);
  if(showY) selected = selected.concat(visY);
  if(showZ) selected = selected.concat(visZ);
  if(selected.length === 0) return;

  const minRaw = Math.min(...selected), maxRaw = Math.max(...selected);
  const center = (minRaw + maxRaw) / 2.0, rawSpan = maxRaw - minRaw;

  let span;
  if(autoMode){
    if(sistemaEmRepouso) span = 0.4;
    else {
      span = Math.max(rawSpan * 1.4, 0.2);
      const spanMin = Math.max(rmsXAuto, rmsYAuto, rmsZAuto, 0.03) * 7.0;
      if(span < spanMin) span = spanMin;
    }
  } else {
    span = Math.max(rawSpan, 2.0) * verticalGain;
  }

  const minVal = center - span / 2.0, maxVal = center + span / 2.0;

  const grad = ctx.createLinearGradient(0, top, 0, top + plotH);
  grad.addColorStop(0, corVar("--canvas-bg1")); grad.addColorStop(1, corVar("--canvas-bg2"));
  ctx.fillStyle = grad; ctx.fillRect(left, top, plotW, plotH);

  const isDark = document.body.classList.contains("dark-theme");
  ctx.save(); ctx.lineWidth = 1.1;
  ctx.strokeStyle = isDark ? "rgba(255,255,255,0.12)" : "rgba(17,24,39,0.12)";
  for(let i=0;i<=10;i++){ const xx=Math.round(left+i*(plotW/10))+0.5; ctx.beginPath(); ctx.moveTo(xx,top); ctx.lineTo(xx,top+plotH); ctx.stroke(); }
  for(let i=0;i<=6;i++){ const yy=Math.round(top+i*(plotH/6))+0.5; ctx.beginPath(); ctx.moveTo(left,yy); ctx.lineTo(left+plotW,yy); ctx.stroke(); }
  ctx.strokeStyle = isDark ? "rgba(255,255,255,0.22)" : "rgba(17,24,39,0.24)"; ctx.lineWidth = 1.3;
  ctx.strokeRect(Math.round(left)+0.5, Math.round(top)+0.5, Math.round(plotW), Math.round(plotH));
  ctx.restore();

  ctx.fillStyle = corVar("--label-text"); ctx.font = "12px -apple-system, BlinkMacSystemFont, sans-serif";
  ctx.textAlign = "right"; ctx.textBaseline = "middle";
  for(let i=0;i<=6;i++){ const py=top+i*(plotH/6); ctx.fillText((maxVal-(i/6)*(maxVal-minVal)).toFixed(2), left-12, py); }
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  for(let i=0;i<=10;i++){ const px=left+i*(plotW/10); const t=samplingFreq>0?(i/10*timeWindow/samplingFreq):0; ctx.fillText(t.toFixed(2)+"s", px, top+plotH+12); }

  ctx.save(); ctx.beginPath(); ctx.rect(left, top, plotW, plotH); ctx.clip();
  if(showX) drawTrace(ctx, visX, "#ff453a", left, top, plotW, plotH, minVal, maxVal);
  if(showY) drawTrace(ctx, visY, "#0a84ff", left, top, plotW, plotH, minVal, maxVal);
  if(showZ) drawTrace(ctx, visZ, "#30d158", left, top, plotW, plotH, minVal, maxVal);
  ctx.restore();

  ctx.textAlign="left"; ctx.textBaseline="top"; ctx.fillStyle=corVar("--canvas-text"); ctx.font="12px -apple-system, BlinkMacSystemFont, sans-serif";
  ctx.fillText(autoMode ? "Modo AUTO" : "Modo MANUAL", left+12, top+16);
  ctx.fillText(eixoFFT==="Repouso" ? "Estado: Repouso" : (autoMode ? "Eixo vibração: "+eixoDominanteAuto+" | FFT: "+eixoFFT : "Eixo manual: "+eixoFFT), left+102, top+16);
}

// ----- Espectro FFT -----
function drawFFT(){
  const c = document.getElementById("fft");
  const ctx = c.getContext("2d");
  const w = c.clientWidth, h = c.clientHeight;
  ctx.clearRect(0, 0, w, h);
  if(!f.length) return;

  const left=62, right=18, top=18, bottom=42;
  const plotW=w-left-right, plotH=h-top-bottom;

  const grad = ctx.createLinearGradient(0,top,0,top+plotH);
  grad.addColorStop(0, corVar("--canvas-bg1")); grad.addColorStop(1, corVar("--canvas-bg2"));
  ctx.fillStyle=grad; ctx.fillRect(left,top,plotW,plotH);

  for(let i=0;i<=10;i++){ const xx=Math.round(left+i*(plotW/10))+0.5; ctx.beginPath(); ctx.strokeStyle=(i%2===0)?corVar("--grid-mid"):corVar("--grid-softer"); ctx.moveTo(xx,top); ctx.lineTo(xx,top+plotH); ctx.stroke(); }
  for(let i=0;i<=6;i++){ const yy=Math.round(top+i*(plotH/6))+0.5; ctx.beginPath(); ctx.strokeStyle=corVar("--grid-soft"); ctx.moveTo(left,yy); ctx.lineTo(left+plotW,yy); ctx.stroke(); }

  let maxVal=Math.max(...f); if(maxVal<=0) maxVal=1;
  const barWidth=plotW/f.length;
  for(let i=0;i<f.length;i++){
    const X=left+i*barWidth, H=(f[i]/maxVal)*plotH, Y=top+plotH-H;
    ctx.fillStyle="rgba(124,58,237,0.82)"; ctx.fillRect(X,Y,Math.max(barWidth-1,1),H);
  }
  ctx.strokeStyle=corVar("--grid-mid"); ctx.strokeRect(left,top,plotW,plotH);

  ctx.fillStyle=corVar("--label-text"); ctx.font="12px -apple-system, BlinkMacSystemFont, sans-serif";
  ctx.textAlign="right"; ctx.textBaseline="middle";
  for(let i=0;i<=6;i++){ const yy=top+i*(plotH/6); ctx.fillText((maxVal-(i/6)*maxVal).toFixed(1), left-8, yy); }
  ctx.textAlign="center"; ctx.textBaseline="top";
  for(let i=0;i<=10;i++){ const xx=left+i*(plotW/10); ctx.fillText((i/10*nyquist).toFixed(1)+" Hz", xx, top+plotH+8); }

  if(resolucaoFFT>0 && f.length>1){
    const binDom=freqDominante/resolucaoFFT;
    const xDom=left+(binDom/(f.length-1))*plotW;
    ctx.beginPath(); ctx.strokeStyle="rgba(255,69,58,0.95)"; ctx.lineWidth=2; ctx.moveTo(xDom,top); ctx.lineTo(xDom,top+plotH); ctx.stroke();
    const label=freqDominante.toFixed(2)+" Hz";
    ctx.font="12px -apple-system, BlinkMacSystemFont, sans-serif";
    const tw=ctx.measureText(label).width+16;
    let tx=Math.min(Math.max(xDom-tw/2, left), left+plotW-tw);
    const ty=top+36, th=24;
    ctx.fillStyle=corVar("--fft-tag-bg"); ctx.strokeStyle="rgba(255,69,58,0.95)"; ctx.lineWidth=1;
    ctx.beginPath();
    if(ctx.roundRect){ ctx.roundRect(tx,ty,tw,th,10); ctx.fill(); ctx.stroke(); }
    else { ctx.fillRect(tx,ty,tw,th); ctx.strokeRect(tx,ty,tw,th); }
    ctx.fillStyle="#ff453a"; ctx.textAlign="center"; ctx.textBaseline="middle"; ctx.fillText(label,tx+tw/2,ty+th/2);
  }

  ctx.textAlign="left"; ctx.textBaseline="top"; ctx.font="12px -apple-system, BlinkMacSystemFont, sans-serif"; ctx.fillStyle=corVar("--canvas-text");
  ctx.fillText("Fs: "+samplingFreq.toFixed(1)+" Hz", left+8, top+8);
  ctx.fillText("Δf: "+resolucaoFFT.toFixed(3)+" Hz/bin", left+120, top+8);
  ctx.fillText("Nyquist: "+nyquist.toFixed(1)+" Hz", left+250, top+8);
  ctx.fillText("Mag Pico: "+magDominante.toFixed(2), left+390, top+8);
}

// ----- Gráfico de aquisição -----
function drawAquisicao(){
  const c = document.getElementById("aquisicaoChart");
  const ctx = c.getContext("2d");
  const w = c.clientWidth, h = c.clientHeight;
  ctx.clearRect(0, 0, w, h);

  const left=82, right=18, top=18, bottom=52;
  const plotW=w-left-right, plotH=h-top-bottom;

  const grad=ctx.createLinearGradient(0,top,0,top+plotH);
  grad.addColorStop(0,corVar("--canvas-bg1")); grad.addColorStop(1,corVar("--canvas-bg2"));
  ctx.fillStyle=grad; ctx.fillRect(left,top,plotW,plotH);

  const isDark=document.body.classList.contains("dark-theme");
  ctx.save(); ctx.lineWidth=1.2;
  ctx.strokeStyle=isDark?"rgba(255,255,255,0.14)":"rgba(17,24,39,0.14)";
  for(let i=0;i<=10;i++){ const xx=Math.round(left+i*(plotW/10))+0.5; ctx.beginPath(); ctx.moveTo(xx,top); ctx.lineTo(xx,top+plotH); ctx.stroke(); }
  for(let i=0;i<=6;i++){ const yy=Math.round(top+i*(plotH/6))+0.5; ctx.beginPath(); ctx.moveTo(left,yy); ctx.lineTo(left+plotW,yy); ctx.stroke(); }
  ctx.strokeStyle=isDark?"rgba(255,255,255,0.24)":"rgba(17,24,39,0.28)"; ctx.lineWidth=1.4;
  ctx.strokeRect(Math.round(left)+0.5, Math.round(top)+0.5, Math.round(plotW), Math.round(plotH));
  ctx.restore();

  if(!aquisicaoPlotX.length && !aquisicaoPlotY.length && !aquisicaoPlotZ.length){
    ctx.fillStyle=corVar("--canvas-text"); ctx.font="14px -apple-system, BlinkMacSystemFont, sans-serif";
    ctx.textAlign="center"; ctx.textBaseline="middle";
    ctx.fillText("Nenhuma aquisição em andamento", w/2, h/2);
    return;
  }

  const all=[].concat(aquisicaoPlotX,aquisicaoPlotY,aquisicaoPlotZ);
  const minRaw=Math.min(...all), maxRaw=Math.max(...all);
  const center=(minRaw+maxRaw)/2.0, rawSpan=maxRaw-minRaw;
  const span=rawSpan<0.05?0.4:Math.max(rawSpan*1.4,0.2);
  const minVal=center-span/2.0, maxVal=center+span/2.0;

  ctx.fillStyle=corVar("--label-text"); ctx.font="12px -apple-system, BlinkMacSystemFont, sans-serif";
  ctx.textAlign="right"; ctx.textBaseline="middle";
  for(let i=0;i<=6;i++){ const py=top+i*(plotH/6); ctx.fillText((maxVal-(i/6)*(maxVal-minVal)).toFixed(2),left-12,py); }
  ctx.textAlign="center"; ctx.textBaseline="top";
  const total=Math.max(aquisicaoPlotX.length,aquisicaoPlotY.length,aquisicaoPlotZ.length);
  for(let i=0;i<=10;i++){ ctx.fillText(Math.round((i/10)*total), left+i*(plotW/10), top+plotH+10); }

  ctx.save(); ctx.beginPath(); ctx.rect(left,top,plotW,plotH); ctx.clip();
  drawTrace(ctx, aquisicaoPlotX, "#ff453a", left, top, plotW, plotH, minVal, maxVal);
  drawTrace(ctx, aquisicaoPlotY, "#0a84ff", left, top, plotW, plotH, minVal, maxVal);
  drawTrace(ctx, aquisicaoPlotZ, "#30d158", left, top, plotW, plotH, minVal, maxVal);
  ctx.restore();

  ctx.textAlign="left"; ctx.textBaseline="top"; ctx.font="12px -apple-system, BlinkMacSystemFont, sans-serif"; ctx.fillStyle=corVar("--canvas-text");
  [["X","#ff453a",10,28,46],["Y","#0a84ff",58,76,94],["Z","#30d158",106,124,142]].forEach(([l,c,tx,lx1,lx2])=>{
    ctx.fillText(l, left+tx, top+8);
    ctx.strokeStyle=c; ctx.lineWidth=3;
    ctx.beginPath(); ctx.moveTo(left+lx1,top+15); ctx.lineTo(left+lx2,top+15); ctx.stroke();
  });
}

// ----- Operador e exportação -----
function definirOperador(){
  const nome = document.getElementById('operadorInput').value.trim();
  if(nome && ws && ws.readyState === WebSocket.OPEN){
    ws.send('OPERADOR:' + nome);
    operadorNome = nome;
    document.getElementById('operadorAtivo').textContent = 'Operador: ' + nome;
    document.getElementById('operadorInput').value = '';
  }
}

function baixarCSV(){ window.open('/csv'); }

// ----- Inicialização -----
window.addEventListener("resize", () => setTimeout(ajustarCanvas, 100));

aplicarTemaSalvo();
atualizarModoControle();

window.addEventListener("load", () => {
  setTimeout(() => { ajustarCanvas(); conectarWebSocket(); }, 300);
});
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}


// =============================================================================
// SEÇÃO 19 — ROTAS HTTP (CSV e LOG DE ANOMALIAS)
// =============================================================================

// GET /csv — exporta o buffer circular atual como arquivo CSV
void enviarCSV() {
  String csv = "data_hora;operador;offset_ms;x;y;z;freq;rmsY;picoPicoY\n";
  unsigned long tBase = bufferT[indexBuffer % TAM];

  for (int i = 0; i < TAM; i++) {
    int idx = (indexBuffer + i) % TAM;
    unsigned long offset = bufferT[idx] - tBase;

    csv += timestampInicioAquisicao + ";" +
           String(bufferOp[idx])    + ";" +
           String(offset)           + ";" +
           String(bufferX[idx], 3) + ";" +
           String(bufferY[idx], 3) + ";" +
           String(bufferZ[idx], 3) + ";" +
           String(freqDominante, 2) + ";" +
           String(rmsY, 3)          + ";" +
           String(picoPicoY, 3)     + "\n";
  }

  server.send(200, "text/csv", csv);
}

// GET /eventos — exporta o log de anomalias detectadas como CSV
void enviarEventosCSV() {
  String csv = "data_hora;operador;rms;freq_hz;pico_a_pico;tipo\n";
  int total = min(numEventos, MAX_EVENTOS);

  for (int i = 0; i < total; i++) {
    csv += String(logEventos[i].dataHora)    + ";" +
           String(logEventos[i].operador)    + ";" +
           String(logEventos[i].rms, 3)      + ";" +
           String(logEventos[i].freq, 2)     + ";" +
           String(logEventos[i].picoPico, 3) + ";" +
           String(logEventos[i].tipo)        + "\n";
  }

  server.send(200, "text/csv", csv);
}


// =============================================================================
// SEÇÃO 20 — EVENTOS WEBSOCKET (COMANDOS DO CLIENTE)
// =============================================================================
//
//  Comandos aceitos via WebSocket:
//   CALIBRAR         → inicia calibração de repouso
//   OPERADOR:<nome>  → define o nome do operador atual
//   FFT_AUTO         → volta para seleção automática de eixo
//   FFT_X / FFT_Y / FFT_Z           → força um único eixo
//   FFT_XY / FFT_XZ / FFT_YZ / FFT_XYZ → combina eixos

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type != WStype_TEXT) return;

  String msg = String((char*)payload);

  if (msg == "CALIBRAR")            { iniciarCalibracao(); return; }
  if (msg.startsWith("OPERADOR:"))  { operadorAtual = msg.substring(9); operadorAtual.trim(); return; }

  if (msg == "FFT_AUTO") { modoFFTManual = false; usarFFT_X = usarFFT_Y = usarFFT_Z = true; return; }
  if (msg == "FFT_X")   { modoFFTManual = true; usarFFT_X=true;  usarFFT_Y=false; usarFFT_Z=false; return; }
  if (msg == "FFT_Y")   { modoFFTManual = true; usarFFT_X=false; usarFFT_Y=true;  usarFFT_Z=false; return; }
  if (msg == "FFT_Z")   { modoFFTManual = true; usarFFT_X=false; usarFFT_Y=false; usarFFT_Z=true;  return; }
  if (msg == "FFT_XY")  { modoFFTManual = true; usarFFT_X=true;  usarFFT_Y=true;  usarFFT_Z=false; return; }
  if (msg == "FFT_XZ")  { modoFFTManual = true; usarFFT_X=true;  usarFFT_Y=false; usarFFT_Z=true;  return; }
  if (msg == "FFT_YZ")  { modoFFTManual = true; usarFFT_X=false; usarFFT_Y=true;  usarFFT_Z=true;  return; }
  if (msg == "FFT_XYZ") { modoFFTManual = true; usarFFT_X=true;  usarFFT_Y=true;  usarFFT_Z=true;  return; }
}


// =============================================================================
// SEÇÃO 21 — SETUP (executa uma vez na inicialização)
// =============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("tempo_ms,x,y,z,freq,rmsY,picoPicoY");

  // I2C nos pinos do ESP32-C3 Super Mini
  Wire.begin(21, 22); //SDA=G21, SCL=G22

  // Inicializa o ADXL345
  if (!accel.begin()) {
    Serial.println("Erro no ADXL345 — verifique as ligações!");
    while (1); // trava aqui se o sensor não responder
  }
  accel.setRange(ADXL345_RANGE_16_G);          // ±16g (máxima sensibilidade para vibração)
  accel.setDataRate(ADXL345_DATARATE_800_HZ);  // 800 amostras/segundo

  // Zera os buffers
  for (int i = 0; i < TAM; i++) {
    bufferX[i] = bufferY[i] = bufferZ[i] = bufferA[i] = 0;
    strncpy(bufferOp[i], "Desconhecido", 32);
  }
  for (int i = 0; i < FFT_SAMPLES / 2; i++) espectro[i] = 0;

  // Conecta ao Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }

  // Sincroniza horário via NTP (fuso de Brasília: UTC-3)
  configTime(-3 * 3600, 0, "pool.ntp.org", "time.google.com");
  Serial.print("\nSincronizando NTP");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) { delay(500); Serial.print("."); }

  Serial.println("\nConectado!");
  Serial.print("Acesse: http://");
  Serial.println(WiFi.localIP());

  // Registra as rotas HTTP
  server.on("/",        paginaPrincipal);
  server.on("/csv",     enviarCSV);
  server.on("/eventos", enviarEventosCSV);
  server.begin();

  // Inicia o WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}


// =============================================================================
// SEÇÃO 22 — LOOP PRINCIPAL
// =============================================================================
//
//  Tarefas executadas continuamente, cada uma com sua própria temporização:
//
//  │ Tarefa              │ Frequência      │ Responsável      │
//  ├─────────────────────┼─────────────────┼──────────────────┤
//  │ webSocket.loop()    │ a cada iteração │ WebSocketsServer │
//  │ tickCalibracao()    │ a cada iteração │ calibração async │
//  │ atualizarBuffer()   │ 800 Hz          │ leitura + filtro │
//  │ processarFFT()      │ 1 Hz            │ FFT + anomalias  │
//  │ enviarTempoReal()   │ ~12,5 Hz        │ WebSocket TX     │
//  │ server.handleClient│ a cada iteração │ HTTP             │

void loop() {
  webSocket.loop();
  tickCalibracao();

  // Leitura do sensor na taxa de amostragem (800 Hz)
  if (micros() - tLeitura >= 1000000UL / SAMPLING_FREQ) {
    if (!calibrando) atualizarBuffer(); // pausa a leitura durante a calibração
    tLeitura = micros();
  }

  // Processamento FFT (1×/segundo)
  if (millis() - tFFT >= 1000) {
    processarFFT();
    tFFT = millis();
  }

  // Envio de dados via WebSocket (~12,5×/segundo = a cada 80 ms)
  if (millis() - tEnvio >= 80) {
    enviarTempoReal();
    tEnvio = millis();
  }

  // Atende requisições HTTP
  server.handleClient();
}
