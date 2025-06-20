#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <Adafruit_VL53L0X.h>
int recStart = 0;
int limite_superior = 5100; // Valores iniciais que serão substituidos assim que chegar
int limite_inferior = 5200;  // o meu vetor via MQTT.
int numero_voltas = 0; // Mesma analogia dos limites.
int magneticoState = 0;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
float media_calibration = 0.0;
float media_medicoes = 0.0;
float alpha = 0.2;  // Fator de suavização (0 < alpha < 1)
const char* ssid = "Rede Wifi";
const char* password = "Credenciais da rede";
const char* mqtt_server = "Rost IP MQTT";
const char* mqtt_topic = "sensor/values";
const char* mqtt_write = "video/command";
const int LED = 2;
const int rele = 0;
int cameraStart = 0;
bool rec = false;
bool calibra = false;
bool estadoMagnetico = false;// Recebe do MQTT autorização para iniciar contagem apos passar pelo sensor magnetico
int contador = 0;
bool memoria_magnetico = false;
const int magnetico = 16;
int x = 0 ;
const size_t bufferSize = JSON_OBJECT_SIZE(6);
unsigned long previusMillis = 0;
const long interval = 2000;
unsigned long currentMillis = 0;
bool trigger = false;
unsigned long previusMillis_2 = 0;
const long interval_2 = 500;
unsigned long currentMillis_2 = 0;
int anterior = 0;
const int maxDataSize = 100; 
char receivedData[maxDataSize]; 
int dataLength = 0; 

#define SOUND_VELOCITY 0.034
#define CM_TO_INCH 0.393701

WiFiClient espClient;
PubSubClient client(espClient);

int numElementos = 0; 
int matriz_elo[100];
int matriz_fim[100];
bool matrizesIguais = true;

unsigned long ledOnTime = 0; // Adicionado para rastrear quando o rele foi ligado
unsigned long heartbeatInterval = 60000; // Intervalo de 1 minuto para o batimento cardíaco
unsigned long lastHeartbeatTime = 0; // Variável para rastrear o tempo do último batimento cardíaco
//=========================== DECLARETION END ===================================

// Função para enviar o batimento cardíaco ao servidor MQTT
void sendHeartbeat() {
  if (client.connected()) {
    client.publish("sensor/heartbeat", "alive");
  }
}

void intervalo() {
  unsigned long currentMillis = millis();
  
  if (digitalRead(rele) == LOW) {
    if (currentMillis - ledOnTime >= interval) {
      digitalWrite(rele, HIGH);
    }
      }else{
    //digitalWrite(rele, LOW);
    ledOnTime = currentMillis; // Atualiza o tempo que o rele foi ligado
  }
  if (currentMillis - previusMillis >= interval) {
    previusMillis = currentMillis;
  }
}
void intervalo_2() {
  unsigned long currentMillis_2 = millis();
  if (currentMillis_2 - previusMillis_2 >= interval_2) {
    previusMillis_2 = currentMillis_2;
    if (trigger == true && measure.RangeMilliMeter > limite_superior) {
      trigger = false;
    }
  }
}
//####################### Setup ##########################
void setup(){ 
  Serial.begin(115200);
  pinMode(magnetico, INPUT);
  digitalWrite(magnetico, HIGH);
  // wait until serial port opens for native USB devices
  while (!Serial) {
    delay(1);
  }
  pinMode(LED, OUTPUT);
  pinMode(rele, OUTPUT);
  digitalWrite(rele, HIGH);
  digitalWrite(LED, HIGH);
  Serial.println("Testando Rele"); 
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  // power
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
  // Conecta-se à rede Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  // Inicializa a conexão MQTT
  client.setServer(mqtt_server, 1887);
  client.setCallback(callback);
  delay(2000);
}
//==================== PRINCIPAL LOOP ===============================
void loop() {
  if (calibra == false){
  int cont_calibration = 0;
  float media_calibration = 0.0;
  float soma_calibration = 0.0;


// Verificar se é hora de enviar o próximo batimento cardíaco
  unsigned long currentMillis = millis();
  if (currentMillis - lastHeartbeatTime >= heartbeatInterval) {
    // É hora de enviar o batimento cardíaco
    sendHeartbeat();
    // Atualizar o tempo do último batimento cardíaco
    lastHeartbeatTime = currentMillis;
  }

  
  while (cont_calibration <10){
    soma_calibration = soma_calibration + media_medicoes;
    cont_calibration++;
    Serial.println("Calibrando...");
  }
  media_calibration = soma_calibration / 10;
  calibra = true;
  //Serial.println("O valor da media_calibration é: "); Serial.print(media_calibration);
  }
  else{
  //Serial.println("O valor no LOOP da media_calibration é: "); Serial.print(media_calibration);
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  unsigned long currentMillis = millis();

  if (measure.RangeStatus != 4) {
    // Cálculo da Média Exponencial Móvel (EMA)
    media_medicoes = alpha * measure.RangeMilliMeter + (1 - alpha) * media_medicoes;

    Serial.print("Distance EMA (mm): "); 
    Serial.println(media_medicoes);
  } else {
    Serial.println(" out of range ");
  }
    Serial.println("Contador: "); 
    Serial.println(contador);
    
  if (media_medicoes <= limite_inferior  && trigger == false && memoria_magnetico == true) {
    digitalWrite(LED, LOW);
    contador++;
    trigger = true;
        
    //=======================================================    
    StaticJsonDocument<bufferSize> jsonBuffer;
    JsonObject root = jsonBuffer.to<JsonObject>();
    root["Local"] = "Transpotador TP10";
    root["Voltas"] = numero_voltas;
    root["Contagem de Elos"] = contador;
    root["cameraStart"] = recStart;
    root["Elo"] = numElementos;
    root["Pintou"] = matriz_fim[x];
    String jsonString;
    serializeJson(root, jsonString);    
    // Envie o JSON via MQTT
    client.publish("sensor/values", jsonString.c_str());
    // ================================================    
  } else {
    digitalWrite(LED, HIGH);
  } 
Serial.print("==============================");   
Serial.println(memoria_magnetico);

  // Verificar se existe um valor igual a 'contador' na matriz 'matriz_elo'
  
  for (int x = 0; x < numElementos; x++) {
    if (matriz_elo[x] == contador) {
      if (contador == int(matriz_elo[x]) && anterior != contador) {
        anterior = contador;
        digitalWrite(rele, LOW);
        matriz_fim[x] = matriz_elo[x];
      }
    }  
  }

  // Conecta-se ao servidor MQTT, se não estiver conectado
  if (!client.connected()) {
    reconnect();
  }
  if (contador > 0) {
    // Suponha que as matrizes são iguais
    for (int i = 0; i < numElementos; i++) {
      if (matriz_elo[i] != matriz_fim[i]) {
        matrizesIguais = false; // Encontrou um elemento diferente
        break; // Não é necessário verificar mais elementos, saia do loop
      } else {
        matrizesIguais = true;
      }
    }
    
if (matrizesIguais){
  // As matrizes são iguais, faça algo aqui
  Serial.println("Matrizes são iguais");
  // Envie sua mensagem de processo finalizado aqui usando client.publish
  client.publish("sensor/values", "Processo finalizado com SUCESSO");

  // Limpa a matriz elo atribuindo zero a cada elemento
  for (int i = 0; i < 100; i++) {
    matriz_elo[i] = 0;
  }

  // Limpa a matriz fim atribuindo zero a cada elemento
  for (int i = 0; i < 100; i++) {
    matriz_fim[i] = 0;
  }

  // Define matrizesIguais como falso
  matrizesIguais = false;
  numElementos = 0;
}

  }
    
  // Mantém a comunicação MQTT ativa
  client.loop();
  }
 intervalo();
 intervalo_2();
 
 int magneticoState = digitalRead(magnetico);
 Serial.print("Senor Magnetico: ");
 Serial.println(magneticoState);
 //== habilita contagem
//if (estadoMagnetico == 1 && magneticoState == HIGH) { 
if (magneticoState == HIGH) {
  Serial.println("Start Contagem");
    memoria_magnetico = true;
  }
  if(estadoMagnetico == 0){
     //memoria_magnetico = false;
  }
  
//============================= Monitoring cont Elo =================
//=========== Initialize cont of Eloe hen action sensor magnetico igul 1 =======

if(memoria_magnetico == 1 && magneticoState == HIGH && contador > 3){
    contador = 0;
    numero_voltas++ ; 
    Serial.println("Fim da Contagem"); 
  if(rec == true && magneticoState == HIGH){
      client.publish(mqtt_write,"stop");
      client.publish("sensor/values","Stop Rec");
      recStart = 0;
      rec = false;
      Serial.println("Finalizado Gravação");
    }
   } 
   
if(recStart == 1 && magneticoState == HIGH && rec == false){
  client.publish(mqtt_write,"start");
  client.publish("sensor/values","Start rec");
  Serial.println("Iniciado Gravação");
  rec = true;
}
}

//==================== =========== ===============================
void callback(char* topic, byte* payload, unsigned int length) {
  // Limpa o JSON antes de preenchê-lo com os novos dados
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload, length);

  int numChaves = doc.size();
  Serial.print("Número de chaves (campos) no JSON: ");
  Serial.println(numChaves);

  if (doc.containsKey("elo")) {
    JsonArray eloArray = doc["elo"].as<JsonArray>();
    numElementos = eloArray.size(); // Atualiza o número de elementos na matriz
    Serial.println("Número de elementos na matriz (array) 'elo': ");
    Serial.println(numElementos);

    for (int x = 0; x < numElementos; x++) { //podemos tirar depois apenas para visualizar
      matriz_elo[x] = eloArray[x].as<int>();
      Serial.print("Valor na posição ");
      Serial.print(x);
      Serial.print(": ");
      Serial.println(matriz_elo[x]);
    }
  }

  if (doc.containsKey("magnetico")) {
    int valorMagnetico = doc["magnetico"].as<int>();
    Serial.print("Valor da chave 'magnetico': ");
    Serial.println(valorMagnetico);

    // Modifique a variável booleana com base no valor da chave "magnetico"
    estadoMagnetico = (valorMagnetico == 1);
  }
//================================================
if (doc.containsKey("cameraStart")) {
    int cameraStart = doc["cameraStart"].as<int>();
    Serial.print("Valor da chave 'cameraStart': ");
    Serial.println(cameraStart);

    // Modifique a variável booleana com base no valor da chave "cameraStart"
    recStart = (cameraStart == 1);
  }
//================================================  
  if (doc.containsKey("voltas")){
  numero_voltas = doc["voltas"].as<int>();
  Serial.print("O número de voltas é: ");
  Serial.println(numero_voltas);
  }
  if (doc.containsKey("lim_inferior")){
  limite_inferior = doc["lim_inferior"].as<int>();
  Serial.print("O limite inferior é: ");
  Serial.println(limite_inferior);
  }
  if (doc.containsKey("lim_superior")){
  limite_superior = doc["lim_superior"].as<int>();
  Serial.print("O limite superior é: ");
  Serial.println(limite_superior);
  }
  //contador = 0;
}
//=============
void reconnect() {
  // Reconecta ao servidor MQTT
  while (!client.connected()) {
    if (client.connect("ESP8266Client")) {
      // Assina o tópico MQTT para receber valores
      client.subscribe(mqtt_topic);
    }else{
      delay(5000);
    }
  }
}
