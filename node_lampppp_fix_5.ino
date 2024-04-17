#include <painlessMesh.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "soc/rtc_cntl_reg.h"

#define MESH_PREFIX "XirkaMesh"
#define MESH_PASSWORD "12345678"
#define MESH_PORT 5555

uint32_t nodeNumber = 2;
uint32_t myLineNumber = 1;
int receivedLineNumber;
char buffValue[100];
StaticJsonDocument<200> doc;
StaticJsonDocument<200> newDoc;
StaticJsonDocument<200> controlLedDoc;
StaticJsonDocument<200> receivedJsonDoc;

unsigned long t1;
unsigned long tl;
uint32_t sender;
uint32_t newId;
bool flag = 0;
String receivedData;

Scheduler userScheduler;
painlessMesh mesh;

const int MAX_GATEWAY_CONNECTIONS = 1;
uint32_t newGatewayNodes[MAX_GATEWAY_CONNECTIONS] = {};
int jumlahGatewayNodes = 0;

const int MAX_NEXT_DIRECT_CONNECTIONS = 1;
uint32_t newNextDirectNodes[MAX_NEXT_DIRECT_CONNECTIONS];
int jumlahNextDirectNodes = 0;

void ledSetup() {
  uint8_t msg = 1;
  controlLedDoc["LAMP"] = msg;
  String jsonLedOn;
  serializeJson(controlLedDoc, jsonLedOn);
  Serial2.println(jsonLedOn);
}

//function for receive data from microcontroller and send data via mesh
void receiveSerial2() {
  while (Serial2.available() > 0) {
    static uint8_t x = 0;
    buffValue[x] = (char)Serial2.read();
    x += 1;
    if (buffValue[x - 1] == '\n') {
      DeserializationError error = deserializeJson(doc, buffValue);
      if (!error) {
        float vsolar = doc["VSOLAR"];
        float vbat = doc["VBAT"];

        newDoc["Node"] = nodeNumber;
        newDoc["Line"] = myLineNumber;
        newDoc["VSOLAR"] = round(vsolar * 100.0) / 100.0;
        newDoc["VBAT"] = round(vbat * 100.0) / 100.0;  // Round vbat to two decimal places
        newDoc["LAMP"] = (controlLedDoc["LAMP"] == 1) ? "ON" : "OFF";

        String jsonString;
        serializeJson(newDoc, jsonString);
        Serial.println("Data: " + jsonString);
        for (int i = 0; i < jumlahGatewayNodes; i++) {
          if (sender == newGatewayNodes[i]) {
            mesh.sendSingle(newGatewayNodes[i], jsonString);
          }
        }
      }
      x = 0;
      memset(buffValue, 0, sizeof(buffValue));
      break;
    }
  }
}


void sendMessage() {
  String msg = receivedData;

  bool isFromNewNode = false;
  for (int i = 0; i < jumlahGatewayNodes; i++) {
    if (sender == newGatewayNodes[i]) {
      isFromNewNode = true;
      break;
    }
  }

  if (isFromNewNode && jumlahGatewayNodes > 0 && jumlahNextDirectNodes > 0) {
    for (int i = 0; i < jumlahNextDirectNodes; i++) {
      mesh.sendSingle(newNextDirectNodes[i], msg);
      Serial.printf("Data Sent to: %u msg=%s\n", newNextDirectNodes[i], msg.c_str());
    }
  } else if (!isFromNewNode && jumlahGatewayNodes > 0 && jumlahNextDirectNodes > 0) {
    for (int i = 0; i < jumlahGatewayNodes; i++) {
      mesh.sendSingle(newGatewayNodes[i], msg);
      Serial.printf("Data Sent to: %u msg=%s\n", newGatewayNodes[i], msg.c_str());
    }
  }
}

void checkConnectionStatus() {
  if (jumlahGatewayNodes > 0) {
    for (int i = 0; i < jumlahGatewayNodes; i++) {
      if (!mesh.isConnected(newGatewayNodes[i])) {
        Serial.printf("Gateway Nodes %u is not connected. Delete from list.\n", newGatewayNodes[i]);
        for (int j = i; j < jumlahGatewayNodes - 1; j++) {
          newGatewayNodes[j] = newGatewayNodes[j + 1];
        }
        jumlahGatewayNodes--;
        Serial.printf("Amount of gateway node connected: %d\n", jumlahGatewayNodes);
      }
    }
  }

  if (jumlahNextDirectNodes > 0) {
    for (int i = 0; i < jumlahNextDirectNodes; i++) {
      if (!mesh.isConnected(newNextDirectNodes[i])) {
        Serial.printf("Next Direcy Node %u is not connected. Delete from list.\n", newNextDirectNodes[i]);
        for (int i = 0; i < MAX_NEXT_DIRECT_CONNECTIONS; i++) {
          newNextDirectNodes[i] = 0;
        }
        jumlahNextDirectNodes = 0;
        Serial.printf("Amount of next direct node: %d\n", jumlahNextDirectNodes);
      }
    }
  }
}

void saveNode() {
  if (jumlahGatewayNodes < MAX_GATEWAY_CONNECTIONS) {
    newGatewayNodes[jumlahGatewayNodes++] = newId;
    Serial.printf("Node Gateway saved in index number-%d\n", jumlahGatewayNodes - 1);
    Serial.printf("Gateway Node: %d\tAmount Gateway Node:%d\n", newGatewayNodes[jumlahGatewayNodes - 1], jumlahGatewayNodes);
  } else if (jumlahGatewayNodes >= MAX_GATEWAY_CONNECTIONS && jumlahNextDirectNodes < MAX_NEXT_DIRECT_CONNECTIONS) {
    newNextDirectNodes[jumlahNextDirectNodes++] = newId;
    Serial.printf("Next Direct node saved in index number-%d\n", jumlahNextDirectNodes - 1);
    Serial.printf("Next Direct Node: %d\n", newNextDirectNodes[jumlahNextDirectNodes - 1]);
  } else {
    Serial.println("Can't save node because both of node type is full");
  }
}

void checkLineNumber() {
  if (receivedLineNumber != myLineNumber) {
    Serial.println("Line number not match");
    updateConnection();
  } else {
    Serial.println("Line number is match");
  }
}

void updateConnection() {
  if (jumlahGatewayNodes > 0 && jumlahNextDirectNodes == 0) {
    jumlahGatewayNodes--;
    Serial.println("Reduced node gateway by one!");
    Serial.printf("Amount of gateway node : %d\n", jumlahGatewayNodes);
  } else if (jumlahGatewayNodes > 0 && jumlahNextDirectNodes > 0) {
    jumlahNextDirectNodes--;
    Serial.println("Reduced node gateway by one!");
    Serial.printf("Amount of next direct node : %d\n", jumlahNextDirectNodes);
  } else {
    Serial.println("No node gateway to reduce!");
  }
}

bool lineChecked = false;
bool disconnectedNodeMessagePrinted = false;
void receivedCallback(uint32_t from, String& msg) {
  bool isSenderConnected = false;
  for (int i = 0; i < jumlahGatewayNodes; i++) {
    if (from == newGatewayNodes[i]) {
      isSenderConnected = true;
      break;
    }
  }
  for (int i = 0; i < jumlahNextDirectNodes; i++) {
    if (from == newNextDirectNodes[i]) {
      isSenderConnected = true;
      break;
    }
  }

  if (!isSenderConnected) {
    if (!disconnectedNodeMessagePrinted) {
      Serial.printf("Ignoring message from disconnected node: %u\n", from);
      disconnectedNodeMessagePrinted = true;
    }
    return;
    disconnectedNodeMessagePrinted = false;
  }
  sender = from;
  receivedData = msg.c_str();
  Serial.printf("received from %u msg=%s\n", from, msg.c_str());

  StaticJsonDocument<200> receivedJsonDoc;
  DeserializationError error = deserializeJson(receivedJsonDoc, msg);
  if (!error && receivedJsonDoc.containsKey("Line")) {
    receivedLineNumber = receivedJsonDoc["Line"];
    String msg2 = "Check line number";
    mesh.sendSingle(from, msg2);
    // Serial.printf("Received Line Number: %d\n", receivedLineNumber);
  } else {
    // Serial.println("Failed to parse received JSON or missing line number.");
  }
  if (!lineChecked) {
    checkLineNumber();
    lineChecked = true;
  }
  return;
  lineChecked = false;
  sendMessage();
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New Connection, nodeId = %u\n", nodeId);
  newId = nodeId;
  saveNode();
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
  checkConnectionStatus();
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  ledSetup();

  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_PREFIX, MESH_PASSWORD);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
}

void loop() {
  mesh.update();
  userScheduler.execute();

  if (millis() - t1 > 1000) {
    receiveSerial2();
    t1 = millis();
  }
}