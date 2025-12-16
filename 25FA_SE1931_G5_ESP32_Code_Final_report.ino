/*************** BLYNK CONFIG ***************/
#define BLYNK_TEMPLATE_ID "TMPL6GlIX1T5U"
#define BLYNK_TEMPLATE_NAME "Traffic Light"
#define BLYNK_AUTH_TOKEN "1FGL5bJGiIjhzNnrVCMEGccnRADRQ9y2"
#define UART_RX_PIN 16  // RX2 (nhận từ Arduino TX qua level shifter 5V->3.3V)
#define UART_TX_PIN 17  // TX2 (gửi sang Arduino RX qua level shifter 3.3V->5V)

HardwareSerial PedSerial(2);  // dùng UART2

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "time.h"

/*************** USER WIFI ***************/
char ssid[] = "Hung 5G";
char pass[] = "0947532540";

// set up real time UTC + 7
const long gmtOffset_sec = 7 * 3600;
const int daylightOffset_sec = 0;

/*************** PIN CONFIG ***************/
#define A_RED_PIN 26
#define A_YELLOW_PIN 25
#define A_GREEN_PIN 33

#define B_RED_PIN 13
#define B_YELLOW_PIN 12
#define B_GREEN_PIN 14

#define LATCH_A_PIN 15
#define CLOCK_A_PIN 2
#define DATA_A_PIN 4

#define LATCH_B_PIN 5
#define CLOCK_B_PIN 18
#define DATA_B_PIN 19

/*************** 7-SEG MAP ***************/
/*
 Common Anode:
 bit0=a bit1=b ... bit6=g bit7=dp
 0 = sáng đoạn
 1 = tắt đoạn
*/
const uint8_t segCode[10] = {
  0b11000000,  // 0
  0b11111001,  // 1
  0b10100100,  // 2
  0b10110000,  // 3
  0b10011001,  // 4
  0b10010010,  // 5
  0b10000010,  // 6
  0b11111000,  // 7
  0b10000000,  // 8
  0b10010000   // 9
};

// pattern tắt hết đoạn
const uint8_t segOff = 0xFF;

/*************** KHAI BÁO TRẠNG THÁI / BIẾN ĐIỀU KHIỂN ***************/

// thời gian phase auto
int greenNS = 26;
int yellowNS = 4;
int greenWE = 26;
int yellowWE = 4;

int redNS = greenWE + yellowWE;
int redWE = greenNS + yellowNS;

boolean pedNS = false;
boolean pedB = false;

int pedestrianStatus = 11;  //PEDESTRIAN_OFF
// đỏ = xanh + vàng của hướng còn lại

// phase tự động
enum Phase { GREEN,
             YELLOW,
             RED };
Phase stateA = GREEN;  // khởi động: A xanh
Phase stateB = RED;    // khởi động: B đỏ

// countdown mỗi lane trong auto
unsigned long tA = 0;
unsigned long tB = 0;
int secA = greenNS;
int secB = redWE;

// chế độ
bool manual = false;     // Manual mode ON/OFF
bool flashMode = false;  // Flash vàng toàn ngã tư ON/OFF

// lệnh từ app
// reqNS: 0=Red A, 1=Green A, 2=Yellow A
// reqWE: 0=Red B, 1=Green B, 2=Yellow B
int reqNS = 0;
int reqWE = 0;

// ưu tiên lệnh mới nhất: A hay B
enum ControlSource { SRC_A,
                     SRC_B };
ControlSource lastControl = SRC_A;

/*************** KHAI BÁO HÀM TRƯỚC (PROTOTYPE) ***************/
void reportLaneStatusToBlynk(int laneAState);
void manualModeProcess();
void updateA_auto();
void updateB_auto();
void clearSegment(char lane);
void showNumber(char lane, int s);
void setA(int r, int y, int g);
void setB(int r, int y, int g);

/*************** LOW-LEVEL SEGMENT CONTROL ***************/
void sendSeg(int latch, int clock, int data, uint8_t v) {
  digitalWrite(latch, LOW);
  shiftOut(data, clock, MSBFIRST, v);
  digitalWrite(latch, HIGH);
}

// hiển thị số đếm cho 1 lane (0..99)
void showNumber(char lane, int s) {
  if (s < 0) s = 0;
  if (s > 99) s = 99;
  int tens = s / 10;
  int ones = s % 10;

  if (lane == 'A') {
    sendSeg(LATCH_A_PIN, CLOCK_A_PIN, DATA_A_PIN, segCode[ones]);
    delay(2);
    sendSeg(LATCH_A_PIN, CLOCK_A_PIN, DATA_A_PIN, segCode[tens]);
  } else {
    sendSeg(LATCH_B_PIN, CLOCK_B_PIN, DATA_B_PIN, segCode[ones]);
    delay(2);
    sendSeg(LATCH_B_PIN, CLOCK_B_PIN, DATA_B_PIN, segCode[tens]);
  }
}

// tắt hoàn toàn 2 digit của lane
void clearSegment(char lane) {
  if (lane == 'A') {
    sendSeg(LATCH_A_PIN, CLOCK_A_PIN, DATA_A_PIN, segOff);
    sendSeg(LATCH_A_PIN, CLOCK_A_PIN, DATA_A_PIN, segOff);
  } else {
    sendSeg(LATCH_B_PIN, CLOCK_B_PIN, DATA_B_PIN, segOff);
    sendSeg(LATCH_B_PIN, CLOCK_B_PIN, DATA_B_PIN, segOff);
  }
}

/*************** LED CONTROL ***************/
void setA(int r, int y, int g) {
  digitalWrite(A_RED_PIN, r);
  digitalWrite(A_YELLOW_PIN, y);
  digitalWrite(A_GREEN_PIN, g);
}
void setB(int r, int y, int g) {
  digitalWrite(B_RED_PIN, r);
  digitalWrite(B_YELLOW_PIN, y);
  digitalWrite(B_GREEN_PIN, g);
}

/*************** GỬI TRẠNG THÁI LÊN BLYNK ***************/
/*
 laneAState / laneBState:
   0 = RED
   1 = GREEN
   2 = YELLOW
  -1 = không cập nhật lane đó
*/
void reportLaneStatusToBlynk(int laneAState) {
  // Lane A
  if (laneAState != -1) {
    Blynk.virtualWrite(V0, (laneAState == 0) ? 1 : 0);  // Red A
    Blynk.virtualWrite(V1, (laneAState == 1) ? 1 : 0);  // Green A
  }
}

/*************** AUTO MODE UPDATE ***************/
void updateA_auto() {
  if (millis() - tA >= 1000) {
    tA = millis();
    secA--;
    showNumber('A', secA);  // hiển thị countdown

    if (secA <= 0) {
      if (stateA == GREEN) {
        stateA = YELLOW;
        secA = yellowNS;
      } else if (stateA == YELLOW) {
        stateA = RED;
        secA = redNS;
      } else {  // stateA == RED
        stateA = GREEN;
        secA = greenNS;
      }
      if (stateA == RED && pedNS) {
        pedestrianStatus = 10;  //PEDESTRIAN_OK;
      }
      // khôi phục thời gian nếu có lưu
      if (pedNS && stateA == GREEN) {
        pedNS = false;
        pedestrianStatus = 11;  //PEDESTRIAN_OFF;
      }
    }



    if (stateA == GREEN) {
      setA(0, 0, 1);
    } else if (stateA == YELLOW) {
      setA(0, 1, 0);
    } else {  // RED
      setA(1, 0, 0);
    }
  }
}

void updateB_auto() {
  if (millis() - tB >= 1000) {
    tB = millis();
    secB--;
    showNumber('B', secB);  // hiển thị countdown

    if (secB <= 0) {
      if (stateB == GREEN) {
        stateB = YELLOW;
        secB = yellowWE;
      } else if (stateB == YELLOW) {
        stateB = RED;
        secB = redWE;
      } else {  // stateB == RED
        stateB = GREEN;
        secB = greenWE;
      }
    }



    if (stateB == GREEN) {
      setB(0, 0, 1);
    } else if (stateB == YELLOW) {
      setB(0, 1, 0);
    } else {  // RED
      setB(1, 0, 0);
    }
  }
}

/*************** MANUAL MODE ***************/
void manualModeProcess() {
  // Tắt countdown trong manual
  clearSegment('A');
  clearSegment('B');

  // FLASH MODE: nháy vàng cả hai rồi tắt
  if (flashMode) {
    setA(0, 1, 0);                // A vàng
    setB(0, 1, 0);                // B vàng
    reportLaneStatusToBlynk(-1);  // nếu muốn cập nhật UI ở flash
    delay(1000);
    setA(0, 0, 0);  // tắt hết A
    setB(0, 0, 0);  // tắt hết B
    delay(1000);
    return;
  }

  // =========================
  // ƯU TIÊN LỆNH MỚI NHẤT
  // =========================
  if (lastControl == SRC_A) {
    // Ưu tiên lệnh từ A (reqNS là ý định chính)

    if (reqNS == 0) {
      if (reqWE == 0) {
        setA(0, 1, 0);  // A vàng
        delay(2000);
        reqWE = 1;
      }
      setA(1, 0, 0);
      setB(0, 0, 1);
      reportLaneStatusToBlynk(0);
    }

    else if (reqNS == 1) {
      if (reqWE == 1) {
        setB(0, 1, 0);
        delay(2000);
        reqWE = 0;
      }
      setB(1, 0, 0);  // B đỏ
      setA(0, 0, 1);  // A xanh
      reportLaneStatusToBlynk(1);
    }

  } else {
    // lastControl == SRC_B
    // Ưu tiên lệnh từ B (reqWE là ý định chính)

    if (reqWE == 0) {
      if (reqNS == 1) {
        setB(0, 1, 0);  // A vàng
        delay(2000);
        reqNS = 0;
      }
      setB(1, 0, 0);
      setA(0, 0, 1);
      reportLaneStatusToBlynk(1);
    }

    else if (reqWE == 1) {
      if (reqNS == 1) {
        setA(0, 1, 0);
        delay(2000);
        reqNS = 0;
      }
      setA(1, 0, 0);
      setB(0, 0, 1);
      reportLaneStatusToBlynk(0);
    }
  }
}


/*************** BLYNK CALLBACKS (8 BUTTONS) ***************/
/*
V0 -> Red A
V2 -> Green A
V1 -> Yellow A
V6 -> Red B
V4 -> Green B
V5 -> Yellow B
V7 -> Manual toggle (0=auto,1=manual)
V8 -> Flash toggle (0=off,1=flash)
*/

// --- Điều khiển RED A ---
BLYNK_WRITE(V0) {              
  if (!manual || flashMode) {  // Nếu không ở manual hoặc đang flash → bỏ qua
    Blynk.virtualWrite(V0, 0);
    return;
  }
  reqNS = 0;
  reqWE = 0;
  lastControl = SRC_A;
}

// --- Điều khiển GREEN A ---
BLYNK_WRITE(V1) {  
  if (!manual || flashMode) {
    Blynk.virtualWrite(V1, 0);
    return;
  }
  reqNS = 1;
  reqWE = 1;
  lastControl = SRC_A;
}

// --- FLASH MODE ---
BLYNK_WRITE(V2) {
  flashMode = param.asInt();  // 0 = off, 1 = on

  if (!manual && flashMode) {
    // Nếu flash bật khi chưa bật manual → tự bật manual
    manual = true;
    Blynk.virtualWrite(V3, 1);
  }

  if (flashMode) {
    // Khi bật flash → tắt các nút manual control
    Blynk.virtualWrite(V0, 0);
    Blynk.virtualWrite(V1, 0);
    Serial.println("Flash mode ON (in manual)");
  } else {
    Serial.println("Flash mode OFF (back to manual)");
  }
}

// --- MANUAL MODE ---
BLYNK_WRITE(V3) {
  manual = param.asInt();  // 0 = auto, 1 = manual

  if (!manual) {
    // Khi tắt manual → tắt flash luôn
    flashMode = false;
    Blynk.virtualWrite(V2, 0);
    Blynk.virtualWrite(V0, 0);
    Blynk.virtualWrite(V1, 0);
    Serial.println("Switched to AUTO mode");
  } else {
    // Khi bật manual → khởi động đèn mặc định
    flashMode = false;
    Blynk.virtualWrite(V2, 0);  // đảm bảo flash tắt lúc mới vào manual

    reqNS = 1;  // A xanh
    reqWE = 0;  // B đỏ
    secA = greenNS;
    secB = redWE;
    stateA = GREEN;
    stateB = RED;
    lastControl = SRC_A;

    Serial.println("Switched to MANUAL mode");
  }
}



/*************** SETUP ***************/
void setup() {
  pinMode(A_RED_PIN, OUTPUT);
  pinMode(A_YELLOW_PIN, OUTPUT);
  pinMode(A_GREEN_PIN, OUTPUT);
  pinMode(B_RED_PIN, OUTPUT);
  pinMode(B_YELLOW_PIN, OUTPUT);
  pinMode(B_GREEN_PIN, OUTPUT);
  pinMode(LATCH_A_PIN, OUTPUT);
  pinMode(CLOCK_A_PIN, OUTPUT);
  pinMode(DATA_A_PIN, OUTPUT);
  pinMode(LATCH_B_PIN, OUTPUT);
  pinMode(CLOCK_B_PIN, OUTPUT);
  pinMode(DATA_B_PIN, OUTPUT);
  // Serial USB để debug trên PC (cổng COM của ESP32)
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 STARTED");
  // UART2 để giao tiếp với Arduino
  PedSerial.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  // khởi động AUTO: A xanh, B đỏ
  setA(0, 0, 1);
  setB(1, 0, 0);

  // hiển thị ban đầu countdown
  showNumber('A', secA);
  showNumber('B', secB);

  // sync ban đầu lên Blynk
  // A xanh (1), B đỏ (0)
  reportLaneStatusToBlynk(1);
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to Wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Synchronizing time...");
    delay(1000);
  }
  Serial.println("finish Synchronize");


  // kết nối Blynk (blocking)
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}


/*************** LOOP ***************/
void loop() {
  Blynk.run();
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Cannot get time!!");
    delay(1000);
    return;
  }
  // Receive data from UNO
  PedSerial.write(pedestrianStatus);
  if (PedSerial.available()) {
    byte data = PedSerial.read();
    //    data.trim();
    Serial.print("UNO byte: ");
    Serial.println(data);

    if (data == 11) {
      Serial.println("Cancle pedestrian!");
      pedNS = false;
      pedestrianStatus = 11;
    }
    if (data == 10) {

      Serial.println("Pedestrian request received");

      // Chỉ xử lý nếu đang Auto mode
      if (!manual && !flashMode) {
        // Hướng A đang đỏ
        if (stateA == RED) {
          pedestrianStatus = 10;  //PEDESTRIAN_OK
        }
        pedNS = true;

        // gửi thông báo lại cho UNO
        Serial.println("Sent: PEDESTRIAN_ON");
      }
    }
  }

  int hourNow = timeinfo.tm_hour;

  // --- ƯU TIÊN CHẾ ĐỘ ---
  // manual là chế độ chính, flashMode nằm bên trong manual
  if (manual) {
    if (flashMode) {
      //  FLASH MODE trong MANUAL
      clearSegment('A');
      clearSegment('B');
      setA(0, 1, 0);  // A vàng
      setB(0, 1, 0);  // B vàng
      reportLaneStatusToBlynk(-1);
      delay(500);
      setA(0, 0, 0);
      setB(0, 0, 0);
      delay(500);
    } else {
      // MANUAL BÌNH THƯỜNG
      manualModeProcess();
    }

  } else {
    // AUTO MODE
    if (hourNow >= 23 || hourNow < 6) {
      // Đêm: chớp vàng tự động
      clearSegment('A');
      clearSegment('B');
      setA(0, 1, 0);                // A vàng
      setB(0, 1, 0);                // B vàng
      reportLaneStatusToBlynk(-1);
      delay(1000);
      setA(0, 0, 0);
      setB(0, 0, 0);
      delay(1000);
    } else {
      // Ban ngày: chạy auto
      reportLaneStatusToBlynk(-1);
      updateA_auto();
      updateB_auto();
    }
  }
}
