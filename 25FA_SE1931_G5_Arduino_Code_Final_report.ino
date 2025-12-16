#include <SoftwareSerial.h>
SoftwareSerial esp(2, 3); // RX, TX

// Pin cấu hình
#define BUTTON_PIN 10  // nút xin đi bộ (PULLUP)
#define IR_PIN     11   // cảm biến IR
#define BUZZER_PIN 12   // buzzer
#define LED_PIN 13 //Led

unsigned long pressTime = 0;
bool waitingConfirm = false;
bool ledOn = false;
bool buzzerPlayed = false;  // dùng thay count
void setup() {
  Serial.begin(115200);
  esp.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP); 
  pinMode(IR_PIN, INPUT);           
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
 
  Serial.println("UNO READY");
}

void beepConfirm() {
  // beep 2 lần ngắn báo người đi bộ đã gửi yêu cầu
  for(int i = 0; i < 2; i++){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

void buzzerFiveTimes(){
      for (int i = 0; i < 5; i++){
          // TODO:buzzer
          digitalWrite(BUZZER_PIN, HIGH);
          delay(500);
          digitalWrite(BUZZER_PIN, LOW);
          delay(500);
        }
     buzzerPlayed = true;


  }

void loop() {
  
  // Nút được nhấn (LOW)
  if (!waitingConfirm && digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("Button pressed! Checking IR...");
    pressTime = millis();
    waitingConfirm = true;
//    esp.println("PEDESTRIAN_REQ");   // gửi sang ESP32
    esp.write(10); //PEDESTRIAN_REQ
  }

  // Sau 2 giây kiểm tra IR
  if (waitingConfirm && millis() - pressTime > 2000) {

    if (digitalRead(IR_PIN) == LOW) { // IR phát hiện người
      Serial.println("Pedestrian confirmed, sending request...");
    } 
    else {
      Serial.println("No pedestrian detected, cancel request.");
//      esp.println("PEDESTRIAN_CANCLE");   // gửi sang ESP32
        esp.write(11); //PEDESTRIAN_CANCLE
    }

    waitingConfirm = false;
  }
  
  // Nhận UART từ ESP32 (debug)
  if (esp.available()) {
    byte msg = esp.read();
    Serial.print("ESP32: ");
    Serial.println(msg);
    if (msg == 10 && !buzzerPlayed) {
      digitalWrite(LED_PIN, HIGH);
  
      if (!buzzerPlayed) {
        buzzerFiveTimes();
      }
    } 
    else if (msg == 11) {
      digitalWrite(LED_PIN, LOW);
      buzzerPlayed = false;
    }
  }
}
