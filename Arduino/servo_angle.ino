#include <Servo.h>

const int SERVO_PIN = 10;          // 서보 모터 핀
const int IR_DETECT_PIN_1 = 3;     // 첫 번째 IR 센서 핀
const int IR_DETECT_PIN_2 = 12;    // 두 번째 IR 센서 핀
const int RESET_BUTTON_PIN = 4;    // 리셋 버튼 핀

Servo myservo;                     // 서보 객체 생성
unsigned long detectTime = 0;      // 물체가 감지된 시간을 저장할 변수
bool detected = false;             // 물체가 감지되었는지 여부를 저장할 변수
bool isOpen = false;               // 서보 모터가 열렸는지 여부를 저장할 변수
bool isFirstObject = true;         // 첫 번째 물체 여부를 저장할 변수

void setup() {
  pinMode(IR_DETECT_PIN_1, INPUT);     // 첫 번째 IR 센서 핀을 입력으로 설정
  pinMode(IR_DETECT_PIN_2, INPUT);     // 두 번째 IR 센서 핀을 입력으로 설정
  pinMode(RESET_BUTTON_PIN, INPUT);    // 리셋 버튼 핀을 입력으로 설정
  myservo.attach(SERVO_PIN);           // 서보 모터를 SERVO 핀에 연결
  myservo.write(0);                    // 서보 모터를 초기 위치로 설정
}

void loop() {
  int detect1 = digitalRead(IR_DETECT_PIN_1);    // 첫 번째 IR 센서 상태 읽기
  int detect2 = digitalRead(IR_DETECT_PIN_2);    // 두 번째 IR 센서 상태 읽기
  int resetButtonState = digitalRead(RESET_BUTTON_PIN); // 리셋 버튼 상태 읽기

  if (resetButtonState == HIGH) {    // 리셋 버튼이 눌렸을 때
    // 모든 상태 변수를 초기화
    detected = false;
    isOpen = false;
    isFirstObject = true;
    myservo.write(0);  // 서보 모터를 초기 위치로 설정
    delay(500); // 버튼 디바운스 처리
    return; // 리셋 후 바로 루프 종료
  }

  if ((detect1 == LOW || detect2 == LOW) && !detected) { // 한 센서라도 LOW이고 아직 감지 상태가 아닐 때
    detectTime = millis();           // 현재 시간을 저장
    detected = true;                 // 물체 감지 상태로 설정

    if (isFirstObject) {
      myservo.write(90);             // 서보 모터를 90도로 회전
      isOpen = true;                 // 서보 모터가 열린 상태로 설정
      detectTime = millis();         // 현재 시간을 다시 저장 (2초 대기를 위해)
      isFirstObject = false;         // 첫 번째 물체 처리 완료
    }
  } 
  else if ((detect1 == HIGH && detect2 == HIGH) && detected) { // 두 센서가 모두 HIGH이고 감지 상태일 때
    detected = false;                // 물체 감지 상태를 해제
  }

  if (isFirstObject && isOpen && millis() - detectTime >= 2000) {
    myservo.write(0);                // 서보 모터를 0도로 회전
    isOpen = false;                  // 서보 모터 상태 초기화
  } 
  else if (!isFirstObject && detected && millis() - detectTime >= 500 && !isOpen) {
    myservo.write(90);               // 서보 모터를 90도로 회전
    isOpen = true;                   // 서보 모터가 열린 상태로 설정
    detectTime = millis();           // 현재 시간을 다시 저장 (2초 대기를 위해)
  } 
  else if (!isFirstObject && isOpen && millis() - detectTime >= 2000) {
    myservo.write(0);                // 서보 모터를 0도로 회전
    isOpen = false;                  // 서보 모터 상태 초기화
    detected = false;                // 물체 감지 상태 초기화
  }

  delay(100); // 센서 읽기 및 모터 움직임 간의 안정성을 위해 잠시 대기
}
