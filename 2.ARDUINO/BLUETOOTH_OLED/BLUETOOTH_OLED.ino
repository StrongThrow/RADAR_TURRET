#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// 블루투스 설정
SoftwareSerial BTSerial(10, 11); // RX=10, TX=11

#define MAX_CM 30
#define SCALE 2              // 1cm = 2px
#define RADAR_RADIUS (MAX_CM * SCALE)
#define CENTER_X (SCREEN_WIDTH / 2)
#define CENTER_Y (SCREEN_HEIGHT - 1)

#define CMD_SIZE 60
#define ARR_CNT 5

char sendBuf[CMD_SIZE];
char receiveBuf[CMD_SIZE];

String receivedData = "";    // 데이터 수신 버퍼
float angle = 0;             // 스캔 라인 각도

int xPos = 0, yPos = 0;
int xPos_before = 0, yPos_before = 0;

bool request_flag = 0;
unsigned long long send_before = 0;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { for(;;); }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Radar Ready");
  display.display();
  delay(1000);
}

void loop() {
  display.clearDisplay();

  // ===== 레이더 외곽 원 + 5cm 눈금 + 숫자 =====
  for(int r=5; r<=MAX_CM; r+=5){
    int radius = r * SCALE;
    display.drawCircle(CENTER_X, CENTER_Y, radius, SSD1306_WHITE);
    int textX = CENTER_X + radius + 2;
    int textY = CENTER_Y - radius;
    display.setCursor(textX, textY);
    display.print(r);
  }

  // ===== 스캔 라인 =====
  float rad = radians(angle);
  int x = CENTER_X + RADAR_RADIUS * cos(rad);
  int y = CENTER_Y - RADAR_RADIUS * sin(rad);
  display.drawLine(CENTER_X, CENTER_Y, x, y, SSD1306_WHITE);

  // ===== DB 데이터 요청 ===== //
  if(request_flag == 0){
    sprintf(sendBuf,"[%s]%s@%s\n","SQL", "GETDB","RADAR");
    BTSerial.write(sendBuf);
    request_flag = 1;
    send_before = millis();
  }

  if(send_before > 3000){
    request_flag = 0;
  }

  

  // ===== DB 데이터 수신 ====== //
  if(BTSerial.available()){
    bluetoothEvent();
  }

  // ====== OLED에 점 찍기 ======= //
  if(xPos != xPos_before || yPos != yPos_before){
    display.fillCircle(xPos_before, yPos_before, 2, BLACK);
    display.fillCircle(xPos, yPos, 2, SSD1306_WHITE);
    xPos_before = xPos;
    yPos_before = yPos;
    display.display();

  }

  // ===== 각도 이동 =====
  angle += 2;
  if(angle >= 180) angle = 0;

  delay(50);
}

void bluetoothEvent()
{
  int i = 0;
  char * pToken;
  char * pArray[ARR_CNT] = {0};
  char recvBuf[CMD_SIZE] = {0};
  int len = BTSerial.readBytesUntil('\n', recvBuf, sizeof(recvBuf) - 1);

#ifdef DEBUG
  Serial.print("Recv : ");
  Serial.println(recvBuf);
#endif

  pToken = strtok(recvBuf, "[@]");
  while (pToken != NULL)
  {
    pArray[i] =  pToken;
    if (++i >= ARR_CNT)
      break;
    pToken = strtok(NULL, "[@]");
  }
  //recvBuf : [XXX_BTM]LED@ON
  //pArray[0] = "XXX_LIN"   : 송신자 ID
  //pArray[1] = "LED"
  //pArray[2] = "ON"
  //pArray[3] = 0x0
   if (!strcmp(pArray[0], "TERMINAL")) {
    return ;
   }
  if (!strcmp(pArray[1], "GETDB")) {
    if (!strcmp(pArray[2], "RADAR")) {
      char *buf = strtok(pArray[3], ",");
      xPos = (int)atof(buf[0]);
      yPos = (int)atof(buf[1]);
    }  
  }
  else if (!strncmp(pArray[1], " New", 4)) // New Connected
  {
    return ;
  }
  else if (!strncmp(pArray[1], " Alr", 4)) //Already logged
  {
    return ;
  }
  else 
  { 
    return;
  }

#ifdef DEBUG
  Serial.print("Send : ");
  Serial.print(sendBuf);
#endif
  BTSerial.write(sendBuf);
}
