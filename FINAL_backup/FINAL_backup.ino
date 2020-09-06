#include <BRCClient.h>
#include <RelayRace.h>
#include <SPI.h>
#include <RFID.h>
#include <Navigator.h>

//WIFI setup start
#define HW_SERIAL Serial3
BRCClient brcClient(&HW_SERIAL);
#define AP_SSID    "KJChen"
#define AP_PASSWD  "kuanjen0"
#define TCP_IP     "192.168.43.1"
#define TCP_PORT   5000
#define MY_COMM_ID 0x36
RelayRace race(&brcClient, MY_COMM_ID);
//WIFI setup end

//RFID setup start
#define SPI_MOSI 51
#define SPI_MISO 50
#define SPI_SCLK 52
#define SPI_SS   53
#define MFRC522_RSTPD 49
RFID rfid(SPI_SS, MFRC522_RSTPD);
static uint8_t status;
static uint16_t card_type;
static uint8_t sn[MAXRLEN], snBytes;
//RFID setup end

//Car setup start
const int TRIG[] = {32,34,40,38,36};
const int ECHO[] = {33,35,41,39,37};
const int MOTOR[] = {7,6,5,4};
const int RR=0, RF=1, LR=2, LF=3, MD=4;
const String POS[] = {"RR","RF","LR","LF","MD"};
const int SENSOR_NUM=5, MOTOR_NUM=2;
const int HX=255, LO=0, BASE=95, LH=BASE+5, RH=BASE, RX=HX, LX=HX; 
const int TIMESTEP=20, CALIBRATE_DELAY=8, TURN_DELAY=320, TURN180_DELAY=TURN_DELAY*2-50, POST_TURN_DELAY=500;
const int PRE_TURN_TIMESTEPS=8, STUCK_RECORD_NUM=20;
const float ROAD_WIDTH = 30.0, BOUND_DIST = 10.0, STUCK_SIDE_DIST=7.5, CALIBRATE_DIST=11;
float dist[SENSOR_NUM], recordDist[SENSOR_NUM][STUCK_RECORD_NUM];
int rightTurnCounter=0, leftTurnCounter=0, stuckCounter=0, sequenceCounter=0;
bool right, left, middle;
//Car setup end

//Navigator setup start
//Navigator navigator(0,0);
//Navigator setup end

void forward     (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void backward    (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],RH); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LH);}
void forwardLeft (){analogWrite(MOTOR[RF],RX); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LO);}
void forwardRight(){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LX); analogWrite(MOTOR[LR],LO);}
void turnRight   (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void turnLeft    (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LO);}
void spinRight   (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],RH); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void spinLeft    (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LH);}
void halt        (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LO);}

void readTag(){
  CommMsg msg;
  if ((status = rfid.findTag(&card_type)) == STATUS_OK) {
    Serial.print("OK! ");
    Serial.println(card_type);
    if ((status = rfid.readTagSN(sn, &snBytes)) == STATUS_OK) {
      digitalWrite(LED_BUILTIN, HIGH);
      halt();
      brcClient.requestMapData(sn);
      CommMsg msg;
      while(1) {
        if(brcClient.receiveMessage(&msg)) {
          if(msg.type == (char)MAP_PARK_1) {
            halt();
            brcClient.complete();
            while(1);
          }
          else break;
        }
        delay(1);
      }
      rfid.piccHalt();
    }
  }else{
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("No tag.");
  } 
}

void scanSurrounding(){
  digitalWrite(TRIG[RF],HIGH);
  digitalWrite(TRIG[LF],HIGH);
  digitalWrite(TRIG[MD],HIGH);  
  delay(TIMESTEP);
  digitalWrite(TRIG[RF], LOW);
  dist[RF] = (float)pulseIn(ECHO[RF],HIGH)/2/29.1;
  digitalWrite(TRIG[MD], LOW);
  dist[MD] = (float)pulseIn(ECHO[MD],HIGH)/2/29.1;
  digitalWrite(TRIG[LF], LOW);
  dist[LF] = (float)pulseIn(ECHO[LF],HIGH)/2/29.1;
  
  right = (dist[RF]>ROAD_WIDTH)? true: false;
  left  = (dist[LF]>ROAD_WIDTH)? true: false;
  middle= (dist[MD]>BOUND_DIST)? true: false;

  recordDist[RF][stuckCounter]=dist[RF];
  recordDist[LF][stuckCounter]=dist[LF];
  (stuckCounter>=STUCK_RECORD_NUM)? stuckCounter=0: ++stuckCounter;
}

bool stuckHandling(){
  for(int i=0; i<STUCK_RECORD_NUM; ++i)
    if(recordDist[RF][i]>STUCK_SIDE_DIST && recordDist[LF][i]>STUCK_SIDE_DIST) 
      return false;

  backward();
  delay(POST_TURN_DELAY*0.7);
  (dist[RF]>dist[LF])? spinRight(): spinLeft();
  delay(TURN_DELAY/3);
  forward();
  delay(POST_TURN_DELAY*0.7);
  for(int i=0; i<STUCK_RECORD_NUM; ++i){
    recordDist[RF][i]=1000;
    recordDist[LF][i]=1000;
  }
  return true;
}

void calibrate(){
  if(dist[LF]>ROAD_WIDTH && dist[RF]<ROAD_WIDTH){
    dist[RF]>CALIBRATE_DIST? forwardRight(): forwardLeft(); 
    delay(CALIBRATE_DELAY);
  }else if(dist[LF]<ROAD_WIDTH && dist[RF]>ROAD_WIDTH){
    dist[LF]>CALIBRATE_DIST? forwardLeft(): forwardRight();
    delay(CALIBRATE_DELAY);
  }else if(dist[LF]<ROAD_WIDTH && dist[RF]<ROAD_WIDTH){
    (dist[LF]<dist[RF])? forwardRight(): forwardLeft();
    delay(CALIBRATE_DELAY);
  }
  forward();
}

void react(){
  if(right || left || dist[MD]<ROAD_WIDTH) readTag();

  if(stuckHandling()) return;
  (right)? ++rightTurnCounter: rightTurnCounter=0;
  (left)? ++leftTurnCounter: leftTurnCounter=0;

  if(leftTurnCounter>=PRE_TURN_TIMESTEPS){
    spinLeft();
    delay(TURN_DELAY);
    forward();
    delay(POST_TURN_DELAY);
  }else if(middle){
    calibrate();
    return;
  }else{
    scanSurrounding();
    right = (dist[RF]>ROAD_WIDTH)? true: false;
    left  = (dist[LF]>ROAD_WIDTH)? true: false;
    middle= (dist[MD]>BOUND_DIST)? true: false;
    if(left){
      spinLeft();
      delay(TURN_DELAY);
      forward();
      delay(POST_TURN_DELAY);
    }else if(middle){
      calibrate();
      return;
    }else if(right){
      spinRight();
      delay(TURN_DELAY);
      forward();
      delay(POST_TURN_DELAY);
    }else{
      (dist[RF]>dist[LF])? spinRight(): spinLeft();
      delay(TURN180_DELAY);
      forward();
      delay(POST_TURN_DELAY);
    }
  }
  rightTurnCounter=0;
  leftTurnCounter=0;
}

void setup()
{
  Serial.begin(9600);
  while (!Serial);
  pinMode(LED_BUILTIN, OUTPUT);

  //Car initialization start
  for(int i=0; i<SENSOR_NUM; ++i){
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
    dist[i]=0;
    for(int j=0; j<STUCK_RECORD_NUM; ++j) recordDist[i][j]=1000;
  }
  for(int i=0; i<MOTOR_NUM*2; ++i) pinMode(MOTOR[i], OUTPUT);
  rightTurnCounter=0;
  leftTurnCounter=0;
  stuckCounter=0;
  sequenceCounter=0;
  //Car initialization end

  //RFID initialization start
  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000L, MSBFIRST, SPI_MODE3));
  rfid.begin();
  //RFID initialization end
  
  //WIFI initialization start
  brcClient.begin(9600);
  brcClient.beginBRCClient(AP_SSID, AP_PASSWD, TCP_IP, TCP_PORT);
  while(!race.registerID()) delay(1000);
  race.waitRoundStart();
  //WIFI initialization end
  
}

void loop()
{
  scanSurrounding();
  react();
}
