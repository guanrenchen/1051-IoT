#include <BRCClient.h>
#include <RelayRace.h>
#include <SPI.h>
#include <RFID.h>

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
const int HX=255, LO=0, BASE=90, LH=BASE, RH=BASE, RX=HX, LX=HX; 
const int TIMESTEP=30, CALIBRATE_DELAY=15, TURN_DELAY=350, TURN180_DELAY=TURN_DELAY*2-100, POST_TURN_DELAY=250;
const int PRE_TURN_TIMESTEPS=5, STUCK_RECORD_NUM=15;
const float ROAD_WIDTH = 30.0, BOUND_DIST = 10.0, STUCK_SIDE_DIST=7.5, CALIBRATE_DIST=11;
float dist[SENSOR_NUM], recordDist[SENSOR_NUM][STUCK_RECORD_NUM];
int rightTurnCounter=0, leftTurnCounter=0, stuckCounter=0, sequenceCounter=0;
//sequence : LBLLRBMLBMMBRMBLRRBR
const int L=0,M=1,R=2,B=3,END=4;
const int SEQUENCE[]={L,B,L,L,R,B,M,L,B,M,M,B,R,M,B,L,R,R,B,R,END};
//Car setup end

void forward     (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void backward    (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],RH); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LH);}
void forwardLeft (){analogWrite(MOTOR[RF],RX); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void forwardRight(){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LX); analogWrite(MOTOR[LR],LO);}
void turnRight   (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void turnLeft    (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LO);}
void spinRight   (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],RH); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void spinLeft    (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LH);}
void halt        (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LO);}

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

  Serial.print(dist[LF]);
  Serial.print("\t");
  Serial.println(dist[RF]);

  recordDist[RF][stuckCounter]=dist[RF];
  recordDist[LF][stuckCounter]=dist[LF];
  (stuckCounter>=STUCK_RECORD_NUM)? stuckCounter=0: ++stuckCounter;
}

bool stuckHandling(){
  for(int i=0; i<STUCK_RECORD_NUM; ++i)
    if(recordDist[RF][i]>STUCK_SIDE_DIST && recordDist[LF][i]>STUCK_SIDE_DIST) 
      return false;

  backward();
  delay(POST_TURN_DELAY);
  (dist[RF]>dist[LF])? spinRight(): spinLeft();
  delay(TURN_DELAY/3);
  forward();
  delay(POST_TURN_DELAY);
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
  bool right, left, middle;
  right = (dist[RF]>ROAD_WIDTH)? true: false;
  left  = (dist[LF]>ROAD_WIDTH)? true: false;
  middle= (dist[MD]>BOUND_DIST)? true: false;

  if(stuckHandling()) return;
  (right)? ++rightTurnCounter: rightTurnCounter=0;
  (left)? ++leftTurnCounter: leftTurnCounter=0;
  /*
  if(rightTurnCounter>PRE_TURN_TIMESTEPS || leftTurnCounter>PRE_TURN_TIMESTEPS || !middle){
    switch(SEQUENCE[sequenceCounter++]){
      case L: 
        spinLeft(); 
        delay(TURN_DELAY); 
        forward();
        delay(POST_TURN_DELAY);
        break;
      case M: 
        forward();
        delay(POST_TURN_DELAY);
        break;
      case R: 
        spinRight(); 
        delay(TURN_DELAY); 
        forward();
        delay(POST_TURN_DELAY);
        break;
      case B: 
        (dist[RF]>dist[LF])? spinRight(): spinLeft();
        delay(TURN180_DELAY);       
        break;
      default: 
        halt();
        digitalWrite(LED_BUILTIN, HIGH);
        while(1);
        break;
    }
    rightTurnCounter=0;
    leftTurnCounter=0;
  }else{
    calibrate();
  }
  */
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
      halt();
    }
  }
  rightTurnCounter=0;
  leftTurnCounter=0;
  
}

void readTag(){
  if ((status = rfid.findTag(&card_type)) == STATUS_OK) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("OK! ");
    Serial.println(card_type);
    if ((status = rfid.readTagSN(sn, &snBytes)) == STATUS_OK) {
      //race.askRFID(sn);
      rfid.piccHalt();
    }
  }else{
    digitalWrite(LED_BUILTIN, LOW); 
    Serial.println("No tag.");
  }
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
  /*
  //WIFI initialization start
  brcClient.begin(9600);
  brcClient.beginBRCClient(AP_SSID, AP_PASSWD, TCP_IP, TCP_PORT);
  while(!race.registerID()) delay(1000);
  race.waitLaunchSignal();
  //WIFI initialization end
  */
}

void loop()
{
  readTag();
  scanSurrounding();
  react();
}
