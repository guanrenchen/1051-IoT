#include <BRCClient.h>
#include <RelayRace.h>
#include <SPI.h>
#include <RFID.h>
#include <Navigator.h>

//WIFI setup
#define HW_SERIAL Serial3
BRCClient brcClient(&HW_SERIAL);
#define AP_SSID    "HMKRL"
#define AP_PASSWD  "hatsune39"
#define TCP_IP     "192.168.43.1"
#define TCP_PORT   5000
#define MY_COMM_ID 0x36
RelayRace race(&brcClient, MY_COMM_ID);

//RFID setup
#define SPI_MOSI 51
#define SPI_MISO 50
#define SPI_SCLK 52
#define SPI_SS   53
#define MFRC522_RSTPD 49
RFID rfid(SPI_SS, MFRC522_RSTPD);
static uint8_t status;
static uint8_t sn[MAXRLEN], snBytes;
static uint16_t card_type;
uint8_t cd_RFID;

//Car setup
const uint8_t TRIG[] = {32,34,40,38,36};
const uint8_t ECHO[] = {33,35,41,39,37};
const uint8_t MOTOR[] = {7,6,5,4};
const uint8_t RR=0, RF=1, LR=2, LF=3, MD=4;
const uint8_t SENSOR_NUM=5, MOTOR_NUM=2;
const uint8_t HX=255, LO=0, BASE=100, LH=BASE+5, RH=BASE, RX=HX, LX=HX; 
const uint8_t TIMESTEP=20, PRE_TURN_TIMESTEPS=11, STUCK_RECORD_NUM=10;
const int CALIBRATE_DELAY=7, HALT_DELAY=300, TURN_DELAY=280, U_TURN_DELAY=500, POST_TURN_DELAY=400;
const float ROAD_WIDTH = 30.0, BOUND_DIST = 9.0, CALIBRATE_DIST=9.0, STUCK_DIST=8.0, STUCK_ERROR=1.0;
bool right, left, middle;
uint8_t rightCounter, leftCounter, stuckCounter, sequenceCounter, loopCounter;
float dist[SENSOR_NUM], recordDist[SENSOR_NUM][STUCK_RECORD_NUM];
const uint8_t SEQUENCE[] = {R,R,R,U,L,U,U,L,U,D,D,D,D,D,U,U,U,U,U,U,U,U,255};

//NAVIGATOR SETUP
//Navigator navigator(0,0);

//MANEUVER FUNCTION
void forward     (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void backward    (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],RH); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LH);}
void forwardLeft (){analogWrite(MOTOR[RF],RX); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LO);}
void forwardRight(){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LX); analogWrite(MOTOR[LR],LO);}
void turnRight   (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void turnLeft    (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LO);}
void spinRight   (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],RH); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void spinLeft    (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LH);}
void halt        (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LO);}

//READ RFID TAG
void readTag(){
  if ((status = rfid.findTag(&card_type)) == STATUS_OK) {
    if ((status = rfid.readTagSN(sn, &snBytes)) == STATUS_OK) {
      digitalWrite(LED_BUILTIN, HIGH);
      cd_RFID=15;
      halt();
      if(race.askRFID(sn,false)) while(1);
      rfid.piccHalt();
    }else{
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

//SCAN SURROUNDING WITH ULTRASONIC SENSOR (HC-SR04)
void scan(){
  digitalWrite(TRIG[RF],HIGH);
  digitalWrite(TRIG[LF],HIGH);
  digitalWrite(TRIG[MD],HIGH);  
  delay(TIMESTEP);
  digitalWrite(TRIG[RF], LOW);
  dist[RF] = (float)pulseIn(ECHO[RF],HIGH,TIMESTEP*1000)/2/29.1;
  digitalWrite(TRIG[MD], LOW);
  dist[MD] = (float)pulseIn(ECHO[MD],HIGH,TIMESTEP*1000)/2/29.1;
  digitalWrite(TRIG[LF], LOW);
  dist[LF] = (float)pulseIn(ECHO[LF],HIGH,TIMESTEP*1000)/2/29.1;
  
  right = (dist[RF]>ROAD_WIDTH)? true: false;
  left  = (dist[LF]>ROAD_WIDTH)? true: false;
  middle= (dist[MD]>BOUND_DIST)? true: false;

  Serial.print(dist[LF]); Serial.print("\t"); Serial.print(dist[MD]); Serial.print("\t"); Serial.println(dist[RF]);

  recordDist[RF][stuckCounter]=dist[RF];
  recordDist[LF][stuckCounter]=dist[LF];
  if(++stuckCounter>=STUCK_RECORD_NUM) stuckCounter=0;
}

//ACTIVATED WHEN STUCK
//EVALUTE SITUATION & EXECUTE CORRECTION SEQUENCE
bool stuckHandling(){
  float minLF=999, maxLF=0, minRF=999, maxRF=0;
  for(int i=0; i<STUCK_RECORD_NUM; ++i){
    if(recordDist[LF][i]<minLF) minLF=recordDist[LF][i];
    if(recordDist[LF][i]>maxLF) maxLF=recordDist[LF][i];
    if(recordDist[RF][i]<minRF) minRF=recordDist[RF][i];
    if(recordDist[RF][i]>maxRF) maxRF=recordDist[RF][i];
  }    
  if(maxLF<1000.0 && minLF>0.0 && maxRF<1000.0 && minRF>0.0)
    if((maxLF>STUCK_DIST && maxRF>STUCK_DIST) || (maxLF-minLF>STUCK_ERROR && maxRF-minRF>STUCK_ERROR)) 
      return false;
  
  backward(); delay(POST_TURN_DELAY*0.7);
  if(minLF<=0.0 || maxLF>=1000.0) spinRight();
  else if(minRF<=0.0 || maxRF>=1000.0) spinLeft();
  else (maxRF>maxLF)? spinRight(): spinLeft();
  delay(TURN_DELAY*0.2);
  forward(); delay(POST_TURN_DELAY*0.7);
  
  for(int i=0; i<SENSOR_NUM; ++i)
    for(int j=0; j<STUCK_RECORD_NUM; ++j) 
      recordDist[i][j]=STUCK_DIST+1;

  return true;
}

//KEEP THE CAR AT THE CENTER OF THE ROAD
void calibrate(){
  if(dist[LF]>ROAD_WIDTH && dist[RF]<ROAD_WIDTH){
    dist[RF]>CALIBRATE_DIST? forwardRight(): forwardLeft(); delay(CALIBRATE_DELAY);
  }else if(dist[LF]<ROAD_WIDTH && dist[RF]>ROAD_WIDTH){
    dist[LF]>CALIBRATE_DIST? forwardLeft(): forwardRight(); delay(CALIBRATE_DELAY);
  }else if(dist[LF]<ROAD_WIDTH && dist[RF]<ROAD_WIDTH){
    (dist[LF]<dist[RF])?     forwardRight(): forwardLeft(); delay(CALIBRATE_DELAY);
  }
  forward();
}

void resetCar(){
  halt();
  cd_RFID=0;
  rightCounter=0;
  leftCounter=0;
  stuckCounter=0;
  sequenceCounter=0;
  loopCounter=0;
  for(int i=0; i<SENSOR_NUM; ++i){
    dist[i]=0;
    for(int j=0; j<STUCK_RECORD_NUM; ++j) recordDist[i][j]=STUCK_DIST+1;
  }
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  delay(1000);
}

void act(){
  
  if(right && left && dist[MD]>ROAD_WIDTH+10){
    resetCar();    
    return;
  }
  
  if(stuckHandling()) return;
  (right)? ++rightCounter: rightCounter=0;
  (left)?  ++leftCounter:  leftCounter=0;

  //SEQUENCE
  if(leftCounter>=PRE_TURN_TIMESTEPS || !middle || rightCounter>=PRE_TURN_TIMESTEPS){
    switch(SEQUENCE[sequenceCounter++]){
      case U: 
        for(int i=0; i<10; ++i){calibrate(); delay(CALIBRATE_DELAY);}
        break;
      case R: halt(); delay(HALT_DELAY); spinRight(); delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY); break;
      case D: halt(); delay(HALT_DELAY); (dist[RF]>dist[LF])? spinRight(): spinLeft(); delay(U_TURN_DELAY); halt(); delay(HALT_DELAY); break;
      case L: halt(); delay(HALT_DELAY); spinLeft();  delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY); break;
      default: halt(); while(1); break;
    }
  }else{
    calibrate(); delay(CALIBRATE_DELAY);
    return;
  }
  /*//LEFT WALL
  if(leftCounter>=PRE_TURN_TIMESTEPS){
    halt(); delay(HALT_DELAY); spinLeft();  delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
  }else if(middle){
    calibrate(); 
    return;
  }else if(rightCounter>=PRE_TURN_TIMESTEPS){
    halt(); delay(HALT_DELAY); spinRight(); delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
  }else{
    if(left){      
      halt(); delay(HALT_DELAY); spinLeft();  delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
    }else if(right){
      halt(); delay(HALT_DELAY); spinRight(); delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
    }else{          
      halt(); delay(HALT_DELAY); (dist[RF]>dist[LF])? spinRight(): spinLeft(); delay(U_TURN_DELAY); halt(); delay(HALT_DELAY);
      backward(); delay(POST_TURN_DELAY*0.4); halt(); delay(HALT_DELAY);
    }
  }*/
  /*//RIGHT WALL
  if(rightCounter>=PRE_TURN_TIMESTEPS){
    halt(); delay(HALT_DELAY); spinRight();  delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
  }else if(middle){
    calibrate(); 
    return;
  }else if(leftCounter>=PRE_TURN_TIMESTEPS){
    halt(); delay(HALT_DELAY); spinLeft(); delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
  }else{
    if(right){      
      halt(); delay(HALT_DELAY); spinRight(); delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
    }else if(left){
      halt(); delay(HALT_DELAY); spinLeft(); delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
    }else{          
      halt(); delay(HALT_DELAY); (dist[RF]>dist[LF])? spinRight(): spinLeft(); delay(U_TURN_DELAY); halt(); delay(HALT_DELAY);
      backward(); delay(POST_TURN_DELAY*0.4); halt(); delay(HALT_DELAY);
    }
  }*/
  /*//LEFT WALL (NO RIGHT)
  if(leftCounter>=PRE_TURN_TIMESTEPS && (loopCounter<3 || middle)){
    halt(); delay(HALT_DELAY); spinLeft();  delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
    ++loopCounter;
  }else if(middle){
    calibrate(); 
    return;
  }else if(rightCounter>=PRE_TURN_TIMESTEPS){
    halt(); delay(HALT_DELAY); (dist[RF]>dist[LF])? spinRight(): spinLeft(); delay(U_TURN_DELAY); halt(); delay(HALT_DELAY);
    loopCounter=loopCounter-2; if(loopCounter<0) loopCounter=0;
  }else{
    if(left && loopCounter>3){      
      halt(); delay(HALT_DELAY); spinLeft();  delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
      ++loopCounter;
    }else{          
      halt(); delay(HALT_DELAY); (dist[RF]>dist[LF])? spinRight(): spinLeft(); delay(U_TURN_DELAY); halt(); delay(HALT_DELAY);
      loopCounter=loopCounter-2; if(loopCounter<0) loopCounter=0;
    }
  }*/
  /*//LEFT WALL (NO RIGHT + NAVIGATOR)
  if(leftCounter>=PRE_TURN_TIMESTEPS && (loopCounter<3 || middle)){
    halt(); delay(HALT_DELAY); spinLeft();  delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
    ++loopCounter;
  }else if(middle){
    calibrate(); 
    return;
  }else if(rightCounter>=PRE_TURN_TIMESTEPS){
    halt(); delay(HALT_DELAY); (dist[RF]>dist[LF])? spinRight(): spinLeft(); delay(U_TURN_DELAY); halt(); delay(HALT_DELAY);
    loopCounter=loopCounter-2; if(loopCounter<0) loopCounter=0;
  }else{
    if(left && loopCounter>3){      
      halt(); delay(HALT_DELAY); spinLeft();  delay(TURN_DELAY); halt(); delay(HALT_DELAY); forward(); delay(POST_TURN_DELAY);
      ++loopCounter;
    }else{          
      halt(); delay(HALT_DELAY); (dist[RF]>dist[LF])? spinRight(): spinLeft(); delay(U_TURN_DELAY); halt(); delay(HALT_DELAY);
      loopCounter=loopCounter-2; if(loopCounter<0) loopCounter=0;
    }
  }*/
  rightCounter=0;
  leftCounter=0;
}

void setup()
{
  Serial.begin(9600);
  while (!Serial);
  pinMode(LED_BUILTIN, OUTPUT);

  //Car initialization
  for(int i=0; i<SENSOR_NUM; ++i){
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
    dist[i]=0;
    for(int j=0; j<STUCK_RECORD_NUM; ++j) recordDist[i][j]=STUCK_DIST+1;
  }
  for(int i=0; i<MOTOR_NUM*2; ++i) pinMode(MOTOR[i], OUTPUT);
  rightCounter=0;
  leftCounter=0;
  stuckCounter=0;
  sequenceCounter=0;
  loopCounter=0;

  //RFID initialization
  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000L, MSBFIRST, SPI_MODE3));
  rfid.begin();
  cd_RFID=0;

  //WIFI initialization
  brcClient.begin(9600);
  brcClient.beginBRCClient(AP_SSID, AP_PASSWD, TCP_IP, TCP_PORT);
  while(!race.registerID()) delay(1000);
  race.waitLaunchSignal();
}

void loop()
{
  if(race.dead()){halt(); while(1);}
  scan();
  if(cd_RFID>0) --cd_RFID;
  if(!right && !left && dist[MD]<ROAD_WIDTH && cd_RFID==0) readTag();
  act();
}
