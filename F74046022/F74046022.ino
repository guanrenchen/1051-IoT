const int TRIG[] = {32,34,40,38,36};
const int ECHO[] = {33,35,41,39,37};
const int MOTOR[] = {11,10,9,8};
const int RR=0, RF=1, LR=2, LF=3, MD=4;
const String POS[] = {"RR","RF","LR","LF","MD"};

const int BASE=85, LH=BASE, RH=LH, RX=RH+80, LX=LH+80, LO=0; 
const int SENSOR_NUM=5, MOTOR_NUM=2;
const int TIMESTEP=30, CALIBRATE_DELAY=15, TURN_DELAY=375, POST_TURN_DELAY=300;
const int PRE_TURN_TIMESTEPS=9, STUCK_RECORD_NUM=15;
const float ERROR_CALIBRATE=2.0;
const float ROAD_WIDTH = 30.0, TARGET_DIST = 10.0, STUCK_SIDE_DIST=6.5;
float dist[SENSOR_NUM], recordDist[SENSOR_NUM][STUCK_RECORD_NUM];
int rightTurnCounter=0, stuckCounter=0;

void forward      (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void backward     (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],RH); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LH);}
void forwardLeft  (){analogWrite(MOTOR[RF],RX); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LO);}
void forwardRight (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LX); analogWrite(MOTOR[LR],LO);}
void turnRight    (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void turnLeft     (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LO);}
void spinRight    (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],RH); analogWrite(MOTOR[LF],LH); analogWrite(MOTOR[LR],LO);}
void spinLeft     (){analogWrite(MOTOR[RF],RH); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LH);}
void halt         (){analogWrite(MOTOR[RF],LO); analogWrite(MOTOR[RR],LO); analogWrite(MOTOR[LF],LO); analogWrite(MOTOR[LR],LO);}

int compareDist(float a, float b, float error){
  if(a>ROAD_WIDTH || b>ROAD_WIDTH) return 0;
  if((a-b)>error) return -1;
  if((b-a)>error) return 1;
  return 0;
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

  recordDist[RF][stuckCounter]=dist[RF];
  recordDist[LF][stuckCounter]=dist[LF];
  (stuckCounter>=STUCK_RECORD_NUM)? stuckCounter=0: ++stuckCounter;
}

bool stuckHandling(){
  for(int i=0; i<STUCK_RECORD_NUM; ++i)
    if(recordDist[RF][i]>STUCK_SIDE_DIST && recordDist[LF][i]>STUCK_SIDE_DIST) 
      return false;
      
  backward();
  delay(250);
  (dist[RF]>dist[LF])? spinRight(): spinLeft();
  delay(100);
  forward();
  delay(300);
  for(int i=0; i<STUCK_RECORD_NUM; ++i){
    recordDist[RF][i]=1000;
    recordDist[LF][i]=1000;
  }
  
  return true;
}

void calibrate(){
  switch(compareDist(dist[LF], dist[RF], ERROR_CALIBRATE)){
    case -1: 
      forwardLeft(); 
      delay(CALIBRATE_DELAY);
      break;
    case 1:  
      forwardRight();
      delay(CALIBRATE_DELAY);
      break;
    default:
      break;
  }
  forward();
}

void react(){
  bool right, left, middle;
  right = (dist[RF]>ROAD_WIDTH) ? true: false;
  left  = (dist[LF]>ROAD_WIDTH) ? true: false;
  middle= (dist[MD]>TARGET_DIST)? true: false;

  if(stuckHandling()) return;
  (right)? ++rightTurnCounter: rightTurnCounter=0;
  
  if(rightTurnCounter>=PRE_TURN_TIMESTEPS){
    spinRight();
    delay(TURN_DELAY);
    forward();
    delay(POST_TURN_DELAY);
  }else if(middle){
    calibrate();
    return;
  }else{
    scanSurrounding();
    right = (dist[RF]>ROAD_WIDTH) ? true: false;
    left  = (dist[LF]>ROAD_WIDTH) ? true: false;
    middle= (dist[MD]>TARGET_DIST)? true: false;
    if(right){
      spinRight();
      delay(TURN_DELAY);
      forward();
      delay(POST_TURN_DELAY);
    }else if(middle){
      calibrate();
      return;
    }else if(left){
      spinLeft();
      delay(TURN_DELAY);
      forward();
      delay(POST_TURN_DELAY);
    }else{
      (dist[RF]>dist[LF])? spinRight(): spinLeft();
      delay(TURN_DELAY*2);
      halt();
    }
  }
  rightTurnCounter=0;
}

void setup() {
  Serial.begin(9600);
  for(int i=0; i<SENSOR_NUM; ++i){
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
    dist[i]=0;
    for(int j=0; j<STUCK_RECORD_NUM; ++j) recordDist[i][j]=1000;
  }
  for(int i=0; i<MOTOR_NUM*2; ++i) pinMode(MOTOR[i], OUTPUT);
}

void loop(){
  scanSurrounding();
  react();
}
