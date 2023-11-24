//Encoder & PWM sample. PID control
const int encoder0PinA = 16;
const int encoder0PinB = 17;
const int M1_IN_A = 33; 
const int M1_IN_B = 32;
volatile int encoder0Pos = 0; 

#define debug_pin 2
#define AVERAGE_NUM 10
#define F_MAX 1.0
#define F_MIN 0.2

// #define P_MAX 0.003
// #define P_MIN 0.001

long prev_Position[AVERAGE_NUM]  = {0};
unsigned long prev_t=0;
int last = 0;
float P_Constant, D_Constant;
long Position,Destination=0;
float Force, Velocity;

void motor(float p)// p must be -1.0 to 1.0
{
  if(p > 0){
    ledcWrite(2, 0);
    ledcWrite(1, (int)(4095*p));
  }
  else{
    ledcWrite(1, 0);
    ledcWrite(2, (int)(4095*-p));
  }
}

void setup() {
  Serial.begin (921600);
  // while (!Serial);
  
  pinMode(debug_pin, OUTPUT);
  
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(M1_IN_A, OUTPUT);
  pinMode(M1_IN_B, OUTPUT);
  attachInterrupt(encoder0PinA, doEncoderA, CHANGE); //割り込み関数指定
  attachInterrupt(encoder0PinB, doEncoderB, CHANGE); //割り込み関数指定

  ledcSetup(1, 19531, 12);
  ledcAttachPin(33, 1);
  ledcSetup(2, 19531, 12);
  ledcAttachPin(32, 2);

  Force = 0.0;

  // P_Constant = P_MIN;
  // D_Constant = 0.00;
}

void loop() {
  unsigned long t = micros();
  int dt = t - prev_t;
  prev_t = t;

  // データ取得
  if(Serial.available() > 1){
    digitalWrite(debug_pin, HIGH);
    // header
    int val = Serial.read();
    if(val == 0xFF){
      // data
      int d = Serial.read();
      if(d) Force = F_MAX;
      else Force = F_MIN;
    }
  }
  else{
    digitalWrite(debug_pin, LOW);
  }
  
  Position = encoder0Pos;
  Velocity = (float)(Position - prev_Position[last]);
  prev_Position[last] = Position;
  last = (last+1)%AVERAGE_NUM;

  // P制御部分
  // Force = P_Constant * (float)(Destination - Position) - D_Constant * (float)Velocity; 
  // Force = P_Constant * (float)(Destination - Position);

  if(Force > 1.0) Force = 1.0;
  else if(Force < -1.0) Force = -1.0;

  // 握ってない時は動かさない
  if(abs(Position) < 20) Force = 0.0;

  // 手の姿勢パラメータ
  Serial.println(abs(Position)/300.0);

  motor(Force);
//  motor(0.0);

  delay(2);
}

void doEncoderA() {
  // look for a low-to-high on channel A 
  if (digitalRead(encoder0PinA) == HIGH){
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW){
      encoder0Pos = encoder0Pos + 1;         // CW
//      Serial.println("CW_A1");
    }else{
      encoder0Pos = encoder0Pos - 1;         // CCW
//      Serial.println("CCW_A1");
    }
  }else{// must be a high-to-low edge on channel A
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH){
      encoder0Pos = encoder0Pos + 1;          // CW
//      Serial.println("CW_A2");
    }else{
      encoder0Pos = encoder0Pos - 1;          // CCW
//      Serial.println("CCW_A2");
    }
  }
  //Serial.println (encoder0Pos, DEC); //デバグ用：コメントアウト
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if(digitalRead(encoder0PinB) == HIGH){
    // check channel A to see which way encoder is turning
    if(digitalRead(encoder0PinA) == HIGH){
      encoder0Pos = encoder0Pos + 1;         // CW
//      Serial.println("CW_B1");
    }else{
      encoder0Pos = encoder0Pos - 1;         // CCW
//      Serial.println("CCW_B1");
    }
  }else{// Look for a high-to-low on channel B
    // check channel B to see which way encoder is turning
    if(digitalRead(encoder0PinA) == LOW){
      encoder0Pos = encoder0Pos + 1;          // CW
//      Serial.println("CW_B2");
    }else{
      encoder0Pos = encoder0Pos - 1;          // CCW
//      Serial.println("CCW_B2");
    }
  }
}
