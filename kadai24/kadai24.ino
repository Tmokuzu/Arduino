#define encoder0PinA 7
#define encoder0PinB 6

const float Target=4.5;
const float Kp=0.1;
const float Ki=0.5;
const float Kd=0.5;

const int MODE = 8;
const int M1_IN_A = 9;
const int M1_IN_B = 10;

struct repeating_timer st_tm1ms;

volatile int encoder0Pos = 0;
float goal = 0.0;


float x;
float dt;
float pretime;
float P, I, D, preP;
uint8_t duty;

uint8_t a;
uint8_t b;

float calc_angle(int pos){
  int slit_count = 500;
  float angle_per_pulse = 2 * PI / (slit_count * 4);
  float angle = pos * angle_per_pulse;
  return angle;
}

void moter(float p){
  int analog_range = 4096;
  if (p>0){
    analogWrite(M1_IN_A, 0);
    analogWrite(M1_IN_B, (int)(p*analog_range));
  }else{
    analogWrite(M1_IN_B, 0);
    analogWrite(M1_IN_A,(int)(-p*analog_range));
  }
}

void doEncoderA(){

  if(digitalRead(encoder0PinA)== HIGH){
    if (digitalRead(encoder0PinB)==LOW){
      encoder0Pos = encoder0Pos + 1;
    }else{
      encoder0Pos = encoder0Pos - 1;
    }
  }else{
    if (digitalRead(encoder0PinB) == HIGH){
      encoder0Pos = encoder0Pos + 1;      
    }else{
      encoder0Pos = encoder0Pos - 1;
    }
  }
}

void doEncoderB(){

  if(digitalRead(encoder0PinB)== HIGH){
    if (digitalRead(encoder0PinA) == HIGH){
      encoder0Pos = encoder0Pos + 1;
    }else{
      encoder0Pos = encoder0Pos - 1;
    }
  }else{
    if (digitalRead(encoder0PinA) == LOW){
      encoder0Pos = encoder0Pos + 1; 
    }else{
      encoder0Pos = encoder0Pos - 1;
    }
  }
}

float p_controller(float current_pos, float goal) 
{ 
    float gain = 0.04 / PI;    // ゲイン 
 
    float power = gain * (goal - current_pos); 
    if (power > 1.0) { power = 1.0; } 
    else if (power < -1.0) { power = -1.0; } 
    return -power; 
} 
 
// PD 制御 
float pd_controller(float current_pos, float goal) 
{ 
    static float prev_pos = 0.0; 
    static unsigned long prev_time = 0; 
 
    float p = 0.04 / PI; 
    float d = 1000.0; 
 
    float dt_us = micros() - prev_time; 
    float v = (current_pos - prev_pos) / (float)dt_us; 
    float power = p * (goal - current_pos) - (d * v); 
    if (power > 1.0) { power = 1.0; } 
    else if (power < -1.0) { power = -1.0; } 
 
    prev_pos = current_pos; 
    prev_time = micros(); 
    return -power; 
}

float pid_controller(float current_pos, float goal) 
{ 
    static float i_component = 0.0; 
    static unsigned long prev_time = 0; 
    static float prev_pos = 0.0; 
 
    float p = 0.04 / PI; 
    float i = 0.0001 / PI; 
    float d = 1000.0; 
 
    float v = (current_pos - prev_pos) / (float)(micros() - prev_time); 
    float p_component = goal - current_pos; 
    i_component = i_component + p_component; 
    prev_time = micros(); 
    prev_pos = current_pos; 
    float power = p * p_component + i * i_component - d * v; 
    if (power > 1.0) { power = 1.0; } 
    else if (power < -1.0) { power = -1.0; } 
 
    return -power; 
} 
 
bool tm1ms(struct repeating_timer *t) 
{ 
    float current_pos = calc_angle(encoder0Pos);    // 現在位置 
 
    float power; 
    //power = p_controller(current_pos, goal);      // P 制御 
    //power = pd_controller(current_pos, goal);     // PD 制御 
    power = pid_controller(current_pos, goal);    // PID 制御 
    if (power != 0.0) { Serial.println(power); } 
    moter(power); 
 
    return true;
}

void auto_set_goal(float rotate_period) 
{ 
    float time_s = micros() / 1000000.0;    // 現在時刻（s） 
    goal = 2.0 * PI * time_s / (float)rotate_period; 
}


void setup() 
{ 
    pinMode(encoder0PinA, INPUT); 
    pinMode(encoder0PinB, INPUT); 
    pinMode(MODE, OUTPUT); 
    digitalWrite(MODE, LOW); 
    analogWriteFreq(2000);    // PWM set to 2kHz 
    analogWriteResolution(12);    // Analog range set to 0-4095 
 
    // 割り込み関数指定 
    attachInterrupt(encoder0PinA, doEncoderA, CHANGE); 
    attachInterrupt(encoder0PinB, doEncoderB, CHANGE); 
 
    // Repeating timer interrupt at 1ms 
    int loop_freq = 100;    // ループ周期周波数 
    add_repeating_timer_us(1000000 / loop_freq, my_timer, NULL, &st_tm1ms); 
 
    Serial.begin(9600); 
} 
 
// ばねの力を返す 
// 引数はばねの伸びx 
float spring(float x) 
{ 
    float k = 0.1;    // ばね係数 
 
    float elasticity = k * x; 
    if(elasticity > 1.0){
      elasticity = 1.0;
    } 
    else if(elasticity < -1.0){
      elasticity = -1.0;
    }
    return elasticity; 
} 
 
bool my_timer(struct repeating_timer *t) 
{ 
    float current_pos = calc_angle(encoder0Pos);    // 現在位置 
     
    float diff = goal - current_pos; 
    if (diff > 0) 
    { 
        float power = -spring(goal - current_pos); 
        if (power != 0.0) { Serial.println(power); } 
        moter(power);
    }
    return true; 
} 
 
void loop() 
{
  int data = -1;
  if(Serial.available()>0){
    int incomingByte = Serial.read();
    if(incomingByte >= 48){
      data = incomingByte - 48;
    }
  }
}