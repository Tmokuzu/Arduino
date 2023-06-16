#define encoder0PinA 7 
#define encoder0PinB 6 
 
const int MODE = 9;    // GP8 
const int M1_IN_A = 9;     // GP9 
const int M1_IN_B = 10;    // GP10 
 
struct repeating_timer st_tm1ms; 
 
volatile int encoder0Pos = 0;    // 正負の値を取れるよう変更 
float goal = 0.0;    // 目標位置(rad) 
 
// 位置カウント数（整数）から角度（ラジアン）に変換 
float calc_angle(int pos) 
{ 
    int slit_count = 500;    // 一周当たりのスリット数
    float angle_per_pulse = 2 * PI / (slit_count * 4); 
    float angle = pos * angle_per_pulse; 
    return angle; 
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

void moter(float p)    // p must be -1.0 to 1.0 
{ 
    int analog_range = 4095; 
    if (p > 0) 
    { 
        analogWrite(M1_IN_A, 0); 
        analogWrite(M1_IN_B, (int)(p * analog_range)); 
    } 
    else 
    { 
        analogWrite(M1_IN_B, 0); 
        analogWrite(M1_IN_A, (int)(-p * analog_range)); 
    } 
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
    add_repeating_timer_us(1000, tm1ms, NULL, &st_tm1ms); 
 
    Serial.begin(9600); 
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
    power = p_controller(current_pos, goal);      // P 制御 
    //power = pd_controller(current_pos, goal);     // PD 制御 
    //power = pid_controller(current_pos, goal);    // PID 制御 
    if (power != 0.0) { Serial.println(power); } 
    moter(power); 
 
    return true;
}
void auto_set_goal(float rotate_period) 
{ 
    float time_s = micros() / 1000000.0;    // 現在時刻（s） 
    goal = 2.0 * PI * time_s / (float)rotate_period; 
} 
 
void loop() 
{ 
    char command = '¥0'; 
    if (Serial.available() > 0) 
    { 
        String str = Serial.readString(); 
        char buf[8]; 
        str.toCharArray(buf, 9); 
        command = buf[0]; 
    } 
 
    float pos_diff = PI / 2.0; 
    switch (command) 
    { 
        case 'r': 
            goal += pos_diff; 
            break; 
        case 'l': 
            goal -= pos_diff; 
            break; 
    } 
 
    auto_set_goal(1.0); 
}