
// Sample encoder code copied from
// https://playground.arduino.cc/Main/RotaryEncoders/
// section: Interrupt Example (the Encoder interrupts the processor)

// doEncoderA,B関数はサンプルと同一
#define encoderOPinA 7
#define encoderOPinB 6

const int MODE = 9;    // GP8
const int M1_IN_A = 9;     // GP9
const int M1_IN_B = 10;    // GP10

struct repeating_timer st_tm1ms;

volatile int encoderOPos = 0;    // 正負の値を取れるよう変更
float goal = 0.0;    // 目標位置(rad)


// 位置カウント数（整数）から角度（ラジアン）に変換
float calc_angle(int pos)
{
    int slit_count = 500;    // 一周当たりのスリット数
    
    // 1パルス当たりの角度
    // A相とB相の2つの出力信号を使用すると、4倍の角度分解能が得られる点に注意
    float angle_per_pulse = 2 * PI / (slit_count * 4);
    float angle = pos * angle_per_pulse;
    return angle;
}

void doEncoderA()
{
    // look for a low-to-high on channel A
    if (digitalRead(encoderOPinA) == HIGH)
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(encoderOPinB) == LOW)
        {
            encoderOPos = encoderOPos + 1;    // CW
        }
        else
        {
            encoderOPos = encoderOPos - 1;    // CCW
        }
    }
    else    // must be a high-to-low edge on channel A
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(encoderOPinB) == HIGH)
        {
            encoderOPos = encoderOPos + 1;    // CW
        }
        else
        {
            encoderOPos = encoderOPos - 1;    // CCW
        }
    }
    //Serial.println(encoderOPos, DEC);    // デバッグ用
}

void doEncoderB()
{
    // look for a low-to-high on channel B
    if (digitalRead(encoderOPinB) == HIGH)
    {
        // check channel A to see which way encoder is turning
        if (digitalRead(encoderOPinA) == HIGH)
        {
            encoderOPos = encoderOPos + 1;    // CW
        }
        else
        {
            encoderOPos = encoderOPos - 1;    // CCW
        }
    }
    else    // must be a high-to-low edge on channel B
    {
        // check channel A to see which way encoder is turning
        if (digitalRead(encoderOPinA) == LOW)
        {
            encoderOPos = encoderOPos + 1;    // CW
        }
        else
        {
            encoderOPos = encoderOPos - 1;    // CCW
        }
    }
    //Serial.println(encoderOPos, DEC);    // デバッグ用
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
    pinMode(encoderOPinA, INPUT); 
    pinMode(encoderOPinB, INPUT); 
    pinMode(MODE, OUTPUT); 
    digitalWrite(MODE, LOW); 
    analogWriteFreq(2000);    // PWM set to 2kHz 
    analogWriteResolution(12);    // Analog range set to 0-4095 
 
    // 割り込み関数指定 
    attachInterrupt(encoderOPinA, doEncoderA, CHANGE); 
    attachInterrupt(encoderOPinB, doEncoderB, CHANGE); 
 
    // Repeating timer interrupt at 1ms 
    int loop_freq = 10000;    // ループ周期周波数 
    add_repeating_timer_us(1000000 / loop_freq, my_timer, NULL, &st_timer); 
 
    Serial.begin(9600); 
}

// P制御
float p_controller(float current_pos, float goal)
{
    float gain = 0.04 / PI;    // ゲイン

    float power = gain * (goal - current_pos);
    if (power > 1.0) { power = 1.0; }
    else if (power < -1.0) { power = -1.0; }
    return -power;
}

// PD制御
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

// PID制御
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

float rotate_val = 1.0;
bool tm1ms(struct repeating_timer *t)
{
    float current_pos = calc_angle(encoderOPos);    // 現在位置

    float power;
    power = p_controller(current_pos, goal);      // P制御
    //power = pd_controller(current_pos, goal);     // PD制御
    //power = pid_controller(current_pos, goal);    // PID制御
    if (power != 0.0) { Serial.println(power); }
    moter(power);

    auto_set_goal(rotate_val);

    return true;
}

// 一定速度で回転させたいときに目標位置を自動設定する関数
// 引数：回転周期、指定時間当たり1回転（単位はs）
void auto_set_goal(float rotate_period)
{
    float time_s = micros() / 1000000.0;    // 現在時刻（s）
    //goal = 2.0 * PI * time_s / (float)rotate_period;
    goal += 2.0 * PI * rotate_period / 1000;
}
float my_spring(float x) 
{ 
    float k = 0.1;    // ばね係数 
 
    float power = k * x; 
    if (power > 1.0) { power = 1.0; } 
    else if (power < -1.0) { power = -1.0; }
    return power; 
}

bool my_timer(struct repeating_timer *t) 
{ 
    float current_pos = calc_angle(encoderOPos);    // 現在位置 
     
    float diff = goal - current_pos; 
    if (diff > 0) 
    { 
        float power = -my_spring(goal - current_pos); 
        if (power != 0.0) { Serial.println(power); } 
        moter(power); 
    } 
 
    return true; 
} 


void loop()
{
    char command = '\0';
    if (Serial.available() > 0)
    {
        String str = Serial.readString();
        char buf[8];
        str.toCharArray(buf, 9);
        command = buf[0];
    }
}
