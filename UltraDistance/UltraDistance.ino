//*******************************************************************************
//*超音波センサと温度センサを使って距離を表示するプログラム
//*******************************************************************************
double Duration = 0; //受信した間隔
double Distance = 0; //距離
void setup() {
  Serial.begin( 9600 );
  pinMode( 2, INPUT );
  pinMode( 3, OUTPUT );
  pinMode( 4, OUTPUT );
  pinMode( 5, INPUT );
}
void loop() {
  int ans , temp , tv ; //温度計測用の変数
  ans = analogRead(0) ; //アナログ０番ピンからセンサ値を読込む
  tv = map(ans,0,1023,0,5000) ; //センサ値を電圧に変換する
  temp = map(tv,300,1600,-30,100) ; //電圧から温度に変換する(LM61は-30度から100度まで計測)
  Serial.print("temp:");
  Serial.print(temp); //温度を表示
  Serial.print("c");
  digitalWrite( 2, HIGH ); //超音波を出力
  delayMicroseconds( 10 ); //
  digitalWrite( 2, LOW );
  Duration = pulseIn( 3, HIGH,5000 );
  if (Duration > 0) {
    Distance = Duration/2;
    float sspeed = 331.5+0.6*temp;
    Serial.print("\tspeed:");
    Serial.print(sspeed);
    Serial.print("m/sec");
    Distance = Distance*sspeed*100/1000000; //
    Serial.print("\tDistance:");
    Serial.print(Distance);
    Serial.print("cm");
  }
  Serial.println("");
  delay(500);
}