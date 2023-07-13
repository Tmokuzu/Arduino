void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(String(IRread(0) * CheckState(1),0));
  //Serial.println(String(IRread(1),0));
  //Serial.println(CheckState(1));
  delay(1);
}

double IRread(int PinNo) {
 double ans ;
 int i ;
 ans = 0 ;
 for (i=0 ; i < 500 ; i++) {
   ans  = ans + analogRead(PinNo) ;   // 指定のアナログピン(0番端子)から読取ります
  }
  //return ans/1000;//センサーの値を1000回読み取った平均値を返します
  return 19501.14 * pow(ans/500 , -1.256676) * 10;
}

int CheckState(int PinNo) {
  int i;
  int ans = 0;
  int State;
  for (i=0; i<10 ; i++) {
    ans = ans + analogRead(PinNo);
  }
  if(ans>1200) State=1;
  else State = 0;
  return State;
}