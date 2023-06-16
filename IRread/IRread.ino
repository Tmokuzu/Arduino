void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(IRread(0));
  delay(500);
}

double IRread(int PinNo) {
 double ans ;
 int i ;
 ans = 0 ;
 for (i=0 ; i < 1000 ; i++) {
   ans  = ans + analogRead(PinNo) ;   // 指定のアナログピン(0番端子)から読取ります
  }
  //return ans/1000;//センサーの値を1000回読み取った平均値を返します
  return 19501.14 * pow(ans/1000 , -1.256676);
}