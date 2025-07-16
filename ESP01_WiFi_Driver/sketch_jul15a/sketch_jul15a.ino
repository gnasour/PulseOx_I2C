void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);
  Serial1.flush();

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial1.available()){
    digitalWrite(RGB_BUILTIN, HIGH);
    while(1);
  }

}
