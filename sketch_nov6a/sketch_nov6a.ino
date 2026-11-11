const int DIR2 = 8;
const int STEP2 = 9;
const int ENABLE2 = 10;

const int DIR1 = 2;
const int STEP1 = 3;
const int ENABLE1 = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(ENABLE2, OUTPUT);
  digitalWrite(DIR2, HIGH);
  
  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(ENABLE1, OUTPUT);
  digitalWrite(DIR1, HIGH);
}

void loop() {
  unsigned long now = millis();
  digitalWrite(ENABLE2, LOW);
  digitalWrite(ENABLE1, LOW);
  while(millis() - now <= 5000) {
    for( int i = 0; i < 500; i++){
      digitalWrite(STEP2, HIGH);
      digitalWrite(STEP1, HIGH);
      delay(1);
      digitalWrite(STEP2, LOW);
      digitalWrite(STEP1, LOW);
      delay(1);
    }
  }
  digitalWrite(ENABLE2, HIGH);
  digitalWrite(ENABLE1, HIGH);
  delay(5000);
  // put your main code here, to run repeatedly:
}
