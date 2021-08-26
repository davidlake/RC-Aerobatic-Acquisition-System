char oldVal = '0';
char newVal;

void setup() {
  pinMode(9, OUTPUT);
  analogWrite(9,33);
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() 
{
  if (Serial.available() > 0) 
  {
    newVal = Serial.read();
    if (newVal != oldVal)
    {
      Serial.println(newVal); 
      switch(newVal)
      {
        case 'a': //-100
          Serial.println(newVal);
          oldVal = newVal; 
          analogWrite(9,33); //12.93%DC
          break;
        case 'b': //0
          Serial.println(newVal); 
          oldVal = newVal; 
          analogWrite(9,26);//10.13%DC
          break;
        case 'c': //+100
          Serial.println(newVal); 
          oldVal = newVal; 
          analogWrite(9,19);//7.33%DC
          break;
      }    
    }
  }
}
