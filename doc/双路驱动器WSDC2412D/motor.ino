未测试，arduino实现代码。
#define PWM  10           
#define INA  11           
#define INB  12           
int i = 0;                
void setup(){
  pinMode(PWM,OUTPUT); 
  pinMode(INA,OUTPUT); 
  pinMode(INB,OUTPUT); 	
}
void loop()
{
	analogWrite(PWM, 0);  
	digitalWrite(INA,HIGH);
	digitalWrite(INB,LOW);
  for(i = 0;i < 255;i++){ 
    analogWrite(PWM, i);  
    delay(20);           
  }
  for(i =255;i > 0; i--){ 
     analogWrite(PWM, i); 
     delay(20);           
  }
	
	digitalWrite(INA,LOW);
	digitalWrite(INB,HIGH);
  for(i = 0;i < 255;i++){ 
    analogWrite(PWM, i);  
    delay(20);           
  }
  for(i =255;i > 0; i--){ 
     analogWrite(PWM, i); 
     delay(20);           
  }
}

  