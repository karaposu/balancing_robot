

void pid_control()
{


//-----------------------------------------------------------------------------------REMOVE DEADBAND OF PWM
if (raw_pid < deadband_pwm_p) { if (0<raw_pid ) { raw_pid=deadband_pwm_p;} }
if (raw_pid > deadband_pwm_n) {if (0>raw_pid ) { raw_pid=deadband_pwm_n; } }
pwm_cikis=raw_pid;
if (pwm_cikis>255){ pwm_cikis=255;}
if (pwm_cikis<-255){pwm_cikis=-255;}

      
      if( ( (aci_eksi_sinir>gx ) && (gx>aci_max_eksi_sinir) ) || ( (aci_arti_sinir<gx ) && (gx<aci_max_arti_sinir ) )  )  {   //-5 ile +5 arasında ise   veya  -35 ten küçük ise veya +35ten büyük ise
         
                         if(raw_pid>=0  ){
                      digitalWrite(STBY, HIGH); digitalWrite(AIN1, LOW);  digitalWrite(AIN2,HIGH );  analogWrite(PWMA, pwm_cikis);
                       digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, pwm_cikis); }
                         else {tasima=-1*pwm_cikis; pwm_cikis=tasima;
                      digitalWrite(STBY, HIGH); digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, pwm_cikis);
                          digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, pwm_cikis); }
        
      }
       else{
//          pwm_cikis=1;
//            digitalWrite(STBY, HIGH); //disable standby
//                                digitalWrite(AIN1, LOW);  digitalWrite(AIN2,LOW );  analogWrite(PWMA, pwm_cikis);
//                                  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, pwm_cikis);


                                   pwm_cikis=1;
                                   digitalWrite(STBY, LOW);
//                                  digitalWrite(STBY, HIGH); //disable standby
//                                digitalWrite(AIN1, HIGH);  digitalWrite(AIN2,HIGH );  analogWrite(PWMA, pwm_cikis);
//                                  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, pwm_cikis);
}




      
      
      
                              #ifdef ROBOT    
                        
                               Serial.print(" gx = ");          Serial.print(gx);              Serial.print("\t");
                               //    Serial.print(" kp = ");          Serial.print(kp);              Serial.print("\t");
                                //     Serial.print(" ki = ");          Serial.print(ki);              Serial.print("\t");
                               Serial.print(" raw_pid = ");         Serial.print(raw_pid);             Serial.print("\t");  
                               Serial.print(" pwm_cikis = ");     Serial.print(pwm_cikis);     Serial.print("\t");    Serial.print("\t");
                               Serial.println("\t");
                                #endif
                        
                        
                                        
                    
                    
             
              
                      delay(1);
              
                 
              
              }




void pid_update()
{

    double angle_error=0-gx;
//-----------------------------------------------------------------------------------CALCULATE RAW_PID
 double pTerm = kp * angle_error;
 iTerm += ki * angle_error * dt;   iTerm = constrain(iTerm, -10.0f, 10.0f); 
 double dTerm = kd  * (angle_error - angle_error_old)/dt;
 angle_error_old=angle_error;
 raw_pid = pTerm; //+ iTerm + dTerm;
  
}






              
