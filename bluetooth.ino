




void bluetooth(int x){  //Serial işlemler içerdiginden bu kodu Kullanılırken ana koddaki #define robot kısmı kapatılmalıdır

 sendvalue(x) ;  //bu fonksyon ile istenilen degişken degeri rakamlara ayrılır ve usart üzerinden gönderilir. 
//Serial.println( x);
 // getvalue();  //bu fonksyon deneme aşamasındadır.  p i d degerlerini bluetoot üzerinden arduinoya yüklemeye yarar.
   
}

void bluetooth(int x,int y){  //Serial işlemler içerdiginden bu kodu Kullanılırken ana koddaki #define robot kısmı kapatılmalıdır

 sendvalue(x,y) ;  //bu fonksyon ile istenilen degişken degeri rakamlara ayrılır ve usart üzerinden gönderilir. 
//Serial.println( x);
 // getvalue();  //bu fonksyon deneme aşamasındadır.  p i d degerlerini bluetoot üzerinden arduinoya yüklemeye yarar.
 
}


void bluetooth(int x,int y,int z){  //Serial işlemler içerdiginden bu kodu Kullanılırken ana koddaki #define robot kısmı kapatılmalıdır

 sendvalue(x,y,z) ;  //bu fonksyon ile istenilen degişken degeri rakamlara ayrılır ve usart üzerinden gönderilir. 
 
}

void parse_to_digits(int number)
{

  

  /********
   * 
   * int digit_1000;
   int digit_100;
   int digit_10;
   int digit_1;
   sign_of_number;
   these values must been declared as global variables. 
   * 
   * 
   * 
   */

    int number_3digits;
    int number_2digits;
    int number_1digits;
    int mod2;
    int mod1;
    int mod3;
   // int sign_of_number;
    
    if(number<0){
      number=-1*number;   //take absolute of it  , not neccesary for adc values 
      sign_of_number=1;}
    else{
      sign_of_number=2;
    }

      
      //here is the math calculations . actually  each of the block'S third line are not neccesaryy but doesnt hurt no body. 
      
      /*************************************calculate 1000  digit ************************************/
        mod1=number%1000; //564
        digit_1000=(number-mod1)/1000 ;  //4. digit   1
        number_3digits =(number-(1000*digit_1000))  ;    // 564 
      
       /*************************************calculate 100  digit ************************************/
        mod2=number_3digits % 100;  //  64
        digit_100=(  number_3digits-mod2   )/100;   // 3. digit   5
       number_2digits =(number_3digits-(100*digit_100)) ;    //2. hane digit_10iz digit_10ayı
      
      
       /*************************************calculate 10 digit ************************************/
      
      mod3=number_2digits % 10;  //63
      digit_10=(  number_2digits-mod3   )/10 ;   //2. hane
      number_1digits= (number_2digits-(10*digit_10))  ;  //ı  
      
       /*************************************calculate 1  digit ************************************/
      digit_1=number_1digits;
      

  
}


uint8_t digit_dec2ascii(uint8_t decimal_digit){

uint8_t ascii_digit=decimal_digit+48;
return ascii_digit;
  

}

void sendvalue(int value)
{

  //Serial.println( value);

  //  packet demo  :  35 0 52 51 50   which is equal to #+432
 
    int asci_digit100= 0; 
     int asci_digit10=  0;
      int asci_digit1=  0;
   
      
    parse_to_digits(value);   

    Serial.write(35);   // #
  

   
   //  asci_digit100=  digit_100+48;  //rakamlar ascii ye çevrildi    
   //  asci_digit10=  digit_10+48;
   //  asci_digit1=  digit_1+48;

asci_digit100=    digit_dec2ascii(digit_100);
  asci_digit10=    digit_dec2ascii(digit_10);
   asci_digit1=    digit_dec2ascii(digit_1);
  


  Serial.write (sign_of_number);     
  Serial.write (asci_digit100);     
    Serial.write (asci_digit10);
    Serial.write (asci_digit1); 

   // Serial.println();
}

void sendvalue(int value,int value2)
{

 
  //  packet demo 2 :  35 0 52 51 20 47 0 52 51 50  which is equal to #+432/+432
    int asci_digit100= 0; 
     int asci_digit10=  0;
      int asci_digit1=  0;
   
      
    parse_to_digits(value);   

   
//     asci_digit100=  digit_100+48;  //rakamlar ascii ye çevrildi
//     asci_digit10=  digit_10+48;
//     asci_digit1=  digit_1+48;

    asci_digit100=    digit_dec2ascii(digit_100);
  asci_digit10=    digit_dec2ascii(digit_10);
   asci_digit1=    digit_dec2ascii(digit_1);


     

    Serial.write(35);   // #
  Serial.write (sign_of_number);     
  Serial.write (asci_digit100);     
    Serial.write (asci_digit10);
    Serial.write (asci_digit1); 


      asci_digit100= 0; 
      asci_digit10=  0;
       asci_digit1=  0;
   
      
    parse_to_digits(value2);   

   
//     asci_digit100=  digit_100+48;  //rakamlar ascii ye çevrildi
//     asci_digit10=  digit_10+48;
//     asci_digit1=  digit_1+48;

      asci_digit100=    digit_dec2ascii(digit_100);
  asci_digit10=    digit_dec2ascii(digit_10);
   asci_digit1=    digit_dec2ascii(digit_1);

    Serial.write(47);   //   /
  Serial.write (sign_of_number);     
  Serial.write (asci_digit100);     
    Serial.write (asci_digit10);
    Serial.write (asci_digit1); 
    

   // Serial.println();
}



void sendvalue(int value,int value2,int value3)
{

 
  //  packet demo 3 :  35 0 52 51 20  0 52 51 50 0 52 51 50  which is equal to #+432+432+432
int asci_digit100= 0;  int asci_digit10=  0; int asci_digit1=  0;
parse_to_digits(value);   
asci_digit100=    digit_dec2ascii(digit_100);asci_digit10=    digit_dec2ascii(digit_10); asci_digit1=    digit_dec2ascii(digit_1);

 Serial.write(35);   // #
 Serial.write (sign_of_number);     
 Serial.write (asci_digit100);     
 Serial.write (asci_digit10);
 Serial.write (asci_digit1); 


asci_digit100= 0;   asci_digit10=  0;  asci_digit1=  0;
parse_to_digits(value2);   
asci_digit100= digit_dec2ascii(digit_100);  asci_digit10=  digit_dec2ascii(digit_10);  asci_digit1= digit_dec2ascii(digit_1);

 
  Serial.write (sign_of_number);     
  Serial.write (asci_digit100);     
  Serial.write (asci_digit10);
  Serial.write (asci_digit1); 


    
asci_digit100= 0;   asci_digit10=  0;  asci_digit1=  0;
parse_to_digits(value3);   
asci_digit100= digit_dec2ascii(digit_100);  asci_digit10=  digit_dec2ascii(digit_10);  asci_digit1= digit_dec2ascii(digit_1);

 
  Serial.write (sign_of_number);     
  Serial.write (asci_digit100);     
  Serial.write (asci_digit10);
  Serial.write (asci_digit1); 

 
}



void getvalue()
{
   
   if(Serial.available()>0)  
   {

    while(Serial.read()!=200){  //ilk deger 200 olucak her zaman 
      int deger0=Serial.read();   
      Serial.write(deger0);
  //    p= deger0;
      
    }
   }

  

 //Serial.write(gx);
}



void protocol_253(int number){
  if(number==253){
    number=254;
  }
    Serial.write(253);
    Serial.write(number);
}



void protocol_253(float pack_raw[]){
/*
  paket prototipleri
 
  { '253' , signbits  ,  datapacket[0], datapacket[0]_fraction , datapacket[1] ,datapacket[1]_fraction , datapacket[2]  }

  -192.52  250.45  255 geldi
  192.52 pozitif oldu
  192.52>192 oldugundan fractionu 0.52 bulundu
  2 basamaklı fractiona izin veriliyor 0.01 ile 0.99 arasında
  0.52  *100 = 52 çıktı
  0.1 olsaydı 10 çıkacaktı
  0.01 olsaydı 1 çıkacaktı
  253 1 192 52  250 45  255  şeklinde gönderildi

*/
float pack[3];
for (int i = 0; i < 3; i++) {
   pack[i]=pack_raw[i];
  }



//------------------------------------------------------take_Absolute and denote which ones negative
  for (int i = 0; i < 3; i++) {
    if(pack[i]<0){ 
      pack[i]=-1*pack[i]; sign_packet[i]=1;
      }
    else{ sign_packet[i]=0; }
  }

 sign_byte=sign_packet[0]*1+sign_packet[1]*2+sign_packet[2]*4;

 

//------------------------------------------------------extract fractions and put them in array and load nonfraction parts to an array
   for (int i = 0; i < 3; i++) {
     if(  floor(pack[i])<pack[i]){ 
             fraction_packet[i]= pack[i]-((uint8_t)(floor(pack[i])));   //extract  fraction part   192.52-192 =0.52
              fraction_packet[i]=  fraction_packet[i]*100;   //make fraction integer by multiplying by 100 
     }
  }

for (int i = 0; i < 3; i++) {
      pack[i]= floor(pack[i]);
    if(pack[i]==253){ pack[i]=254;}
 
  }

//------------------------------------------------------print packet over usart



   
 Serial.write(253);  //delimeter byte
 Serial.write(sign_byte);    //sign byte
       Serial.write((uint8_t)(pack[0]));
       Serial.write((uint8_t)(fraction_packet[0]));
       Serial.write((uint8_t)(pack[1]));
       Serial.write((uint8_t)(fraction_packet[1]));
       Serial.write((uint8_t)(pack[2]));
   
//integer_part = floor(sign_packet[0]); 
//decimal_part = fmod(num,1)*10^whatever;
    


}






//double x; 
//double y; 
//double z; 
//
//void printDouble( double val, unsigned int precision){
//// prints val with number of decimal places determine by precision
//// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
//// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)
//
//   Serial.print (int(val));  //prints the int part
//   Serial.print("."); // print the decimal point
//   unsigned int frac;
//   if(val >= 0)
//       frac = (val - int(val)) * precision;
//   else
//       frac = (int(val)- val ) * precision;
//   Serial.println(frac,DEC) ;
//} 
//
//void  setup(){
// Serial.begin(9600);
// Serial.println("Print floating point example");   
// printDouble( 3.1415, 100);  // example of call to printDouble to print pi to two decimal places
// x = 10; 
// y = 3.1; 
// z = x / y;   
//}
//
//
//void loop(){
//  printDouble(z,10);   // one decimal place
//  printDouble(z,100);  // two decimal places
//  printDouble(z,1000); // three decimal places
//  z = z + .1;
//  delay(100);
//}
//
//




