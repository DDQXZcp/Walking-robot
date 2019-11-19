#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  119 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  445 // this is the 'maximum' pulse length count (out of 4096)
#define left left
#define right right

int offset[16];
int lastpos[16];
int moveServo(int servonum, int pos)
{
    //offset[servonum]
    int pulselength = map(pos+offset[servonum], 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servonum, 0, pulselength);
    lastpos[servonum] = pos;
    return pulselength;
}

uint8_t servonum = 0;

int light=8;

char Received='o';
int state ; //0 for normal, 1 for forward, 2 for backward
int tilt ;//0 for normal, 1 for left, 2 for right
int hstate ;//0 for normal, 1 for arm raised once, 2 for arm raise twice
int kstate ;//0 for normal, 1 for kick
int hold ;//0 for normal, 1 for ready to kick

void setup(){
   ///////define offsets of individual servos///////
   offset[0]  = 0; 
   offset[1]  = 0;
   offset[2]  = 0;
   offset[3]  = 0;
   offset[4]  = 0;
   offset[5]  = 0;
   offset[6]  = 0;
   offset[7]  = 0;
   offset[8]  = 0;
   offset[9]  = 0;
   offset[10] = 0;
   offset[11] = 0;   
     
  Serial.begin(9600);
  Serial.println("Product from PolyU");
  pinMode(light,OUTPUT);

  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
  for (int i=0;i<16;i++)
  { 
    lastpos[i]=0;
  }
  char data[5];
  int numero;
  int servono;
   
    data[numero] = 0;

    int angulo = atoi(data);
    
    lastpos[4]=60;
    lastpos[9]=105;
    delay(500); 
    Serial.println("Setup Completed");
    reset();
}

void loop() { 
 if(Serial.available()>0)
 { 
    Received = Serial.read();
 }
 //////////reply to the bluetooth module//////////
  echo();  
 //////////Action Decision////////// 
     if (Received=='f') {
        moveForward();
        }
        
     if (Received=='b') {
        moveBackward(); 
        }
                
     if (Received=='A') {
      if(state==1){
        forwardRestore();
        holdKick();
      }else if(state==2){
        backwardRestore();
        holdKick();
      }else{
         holdKick();
      }       
        }
        
     if (Received=='B') {
          raiseArm();
        }
        
     if (Received=='l') {
        if(state==1){
        forwardRestore();
         turnLeft();
      }else if(state==2){
        backwardRestore();
         turnLeft();
      }else{
          turnLeft();
      }
        }
        
     if (Received=='r') {
        if(state==1){
        forwardRestore();
         turnRight();
      }else if(state==2){
        backwardRestore();
         turnRight();
      }else{
          turnRight();
      }
        }

     if (Received=='X') {
        walkKick();
        }

    if (Received=='Y') {
         restoreArm();
        }
        
    if (Received=='0') {
        reset();
        }
                
//////////respond to the bluetooth module//////////
      Received='o';
      echo();
}

      void echo(){
      //Serial.print(Received);
      }
      
      void reset(){
      for (int i =0 ; i<16 ; i++)
      {
          int pulselength = moveServo(i, 90);
          delay(300);
      }
          moveServo(12,10);//put down left hand
          moveServo(14,10);//put down right hand
          state = 0;//set state to default
          tilt = 0;//set tilt to default
          hstate = 0;//set hstate to default
          kstate = 0;//set kstate to default
          hold = 0;//set hold to default
      }

      void tiltLegL1(){
        if(kstate==1){
          for(int i=0;i<10;i++)
         {
          moveServo(5,90+i);//turn servo 5 from 90 to 100
          moveServo(9,90-2*i);//turn servo 9 from 90 to 70
          moveServo(4,90-2*i);//turn servo 4 from 90 to 70
          delay(30);        
        }
        }else{
          for(int i=0;i<10;i++)
         {
          moveServo(9,90-2*i);//turn servo 9 from 90 to 70
          moveServo(4,90-2*i);//turn servo 4 from 90 to 70
          delay(30);        
        }
        }       
       }

      void tiltLegL2(){
        if(kstate==1){
          for(int i=0;i<10;i++)
        {
          moveServo(5,100-i);//turn servo 5 from 100 to 90
          moveServo(9,70+2*i);//turn servo 9 from 70 to 90
          moveServo(4,70+2*i);//turn servo 4 from 70 to 90
          delay(30);
        }
        }else{
           for(int i=0;i<10;i++)
        {
          moveServo(9,70+2*i);//turn servo 9 from 70 to 90
          moveServo(4,70+2*i);//turn servo 4 from 70 to 90
          delay(30);
        }
        }
       }
      
      void tiltLegR1(){
        for(int i=0;i<10;i++)
        {
          moveServo(4,90+2*i);//turn servo 4 from 90 to 110
          moveServo(9,90+2*i);//turn servo 9 from 90 to 110
          delay(30);
        }
       }

      void tiltLegR2(){
        for(int i=0;i<10;i++)
       { 
        moveServo(4,110-2*i);//turn servo 4 from 110 to 90
        moveServo(9,110-2*i);//turn servo 9 from 110 to 90
        delay(30);
       }
      }

      void raiseLeg1(){
        for(int i=0;i<10;i++)
       {
        moveServo(1,87+2.5*i);//turn servo 1 from 90 to 115, 1 modify 87 to 112
        moveServo(2,90-2.5*i);//turn servo 2 from 90 to 65
        moveServo(7,90+2.5*i);//turn servo 7 from 90 to 115
        moveServo(8,90+2.5*i);//turn servo 8 from 90 to 115
        delay(30);
       }
       tilt = 1;
      }

     void raiseLeg2(){
        for(int i=0;i<10;i++)
       {   
        moveServo(1,87+2.5*i);//turn servo 1 from 90 to 115, 1 modify
        moveServo(3,65+2.5*i);//turn servo 3 from 65 to 90
        moveServo(6,68+2.5*i);//turn servo 6 from 65 to 90, 6 modify 68 to 93
        moveServo(8,90+2.5*i);//turn servo 8 from 90 to 115
        delay(30);
       }
       tilt = 1;
      }

     void raiseLeg3(){
        for(int i=0;i<10;i++)
       {  
        moveServo(1,112-2.5*i);//turn servo 1 from 115 to 90, 1 modify 112 to 87
        moveServo(3,90-2.5*i);//turn servo 3 from 90 to 65
        moveServo(6,93-2.5*i);//turn servo 6 from 90 to 65, 6 modify 93 to 68
        moveServo(8,115-2.5*i);//turn servo 8 from 115 to 90
        delay(30);
       }
       tilt = 1;
      }

      void moveForward(){        
         tiltLegL1();//tilt left leg
        //raise leg
        if (state==0){
          raiseLeg1();//move right leg forward
        }else{      
          raiseLeg2();//move left leg forward
       }      
         tiltLegL2();//restore tilt left leg     
    //////////////////second step////////////////////    
        tiltLegR1();//tilt right leg                
        raiseLeg3();//move left leg forward       
        tiltLegR2();//restore tile right leg
        state=1;//change state for next action
      }
      
      void forwardRestore(){
        for(int i=0;i<10;i++)
     {  moveServo(4,90-2*i);//turn servo 4 from 90 to 70
        moveServo(9,90-2*i);//turn servo 9 from 90 to 70
        moveServo(6,68+2.5*i);//turn servo 6 from 65 to 90, 6 modify
        moveServo(7,115-2.5*i);//turn servo 7 from 115 to 90
        delay(30);
        }
        for(int i=0;i<10;i++)
    {   moveServo(4,70+2*i);//turn servo 4 from 70 to 90
        moveServo(9,70+2*i);//turn servo 9 from 70 to 90
        moveServo(2,65+2.5*i);//turn servo 2 from 65 to 90
        moveServo(3,65+2.5*i);//turn servo 3 from 65 to 90
        delay(30);
    }
        state=0;//change state for next action
      }
      
      void moveBackward(){  
      //////////////////////first step////////////////////////
        tiltLegR1();//tilt right leg        
        //raise leg
        if(state==0){
        raiseLeg1();//move left leg backward
      }else{
        raiseLeg2();//move right leg backward   
        }       
        tiltLegR2();//restore tilt right leg
     //////////////////second step///////////////////     
        tiltLegL1();//tilt left leg
        raiseLeg3();//move right leg backward     
        tiltLegL2();//restore tilt left leg
        state=2;//change state for next action  
  }

  void backwardRestore(){
    for(int i=0;i<10;i++)
     {  moveServo(4,90+2*i);//turn servo 4 from 90 to 110
        moveServo(9,90+2*i);//turn servo 9 from 90 to 110
        moveServo(2,65+2.5*i);//turn servo 2 from 65 to 90
        moveServo(3,65+2.5*i);//turn servo 3 from 65 to 90
        delay(30);
        }
        for(int i=0;i<10;i++)
    {   moveServo(4,110-2*i);//turn servo 4 from 110 to 90
        moveServo(9,110-2*i);//turn servo 9 from 110 to 90
        moveServo(6,68+2.5*i);//turn servo 6 from 65 to 90, 6 modify
        moveServo(7,115-2.5*i);//turn servo 7 from 65 to 90
        delay(30);
    }
    state=0;//change state for next action
  }
 
 void turnRight(){    
        tiltLegL1();//tilt left leg       
      //turn right waist      
        for(int i=0;i<10;i++)
       {  
        moveServo(10,90-3*i);//turn servo 10 from 90 to 60       
        delay(40);
       }
        tiltLegL2();//restore tilt left leg
        delay(200);     
    //////////////////second step////////////////////    
       tiltLegR1();//tilt right leg      
      //turn left waist
       for(int i=0;i<10;i++)
      {  
       moveServo(10,60+3*i);//turn servo 10 from 60 to 90
       delay(40);
      }
      tiltLegR2();//restore tilt right leg
     }

    void turnLeft(){
      tiltLegR1();//tilt right leg
      //turn left waist      
        for(int i=0;i<10;i++)
       {  
        moveServo(11,90+3*i);//turn servo 11 from 90 to 120 
        delay(40);
       }            
      tiltLegR2();//restore tilt right leg
      delay(200);
     //////////////////second step///////////////////     
      tiltLegL1();//tilt left leg
       //turn left waist
       for(int i=0;i<10;i++)
      { 
       moveServo(11,120-3*i);//turn servo 11 from 120 to 90 
       delay(40);
      }  
     tiltLegL2();//restore tilt left leg
     state=0;//change state for next action      
      }
void leanLeft(){
  for(int i=0;i<10;i++){
    moveServo(10,90+3*i);
    moveServo(11,90+3*i);
    delay(40);
  }
}

 void walkKick(){
      moveForward();
      kstate = 1;
      leanLeft();
      delay(200);        
      if(kstate==1){
          for(int i=0;i<10;i++)
         {
          moveServo(5,90+i);//turn servo 5 from 90 to 100
          moveServo(9,90-2.5*i);//turn servo 9 from 90 to 70
          moveServo(4,90-2*i);//turn servo 4 from 90 to 70
          delay(30);        
        }
        }else{
          for(int i=0;i<10;i++)
         {
          moveServo(9,90-2.5*i);//turn servo 9 from 90 to 70
          moveServo(4,90-2*i);//turn servo 4 from 90 to 70
          delay(50);        
        }
        }       
      //tilt left leg
      for(int i=0;i<10;i++)
     {  moveServo(6,68+2.5*i);//turn servo 6 from 65 to 90, 6 modify
        moveServo(7,115-2.5*i);//turn servo 7 from 115 to 90
        moveServo(2,65-0.5*i);//turn servo2 from 65 to 60
        delay(50);
        }    
    //raise right leg
      for(int i=0;i<15;i++)
     { moveServo(1,87-2*i);//turn servo 1 from 90 to 60, 1 modify 87 to 57
       moveServo(2,60-2*i);//turn servo 2 from 90 to 30 
       delay(30);
     }
   /////////////// KICK ///////////////
     for(int i=0;i<15;i++)
     { 
       moveServo(1,57+6*i);//turn servo 1 from 60 to 150, 1 modify 57 to 147
       moveServo(2,30+4*i);//turn servo 2 from 30 to 90
       delay(30);
     }
       delay(500);
       ///////////restore////////////////
       for(int i=0;i<15;i++)
      { 
        moveServo(2,90-4*i); //turn servo 2 from 90 to 30
        delay(30);
      }
      for(int i=0;i<10;i++)
      {
        moveServo(3,65+2.5*i);//turn servo 3 from 65 to 90  
      }
      for(int i=0;i<15;i++)
     {
       moveServo(1,147-4*i);//turn servo 1 from 150 to 90, 1 modify
       moveServo(2,30+4*i);//turn servo 2 from 30 to 90
       delay(30);
     }
     tiltLegL2();//restore tilt left leg
     kstate = 0;//reset kstate for next action
     state = 0;//teset state for next action
    }

    void holdKick(){
        /////////////// KICK ///////////////
        kstate = 1;//change kstate for kick
     if(hold==0){
      tiltLegL1();//tilt left leg    
    //raise right leg
      for(int i=0;i<15;i++)
     { moveServo(1,87-2*i);//turn servo 1 from 90 to 60, 1 modify
       moveServo(2,90-4*i);//turn servo 2 from 90 to 30 
       delay(30);
     }
     hold = 1;
     }else if(hold==1){
      for(int i=0;i<15;i++)
     { 
       moveServo(1,57+6*i);//turn servo 1 from 60 to 150, 1 modify
       moveServo(2,30+4*i);//turn servo 2 from 30 to 90
       delay(30);
     }
       delay(500);
       ///////////restore////////////////
       for(int i=0;i<15;i++)
      { 
        moveServo(2,90-4*i); //turn servo 2 from 90 to 30
        delay(30);
      }
        
      //put down right leg
      for(int i=0;i<15;i++)
     {
       moveServo(1,147-4*i);//turn servo 1 from 150 to 90, 1 modify
       moveServo(2,30+4*i);//turn servo 2 from 30 to 90
       delay(30); 
     }
     tiltLegL2();//restore tilt left leg
     hold = 0;//reset hold for next action
     kstate = 0;//reset kstate for next action
     }
     }
         
      
    void raiseArm(){
      if(hstate==0){
         for(int i=0;i<15;i++)
        {
          moveServo(12,10+2*i);//turn servo 12 from 40 to 70
          moveServo(14,10+2*i);//turn servo 14 from 10 to 40
          hstate = 1;//change hstate for next action
          delay(30);
        }
      }else if(hstate==1){
         for(int i=0;i<15;i++)
        {
          moveServo(12,40+2*i);//turn servo 12 from 70 to 100
          moveServo(14,40+2*i);//turn servo 14 from 40 to 70
          hstate = 2;//change hstate for next action
          delay(30);
         }
      }else if(hstate==2){
         for(int i=0;i<15;i++)
        {
          moveServo(12,70+2*i);//turn servo 12 from 100 to 130
          moveServo(14,70+2*i);//turn servo 14 from 70 to 100
          hstate=0;//reset hstate for next action
          delay(30);
         }
        }
      }

    void restoreArm(){
        moveServo(12,10);//reset servo 12
        moveServo(14,10);//reset servo 14
        hstate=0;//reset hstate for next action   
       }

  void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 50;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}
