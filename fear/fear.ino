//for fear conditioning, use with matlab: posTracker1.9
//modified 7/27/2020 by YP

//chanels 
#define Tone_PIN 5
#define Shock_PIN 8
#define Laser_PIN 13

//default parameters
int tonePars[2]={10000,10000};    //parameters for tone: channel,Hz,duration(ms)
int shockPars[2]={8000,2000};    //parameters for tone: delay-after-tone,duration(ms)
int laserPattern[3] = {20,10,120}; //Hz/duration/totalTime(sec)
//define variables
int state = 0;
int numBuffer[8];
int numPos = 0;
int numType = 0;    //1=duration, 2=Hz, 0=none
int numVal = 0;
//timers
unsigned long timLaser=0;    //timer for laser
unsigned long timTone=0;    //timer for tone
unsigned long timShock=0;    //timer for shock

void setup() {
  // put your setup code here, to run once:
  pinMode(Laser_PIN,OUTPUT);
  pinMode(Shock_PIN,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0)
  {
    state = Serial.read();
    //deal with numbers
    if (state>='0' && state<='9' && numType>0)
    {
      numBuffer[numPos] = state-'0';
      numPos++;
    }
    switch (state)
    {
      case 't':       //numbers as tone duration
        numType=1;
        numVal=0;
        numPos=0;
        break;
      case 's':
        numType=2;    //numbers as shock duration
        numVal=0;
        numPos=0;
        break;  
      case '/':       //end of numbers
        //calculate the actual value
        for (int i=0;i<numPos;i++)
        {
          numVal=numVal+numBuffer[i]*pow(10,numPos-i-1);
        }
        //update tonePars
        if (numType==1)
        {
          tonePars[1]=numVal*1000;
          timTone = 0;
          shockPars[0]=tonePars[1]-shockPars[1];
        }
        else if (numType==2)
        {
          shockPars[1]=numVal*1000;
          shockPars[0]=tonePars[1]-shockPars[1];
        }
        numType=0;
        numPos=0;
        numVal=0;
        break;
      case '0':
        if (numType==0)
        {
        digitalWrite(Laser_PIN,LOW);
        timLaser = 0;
        }
        break;
      case 'X':
        timLaser=millis();
        break;   
      case 'T': //'T'=tone on
        tone(Tone_PIN,tonePars[0]);  
        timTone=millis();
        break;
      case 'S':  //S=Shock
        timShock=millis();
        break;  
      case 'C': //cancel tone and shock 
        noTone(Tone_PIN);
        timTone=0;
        digitalWrite(Shock_PIN,LOW);
        timShock=0;
        break;
    }
  }
  unsigned long now=millis();
  if (timLaser>0)
  {
    startFlash(Laser_PIN,laserPattern,now-timLaser); 
  }
  if (now-timTone>tonePars[1] && timTone>0)
  {
    noTone(Tone_PIN);
    timTone=0;
  }  
  if (timShock>0)
  {
    if (now-timShock>=shockPars[0] && now-timShock<(shockPars[0]+shockPars[1]))
    {
      digitalWrite(Shock_PIN,HIGH);
    }
    else if (now-timShock>=(shockPars[0]+shockPars[1]))
    {
      digitalWrite(Shock_PIN,LOW);
      timShock=0;
    }
    else
    {
      digitalWrite(Shock_PIN,LOW);
    }
  }
}


//for laser stimulation
void startFlash(int ch, int *pattern,unsigned long delTime)
{
  
  if(delTime<1000*pattern[2]/pattern[0])
  {
    if(delTime%(1000/pattern[0])<pattern[1])
      digitalWrite(ch,HIGH);
    else
      digitalWrite(ch,LOW);
  }
  else
    digitalWrite(ch,LOW);
    
}
