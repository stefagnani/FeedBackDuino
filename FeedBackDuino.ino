//------------------------------------------------------------------------
//
// FeedBackDuino - RocRail Feedback bus with Arduino
//
// Copyright (c) 2013 Stefano Fagnani
//
// This source file is subject of the GNU general public license 3,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      FeedBackDuino.ino
// version:   1.0
// author:    Stefano Fagnani
// webpage:   http://ilplasticomoduare.blogspot.it/
// history:   2013-06-15 Initial Version 
//
// todo: 
//------------------------------------------------------------------------
//
// purpose:   Act as feedback bus with PC Serial Interface to RocRail
//  		 
//------------------------------------------------------------------------


#define XPNET_ADDRESS 29   // Indirizzo XpressNet



char val = 0;
int rx_buf[20];
int rx_position = 0;
int header = 0;
String readString ;
String compString ;
byte InByte;
unsigned long time;
unsigned long lasttx;
unsigned long TX_FEEDBACK_INTERVAL = 2000;
int sensors;
byte lastPINBstate;

int XPNET_PC_REQ_IFVERSION[] = {2,0xF0,0xF0};                  // Informazioni sulla versione dell'interfaccia
int XPNET_PC_REQ_STATIONVERSION[] = {3,0x21,0x21,0x00};        //Informazioni sulla versione dell'interfaccia
int XPNET_PC_REQ_IFADDRESS[] = {4,0xF2,0x01,0x55,0xA6};        // indrizzo interfaccia
int XPNET_PC_REQ_PWON[] = {3,0x21,0x81,0xA0};                  // indrizzo interfaccia
int XPNET_PC_REQ_PWOFF[] = {3,0x21,0x80,0xA1};                 // indrizzo interfaccia

int CalcolaXOR(byte var1,byte var2,byte var3) {
  int XOR = 0;
  
  XOR = var1^var2^var3;
  return XOR;
  
}


//------------------------------------------------------------------------
//
// purpose:  Crea una stringa partendo da un Array di caratteri
//			 
//------------------------------------------------------------------------

String PacchettoXpress(int packet[]){
    
    
    String strPacchetto = "";
    
    for (int i = 1;i <= packet[0]; i++) 
    {
      
      strPacchetto += char(packet[i]);
    }
      
    return strPacchetto;
}

//------------------------------------------------------------------------
//
// purpose:  
//Crea un comando di feedback partendo da un byte di ingresso.
//ogni byte rappresenta un ingresso partendo dal MSB.
//Typenibble indica se si tratta del nibble alto o basso come da spec XpressNet
//			 
//------------------------------------------------------------------------


int CreatecmdFeedback(byte data,boolean TypeNibble){
  
  int cmd = 0;
  
  if (TypeNibble == true) {              // Se nibble alto
    cmd = 0x40;
    bitWrite(cmd,0,!bitRead(data,0));
    bitWrite(cmd,1,!bitRead(data,1));
    bitWrite(cmd,2,!bitRead(data,2));
    bitWrite(cmd,3,!bitRead(data,3));
    
  }else{                                // Se nibble basso
    cmd = 0x50;
    bitWrite(cmd,0,!bitRead(data,4));
    bitWrite(cmd,1,!bitRead(data,5));
    bitWrite(cmd,2,!bitRead(data,6));
    bitWrite(cmd,3,!bitRead(data,7));
  }
 
  
  return cmd;
  
}

//------------------------------------------------------------------------
//
// purpose:  
//Crea il pacchetto di feedback 

//			 
//------------------------------------------------------------------------
  
void FeedbackPacket(){
    
    
    String strFeedback = "";
    int XorByte ;
    
    int cmd[10];
    
    cmd[0] = 0x44;
    cmd[1] = 0x41;
    cmd[2] = CreatecmdFeedback(PINB,true);
    cmd[3] = 0x41;
    cmd[4] = CreatecmdFeedback(PINB,false);
    
       
      XorByte  = cmd[0]^cmd[1]^cmd[2]^cmd[3]^cmd[4];   // Xor sui pacchetti 
    
    
    for (int i = 0;i < 5; i++)                          // Invia alla seriale
    {
      Serial.write(cmd[i]);
    }
    Serial.write(XorByte);
}

//------------------------------------------------------------------------
//
// purpose:  
//Routine di inizializzazione
// Viene seguita una sola volta lla'avvio di Arduino
//			 
//------------------------------------------------------------------------


void setup() {
  
  // initialize serial:
  Serial.begin(9600);
  
  // Inizializza gli ingressi 
  
  pinMode(8,INPUT);
  pinMode(9,INPUT);
  pinMode(10,INPUT);
  pinMode(11,INPUT);
  pinMode(12,INPUT);
  pinMode(13,OUTPUT);
  
  // Imposta la resistenza di pullup
  digitalWrite(8,HIGH);
  digitalWrite(9,HIGH);
  digitalWrite(10,HIGH);
  digitalWrite(11,HIGH);
  digitalWrite(12,HIGH);
  
  
  
 
}


//------------------------------------------------------------------------
//
// purpose:  Main Loop
//
//			 
//------------------------------------------------------------------------

void loop() {
  // Legge il dato i seriale:
 

      while (Serial.available()) {
        delay(10);  
          if (Serial.available() >0) {
        InByte = Serial.read();
        readString += char(InByte);}
        }
        
     
     
     // Effettua il parsing dei pacchetti ricevuti dalla serial
     
       if (readString == PacchettoXpress(XPNET_PC_REQ_IFVERSION)) {         // Richiesta versione interfccia
            Serial.write(0x02);
            Serial.write(0x30);
            Serial.write(0x01);
            Serial.write(0x33);
          }
       
    
       if (readString == PacchettoXpress(XPNET_PC_REQ_STATIONVERSION)) {  // Richiesta versione CommandStation
            Serial.write(0x63);
            Serial.write(0x21);
            Serial.write(0x30);
            Serial.write(0x01);
            Serial.write(0x63^0x21^0x30^0x01);
          }
       
        if (readString == PacchettoXpress(XPNET_PC_REQ_IFADDRESS)) {     // Richiesta Indirizzo interfaccia
            Serial.write(0xF2);
            Serial.write(0x01);
            Serial.write(XPNET_ADDRESS);
            Serial.write(0xF2^0x01^XPNET_ADDRESS);
            
          }
      
        if (readString == PacchettoXpress(XPNET_PC_REQ_PWOFF)) {          // Richiesta Poweroff
          //Serial.write(0x60);
            Serial.write(0x61);
            Serial.write((byte)0x0);                                      // da scrivere 0x00, questo formato va riportato così per problemi con compiler.
            Serial.write(0x61);
            digitalWrite(13,LOW);
            
          }
      
      
        if (readString == PacchettoXpress(XPNET_PC_REQ_PWON)) {           // Richiesta PowerON
          //Serial.write(0x60);
            Serial.write(0x61);
            Serial.write(0x01);
            Serial.write(0x60);
            digitalWrite(13,HIGH);
            
          }
    
    
      
      readString="";
     
     
      //Trasmette il pacchetto di feedback su cambio di stato ingresso
      
      if (lastPINBstate != PINB){
        
      FeedbackPacket();
      
      }
         
         lastPINBstate = PINB; 
} 
