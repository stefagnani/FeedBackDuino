

  
/*
Basato sullo sketch pubblicato da "antogar" a
http://www.rmweb.co.uk/community/index.php?/topic/92932-arduino-loconet-occupancy-detector/

Le librerie sono parte del progetto MRRWA (LOCONET) a 
https://github.com/mrrwa

Modificato da Colin F. Jones, 13/6/2015
STATUS = Practical Completion

Modificato tradotto testato ed adattato da Stefano Fagnani
STATO = In verifica

18/06/2023 V02 - Monitor Seriale implementa indicazione dello stato 
08/06/2016 V01 - Modificata trasmissione stato sensore inversione stato. Trasmette alto quando il sensore è occupato, basso quando libero.
                In setup() venivano inizializzati correttamenti solo 14 ingressi qualunque valore si assegnasse alla variabile numSensors              
12/10/2016      Adattamento codice per Arduino NANO da intallare su cheda ARLN_FD8_NANO
20/11/2017      Disabilitazione reporting sensor su comando scambio per problemi di trasmissione.
10/09/2018      Eliminazione ritardi di lettura
Comunicazione LOCONET tramite: 
  OPC_INPUT_REP 0xB2 ; General SENSOR Input codes
    Packet format= <0xB2>, <IN1>, <IN2>, <CHK>

<IN1> =<0,A6,A5,A4 - A3,A2,A1,A0> ; sono i 7 bit meno significativi dell'indirizzo
<IN2> =<0,X,I,L - A10,A9,A8,A7> ; Report/status bits e 4 bit più significativi dell'indirizzo.
"X"=1, control bit , 0 è riservato per uso futuro!   Default = 1.
"I"=0 per DS54 "aux" inputs e 1 per "switch" inputs mappati to 4K SENSOR space.  Default = 1.
"L"=0 for input SENSOR 0V (LOW) , 1 for Input sensor >=+5V (HIGH)
"A10,A9,A8,A7" are 4 most significant address bits

L'indirizzo di base è assegnato gli altri indirizzi vengono allocati sequenzialmente
CVs 1 e 9 ( EEPROM 0 and 8) sono usati per memorizzare l'indirizzo di base per i decoder/encoder accessori
Il numero massimo di indirizzi per i sensori e dato dall'indirizzo di base più il numero di sensori (14 in questo caso)

Le CV dalla  33 alla 81 (EEPROM 32 e 80) sono disponibili per memorizzare informazioni utente o del costruttore. 
Le CV partono dalla 33, 2 CV per indirizzo.

Usare il pin 8 per la ricezione (RX pin).  Obbligatorio per questa libreria in quanto utilizza il timer interrupt e non è possibile cambiarlo.
Il Pin 11 è usato per la trasmissione (TX pin) ma qualsiasi altro pin può essere utuilizzato per questo scopo.

ASSEGNARE L'INDIRIZZO DI BASE
Per impostare l'indirizzo di base del decoder premere il pulsante di programmazione per 2 secondi fino a quando si accende il LED
L'unità attende il rilascio del pulsante, a quel punto il LED inzia a lampeggiare.
E' possibile uscire dalla programmazione premendo di nuyovo il pulsante per due secondi
Nello stato di programmazione inviare un comando scambio sul bus LocoNet con indirizzo desiderato.
L'indirizzo viene memorizzato nella EEPROM e viene generato un broadcast dello stato corrente dei sensori.

USO
Inviando un comando scambio all'indirizzo di base genera un segnale di broadcast con lo stato corrente dei sensori.
Inviando un comando scambio all'indirizzo specifico del sensore viene trasmesso lo stato del solo sensore specificato.
la variabile "interval" controlla l'intervallo di scansione dello stato degli ingressi per la trasmissione.
Di base impostato ogni 1,5 secondi

NOTA: commentare tutte le rige Serial.print nella versione di uso finale.

*/
////////////////////////////////////////////////////////////////
// Included files

#include <LocoNet.h>
#include <EEPROM.h>



////////////////////////////////////////////////////////////////////
//  Definitions and structures


#define TX_PIN   11
#define CUR_VER_MAJOR "6."                //Versione corrente, revisoone principale
#define CUR_VER_MINOR "1b2"                //Versione corrente, revisione secondaria
#define BOARD_TYPE  0x01               // Tipo Scheda   TYPE_ARLN_FD  0x01   TYPE_ARLN_FDH 0x02  TYPE_ARLN_SW  0x03 
            

// Commentare per eliminare il debug su monitor seriale

#define DEBUG


int progButton = 12;
int led = 13;

static   LnBuf        LnTxBuffer;
static   lnMsg        *LnPacket;

unsigned int address_received;
unsigned int myAddress;

int LEDdelay = 200;  // set blink rate of LED on pin 13
byte decoderStatus = 0;  // default normal operation, = 1 is change base address
int i;
byte numSensors = 8;  // can change to suit the number of sensors used, optional
int sensorPin[8] = {2,3,4,5,6,7,9,10};
boolean sensorValue[8] = {1,1,1,1,1,1,1,1};  // default high
boolean priorSensorValue[8] = {1,1,1,1,1,1,1,1};  // default high

unsigned long prevmillis;
unsigned long currentmillis;
unsigned long interval = 500; // delay between consecutive input checks, 1.5sec


   
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

void setup()
{
  // Inizializza la funzione dei pin e li inizializza
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // Impostando ad HIGH un ingresso attiva la resistenza di pull-up interna 
  pinMode(progButton, INPUT);
  digitalWrite(progButton, HIGH);

  // Ciclo che setta i PIN indicati nell'array sensorPin[] ad ingresso ed attiva la relativa resistenza di pull-up
  // Il numero di iterazioni è dato dalla variabile numSensors
  
  for(i=0;i<numSensors;i++)
  {
    pinMode(sensorPin[i], INPUT);
    digitalWrite(sensorPin[i], HIGH);
  }
  //***********************************
   
    // Attiva e configura la porta seriale alla velocità di 57600 baud
  Serial.begin(57600);
  
    // Inizializza l'interfaccia LocoNet indicando il Pin di trasmissione, quello di ricezione è fisso e non modificabile
  LocoNet.init(TX_PIN);
  
    // Inizializza il buffer di ricezione diei pacchetti LocoNet 
  initLnBuf(&LnTxBuffer) ;

    // Legge l'indirizzo di base dalla EEPROM
  myAddress = EEPROM.read(0) + 256 * EEPROM.read(8);

 // #ifdef DEBUG  ---->>>6.1.b2
    // Codice didebug, commentare nella versone di uso finale
      Serial.println("ilplasticomodulare.blogspot.it");
      Serial.println("Loconet Feedback Module 8");
      Serial.print("Current Version:");Serial.print(CUR_VER_MAJOR);Serial.println(CUR_VER_MINOR);
      Serial.println("Good Day");
      Serial.print("Decoder Base Address");
      Serial.print("\t");
      Serial.println(myAddress);
 // #endif

  
} // **** Fine funzione Setup() ****

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void loop()
{  
  LnPacket = LocoNet.receive() ;  // Controlla se sono diponibili pacchetti LocoNet ricevuti nel buffer.
   
  if(LnPacket)  // Se il pacchetto è presente...
  {
    LocoNet.processSwitchSensorMessage(LnPacket);  // Processa il pacchetto per ottenere l'indirizzo ricevuto
    #ifdef DEBUG
      printRXpacket (); 
    #endif
    
    // Se il pacchetto ricevuto è un comando di Global Power-ON vengono inviati tutti gli stati correnti dei sensori per allineamento del campo con il software di gestione
    if ( LnPacket -> data[0] == 0x83 ) sendAllsensors (); 

     // Se si tratta di un comando scambi gestisce per l'invio degli stati.
    if ( LnPacket -> data[0] == 0xB0 ) packetHandling();    
  }
 
  if(digitalRead(progButton) == LOW) changeAddress();  // Se il pulsante è premuto avvia la gestione di cambio indirizzo.
    
  currentmillis = millis ();
  if (currentmillis - prevmillis > interval)  // Il tempo impostato nella variabile interval è scaduto, quindi è tempo di inviare lo stato dei sensori.
  {
    readAllSensors ();         // Legge stato fisico degli ingressi
    sendChangedSensors ();     // Invia lo stato degli ingressi che sono cambiati di stato.
  }
 
  if ((decoderStatus != 0))  // Fa lampeggiare il led SE IL DISPOSITIVO SI TROVA IN STATO DI PROGRAMMAZIONE
    {
      digitalWrite(led, HIGH);
      delay (LEDdelay);
      digitalWrite(led, LOW);
      delay (LEDdelay);
     }
   
  
  serialEvent();
  
}   // **** Fine funzione Loop() ****

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

void printRXpacket () // Stampa il pacchetto Loconet, solo per scopi di debug.
{
    uint8_t Length = getLnMsgSize( LnPacket ) ;  
    Serial.print("RX: ");
    for( uint8_t Index = 0; Index < Length; Index++ )
    {
      Serial.print(LnPacket->data[ Index ], DEC);
      Serial.print("  ");
     } 
    Serial.println();
}

//////////////////////////////////////////////////////////////////////////////
void packetHandling()
{
   /*byte LSB (meno significativo) = LnPacket -> data[1];
   byte MSB (più significativo = LnPacket -> data[2];
   MSB = (MSB & B00001111);
   LSB = LSB << 1;
   address_received = MSB * 256;
   address_received = address_received + LSB;
   address_received = address_received >> 1;
   address_received += 1; // change system address to display address
   */

   #ifdef DEBUG
      Serial.print("Address : ");
      Serial.println(address_received);
   #endif
   
   switch (decoderStatus)
    { 
      // Se l'interfaccia si trova in stato di funzionamento normale e l'indirizzo ricevuto è uguale all'indirizzo di base viene inviato un broadcast dello stato dei sensori    
      case 0:     
        /*if (address_received == myAddress)
        {
          Serial.println("Status 0 - broadcast request");
          sendAllsensors ();
        }
        
        // Se l'indirizzo ricevuto è compreso tra l'indirizzo di base e minore del numero di sensori viene inviato lo stato dell'indirzzo fornito
        if ( (address_received > myAddress) && (address_received < myAddress + numSensors) )
        {
          i = address_received - myAddress - 1;  // Ottiene l'indirizzo del sensore da aggiornare
          priorSensorValue[i] = sensorValue[i]; // Memorizza lo stato corrente
          sensorValue[i] = digitalRead(sensorPin[i]);  // Legge lo stato corrente
          LocoNet.reportSensor(address_received, sensorValue[i]); // invia il nuovo stato
          Serial.print("Sent sensor : ");
          Serial.println(i);
          //Serial.println(tempaddr);
        }  ///// Commentato per problemi di trasmissione stati 20/11/2017*/
        break;       

        // Se l'interfaccia si trova in stato di programmazione ottiene l'indirizzo lo assegna come indirizzo di base e lo memorizza nella EEPROM
      case 1:            
        EEPROM.write(0, lowByte(address_received)); // CV1
        EEPROM.write(8, highByte(address_received)); // CV9
        myAddress = EEPROM.read(0) + 256 * EEPROM.read(8);
        
        #ifdef DEBUG 
          Serial.println("Status 1 - memorising the address");
          Serial.print(address_received, HEX);
          Serial.print("\t");
          Serial.print(lowByte(address_received), HEX);
          Serial.print("\t");
          Serial.print(highByte(address_received), HEX);
          Serial.println();
        #endif
        decoderStatus = 0; // ritorna al funzionamento normale
        digitalWrite(led, LOW);// spegne il LED di programmazione
        break;
     } 
      
}

///////////////////////////////////////////////////////////////////////////////
void changeAddress()
{  // Commentare tutte le righe Serial.println nella versione di uso finale

   if (decoderStatus == 0)
   {
     delay (3000); // controlla se dopo tre secondi il pulsante è ancora premuto
     if (digitalRead(progButton) == HIGH) return; // Falso allarme è stato premuto accidentalmente
     digitalWrite(led, HIGH);  //Accende il LED per indicare l'ingresso in programmazione 
     while (digitalRead(progButton) == LOW) { }  // attende il rilascio del pulsante
     
   }
  
   else {delay (250);}
   
    switch (decoderStatus)
      {
        case 0:
          decoderStatus = 1; // Acquisisce il nuovo indirizzo
          #ifdef DEBUG
            Serial.println("STATUS 1 - Acquire new decoder address");
            Serial.println(decoderStatus);
            Serial.println(LEDdelay); 
          #endif    
        break;
      
        // Cancellazione stato di programmazione    
        case 1:
          decoderStatus = 0; // ritorna al funzionamento normale
          digitalWrite(led, LOW);// spegne il LED
          #ifdef DEBUG
            Serial.println("STATUS 0 - cancelled address change");
            Serial.println(decoderStatus);
            Serial.println(LEDdelay);
          #endif
          break;
        }
  }
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
void sendAllsensors ()
{
  unsigned int tempaddr;
  
  readAllSensors ();
  tempaddr = myAddress + 1;  // point to first sensor address
  for (i = 0; i<numSensors; i++)
    {   
      LocoNet.reportSensor(tempaddr + i, ToggleSensorStatus(sensorValue[i]));
    }
}
  
///////////////////////////////////////////////////////////////////
void readAllSensors ()
{
  for (i=0; i<numSensors; i++)
  {
    priorSensorValue[i] = sensorValue[i]; //memorizza il vecchio stato
    sensorValue[i] = digitalRead(sensorPin[i]);  // Legge il nuovo stato
  }
  prevmillis =  millis (); // Reset clock per intervallo di lettura
}

/////////////////////////////////////////////////////////////////////
void sendChangedSensors ()
{
  unsigned int tempaddr;
  boolean sensValueTX;
  
  tempaddr = myAddress;  //Punta al primo indirizzo dei sensori     //<<<<<<-------------------- Modifica per numero di sensori
  for (i=0; i<numSensors; i++)
  {
    if (sensorValue[i] != priorSensorValue[i])  
    // se lo stato corrente è differente dal precedente stato invia il nuovo stato
    {
      
      LocoNet.reportSensor(tempaddr + i, ToggleSensorStatus(sensorValue[i]));  // Invia lo stato dopo aver invertito il valore
      #ifdef DEBUG
        Serial.println("Sent sensor : " + String(i) + " - Address: " + String(tempaddr) + " Status: " + String(sensorValue[i]));
      #endif
    }
  }
}

/////////////////////////////////////////////////////////////////////

// Ottiene l'indirizzo dal pacchetta passato in argomento
void notifySwitchRequest( uint16_t AddressMRRWA, uint8_t Output, uint8_t Direction ) 

{
      address_received = AddressMRRWA;
      
      #ifdef DEBUG
        Serial.println();
        Serial.print("Address: " + String(AddressMRRWA));
        Serial.print("Output:  ");
        Serial.println(Output >> 4,HEX);
        Serial.print("Direction:  ");
        Serial.println(Direction >> 5,HEX);
      #endif
}

/////////////////////////////////////////////////////////////////////

// Inverte lo stato della variabile passata in ingresso
bool ToggleSensorStatus(bool Value)

{
      if (Value)
      {
        return false;
      }
      else
      {
        return true;
      }
}

/////////////////////////////////////////////////////////////////////

// Stampa su seriale l'help per la programmazione


void serialEvent()
{

   byte caratteri; //contiene il numero di caratteri ricevuti
   byte tempChar; //contiene il carattere letto
   unsigned long tempMillis; //serve per il timeout sulla ricezione del valore seriale
   char c;
   String comando;
   
   if(Serial.available()){
      do {
        if(Serial.available()) {
          c = Serial.read();
          comando += c;
         }
      } while(c != '\n');
   Serial.print(cmParse(comando));   
   }
   
   
  

}


String cmParse(String request){

  String strParsed ;
  char c;
  char EOC;
  //Serial.println(request);
  
  c = request.charAt(0);              //Ottiene il primo carattere
  request.remove(request.length()-1,1);
  
//Serial.println(request);
  
  if (!request.endsWith("#")) {       //Se nonesiste il carattere di controllo alla fine del comando ritorna errore
    return "NOK";
  }
    
  switch(c){
    case 'f':   //report versione del firmware
      strParsed = reportFirmwareVersion();
      break;
    case 'a':  //Report indirizzo
      strParsed = reportLNAddress();
      break;
     case 'i':  //report Check interval
      strParsed = reportCheckInterval();
      break;
    default:
       strParsed = "NOK";
    
  }

  if (strParsed == "NOK"){
    return "NOK " + request;
  }else{ 
    return c + strParsed + '#';
  }
  
}

String reportFirmwareVersion(){           // Ottiene e formatta la richiesta di versione del Firmware

  String mjVer;
  String mnVer;
  String bdType;

  bdType = String(BOARD_TYPE);          // Tipo hardware di scheda
  if(int(CUR_VER_MAJOR) < 10){
    mjVer = '0' + String(CUR_VER_MAJOR);
  } else{
    mjVer = String(CUR_VER_MAJOR);
  }

  if(int(CUR_VER_MINOR) < 10){
    mnVer = '0' + String(CUR_VER_MINOR);
  } else{
    mnVer = String(CUR_VER_MINOR);
  } 
  return bdType + mjVer + mnVer;
}

String reportLNAddress(){

  char data[4];
  
  sprintf(data,"%04d",myAddress);
  return data;
}


String reportCheckInterval(){

  String sValue;
  int len;
  

 sValue = String(interval);
 len = sValue.length();
 sValue = String(len) + sValue;
 return sValue;

 
  
}

/*************************** END PROGRAM ***********************************************/
