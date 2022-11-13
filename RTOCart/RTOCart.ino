/*
//                       RTO-CART by Andrea Ottaviani 
// Intellivision multicart based on Teensy 4.1 board -
// v. 0.1 2022-11-11 : Initial version
//
// Thanks to John Dullea (dwarfaxe@yahoo.com) for his support and his ACC projects, from wich i get some bits of code
//
*/
// include for Oled display
#include <Wire.h>               // SCL pin 19, SDA pin 18 SCL2 25, SDA2 24
#include <Adafruit_GFX.h>      
#include <Adafruit_SSD1306.h>   
 // include the SD library:

#include <SD.h>
//#include <SdFat.h>
#include <SPI.h>

// set up variables using the SD utility library functions:
//Sd2Card card;
//SdVolume volume;
File root;
File entry;
Adafruit_SSD1306 display(128, 32, &Wire2, -1, 1000000);  // 1MHz I2C clock

// here is one continuous block of 16 bits
// should be able to read like this
// uint16_t addr= (GPIO6_DR & 0xFF00) / 256;

// Teensy pin usage definitions
#define D0_PIN    19 //GPIO6_16
#define D1_PIN    18 //GPIO6_17
#define D2_PIN    14 //GPIO6_18
#define D3_PIN    15 //GPIO6_19
#define D4_PIN    40 //GPIO6_20
#define D5_PIN    41 //GPIO6_21
#define D6_PIN    17 //GPIO6_22
#define D7_PIN    16 //GPIO6_23
#define D8_PIN    22 //GPIO6_24
#define D9_PIN    23 //GPIO6_25
#define D10_PIN   20 //GPIO6_26
#define D11_PIN   21 //GPIO6_27
#define D12_PIN   38 //GPIO6_28
#define D13_PIN   39 //GPIO6_29
#define D14_PIN   26 //GPIO6_30
#define D15_PIN   27 //GPIO6_31

int D_PIN[] ={19,18,14,15,40,41,17,16,22,23,20,21,38,39,26,27};

#define BDIR_PIN  2 //GPIO4_4
#define BC1_PIN   3 //GPIO4_5
#define BC2_PIN   4 //GPIO4_6
#define DIR_PIN   5 //GPIO4_8 - TO PIN 1 LVC245, HIGH= A->B (INPUT 5V TO 3.3V) LOW= B->A (OUTPUT 3.3V TO 5V)
#define BT2_PIN  35 //GPIO2_26
#define BT1_PIN  36 //GPIO2_28
#define Status_PIN   37 //GPIO2_19


// Inty bus values (BDIR+BC2+BC1)

#define BUS_NACT  0
#define BUS_ADAR  1
#define BUS_IAB   2
#define BUS_DTB   3
#define BUS_BAR   4
#define BUS_DW    5
#define BUS_DWS   6
#define BUS_INTAK 7


long romLen;
byte RBLo,RBHi;
//EXTMEM unsigned int ROM[65535];
 unsigned int ROM[65536];

// change this to match your SD shield or module;
// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
const int chipSelect = BUILTIN_SDCARD;

uint32_t selectedfile_size;       // BIN file size
char selFileName[36];


char* SelectBinFile() // adapted from my SDLEP-TFT project  http://dcmoto.free.fr/   
{
  int i; 
 
  char suf[10];                 // extension file 
  char longfilename[36];        // long file name (trunked) 
  
  uint64_t file_size;        // tailles des fichiers affiches
  char* file_name;  // premiers blocs des fichiers affiches
  // Initialisations
  selectedfile_size = 0;          // initialisation de la taille du fichier choisi
  file_size = 0;
  bool selected = false;
  bool filefound = false;
  int subfolder = 0;    // nexting subfolders counter

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(1);
      display.setCursor(0, 1);
      display.print("Reading card..");
      display.display();
    
    // Boucle de choix du fichier
   root=SD.open("/");
  
  while (!selected)
  {
    while (!filefound) 
    {
      //read next file
      entry=root.openNextFile();
      if(!entry)
      {
        if (subfolder>0) {
          strcpy(longfilename,"/..");
        root.rewindDirectory();
        entry=root.openNextFile();
        filefound=true;
        break; 
        } else {
        root.rewindDirectory();
        entry=root.openNextFile();
        break;
        }
      //entry.close();
        Serial.println("Rewind");
        continue;      
      }
        if(entry.isDirectory()) {   //handling subdirectory
          file_name=entry.name();
          strcpy(longfilename,file_name);
          if (strcmp(file_name,"System Volume Information")==0) {
           continue;
         } else {
           filefound=true;
           break; 
         }
        }

      file_name=entry.name();
      strcpy(longfilename,file_name);
     
      file_size = entry.size();
      for(i=0; i<35; i++)
        if (longfilename[i]=='.')
          break;  //point trouve
      
      if(i==35)
        continue; //dot not found, get next file
   
      //test du suffixe
      suf[0]=longfilename[i+1];
      suf[1]=longfilename[i+2];
      suf[2]=longfilename[i+3];
      suf[3]=0;
      
      if(strcmp(suf,"bin")!=0  && strcmp(suf,"BIN")!=0 )
        continue; //not BIN file
      
      filefound=true;
    }
      // .bin found
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(1);
      display.setCursor(0, 1);
      display.print(longfilename);
      display.display();
    filefound=false;
    //check user button and time
    while (1)
    {
      //Next button
      if (digitalRead(BT1_PIN)) {
          digitalWriteFast(Status_PIN,HIGH);
          delay(400);
          digitalWriteFast(Status_PIN,LOW);
          break; // readnext file
      }

      //SELECT Button
      if (digitalRead(BT2_PIN)) {
          Serial.println(longfilename);
          digitalWriteFast(Status_PIN,HIGH);
          delay(400);
          digitalWriteFast(Status_PIN,LOW);
          if (entry.isDirectory()) {
            root=SD.open(longfilename);
            subfolder++;
            Serial.println("dir down");
            break;
          } else 
          if (strcmp(longfilename,"/..")==0) {
            root=SD.open("/..");
            subfolder--;
            Serial.println("dir up");
            break;
          } else {
          strcpy(file_name,longfilename);
          selected=true;
          break; // readnext file
          }
        }
      } 
    } //fin de la boucle de choix du fichier (on en sort si le fichier a ete choisi)
  //root.close();  //fermeture du repertoire de la carte SD
  Serial.print("Returning:");
  Serial.print(file_name);
  return file_name;
}


void setup() {

int fileOffset;

  long MUX = 21;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_00 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_01 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_12 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_13 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 = MUX;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = MUX;

  long PAD = 0B00000000000000000000000010111001;
  //           |||||||||||||||||||||||||||||||+-FAST
  //           |||||||||||||||||||||||||||||++--RESERVED
  //           ||||||||||||||||||||||||||+++----DSE
  //           ||||||||||||||||||||||||++-------SPEED 
  //           |||||||||||||||||||||+++---------RESERVED
  //           ||||||||||||||||||||+------------ODE (OPEND RAIN) 
  //           |||||||||||||||||||+-------------PKE
  //           ||||||||||||||||||+--------------PUE
  //           |||||||||||||||||+---------------PUS
  //           ||||||||||||||||+----------------HYS
  //           ++++++++++++++++-----------------RESERVED
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_00 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_01 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_04 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_05 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_06 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_07 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_08 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_09 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_10 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_11 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_12 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_14 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_15 = PAD;

//  IOMUXC_GPR_GPR26 &=0xffff0000;

  Serial.begin(115200);

  //while(!Serial);
  Serial.println("Serial started");

  for (long i=0; i<65536; i++) {
    ROM[i]=0xfff;
  }

  while (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C));
  delay(100);
  display.clearDisplay();
  display.setCursor(0, 1);
  display.print("Oled intitialized");
  display.display();

  while (!SD.begin(chipSelect)) 
  {
    Serial.println("initialization failed!");
    display.clearDisplay();
    display.setCursor(0, 1);
    display.print("Insert SDCard");
    display.display();
    delay(2000);
  }

  for (int i=0; i<16; i++) {pinMode(D_PIN[i], OUTPUT);} 

  pinMode(Status_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BDIR_PIN, INPUT);
  pinMode(BC1_PIN, INPUT);
  pinMode(BC2_PIN, INPUT);
  pinMode(BT1_PIN,INPUT_PULLDOWN);
  pinMode(BT2_PIN,INPUT_PULLDOWN);
  
  digitalWriteFast(Status_PIN,HIGH);




  char* sfn=SelectBinFile();
  Serial.print("Selected: ");
  Serial.println(sfn);

  strcpy(selFileName,sfn);
  Serial.println(selFileName);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 1);
  display.print("Reading:");
  display.setCursor(0,16);
  display.print(selFileName);
  display.display();

  if (entry) {
    // read from the file until there's nothing else in it:
    romLen=0;
    while (entry.available()) {
      RBHi=entry.read();
      RBLo=entry.read();
      fileOffset=romLen + 0x5000;
      ROM[fileOffset]=RBLo | (RBHi << 8);
      Serial.print(fileOffset,HEX);
      Serial.print("-");
      Serial.println(ROM[fileOffset],HEX);
      romLen++;
    }
    // close the file:
    entry.close();
  } else {
    // if the file didn't open, print an error:
    display.clearDisplay();
    display.setCursor(0, 1);
    display.print("Error opening file:");
    display.setCursor(0,16);
    display.print(selFileName);
    display.display();
    return; 
     }
}

FASTRUN void loop()
{

  unsigned int lastBusState, busState1;
  unsigned int romOffset=0;
  unsigned int parallelBus,dataOut;
  bool deviceAddress = false;  
    display.clearDisplay();
    display.setCursor(0, 1);
    display.print("Emulating ROM:");
    display.setCursor(0,16);
    display.print(selFileName);
    display.display();
  // Initialize the bus state variables

  lastBusState = BUS_NACT;
  dataOut=0;
  // Set the parallel port pins to defaults
  for (int i=0; i<16; i++) {pinMode(D_PIN[i], INPUT);} // SET DIR TO INPUT
  GPIO6_GDIR &= 0x00000000; // to set pins to inputs (bit16-31)
        
    digitalWriteFast(DIR_PIN,HIGH); // set dir to input from 5V INTV
    //wait for lvc245 to stabilize
  // reading data
    parallelBus = (GPIO6_PSR >> 16);     
    
   // Main loop   
  digitalWriteFast(Status_PIN,LOW);
  while (true)
  {

    // Wait for the bus state to change

    do
    {
      busState1=((digitalReadFast(BDIR_PIN)<<2)|(digitalReadFast(BC2_PIN)<<1)|digitalReadFast(BC1_PIN));
    } while ( busState1 == lastBusState);

    // We detected a change, but reread the bus state to make sure that all three pins have settled
    busState1 = (digitalReadFast(BDIR_PIN)<<2)|(digitalReadFast(BC2_PIN)<<1)|digitalReadFast(BC1_PIN);
    lastBusState=busState1;
    
   
    
    // Avoiding switch statements here because timing is critical and needs to be deterministic

    if (busState1==BUS_DTB)
    {
      // -----------------------
      // DTB
      // -----------------------

      // DTB needs to come first since its timing is critical.  The CP-1600 expects data to be
      // placed on the bus early in the bus cycle (i.e. we need to get data on the bus quickly!)
 
      if (deviceAddress)
      {
        // The data was prefetched during BAR/ADAR.  There isn't nearly enough time to fetch it here.
        // We can just output it.
        
        // for (int i=0; i<16; i++) {pinMode(D_PIN[i], OUTPUT);} // SET DIR TO OUTPUT
        GPIO6_GDIR |= 0xFFFF0000; // to set pins to outputs (bit16-31)
        // writing data (and preserve other pins data)
        GPIO6_DR = (GPIO6_DR & 0x0000FFFF) | (dataOut << 16);
 
        //delayNanoseconds(32);   // to tune ok a 32
        digitalWriteFast(DIR_PIN,LOW); // set dir to Output from 5V INTV
        
        delayNanoseconds(482);   // to tune ok a 360 / 380
     
        // See if we can wait with NOPs and then switch our bus direction back to
        // input here instead of waiting for a NACT.  That would free up the NACT
        // afterward for emulation.
        // At 60 NOPs, only the low byte of the data occasionally makes it back
        // to the Intellivision.  All 16 bits of it get back to the Intellivision
        // when we NOP 67 times, but we're going to do 82 for a little extra margin.
        // 82 NOPs amounts to a wait of 482ns at 170MHz.  This works for an NTSC
        // Intellivision II.
        GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
        digitalWriteFast(DIR_PIN,HIGH); // return to input to bus   
      }
    }
    else
    {
      if ((busState1==BUS_BAR)||(busState1==BUS_ADAR))
      {
        // -----------------------
        // BAR, ADAR
        // -----------------------

 
        // Prefetch data here because there won't be enough time to get it during DTB.
        // However, we can't take forever because of all the time we had to wait for
        // the address to appear on the bus.

       // We have to wait until the address is stable on the bus

        digitalWriteFast(DIR_PIN,HIGH); // set dir to input from 5V INTV
       //wait for lvc245 to stabilize
        delayNanoseconds(600); // 64 last, 70 buono last funziona con display on
       
       GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
       parallelBus = (GPIO6_PSR >> 16);     

       
      }
      else
      {
          // -----------------------
          // NACT, IAB, DW, INTAK
          // -----------------------
        GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
        digitalWriteFast(DIR_PIN,HIGH); // input to bus
        // reconnect to bus
          
          // Load data for DTB here to save time
          romOffset=parallelBus;
         
          dataOut=ROM[romOffset];
          if (dataOut != 0xfff) {
          deviceAddress = true;
          digitalWriteFast(Status_PIN,HIGH);
          } else {
            deviceAddress = false;
            digitalWriteFast(Status_PIN,LOW);
          }  

       } // SET DIR TO INPUT
      } 
    }
   } 
    
  
