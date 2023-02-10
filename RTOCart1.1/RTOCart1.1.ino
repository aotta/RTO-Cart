/*
//                       RTO-CART by Andrea Ottaviani 
// Intellivision multicart based on Teensy 4.1 board -
// v. 1.1 2023-01-24 : added reset handling
//                     fixed pin 8 video bug (connected to GND)
//                     added third button for prev file
//                     to do: check for extmem for dimensioning ROM size buffer
// v. 0.1 2022-11-11 : Initial version
//
// Thanks to John Dullea (dwarfaxe@yahoo.com) for his support and his ACC projects, from wich i get some bits of code
// 
//  More info on https://github.com/aotta/RTO-Cart
*/

// include for Oled display
#include <SPI.h>
#include <Wire.h>               // SCL pin 19, SDA pin 18 SCL2 25, SDA2 24

#include <Adafruit_GFX.h>      
#include <Adafruit_SSD1306.h>   

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

 // include the SD library:
//-#include <SD.h>

#include "splash.h"
#include <FS.h>
#include <SdFat.h>

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN


// change this to match your SD shield or module;
// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
//const int chipSelect = BUILTIN_SDCARD;


// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(50)

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif  ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS

// set up variables using the SD utility library functions:
SdFs sd;
FsFile root;
FsFile entry;
FsFile mapfile;
//-File root;
//-File entry;
//-File mapfile;

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
#define DIR2_PIN  6 //GPIO2_10 - TO PIN 1 LVC245, HIGH= A->B (INPUT 5V TO 3.3V) LOW= B->A (OUTPUT 3.3V TO 5V)
#define BT2_PIN  35 //GPIO2_26
#define BT1_PIN  36 //GPIO2_28
#define BT3_PIN  34 //GPIO2_29
#define Status_PIN   37 //GPIO2_19
#define RST_PIN   7 //GPIO2_17
#define MSYNC_PIN   8 //GPIO2_16

// Inty bus values (BC2+BC1+BDIR)
#define BUS_NACT  0b000  //0
#define BUS_ADAR  0b010  //2
#define BUS_IAB   0b100  //4
#define BUS_DTB   0b110  //6
#define BUS_BAR   0b001  //1
#define BUS_DW    0b011  //3
#define BUS_DWS   0b101  //5
#define BUS_INTAK 0b111  //7

unsigned char busLookup[8];

#define BUS_STATE_MASK  0x00000070L // gpio9_4.5.6 pins 5.6.7
#define BUS_DIR_MASK 0x00000100L // gpio9_8 pin 9 


char RBLo,RBHi;
#define BINLENGTH  224000L
#define RAMSIZE  8192
//EXTMEM unsigned int ROM[BINLENGTH];
uint16_t ROM[BINLENGTH] DMAMEM;
uint16_t RAM[RAMSIZE] DMAMEM;
#define maxHacks 32
uint16_t HACK[maxHacks];
uint16_t HACK_CODE[maxHacks];

unsigned int romLen;

unsigned int ramfrom = 0;
unsigned int ramto =   0;
unsigned int mapfrom[80];
unsigned int mapto[80];
unsigned int maprom[80];
unsigned int addrto[80];
unsigned int RAMused = 0;

unsigned int tipo[80];  // 0-rom / 1-page / 2-ram
unsigned int page[80];  // page number
int slot;
int hacks;

uint32_t selectedfile_size;       // BIN file size
char longfilename[46];        // long file name (trunked) 
char mapfilename[46];          // map cfg file name
int lenfilename; 
char path[50];

/////////////////////////////////////////////////////////////////////
//                   SelectBinFile()                               //
/////////////////////////////////////////////////////////////////////


void SelectBinFile() // adapted from my SDLEP-TFT project  http://dcmoto.free.fr/   
{
  int i; 
  int rootpos[255];
  int curpos=0;
  int lastpos;
 
  char suf[10];                 // extension file 
  uint64_t file_size;        // tailles des fichiers affiches
  // Initialisations
  selectedfile_size = 0;          // initialisation de la taille du fichier choisi
  file_size = 0;
  bool selected = false;
  bool filefound = false;
  int subfolder = 0;    // nexting subfolders counter
  int dot = 0;
 
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(1);
      display.setCursor(0, 1);
      display.print("Reading card..");
      display.display();
      //delay(1000);
    
    // Boucle de choix du fichier
   path[0]=0;
   if (!(root=sd.open("/"))) {
    Serial.println("Error opening Root");
   }
   // wait all buttons are released
  while(digitalReadFast(BT1_PIN)|digitalReadFast(BT2_PIN)|digitalReadFast(BT3_PIN)) 
  while (!selected)
  {
     while (!filefound) 
    {
        //read next file
        lastpos=root.curPosition();
        entry=root.openNextFile();
 
      if(!entry)
      {
        if (subfolder>0) {
          strcpy(longfilename,"..");
        root.rewindDirectory();
         break; 
        } else {
        root.rewindDirectory();
        lastpos=root.curPosition();
        entry=root.openNextFile();
        entry.getName(longfilename,45);
        break;
        }
      //entry.close();
        continue;      
      }

        if(entry.isDirectory()) {   //handling subdirectory
          entry.getName(longfilename,45);
          if (strcmp(longfilename,"System Volume Information")==0) {
           continue;
         } else {
           filefound=true;
           break; 
         }
        }

      for(i=0; i<46; i++) longfilename[i]=0; 
      entry.getName(longfilename,45);
      dot=0;
      for(i=0; i<43; i++) {
        if (longfilename[i]=='.') {
          dot=i;
        }
      }    
      if(dot==0) 
      {
        continue; //dot not found, get next file
      }
      suf[0]=0;
      //test du suffixe
      suf[0]=longfilename[dot+1];
      suf[1]=longfilename[dot+2];
      suf[2]=longfilename[dot+3];
      suf[3]=0;
    
      if(strcmp(suf,"bin")!=0  && strcmp(suf,"BIN")!=0 )
        continue; //not BIN file
      filefound=true;
    }

     // .bin found
     // save position
        rootpos[curpos]=lastpos;
        curpos++;
     
      file_size = entry.size();
      int kb=(file_size >>10);
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(1);
      display.setCursor(0, 1);
      display.print(longfilename);
      display.setCursor(0,24);
      if (entry.isDirectory()) {
        display.print("DIR");
      } else 
      { display.print("KB:");
      display.setCursor(22,24);
      display.print(kb,DEC);
      }
      display.display();
    
    filefound=false;
    
    
    //check user button and time
    while (1)
    {
      //Next button
      if (digitalRead(BT1_PIN)) {
          digitalWriteFast(Status_PIN,HIGH);
          delay(300);
          digitalWriteFast(Status_PIN,LOW);
          break; // readnext file
       }

      //Prev button
      if (digitalRead(BT3_PIN)) {
          digitalWriteFast(Status_PIN,HIGH);
          delay(300);
          digitalWriteFast(Status_PIN,LOW);
          if (curpos>1) {
            curpos-=2;
          } else {
            curpos=0;
          }
          root.seekSet(rootpos[curpos]);
          break;
      }
      //SELECT Button
      if (digitalRead(BT2_PIN)) {
          digitalWriteFast(Status_PIN,HIGH);
          delay(300);
          digitalWriteFast(Status_PIN,LOW);
          if (entry.isDirectory()) {
            root=sd.open(longfilename);
            strcat(path,longfilename);
            subfolder++;
            break;
          } else 
          if (strcmp(longfilename,"..")==0) {
            root=sd.open("/");
            for(i=0;i<50;i++) path[i]=0;
            subfolder--;
            break;
          } else {
          selected=true;
          break; // readnext file
          }
        }
      } 
    } //fin de la boucle de choix du fichier (on en sort si le fichier a ete choisi)
  //root.close();  //fermeture du repertoire de la carte SD
  //return file_return;
}

////////////////////////////////////////////////////////////////////////////////////
//                     LOAD ROM
////////////////////////////////////////////////////////////////////////////////////

void loadROM() {

  
  String riga;
  char tmphex[] {0,0,0,0,0,0};
  int linepos;

 
  for(int i=0; i<45; i++) {
        if (longfilename[i]=='.') {
          lenfilename= i;  //dot found
        }
  }
  
  memcpy(mapfilename,longfilename,lenfilename);
  strcat(mapfilename,".cfg");
 
  strcat(path,"/");
  strcat(path,mapfilename);
  mapfile=sd.open(path);
  //  mapfile=SD.open(mapfilename);
  if (!mapfile) {
    mapfile.openNextFile();
    mapfile=sd.open("0.cfg");
    }
  if (!mapfile) {
    display.clearDisplay();
    display.setCursor(0, 1);
    display.print("Map CFG file error");
    display.setCursor(0,16);
    display.print("restart RTO");
    display.display();
    while(1);
  }

  slot=0;
  hacks=0;
  
  while (mapfile.available()) {
    riga=mapfile.readStringUntil('\n');
    if (riga.length()>1) {
      String tmp=riga.substring(0,9);
      if (slot==0) {
        if (tmp=="[mapping]") {
          riga=mapfile.readStringUntil('\n');
        } else {
          display.clearDisplay();
          display.setCursor(0, 1);
          display.print("[mapping] not found");
          display.display();
          while(1);
        }
      }
    
      if (tmp=="[memattr]") {
        riga=mapfile.readStringUntil('\n');
        Serial.println("Memattr");
        tmp=riga.substring(1,5);
        tmp.toCharArray(tmphex,5);
        ramfrom=strtoul(tmphex,NULL,16);
        mapfrom[slot]=ramfrom;
        tmp=riga.substring(9,13);
        tmp.toCharArray(tmphex,5);
        ramto=strtoul(tmphex,NULL,16);
        mapto[slot]=ramto;
        maprom[slot]=ramfrom;
        addrto[slot]=maprom[slot]+(mapto[slot]-mapfrom[slot]);
        tipo[slot]=2; // RAM
        RAMused=1;
        slot++;
      } else {
        tmp=riga.substring(0,1);
        if (tmp=="p") {
          // [MACRO]
          tmp=riga.substring(2,6);
          tmp.toCharArray(tmphex,6);
          HACK[hacks]=strtoul(tmphex,NULL,16);
          tmp=riga.substring(7,10);
          tmp.toCharArray(tmphex,4);
          HACK_CODE[hacks]=strtoul(tmphex,NULL,16);
          Serial.print(HACK[hacks],HEX);Serial.print("-");
          Serial.println(HACK_CODE[hacks],HEX);
          hacks++;
        } else {
        //mapping
        linepos=riga.indexOf("-");
        if (linepos>=0) {
          tmp=riga.substring(1,linepos-1);
          tmp.toCharArray(tmphex,6);
          mapfrom[slot]=strtoul(tmphex,NULL,16);
          tmp=riga.substring(linepos+3,linepos+9);
          tmp.toCharArray(tmphex,6);
          mapto[slot]=strtoul(tmphex,NULL,16);
          if (linepos==6) {
            tmp=riga.substring(linepos+11,linepos+15);
          } else {
            tmp=riga.substring(linepos+12,linepos+17);
          }
          tmp.toCharArray(tmphex,6);
          maprom[slot]=strtoul(tmphex,NULL,16);
          if (linepos==6) {
            tmp=riga.substring(22,26);
          } else {
            tmp=riga.substring(24,28);
          }  
          tmp.toCharArray(tmphex,6);
          addrto[slot]=maprom[slot]+(mapto[slot]-mapfrom[slot]);
          if (!(strcmp(tmphex,"PAGE"))) {
            tipo[slot]=1;
            if (linepos==6) {
              tmp=riga.substring(27,28);
            } else {
              tmp=riga.substring(29,31);
            }   
          tmp.toCharArray(tmphex,2);
          page[slot]=strtoul(tmphex,NULL,16);
          } else {
            tipo[slot]=0;
          }
          slot++;
          }
        }
        Serial.print("Slot:");Serial.print(slot-1);
        Serial.print("-From:");Serial.print(mapfrom[slot-1],HEX);
        Serial.print(" to:");Serial.print(mapto[slot-1],HEX);
        Serial.print(" IN:");Serial.print(maprom[slot-1],HEX);
        Serial.print(" tipo:");Serial.print(tipo[slot-1]);
        Serial.print(" page:");Serial.print(page[slot-1]);
        Serial.print(" addr-to:");Serial.println(addrto[slot-1],HEX);
        }
     }
  }
     
  slot--;  // conservo il numero di slot!
  Serial.print("Slots:");Serial.println(slot);
  if (entry) {
    // read from the file until there's nothing else in it:
    romLen=0;
    while (entry.available()) {
      RBHi = entry.read();
      RBLo = entry.read();
      unsigned int dataW= RBLo | (RBHi << 8);
      //fileOffset=romLen - mapfrom[slot] + maprom[slot];
      ROM[romLen]=dataW;
      romLen++;
         if (romLen>BINLENGTH) {
          display.clearDisplay();
          display.setCursor(0, 1);
          display.print("File too big!");
          display.setCursor(0,16);
          display.print(longfilename);
          display.display();
          while(1);
      }
  
    }
    //verify
  //  slot=0;
    romLen=0;
    entry.seek(0);
    while (entry.available()) {
      RBHi=entry.read();
      RBLo=entry.read();
      unsigned int dataW= RBLo | (RBHi << 8);
      //fileOffset=romLen - mapfrom[slot] + maprom[slot];
      if (ROM[romLen]!= dataW) {
        display.clearDisplay();
        display.setCursor(0, 1);
        display.print("Verify file error!");
        display.display();
        while(1);
      } 
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
    display.print(longfilename);
    display.display();
    return; 
     }
  if (hacks>0) {
    display.clearDisplay();
    display.setCursor(0, 1);
    display.print("Hack found: use it?");
    display.setCursor(0,16);
    display.print("OK to confirm, Next or Prev to deny");
    display.display();
    delay(500); // wait to clear buttons
    while(((digitalReadFast(BT1_PIN)==0) & (digitalReadFast(BT2_PIN)==0) & (digitalReadFast(BT3_PIN)==0))) {
      if ((digitalReadFast(BT1_PIN)==1) | (digitalReadFast(BT3_PIN)==1)) hacks=0;
    }
  } 
}

////////////////////////////////////////////////////////////////////////////////////
//                     SETUP
////////////////////////////////////////////////////////////////////////////////////
void setup() {

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
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 = MUX;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = MUX; //GPIO4_4  BDIR
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05 = MUX; //GPIO4_5  BC1
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06 = MUX; //GPIO4_6  BC2

  long PAD = 0B00000000000000010011000000111000;
  //           |||||||||||||||||||||||||||||||+-FAST
  //           |||||||||||||||||||||||||||||++--RESERVED
  //           ||||||||||||||||||||||||||+++----DSE
  //           ||||||||||||||||||||||||++-------SPEED 
  //           |||||||||||||||||||||+++---------RESERVED
  //           ||||||||||||||||||||+------------ODE (OPEN DRAIN) 
  //           |||||||||||||||||||+-------------PKE
  //           ||||||||||||||||||+--------------PUE
  //           ||||||||||||||||++---------------PUS
  //           |||||||||||||||+-----------------HYS
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
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_13 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_14 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_15 = PAD;

  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_04 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_05 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_06 = PAD;

 // Initialize the bus state variables

  busLookup[BUS_NACT]  = 4; // 100
  busLookup[BUS_BAR]   = 1; // 001
  busLookup[BUS_IAB]   = 4; // 100
  busLookup[BUS_DWS]   = 2; // 010   // test without dws handling
  busLookup[BUS_ADAR]  = 1; // 001
  busLookup[BUS_DW]    = 4; // 100
  busLookup[BUS_DTB]   = 0; // 000
  busLookup[BUS_INTAK] = 4; // 100
  
//  IOMUXC_GPR_GPR26 &=0xffff0000; not needed in teensy 4.1, it's default

 //Serial.begin(115200);
  
  pinMode(Status_PIN, OUTPUT);
  pinMode(BT1_PIN,INPUT_PULLDOWN);
  pinMode(BT2_PIN,INPUT_PULLDOWN);
  pinMode(BT3_PIN,INPUT_PULLDOWN);
  pinMode(DIR2_PIN,OUTPUT);
  pinMode(RST_PIN,OUTPUT);
  pinMode(MSYNC_PIN,INPUT_PULLDOWN);
    
  digitalWriteFast(RST_PIN,LOW);   //???????????
  digitalWriteFast(DIR2_PIN,HIGH); // A->B, TEENSY to INTV BUS
  
  for (long i=0; i<BINLENGTH; i++) {
    ROM[i]=0;
  }
 
 for (long i=0; i<RAMSIZE; i++) {
    RAM[i]=0;
  }

 for (int i=0; i<maxHacks; i++) {
  HACK[i]=0;
  HACK_CODE[i]=0;
 }
  
  while (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS));
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(1);
 
  display.setCursor(12, 1);
  display.print("RTO Cart");
  display.setTextSize(1);
  display.setCursor(110,22);
  display.print("1.0");
  display.display();
  //delay(500); // Pause for 2 seconds
  
  while (!sd.begin(SD_CONFIG)) 
  {
    display.clearDisplay();
    display.setCursor(0, 1);
    display.print("Insert SDCard");
    display.display();
  //  delay(1000);
  }

  for(int i=0;i<1280;i++){
    unsigned int dataW= splash[(i*2)+1] | (splash[i*2] << 8);
    ROM[i]=dataW;
  }
  slot=0;
  mapfrom[0]=0x0;
  mapto[0]=0x4ff;
  maprom[0]=0x5000;
  tipo[0]=0;
  page[0]=0;
  addrto[0]=0x54ff;
    
  display.clearDisplay();
  display.setCursor(0, 1);
  display.print("Press a button on RTO");
  display.display();
  resetCart();
  HandleBUS();
  
  Serial.println("exit splash");
  SelectBinFile();
  loadROM();
  display.clearDisplay();
  display.setCursor(0, 1);
  display.print("Emulating ROM:");
  display.setCursor(0,16);
  display.print(longfilename);
  if (hacks>0) {
    display.setCursor(92,1);
    display.setTextSize(1.5);
    display.print("(H)");
  }
  display.display();
  
  
}

void yield () {} //Get rid of the hidden function that checks for serial input and such.

void doReboot() {
  SCB_AIRCR = 0x05FA0004;
}

////////////////////////////////////////////////////////////////////////////////////
//                     RESET
////////////////////////////////////////////////////////////////////////////////////
void resetCart() {
 
  digitalWriteFast(Status_PIN,HIGH);
  Serial.println(digitalReadFast(MSYNC_PIN));

  do {
    digitalWriteFast(Status_PIN,digitalReadFast(MSYNC_PIN));
    digitalWriteFast(RST_PIN,HIGH);
    delay(6);
    if (digitalReadFast(MSYNC_PIN)==0) break;
    digitalWriteFast(RST_PIN,LOW);
    delay(2);
    if (digitalReadFast(MSYNC_PIN)==0) break;
    
  } while (digitalReadFast(MSYNC_PIN)==1);

//  do {
   digitalWriteFast(Status_PIN,digitalReadFast(MSYNC_PIN));
   digitalWriteFast(RST_PIN,LOW); 
 // } while (digitalReadFast(MSYNC_PIN)==0);
  
 
  Serial.println(digitalReadFast(MSYNC_PIN));
  
  digitalWriteFast(Status_PIN,LOW);
   //pinMode(RST_PIN,INPUT);
  //Serial.println("Reset");
}

////////////////////////////////////////////////////////////////////////////////////
//                     HANDLE BUS
////////////////////////////////////////////////////////////////////////////////////

FASTRUN void HandleBUS()
{
  unsigned int lastBusState, busState1;
  unsigned int parallelBus, dataOut;
  unsigned char busBit;
  bool deviceAddress = false; 
  unsigned int delRD=482;
  unsigned int delWR=540;
  unsigned int delADAR=0;
  unsigned int curPage=0;
  unsigned int checkPage=0;
  int exitBUS;
   
  busState1 = BUS_NACT;
  lastBusState = BUS_NACT;
  
  dataOut=0;
  // Set the parallel port pins to defaults
  GPIO6_GDIR &= 0x0000ffff; // to set pins to inputs (bit16-31)
  GPIO9_GDIR &= 0xffffff00; // to set pins to inputs (bit5-7)
  GPIO9_GDIR |= 0x00000100; // to set pins 9 to output (dir)
  
  GPIO9_DR |= BUS_DIR_MASK;  // set bit 9 ->set dir to input 
  //wait for lvc245 to stabilize?
  // reading data
  parallelBus = (GPIO6_PSR >> 16);     
  
  // Main loop   

  exitBUS=1;

  while (exitBUS)
  {

  
    // Wait for the bus state to change
  
    do
    {
    if (digitalReadFast(BT1_PIN)) {
      exitBUS=0;
      break;
      }
    if (digitalReadFast(BT3_PIN)) doReboot();
    
    } while (!((GPIO9_PSR ^ lastBusState) & BUS_STATE_MASK));
    // We detected a change, but reread the bus state to make sure that all three pins have settled
     lastBusState = GPIO9_PSR; //if gpio9

 
    
    busState1 = (lastBusState >> 4) & 7; //if gpio9    
    busBit = busLookup[busState1];
    // Avoiding switch statements here because timing is critical and needs to be deterministic
    if (!busBit)
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
                
        GPIO6_GDIR |= 0xFFFF0000; // to set pins to outputs (bit16-31)
        // writing data (and preserve other pins data)
        GPIO6_DR = (GPIO6_DR & 0x0000FFFF) | (dataOut << 16);
        delayNanoseconds(25);
        // latch
        GPIO9_DR &= ~BUS_DIR_MASK;  // clr bit 9 ->set dir to output (low)
           
        // Wait of 482ns 
        delayNanoseconds(delWR);   // to tune ok a 360 / 380
        GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
        GPIO9_DR |= BUS_DIR_MASK;  // set bit 9 ->set dir to input (high)
      }
    }
    else
    {
      busBit >>= 1;
      if (!busBit)
      {
        // -----------------------
        // BAR, ADAR
        // -----------------------
        if ((deviceAddress) and (busState1==BUS_ADAR)) 
      {
        // The data was prefetched during BAR/ADAR.  There isn't nearly enough time to fetch it here.
        // We can just output it.
                
        GPIO6_GDIR |= 0xFFFF0000; // to set pins to outputs (bit16-31)
        // writing data (and preserve other pins data)
        GPIO6_DR = (GPIO6_DR & 0x0000FFFF) | (dataOut << 16);
         // latch
        GPIO9_DR &= ~BUS_DIR_MASK;  // clr bit 9 ->set dir to output (low)
        
        delayNanoseconds(delWR);   // to tune ok a 360 / 380
      }
        // Prefetch data here because there won't be enough time to get it during DTB.
        // However, we can't take forever because of all the time we had to wait for
        // the address to appear on the bus.
      
        GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
        GPIO9_DR |= BUS_DIR_MASK;  // set bit 9 ->set dir to input (high)   
       // We have to wait until the address is stable on the bus
        delayNanoseconds(delRD-delADAR); // waiting bus is stable
        
        parallelBus = GPIO6_PSR >> 16; 
       
       // Load data for DTB here to save time
    
        deviceAddress = false;

        for (int i=0; i < slot+1; i++) {
            if ((parallelBus >= maprom[i]) && (parallelBus<=addrto[i])) {
              if (tipo[i]==2) {
                dataOut=RAM[parallelBus - ramfrom];
                deviceAddress = true;
                break;
              }
              if ((tipo[i]==1) && ((parallelBus & 0xfff)==0xfff)) {
                checkPage=1;
                deviceAddress = true;
                break;
              }
              if ((page[i]==curPage) && (tipo[i]==1)) {
                dataOut=ROM[(parallelBus - maprom[i]) + mapfrom[i]];
                deviceAddress = true;
                break;
              }
              if (tipo[i]==0) {
                dataOut=ROM[(parallelBus - maprom[i]) + mapfrom[i]];
                deviceAddress = true;
                break;
              }
            } 
                      
        }
        
        if (hacks>0) {
          for (int i=0; i<maxHacks;i++) {
            if (parallelBus==HACK[i]) {
              dataOut=HACK_CODE[i];
              deviceAddress = true;
            }
            break;
          }
        } 
       
      }
      else
      {
         busBit >>= 1;
        if (!busBit)
        {
          // -----------------------
          // DWS
          // -----------------------
          
          if (deviceAddress) {
            GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
            GPIO9_DR |= BUS_DIR_MASK;  // set bit 9 ->set dir to input (high)   
            unsigned int dataWrite = GPIO6_PSR >> 16;
            if (RAMused == 1) RAM[parallelBus-ramfrom]=dataWrite;
            if ((checkPage == 1) && (((dataWrite >> 4) & 0xff) == 0xA5)) {
            //  if (checkPage == 1) {
                curPage=dataWrite & 0xf;
              checkPage=0;
             
          }
        }
        else
        {
         // -----------------------
         // NACT, IAB, DW, INTAK
         // -----------------------
         // reconnect to bus
          GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
          GPIO9_DR |= BUS_DIR_MASK;  // set bit 9 ->set dir to input (high)
    
        }
        
        }
      } 
    }    
   }
} 


////////////////////////////////////////////////////////////////////////////////////
//                     MAIN LOOP
////////////////////////////////////////////////////////////////////////////////////    
  
//FASTRUN void loop()
void loop()
{
resetCart();  
HandleBUS();
}
