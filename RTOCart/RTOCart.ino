/*
//                       RTO-CART by Andrea Ottaviani 
// Intellivision multicart based on Teensy 4.1 board -
// v. 0.1 2022-11-11 : Initial version
//
// Thanks to John Dullea (dwarfaxe@yahoo.com) for his support and his ACC projects, from wich i get some bits of code
//
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
#include <SD.h>
//#include <SdFat.h>


// set up variables using the SD utility library functions:
//Sd2Card card;
//SdVolume volume;
File root;
File entry;
File mapfile;

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

#define BUS_STATE_MASK  0x00000070L

long romLen;
char RBLo,RBHi;
//EXTMEM unsigned int ROM[65535];
 unsigned int ROM[65536];

// change this to match your SD shield or module;
// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
const int chipSelect = BUILTIN_SDCARD;

uint32_t selectedfile_size;       // BIN file size
char longfilename[46];        // long file name (trunked) 
char mapfilename[46];          // map cfg file name
int lenfilename; 
char path[50];

void SelectBinFile() // adapted from my SDLEP-TFT project  http://dcmoto.free.fr/   
{
  int i; 
 
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
      delay(1000);
    
    // Boucle de choix du fichier
   path[0]=0;
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
          strcpy(longfilename,"..");
        root.rewindDirectory();
         break; 
        } else {
        root.rewindDirectory();
        entry=root.openNextFile();
        strcpy(longfilename,entry.name());
        break;
        }
      //entry.close();
        continue;      
      }

        if(entry.isDirectory()) {   //handling subdirectory
          strcpy(longfilename,entry.name());
          if (strcmp(longfilename,"System Volume Information")==0) {
           continue;
         } else {
           filefound=true;
           break; 
         }
        }

      for(i=0; i<46; i++) longfilename[i]=0; 
      strcpy(longfilename,entry.name());
      
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
      if (digitalRead(BT2_PIN)) {
          digitalWriteFast(Status_PIN,HIGH);
          delay(400);
          digitalWriteFast(Status_PIN,LOW);
          break; // readnext file
      }

      //SELECT Button
      if (digitalRead(BT1_PIN)) {
          digitalWriteFast(Status_PIN,HIGH);
          delay(400);
          digitalWriteFast(Status_PIN,LOW);
          if (entry.isDirectory()) {
            root=SD.open(longfilename);
            strcat(path,longfilename);
            subfolder++;
            break;
          } else 
          if (strcmp(longfilename,"..")==0) {
            root=SD.open("/");
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

  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = MUX; //GPIO4_4  BDIR
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05 = MUX; //GPIO4_5  BC1
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06 = MUX; //GPIO4_6  BC2

  long PAD = 0B00000000000000010011000011111001;
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
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_14 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_15 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_04 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_05 = PAD;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_06 = PAD;

//  IOMUXC_GPR_GPR26 &=0xffff0000; not needed in teensy 4.1, it's default

String riga;
int slot;
char tmphex[] {0,0,0,0,0};
long mapfrom[5];
long mapto[5];
long maprom[5];

  Serial.begin(115200);
 // while(!Serial);
  Serial.println("Serial started");


  pinMode(Status_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BDIR_PIN, INPUT);
  pinMode(BC1_PIN, INPUT);
  pinMode(BC2_PIN, INPUT);
  pinMode(BT1_PIN,INPUT_PULLDOWN);
  pinMode(BT2_PIN,INPUT_PULLDOWN);
  
  digitalWriteFast(Status_PIN,HIGH);

  for (long i=0; i<65536; i++) {
    ROM[i]=0xfff;
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
  delay(2000); // Pause for 2 seconds
  
  while (!SD.begin(chipSelect)) 
  {
    display.clearDisplay();
    display.setCursor(0, 1);
    display.print("Insert SDCard");
    display.display();
    delay(2000);
  }
  
  SelectBinFile();
  
  for(int i=0; i<45; i++) {
        if (longfilename[i]=='.') {
          lenfilename= i;  //dot found
        }
  }
  
  memcpy(mapfilename,longfilename,lenfilename);
  Serial.print("mapname:");
  Serial.println(mapfilename);
  strcat(mapfilename,".cfg");
  mapfile=SD.open(path);
  if (!mapfile) Serial.println("path open error");

  strcat(path,"/");
  strcat(path,mapfilename);
  Serial.println(path);
  mapfile=SD.open(path);
  //  mapfile=SD.open(mapfilename);
  if (!mapfile) {
    mapfile.openNextFile();
    mapfile=SD.open("0.cfg");
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
  Serial.println(mapfile.name());
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 1);
  display.print("Reading:");
  display.setCursor(0,16);
  display.println(longfilename);
  display.display();

  slot=0;
  
  while (mapfile.available()) {
    riga=mapfile.readStringUntil('\n');
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
    if (riga=="[memattr]") {
      display.clearDisplay();
      display.setCursor(0, 1);
      display.print("[memattr]");
      display.display();
      while(1);
    }
  
    tmp=riga.substring(1,5);
    tmp.toCharArray(tmphex,5);
    mapfrom[slot]=strtoul(tmphex,NULL,16);
    tmp=riga.substring(9,13);
    tmp.toCharArray(tmphex,5);
    mapto[slot]=strtoul(tmphex,NULL,16);
    tmp=riga.substring(17,21);
    tmp.toCharArray(tmphex,5);
    maprom[slot]=strtoul(tmphex,NULL,16);
    Serial.print(mapfrom[slot],HEX);Serial.print("-");
    Serial.print(mapto[slot],HEX);Serial.print("-");
    Serial.println(maprom[slot],HEX);
    slot++;
  }
     
  slot=0;
  if (entry) {
    // read from the file until there's nothing else in it:
    romLen=0;
    while (entry.available()) {
      RBHi = entry.read();
      RBLo = entry.read();
      unsigned int dataW= RBLo | (RBHi << 8);
      fileOffset=romLen - mapfrom[slot] + maprom[slot];
      ROM[fileOffset]=dataW;
      Serial.print(fileOffset,HEX);
      Serial.print("-");
      Serial.println(ROM[fileOffset],HEX);
      romLen++;
      if (romLen>mapto[slot]) slot++;
    }
    //verify
    slot=0;
    romLen=0;
    entry.seek(0);
    while (entry.available()) {
      RBHi=entry.read();
      RBLo=entry.read();
      unsigned int dataW= RBLo | (RBHi << 8);
      fileOffset=romLen - mapfrom[slot] + maprom[slot];
      if (ROM[fileOffset]!= dataW) {
        Serial.print("Verify error at:");
        Serial.println(fileOffset,HEX);
      } 
      romLen++;
      if (romLen>mapto[slot]) slot++;
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
}

FASTRUN void loop()
{

  unsigned int lastBusState, busState1;
  unsigned int romOffset=0;
  unsigned int parallelBus, dataOut;
  //unsigned int tempbus;
  unsigned char busBit;
  bool deviceAddress = false; 
  unsigned int delRD=520;
  unsigned int delWR=550;
  
  display.clearDisplay();
  display.setCursor(0, 1);
  display.print("Emulating ROM:");
  display.setCursor(0,16);
  display.print(longfilename);
  display.display();
  
  // Initialize the bus state variables

  busLookup[BUS_NACT]  = 4; // 100
  busLookup[BUS_BAR]   = 1; // 001
  busLookup[BUS_IAB]   = 4; // 100
  busLookup[BUS_DWS]   = 2; // 010
  busLookup[BUS_ADAR]  = 1; // 001
  busLookup[BUS_DW]    = 4; // 100
  busLookup[BUS_DTB]   = 0; // 000
  busLookup[BUS_INTAK] = 4; // 100

  busState1 = BUS_NACT;
  lastBusState = BUS_NACT;

  dataOut=0;
  // Set the parallel port pins to defaults
  GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
  GPIO9_GDIR &= 0b11111111111111111111111110001111; // to set pins to inputs (bit16-31)
  
  digitalWriteFast(DIR_PIN,HIGH); // set dir to input from 5V INTV
  //wait for lvc245 to stabilize?
  // reading data
  parallelBus = (GPIO6_PSR >> 16);     
    
  // Main loop   
  digitalWriteFast(Status_PIN,LOW);
  while (true)
  {
    // Wait for the bus state to change
    do
    {
    } while (!((GPIO9_PSR ^ lastBusState) & BUS_STATE_MASK));
    
    // We detected a change, but reread the bus state to make sure that all three pins have settled
       lastBusState = GPIO9_PSR;

 //    lastBusState2 = GPIO9_PSR;
 //   if ((lastBusState2 & BUS_STATE_MASK) != (lastBusState & BUS_STATE_MASK)) {
 //     display.clearDisplay();
 //     display.setCursor(0, 1);
 //     display.setCursor(0,16);
 //     display.print(lastBusState2,BIN);
 //     display.display();      
 //   }

    busState1 = (lastBusState >> 4) & 7;     
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
        
        digitalWriteFast(DIR_PIN,LOW); // set dir to Output from 5V INTV
       
        // See if we can wait with NOPs and then switch our bus direction back to
        // input here instead of waiting for a NACT.  That would free up the NACT
        // afterward for emulation.
        // At 60 NOPs, only the low byte of the data occasionally makes it back
        // to the Intellivision.  All 16 bits of it get back to the Intellivision
        // when we NOP 67 times, but we're going to do 82 for a little extra margin.
        // 82 NOPs amounts to a wait of 482ns at 170MHz.  This works for an NTSC
        // Intellivision II.
      
        delayNanoseconds(delWR);   // to tune ok a 360 / 380
           
        digitalWriteFast(DIR_PIN,HIGH); // return to input to bus   
        GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
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

 
        // Prefetch data here because there won't be enough time to get it during DTB.
        // However, we can't take forever because of all the time we had to wait for
        // the address to appear on the bus.

        //digitalWriteFast(DIR_PIN,HIGH); // set dir to input from 5V INTV
        //GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
  
       // We have to wait until the address is stable on the bus
        delayNanoseconds(delRD); // 64 last, 70 buono last funziona con display on
        parallelBus = (GPIO6_PSR >> 16);     
        /*
        delayNanoseconds(10);
        tempbus = (GPIO6_PSR >> 16);     
       
       if (tempbus != parallelBus) {
          display.clearDisplay();
          display.setCursor(0, 1);
          display.print(parallelBus,HEX);
          display.setCursor(0,16);
          display.print(tempbus,HEX);
          display.display(); 
       }

       */

       // Load data for DTB here to save time
       
        romOffset=parallelBus;

        dataOut=ROM[romOffset];
        if (dataOut != 0xfff) {
          deviceAddress = true;
          //digitalWriteFast(Status_PIN,HIGH);
        } else {
          deviceAddress = false;
          //digitalWriteFast(Status_PIN,LOW);
        }
      
         /* 
        if ((romOffset > 0x4fff)&&(romOffset < 0x7000)) {
         deviceAddress = true;
         dataOut=ROM[romOffset];
         } else {
        deviceAddress = false;
        }
        */ 
      }
      else
      {
        busBit >>= 1;
        if (!busBit)
        {
          // -----------------------
          // DWS
          // -----------------------
          /*
          if (deviceAddress) {
          display.clearDisplay();
          display.setCursor(0, 1);
          display.print(busState1,BIN);
          display.setCursor(0,16);
          display.print(parallelBus,HEX);
          display.display(); 
          }
          */
        }
       else
       {  
          // -----------------------
          // NACT, IAB, DW, INTAK
          // -----------------------
         // reconnect to bus
         //GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
         //digitalWriteFast(DIR_PIN,HIGH); // input to bus
  
         // reconnect to bus
      
         if (digitalReadFast(BT1_PIN)) 
         {
          delWR--;
          display.clearDisplay();
          display.setCursor(0, 1);
          display.print("delWR:");
          display.setCursor(0, 16);
          display.print(delWR);
          display.display();
          delay(100);
         }    
         if (digitalReadFast(BT2_PIN)) 
         {
          delWR++;
          display.clearDisplay();
          display.setCursor(0, 1);
          display.print("delWR:");
          display.setCursor(0, 16);
          display.print(delWR);
          display.display();
          delay(100);
         }    
        } 
      } 
    }
   }
  } 
    
  
