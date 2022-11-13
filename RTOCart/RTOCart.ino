#include <Wire.h>               // SCL pin 19, SDA pin 18 SCL2 25, SDA2 24
#include <Adafruit_GFX.h>      
#include <Adafruit_SSD1306.h>   
 // include the SD library:
#include <SD.h>
#include <SPI.h>
File romFile;
// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
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


// Teensy pin usage masks

#define D0_PIN_MASK     0b0000000000000001
#define D1_PIN_MASK     0b0000000000000010
#define D2_PIN_MASK     0b0000000000000100
#define D3_PIN_MASK     0b0000000000001000
#define D4_PIN_MASK     0b0000000000010000
#define D5_PIN_MASK     0b0000000000100000
#define D6_PIN_MASK     0b0000000001000000
#define D7_PIN_MASK     0b0000000010000000
#define D8_PIN_MASK     0b0000000100000000
#define D9_PIN_MASK     0b0000001000000000
#define D10_PIN_MASK    0b0000010000000000
#define D11_PIN_MASK    0b0000100000000000
#define D12_PIN_MASK    0b0001000000000000
#define D13_PIN_MASK    0b0010000000000000
#define D14_PIN_MASK    0b0100000000000000
#define D15_PIN_MASK    0b1000000000000000


// Inty bus values (BDIR+BC2+BC1)

#define BUS_NACT  0
#define BUS_ADAR  1
#define BUS_IAB   2
#define BUS_DTB   3
#define BUS_BAR   4
#define BUS_DW    5
#define BUS_DWS   6
#define BUS_INTAK 7

unsigned char busLookup[8];
long romLen;
byte RBLo,RBHi;
//EXTMEM unsigned int ROM[65535];
 unsigned int ROM[65536];
// change this to match your SD shield or module;
// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD

const int chipSelect = BUILTIN_SDCARD;


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

 // Serial.begin(115200);

 //while(!Serial);

  for (long i=0; i<65536; i++) {
    ROM[i]=0xfff;
  }

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(100);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 1);
  display.print("Reading: Last.bin");
  display.display();

  for (int i=0; i<16; i++) {pinMode(D_PIN[i], OUTPUT);} 

  pinMode(Status_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BDIR_PIN, INPUT);
  pinMode(BC1_PIN, INPUT);
  pinMode(BC2_PIN, INPUT);
  digitalWriteFast(Status_PIN,HIGH);


  if (!SD.begin(chipSelect)) {
    //Serial.println("initialization failed!");
    display.clearDisplay();
    display.setCursor(0, 1);
    display.print("SD init fail!");
    display.display();
    return;
  }
  romFile = SD.open("last.bin");
  if (romFile) {
    // read from the file until there's nothing else in it:
    romLen=0;
    while (romFile.available()) {
      RBHi=romFile.read();
      RBLo=romFile.read();
      fileOffset=romLen + 0x5000;
      ROM[fileOffset]=RBLo | (RBHi << 8);
    //  Serial.print(fileOffset,HEX);
    //  Serial.print("-");
    //  Serial.println(ROM[fileOffset],HEX);
      romLen++;
    }
    // close the file:
    romFile.close();
  } else {
    // if the file didn't open, print an error:
    display.clearDisplay();
    display.setCursor(0, 1);
    display.print("Error opening file");
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
  
  // Initialize the bus state variables

  lastBusState = BUS_NACT;
  dataOut=0;
  // Set the parallel port pins to defaults
  for (int i=0; i<16; i++) {pinMode(D_PIN[i], INPUT);} // SET DIR TO INPUT
  GPIO6_GDIR &= 0x00000000; // to set pins to inputs (bit16-31)
        
    digitalWriteFast(DIR_PIN,HIGH); // set dir to input from 5V INTV
    //wait for lvc245 to stabilize
  delayNanoseconds(600);
  // reading data
    parallelBus = (GPIO6_PSR >> 16);     
    //parallelBus=(digitalReadFast(D0_PIN) | digitalReadFast(D1_PIN)<<1 | digitalReadFast(D2_PIN)<<2 | digitalReadFast(D3_PIN)<<3 |
    //             digitalReadFast(D4_PIN)<<4 | digitalReadFast(D5_PIN)<<5 | digitalReadFast(D6_PIN)<<6 |
    //             digitalReadFast(D7_PIN)<<7 | digitalReadFast(D8_PIN)<<8 | digitalReadFast(D9_PIN)<<9 |
    //             digitalReadFast(D10_PIN)<<10 | digitalReadFast(D11_PIN)<<11 | digitalReadFast(D12_PIN)<<12 |
    //             digitalReadFast(D13_PIN)<<13| digitalReadFast(D14_PIN)<<14 | digitalReadFast(D15_PIN)<<15);
    display.clearDisplay();
    display.setCursor(0, 1);
    display.print(parallelBus,HEX);
    display.display();
    
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
        
        delayNanoseconds(780);   // to tune ok a 360 / 380
     
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
        delayNanoseconds(780); // 64 last, 70 buono last funziona con display on
       
       GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
       parallelBus = (GPIO6_PSR >> 16);     
    
         
          romOffset=parallelBus;
         
          dataOut=ROM[romOffset];
          if (dataOut != 0xfff) {
          deviceAddress = true;
          //digitalWriteFast(Status_PIN,HIGH);
          } else {
            deviceAddress = false;
            //digitalWriteFast(Status_PIN,LOW);
          }  
       
      }
      else
      {
        
          // -----------------------
          // NACT, IAB, DW, INTAK
          // -----------------------
        GPIO6_GDIR &= 0x0000FFFF; // to set pins to inputs (bit16-31)
       } // SET DIR TO INPUT
      } 
    }
   } 
    
  
