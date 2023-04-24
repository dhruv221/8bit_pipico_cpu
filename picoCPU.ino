#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>




//--------human interface definitions--------//
#define i2c_Address 0x3c
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     //   QT-PY / XIAO
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);
//Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//for esp32:
#define left_btn  12
#define right_btn 13
#define up_btn    27
#define down_btn  14
#define d0_btn     2
#define d1_btn     4
#define d2_btn     5
#define d3_btn    18
#define d4_btn    19
#define d5_btn    15
#define d6_btn    23
#define d7_btn    26
#define mode_btn  25
#define step_btn  33
#define srst_btn  32
//for pico:
// #define left_btn  0
// #define right_btn 1
// #define up_btn    2
// #define down_btn  3
// #define d0_btn     6
// #define d1_btn     7
// #define d2_btn     8
// #define d3_btn    9
// #define d4_btn    10
// #define d5_btn    11
// #define d6_btn    12
// #define d7_btn    13
// #define mode_btn  14
// #define step_btn  15
// #define srst_btn  16





//--------mode vars--------//
//programming mode
bool inp[8];
int page = 0;
int pointer = 0;
bool enableRamUpdate = false;
//run mode
int val;
bool softReset;




//--------CPU Architecture--------//
uint8_t PC;              //program counter
uint8_t A_R, B_R, I_R;   //registers
uint8_t MAR;             //memory address register
uint8_t RAM[256];        //RAM
bool E, C, N, Z, V;      //flags
bool hlt;                //hlt signal
uint8_t OUTPUT_R;        //output reg




void setup() {
  //Serial init
  Serial.begin(115200);
  //pin modes:
  pinMode(left_btn,  INPUT_PULLUP);
  pinMode(right_btn, INPUT_PULLUP);
  pinMode(up_btn,    INPUT_PULLUP);
  pinMode(down_btn,  INPUT_PULLUP);
  pinMode(d0_btn,    INPUT_PULLUP);
  pinMode(d1_btn,    INPUT_PULLUP);
  pinMode(d2_btn,    INPUT_PULLUP);
  pinMode(d3_btn,    INPUT_PULLUP);
  pinMode(d4_btn,    INPUT_PULLUP);
  pinMode(d5_btn,    INPUT_PULLUP);
  pinMode(d6_btn,    INPUT_PULLUP);
  pinMode(d7_btn,    INPUT_PULLUP);
  pinMode(mode_btn,  INPUT_PULLUP);
  pinMode(step_btn,  INPUT_PULLUP);
  pinMode(srst_btn,  INPUT_PULLUP);
  //cpu reset sequence
  cpureset();
  //disp init
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  //display.begin(i2c_Address, true);
  if (digitalRead(mode_btn) == LOW) {
    printcpu();
  }
  //fibonacci();
  examples();
}

void loop() {
  //Serial.println("pico boo");
  //run mode
  if (digitalRead(mode_btn)==LOW) {
    printcpu();
    if (digitalRead(step_btn)==LOW) {
      fetch();
      execute();
      printcpu();
      //debug();
//      while(digitalRead(step_btn)==LOW){
//      }
      delay(150);
    }

    while(hlt && digitalRead(mode_btn)==LOW){
      if (digitalRead(srst_btn)==LOW){
        PC = 0;
        hlt = false;
        int timer = 0;
        while(digitalRead(srst_btn)==LOW){
          timer++;
          delay(200);
          if (timer>15) {
            cpureset();
            printcpu();
          }
        }
        delay(150);
      }
    }

    if (digitalRead(srst_btn)==LOW){
        PC = 0;
        hlt = false;
        int timer = 0;
        while(digitalRead(srst_btn)==LOW){
          timer++;
          delay(200);
          if (timer>15) {
            cpureset();
            printcpu();
          }
        }
        delay(150);
      }
  }

  //programming mode
  if (digitalRead(mode_btn)==HIGH) {
    programmingMode();
  }  

}


//pgm mode
void programmingMode(){
  navInput();  //get nav buttons input
  byte addrs = 8*page+pointer;  //ram addrs
  //inputs = ram value at that location
  for (int i=0; i<8; i++){
    inp[i] = bitRead(RAM[addrs], i);
  }
  
  dataInput(); //get data from buttons
  //convert inp array to byte
  byte dataInput = (inp[7]*pow(2,7))+(inp[6]*pow(2,6))+(inp[5]*pow(2,5))+(inp[4]*pow(2,4))+(inp[3]*pow(2,3))+(inp[2]*pow(2,2))+(inp[1]*pow(2,1))+(inp[0]*pow(2,0));
  
  //update ram data only if button pressed
  if (enableRamUpdate){
    RAM[addrs] = dataInput;
    enableRamUpdate = false;
  }
  // Serial.print("dataInput: ");
  // Serial.print(dataInput);
  // Serial.print(", RAM[addrs]: ");
  // Serial.println(RAM[addrs]);
  //display output code
  display.setCursor(0, 0);
  for (int i = 8*page; i < 8*page+8; i++) {
    if (i==(addrs)){
      display.setTextColor(BLACK, WHITE);
      //display.setTextColor(SH110X_BLACK, SH110X_WHITE);
    }
    else {
      display.setTextColor(WHITE);
      //display.setTextColor(SH110X_WHITE);
    }
    if (i < 16) {
      display.print("0x0");
    } else {
      display.print("0x");
    }
    display.print(i, HEX);
    display.print(": ");
    String data = "00000000" + String(RAM[i], BIN);
    display.print(data.substring(data.length()-8,data.length()-4));
    display.print("-");
    display.println(data.substring(data.length()-4,data.length()));
  }

  display.display();
  //delay(100);
  display.clearDisplay();
}
void navInput(){
  if (digitalRead(right_btn)==LOW){
    page++;
    pointer = 0;
    delay(150);
    resetInp();
  }
  if (digitalRead(left_btn)==LOW){
    page--;
    delay(150);
    resetInp();
  }
  if (page>=32){page = 0;}
  if (page<0){page = 31;}

  if (digitalRead(up_btn)==LOW){
    pointer--;
    delay(150);
    resetInp();
  }
  if (digitalRead(down_btn)==LOW){
    pointer++;
    delay(150);
    resetInp();
  }
  if (pointer>7){pointer = 0;}
  if (pointer<0){pointer = 7;}

}
void resetInp(){
  for (int i=0; i<7; i++){
    inp[i]=0;
  }
}
void dataInput() {
  if (digitalRead(d0_btn)==LOW){
    inp[0] = !inp[0];
    delay(150);
    enableRamUpdate = true;
  }
  else if (digitalRead(d1_btn)==LOW){
    inp[1] = !inp[1];
    delay(150);
    enableRamUpdate = true;
  }
  else if (digitalRead(d2_btn)==LOW){
    inp[2] = !inp[2];
    delay(150);
    enableRamUpdate = true;
  }
  else if (digitalRead(d3_btn)==LOW){
    inp[3] = !inp[3];
    delay(150);
    enableRamUpdate = true;
  }
  else if (digitalRead(d4_btn)==LOW){
    inp[4] = !inp[4];
    delay(150);
    enableRamUpdate = true;
  }
  else if (digitalRead(d5_btn)==LOW){
    inp[5] = !inp[5];
    delay(150);
    enableRamUpdate = true;
  }
  else if (digitalRead(d6_btn)==LOW){
    inp[6] = !inp[6];
    delay(150);
    enableRamUpdate = true;
  }
  else if (digitalRead(d7_btn)==LOW){
    inp[7] = !inp[7];
    delay(150);
    enableRamUpdate = true;
  }
  

}


//disp
void printcpu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  //display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("MAR: "); display.print(MAR, HEX);
  display.print("  ");
  display.print("PC: "); display.println(PC, HEX);
  display.println();

  display.print("RAM: "); display.print(RAM[MAR], HEX);
  display.print("  ");
  display.print("A:  "); display.println(A_R, HEX);
  display.println();

  display.print("IR:  "); display.print(I_R, HEX);
  display.print("  ");
  display.print("B:  "); display.println(B_R, HEX);
  display.println();

  display.print("FLG: ");
  display.print(E);
  display.print(C);
  display.print(N);
  display.print(Z);
  display.print(V);
  display.print("  ");
  display.print("OUT: ");
  display.println(OUTPUT_R);
  display.display();
}


//cpu
void cpureset() {
  PC = 0;
  MAR = 0;
  A_R = 0;
  B_R = 0;
  I_R = 0;
  N = 0;
  C = 0;
  Z = 0;
  OUTPUT_R = 0;
}

//-----fetch-----//
void fetch() {
  MAR = PC;
  I_R = RAM[MAR];
}

//-----execute-----//
void execute() {
  switch (I_R) {
    
    //No Operation
    case 0x00:
    break;
    
    //HLT
    case 0x01:
    hlt = true;
    break;
    
    //LDA imm
    case 0x02:
    PC++;
    MAR = PC;
    A_R = RAM[MAR];
    break;
    
    //LDA abs
    case 0x03:
    PC++;
    MAR = PC;
    MAR = RAM[MAR];
    A_R = RAM[MAR];
    break;
    
    //LDB imm
    case 0x04:
    PC++;
    MAR = PC;
    B_R = RAM[MAR];
    break;
    
    //LDB abs
    case 0x05:
    PC++;
    MAR = PC;
    MAR = RAM[MAR];
    B_R = RAM[MAR];
    break;
    
    //STA
    case 0x06:
    PC++;
    MAR = PC;
    MAR = RAM[MAR];
    RAM[MAR] = A_R;
    break;
    
    //STB
    case 0x07:
    PC++;
    MAR = PC;
    MAR = RAM[MAR];
    RAM[MAR] = B_R;
    break;
    
    //ADD imm
    case 0x08:
    PC++;
    MAR = PC;
    val = A_R + RAM[MAR];
    A_R = A_R + RAM[MAR];
    flagUpdate(val);
    break;
    
    //SUB imm
    case 0x09:
    PC++;
    MAR = PC;
    val = A_R - RAM[MAR];
    A_R = A_R - RAM[MAR];
    flagUpdate(val);
    break;
    
    //ADD B
    case 0x0a:
    val = A_R + B_R;
    A_R = A_R + B_R;
    flagUpdate(val);
    break;
    
    //SUB B
    case 0x0b:
    val = A_R - B_R;
    A_R = A_R - B_R;
    flagUpdate(val);
    break;
    
    //JMP imm
    case 0x0c:
    PC++;
    MAR = PC;
    PC = RAM[MAR]-1;
    break;
    
    //CMP
    case 0x0d:
    if (A_R == B_R) {
      E = 1;
    }
    else {
      E = 0;
    }
    if (A_R > B_R) {
      C = 1;
    }
    else {
      C = 0;
    }
    break;
    
    //BRE
    case 0x0e:
    if (E == 1) {
      PC++;
      MAR = PC;
      PC = RAM[MAR]-1;
    }
    else {
      PC++;
    }
    break;
    
    //BRG
    case 0x0f:
    if (C == 1) {
      PC++;
      MAR = PC;
      PC = RAM[MAR]-1;
    }
    else {
      PC++;
    }
    break;
    
    //BRN
    case 0x10:
    if (N == 1) {
      PC++;
      MAR = PC;
      PC = RAM[MAR]-1;
    }
    else {
      PC++;
    }
    break;
    
    //BRZ
    case 0x11:
    if (Z == 1) {
      PC++;
      MAR = PC;
      PC = RAM[MAR]-1;
    }
    else {
      PC++;
    }
    break;
    
    //BRV
    case 0x12:
    if (V == 1) {
      PC++;
      MAR = PC;
      PC = RAM[MAR]-1;
    }
    else {
      PC++;
    }
    break;

    //OUT
    case 0x13:
    OUTPUT_R = A_R;
    break;
    
    
    
      
  }
  PC++;
}

void flagUpdate(int value){
  if (value==0){
    Z = 1;
  }
  else {
    Z = 0;
  }
  if (value > 255){
    V = 1;
  }
  else {
    V = 0;
  }
  if (value < 0){
    N = 1;
  }
  else {
    N = 0;
  }
  
}


void debug(){
  Serial.print("A: "); Serial.println(A_R);
  Serial.print("B: "); Serial.println(B_R);
  Serial.print("NVZ: "); 
  Serial.print(N);
  Serial.print(V);
  Serial.println(Z);
  Serial.print("PC: "); Serial.println(PC);
  Serial.print("MAR: "); Serial.println(MAR);
  Serial.print("IR: "); Serial.println(I_R);
  Serial.print("RAM: "); Serial.println(RAM[MAR]);
  Serial.println();
}


void fibonacci() {
  RAM[0x00] = 0x03;  //LDA ABS
  RAM[0x01] = 0xFF;  //255
  RAM[0x02] = 0x08;  //ADD IMM
  RAM[0x03] = 0x01;  //1
  RAM[0x04] = 0x13;  //OUT
  RAM[0x05] = 0x05;  //LDB ABS
  RAM[0x06] = 0x03;  //3
  RAM[0x07] = 0x07;  //STB
  RAM[0x08] = 0xFF;  //255
  RAM[0x09] = 0x06;  //STA
  RAM[0x0A] = 0x03;  //3
  RAM[0x0B] = 0x04;  //LDB IMM
  RAM[0x0C] = 0xE9;  //233
  RAM[0x0D] = 0x0D;  //CMP
  RAM[0x0E] = 0x0E;  //BRE
  RAM[0x0F] = 0x12;  //12
  RAM[0x10] = 0x0C;  //JMP IMM
  RAM[0x11] = 0x00;  //0
  RAM[0x12] = 0x04;  //LDB IMM
  RAM[0x13] = 0x00;  //0
  RAM[0x14] = 0x07;  //STB
  RAM[0x15] = 0xFF;  //255
  RAM[0x16] = 0x04;  //LDB IMM
  RAM[0x17] = 0x01;  //1
  RAM[0x18] = 0x07;  //STB
  RAM[0x19] = 0x03;  //3
  RAM[0x1A] = 0x0C;  //JMP IMM
  RAM[0x1B] = 0x00;  //00
}

void examples() {
  RAM[0x00] = 0x0c;
  RAM[0x01] = 0x28;
  //Addition 0x08
  RAM[0x08] = 0x02; //LDA imm
  RAM[0x09] = 0xff; //255
  RAM[0x0a] = 0x08; //ADD imm
  RAM[0x0b] = 0x01; //1
  RAM[0x0c] = 0x13; //OUT
  //Multiplication 0x10
  RAM[0x10] = 0x03; //LDA abs
  RAM[0x11] = 0x27; //prod
  RAM[0x12] = 0x05; //LDB abs
  RAM[0x13] = 0x25; //X
  RAM[0x14] = 0x0a; //ADD B
  RAM[0x15] = 0x06; //STA
  RAM[0x16] = 0x27; //prod
  RAM[0x17] = 0x03; //LDA abs
  RAM[0x18] = 0x26; //Y
  RAM[0x19] = 0x09; //SUB imm
  RAM[0x1a] = 0x01; //0x01
  RAM[0x1b] = 0x06; //STA 
  RAM[0x1c] = 0x26; //Y
  RAM[0x1d] = 0x11; //BRZ
  RAM[0x1e] = 0x21; //0x21
  RAM[0x1f] = 0x0c; //JMP imm
  RAM[0x20] = 0x10; //0x10
  RAM[0x21] = 0x03; //LDA abs
  RAM[0x22] = 0x27; //prod
  RAM[0x23] = 0x13; //OUT
  RAM[0x24] = 0x00; 
  RAM[0x25] = 0x03; //X
  RAM[0x26] = 0x03; //Y
  RAM[0x27] = 0x00; //prod
  //Fibonacci 0x28
  RAM[0x28] = 0x03;  //LDA ABS
  RAM[0x29] = 0xFF;  //255
  RAM[0x2a] = 0x08;  //ADD IMM
  RAM[0x2b] = 0x01;  //1
  RAM[0x2c] = 0x13;  //OUT
  RAM[0x2d] = 0x05;  //LDB ABS
  RAM[0x2e] = 0x2b;  //2b
  RAM[0x2f] = 0x07;  //STB
  RAM[0x30] = 0xFF;  //255
  RAM[0x31] = 0x06;  //STA
  RAM[0x32] = 0x2b;  //2b
  RAM[0x33] = 0x04;  //LDB IMM
  RAM[0x34] = 0xE9;  //233
  RAM[0x35] = 0x0D;  //CMP
  RAM[0x36] = 0x0E;  //BRE
  RAM[0x37] = 0x3a;  //3a
  RAM[0x38] = 0x0C;  //JMP IMM
  RAM[0x39] = 0x28;  //28
  RAM[0x3a] = 0x04;  //LDB IMM
  RAM[0x3b] = 0x00;  //0
  RAM[0x3c] = 0x07;  //STB
  RAM[0x3d] = 0xFF;  //255
  RAM[0x3e] = 0x04;  //LDB IMM
  RAM[0x3f] = 0x01;  //1
  RAM[0x40] = 0x07;  //STB
  RAM[0x41] = 0x2b;  //2b
  RAM[0x42] = 0x0C;  //JMP IMM
  RAM[0x43] = 0x28;  //28
}
