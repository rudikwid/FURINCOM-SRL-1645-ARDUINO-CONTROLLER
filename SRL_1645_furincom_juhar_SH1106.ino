/*
 * 
 *  FURUNO, FURINCOM, RANGER SRL-1645 Controller
 *  by Juhar / Widi Gresik - YC3TKM 
 *  ADAPTASI KE OLED 1.3" SH1106, PLL CONTROL TO PLL MAPPING TEST SUCCESSFUL FOR 140-150MHz
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.

 - Encoder A         = 2;
 - Encoder B         = 3;
 - tombol_Dial.pin   = 4;
 - tombol_TX.pin     = 5;
 - tombol_MemVFO.pin = 6;
 - tombol_Pow.pin    = 7;

 * 
 */ 
 
#include <Rotary.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <Adafruit_SH1106.h>
#include<avr/wdt.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_SH1106 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

/**************************************************************************
*  Definitions
**************************************************************************/
#define dial_A          2       // Encoder pin A INT0/PCINT18 D2 aslinya 2
#define dial_B          3       // Encoder pin B INT1/PCINT19 D3 aslinya 3
#define switchPower     A1
#define smeter          A0
#define strobebit       13

#define freq_min        139000UL      // batas bawah
#define freq_max        155000UL      // batas atas
#define offset_min      -10000        // offset minimum
#define offset_max      10000
#define max_mem         30              //aslinya=69  
#define min_mem         1  

#define NAIK   1
#define TURUN -1
#define BENAR 1
#define TIDAK 0
#define SELESAI 0
#define modeLow 0
#define modeHigh 1
#define modeVFO 0
#define modeMEM 1
#define modeDupAdj 2

#define Ditekan LOW
#define Netral HIGH
#define tekanCepat 100
#define tekanLama 500

/**************************************************************************
*  EEPROM data locations 
**************************************************************************/
#define EE_SAVED_LASTMODE   0    // Memori, atau VFO
#define EE_SAVED_LASTCH     4    // nomor Memori

#define EE_SAVED_VFO            8    // Frekuensi kerja terakhir (VFO)
#define EE_SAVED_VFO_STEP       12   // dial step
#define EE_SAVED_VFO_POWER      16   // Setting Power
#define EE_SAVED_VFO_DUPE       20   // Offset duplex

typedef struct tombol {
    byte pin;
    const int debounce = 10;
    unsigned long hitungan=0;
    bool sts_lama = Netral;
    bool sts_kini;
} TombolTekan;

TombolTekan tom_Dial,tom_TX, tom_MemVFO, tom_Pow;

Adafruit_SH1106 Layar(1);

//Rotary r = Rotary(2, 3);

Rotary DIAL = Rotary(dial_A, dial_B);

/*  mapping pin arduino ke pllnya (ke socket DIP ex EPROM), RUDIKWID YD0AYA, 13-12-22)
DATA PLL -> ARDUINO -> KAKI EPROM
D0=2=11  //ARDUINO D8  -> EPROM 11
D1=1=12  //ARDUINO D9  -> EPROM 12
D2=20=13 //ARDUINO D10 -> EPROM 13
D3=19=15 //ARDUINO D11 -> EPROM 15
A2=11=8  //ARDUINO D12 -> EPROM 8
A1=10=9  //ARDUINO A3  -> EPROM 9
A0=9=10  //ARDUINO A2  -> EPROM 10
ST=STROBE=ARDUINO PIN 13 LANGSUNG KE PIN 12 MC145146
 */
int dataPort[] = {8,9,10,11,12,A3,A2,13}; //D0-D1-D2-D3-A2-A1-A0-ST

const byte Data5 = B01010000, Data6 = B01101100, Data7 = B01110011;
byte Data4,Data3,Data2,Data1,Data0;
byte data_PLL;
  
word stepDial = 100L, stepSemula = 100L, alamat_Memori;                 
byte nomor_Memori = 1;
boolean pake_Duplex = TIDAK, last_Mode = modeVFO, power_mode = modeLow;
boolean freq_berubah = TIDAK, ubah_step = TIDAK, ubah_offset = TIDAK, ubah_TX = TIDAK, ubah_Power = TIDAK, ada_perubahan = TIDAK, sedang_TX = TIDAK;
int mode_operasi;


volatile uint32_t freqKerja = 145000UL; // frekuensi 145.000 Mhz
volatile int offsetTX;

/*
  Setting interrupt handler, baca rotary dial
 */
ISR(PCINT2_vect) {
  char result = DIAL.process();
  if (result == DIR_CW)
    set_frequency(NAIK);
  else if (result == DIR_CCW)
    set_frequency(TURUN);
}

/*
  Digunakan untuk menangani interrupt akibat gerakan rotary dial
 */
void set_frequency(short arah)
{ 
  switch (mode_operasi)
  { 
    case modeVFO:              // Tx frequency
      if (!sedang_TX) {
          if (arah == NAIK)  freqKerja += stepDial;
          if (arah == TURUN) freqKerja -= stepDial;
          if(freqKerja > freq_max)  freqKerja = freq_min;
          if(freqKerja < freq_min) freqKerja = freq_max;
          ada_perubahan = BENAR;
      }
      break;
    case modeMEM:
      if (!sedang_TX) {
          if (arah == NAIK)  nomor_Memori++;
          if (arah == TURUN) nomor_Memori--;
          if(nomor_Memori > max_mem) nomor_Memori = min_mem;
          if(nomor_Memori < min_mem) nomor_Memori = max_mem;
          ada_perubahan = BENAR;
      }
     break;
    case modeDupAdj:             // Offset TX
      if (arah == NAIK) offsetTX += stepDial;
      if (arah == TURUN)  offsetTX -= stepDial;
      if (freqKerja+offsetTX < freq_min) offsetTX = freq_min-freqKerja;
      if (freqKerja+offsetTX > freq_max) offsetTX = freq_max-freqKerja;    
      break;
  }
  freq_berubah = BENAR;
  
}



void simpan_Status() {
 
  if (mode_operasi == modeMEM) {
     eeprom_update_dword((uint32_t *)EE_SAVED_LASTMODE,modeMEM); 
     eeprom_update_dword((uint32_t *)EE_SAVED_LASTCH,nomor_Memori); 
     // simpan data power
     alamat_Memori = (nomor_Memori * 12) + 16;
     eeprom_update_dword((uint32_t *)alamat_Memori,power_mode);  
     // simpan data offset 
     alamat_Memori = (nomor_Memori * 12) + 20;
     eeprom_update_dword((uint32_t *)alamat_Memori,offsetTX); 
  }

  if (mode_operasi == modeVFO) {
    eeprom_update_dword((uint32_t *)EE_SAVED_LASTMODE,modeVFO); 
    eeprom_update_dword((uint32_t *)EE_SAVED_VFO,freqKerja);   
    eeprom_update_dword((uint32_t *)EE_SAVED_VFO_STEP,stepDial);   
    eeprom_update_dword((uint32_t *)EE_SAVED_VFO_POWER,power_mode);  
    eeprom_update_dword((uint32_t *)EE_SAVED_VFO_DUPE,offsetTX);  
  }
 
}

/*
 * Digunakan untuk mbaca data vfo sesuai kondisi terakhir
 */
void baca_dataVFO() {
      mode_operasi = modeVFO;
      // baca data step
      stepDial = eeprom_read_dword((const uint32_t *)EE_SAVED_VFO_STEP);
      if ((stepDial < 5UL) | (stepDial > 1000UL)) stepDial = 10UL;  
      
      // ambil data freq kerja terakhir 
      freqKerja = eeprom_read_dword((const uint32_t *)EE_SAVED_VFO);
      if ((freqKerja < freq_min) | (freqKerja > freq_max)) freqKerja = 145000UL;
      
      // mode power
      power_mode = eeprom_read_dword((const uint32_t *)EE_SAVED_VFO_POWER);    
      if (power_mode != modeHigh) power_mode = modeLow;
      
      // ambil data offset 
      offsetTX = eeprom_read_dword((const uint32_t *)EE_SAVED_VFO_DUPE);
      
      // Serial.println(offsetTX);
      if ((offsetTX < offset_min) | (offsetTX > offset_max)) offsetTX = 0;
      if (offsetTX == 0) pake_Duplex = TIDAK; else pake_Duplex = BENAR;
      
      // simpan status terakhir
      eeprom_update_dword((uint32_t *)EE_SAVED_LASTMODE,modeVFO); 
}

/*
 * Digunakan untuk mbaca data memori sesuai nomor memori yang dipilih
 */
void baca_dataMEM() {
      mode_operasi = modeMEM;  
      if (nomor_Memori > max_mem) nomor_Memori = 1;
      
      // start dari lokasi 24,28,32 next 36,40,44 next 48,52,56
      // ambil data freq kerja
      alamat_Memori = (nomor_Memori * 12) + 12;
      freqKerja = eeprom_read_dword((const uint32_t *)alamat_Memori); /// check
      if ((freqKerja < freq_min) | (freqKerja > freq_max)) freqKerja = 145000UL;

       // ambil data power
      alamat_Memori = (nomor_Memori * 12) + 16;
      power_mode = eeprom_read_dword((const uint32_t *)alamat_Memori);    
      if (power_mode != modeHigh) power_mode = modeLow;
      
      // ambil data offset 
      alamat_Memori = (nomor_Memori * 12) + 20;
      offsetTX = eeprom_read_dword((const uint32_t *)alamat_Memori);    
      if ((offsetTX < offset_min) | (offsetTX > offset_max)) offsetTX = 0;
      if (offsetTX == 0) pake_Duplex = TIDAK; else pake_Duplex = BENAR;
      
      //simpan status terakhir    
      eeprom_update_dword((uint32_t *)EE_SAVED_LASTMODE,modeMEM); 
      eeprom_update_dword((uint32_t *)EE_SAVED_LASTCH,nomor_Memori); 
}

/*
 * Digunakan untuk simpan frekuensi, power level dan offset ke memori
 */
void simpan_memori(){
       // simpan data freq kerja
      alamat_Memori = (nomor_Memori * 12) + 12;
      eeprom_update_dword((uint32_t *)alamat_Memori,freqKerja);   
 
      // simpan data power
      alamat_Memori = (nomor_Memori * 12) + 16;
      eeprom_update_dword((uint32_t *)alamat_Memori,power_mode);  
       
      // simpan data offset 
      alamat_Memori = (nomor_Memori * 12) + 20;
      eeprom_update_dword((uint32_t *)alamat_Memori,offsetTX); 
 
}

/*
 * Fungsi reverse monitoring
 */
void reverseFreq() {
   if (offsetTX != 0)
   {
      freqKerja = freqKerja + offsetTX;
      offsetTX = -offsetTX;
      update_dataPLL(freqKerja); 
   }
}

/**************************************************/
/* Baca tombol dial (step dan offset adjust)      */
/**************************************************/
boolean tombol_ditekan()
{ boolean ubah_Mode = TIDAK;

   ubah_TX = TIDAK;
   tom_TX.sts_kini = digitalRead(tom_TX.pin);
   if (tom_TX.sts_kini != tom_TX.sts_lama) {
      delay(tom_TX.debounce);
   tom_TX.sts_kini = digitalRead(tom_TX.pin);
      if (tom_TX.sts_kini == Ditekan) sedang_TX = BENAR;
        else sedang_TX = TIDAK;
   tom_TX.sts_lama = tom_TX.sts_kini;
      ubah_TX = BENAR;
  }

   tom_Pow.sts_kini = digitalRead(tom_Pow.pin);
   if (tom_Pow.sts_kini != tom_Pow.sts_lama) {
      delay(tom_Pow.debounce);
      tom_Pow.sts_kini = digitalRead(tom_Pow.pin);
    //  if (tom_Pow.sts_kini == Ditekan) ubah_Power = BENAR;
    //  tom_Pow.sts_lama = tom_Pow.sts_kini;
     if (tom_Pow.sts_kini == Ditekan) tom_Pow.hitungan = millis();
       
       // tombol sudah dilepas
   if (tom_Pow.sts_kini == Netral) {
        unsigned long millis_skrg = millis();
   if ((millis_skrg - tom_Pow.hitungan >= tekanCepat) && !(millis_skrg - tom_Pow.hitungan >= tekanLama)) {
                
   // tombol ditekan sebentar 
      ubah_Power = BENAR;
      }
     if ((millis_skrg - tom_Pow.hitungan >= tekanLama)) {
     
     // tombol ditekan lama
               reverseFreq(); 
            }       
      }
      tom_Pow.sts_lama = tom_Pow.sts_kini;
   }
   
   // cek tombol push on di dial
   tom_Dial.sts_kini = digitalRead(tom_Dial.pin);
   
   // jika tombol ditekan
   if (tom_Dial.sts_kini != tom_Dial.sts_lama) {
      delay(tom_Dial.debounce);
      tom_Dial.sts_kini = digitalRead(tom_Dial.pin);
   if (tom_Dial.sts_kini == Ditekan) tom_Dial.hitungan = millis();
       
   // tombol sudah dilepas
   if (tom_Dial.sts_kini == Netral) {
   unsigned long millis_skrg = millis();
   if ((millis_skrg - tom_Dial.hitungan >= tekanCepat) && !(millis_skrg - tom_Dial.hitungan >= tekanLama)) {
                
   // tombol ditekan sebentar 
   if (!sedang_TX) ubah_step = BENAR; 
       }
   if ((millis_skrg - tom_Dial.hitungan >= tekanLama)) {
                
   // tombol ditekan lama
   if (!sedang_TX) ubah_offset = BENAR; 
        }       
      }
      tom_Dial.sts_lama = tom_Dial.sts_kini;
   }

   // cek tombol VFO/MEM
   tom_MemVFO.sts_kini = digitalRead(tom_MemVFO.pin);
   
   // jika tombol ditekan
   if (tom_MemVFO.sts_kini != tom_MemVFO.sts_lama) {
      delay(tom_MemVFO.debounce);

      tom_MemVFO.sts_kini = digitalRead(tom_MemVFO.pin);
      tom_MemVFO.sts_lama = tom_MemVFO.sts_kini;
      ubah_Mode = BENAR;
      
      if (tom_MemVFO.sts_kini == Ditekan) tom_MemVFO.hitungan = millis();
      
   // tombol sudah dilepas
      if (tom_MemVFO.sts_kini == Netral) {
        unsigned long millis_skrg = millis();
            if ((millis_skrg - tom_MemVFO.hitungan >= tekanCepat) && !(millis_skrg - tom_MemVFO.hitungan >= tekanLama)) {
                
                // tombol ditekan sebentar 
                if (!sedang_TX) {
                  if (mode_operasi == modeVFO) {
                       
                       // simpan data VFO terakhir
                       eeprom_update_dword((uint32_t *)EE_SAVED_VFO, freqKerja); 
                       
                       // pindah ke memori
                       baca_dataMEM();
                         }
                       else 
                    {
                      // dari mode MEM, balik ke mode VFO
                      baca_dataVFO();
                    }
                   update_dataPLL(freqKerja); 
                  }      
                }
            if ((millis_skrg - tom_MemVFO.hitungan >= tekanLama)) {
                
                // tombol ditekan lama
                if (!sedang_TX) simpan_memori(); 
            }       
       }
   }
   
  if (ubah_step | ubah_offset | ubah_TX | ubah_Mode | ubah_Power) return BENAR;
  else return SELESAI;
}

/*
  Menampilkan frekuensi di layar (bagian tengah)
*/
void display_frequency()
{
  display.setTextSize(2);  
  char Angka_layar[12]; 
  char Mhz[5], Herz[12];
  int panjang,n_Mhz;
  long freq;

  display.clearDisplay();
  // pilih angka yang akan ditampilkan
  if (mode_operasi == modeVFO || mode_operasi == modeMEM)
  {
        if (sedang_TX) {
          if (pake_Duplex) freq = freqKerja + offsetTX;
          else freq = freqKerja;
        }
        else freq = freqKerja;
        
        // penampung kilohertz sampai hertz / belakang koma  
        Herz[1]='\0';                           
        
        // konversikan ke array
        sprintf(Angka_layar, "%ld", freq); 
        
        // hitung panjang tampilan         
        panjang=strlen(Angka_layar);                       
        
        // ambil bagian Megahertz dari angka layar, array mulai dari 0
        strncpy(Mhz,Angka_layar,(panjang-3)); 
        
        // Mhz berisi angka Megahertz             
        n_Mhz=panjang-3;
        
        //tutup bagian Mhz dengan null character
        Mhz[n_Mhz]='\0';                         
        strcpy(Herz,Angka_layar);
        strcpy(Angka_layar+n_Mhz,Herz+(n_Mhz-1));       
        Angka_layar[n_Mhz]='.';                         
        
        display.setCursor(4,20);
        display.print(Angka_layar);
        display.setCursor(97,20);
        if (nomor_Memori < 10) display.print("0");
        display.print(nomor_Memori);
       
  }
  else 
  {
    display.setCursor(40,20);
    display.println(offsetTX);     
  }
  display_settings();
}

/*
   Menampilkan status dan s-meter di layar
*/
void display_settings()
{

   unsigned int sample;
                                                          
   sample = analogRead(smeter);         // baca sinyal input untuk s meter
              
   float db = map(sample,400,700,0,511);         
   
   // garis vertikal skala
   for(int x =5;x<120;x=x+17){                             
      display.drawLine(x, 54, x, 51, WHITE);
   }
  
   // tulisan angka s-meter
   display.setCursor(1,57);
   display.setTextSize(1);
   display.println("S1 3  5  9 10 20 30+");
  
   display.drawRect(0, 40, 126, 9, WHITE);        // bingkai bargraph
   int r = map(db,50,550,5,126);                  // pemetaan panjang bargraph dari nilai smeter
   display.fillRect(1, 41, r, 8, WHITE);          // gambar bargraph
   
  // Kotak Frekwensi
  display.drawRect(1, 16, 125, 22, WHITE);
  if (mode_operasi != modeDupAdj) display.drawLine(92, 16, 92, 37, WHITE);
  
  // Kotak Kuning
  display.drawRect(1, 0, 125, 15, WHITE);     
  
  // Stepsize, pojok kiri atas  
  display.setCursor(1, 4 );  
  display.setTextSize(1);  
  switch (stepDial)
  {
    case 5:
       display.println(F("   5K"));
       break;
    case 10:
      display.println(F("  10K"));
      break;
    case 100:
      display.println(F(" 100K"));
      break;
    case 1000:
      display.println(F("   1M"));
      break;
   }

  display.setCursor(38, 4);
  display.setTextSize(1);  
  
  switch (mode_operasi)
  {
    case modeVFO:
      display.println(F("VFO"));
      break;
    case modeMEM:
      display.print(F("MEM "));
      // diikuti nomor memori
      break;
    case modeDupAdj:
      display.println(F("OFFSET RIPITER"));
      break;
  } 

  display.setCursor(77, 4);
  display.setTextSize(1);
  if (mode_operasi == modeDupAdj) {
     display.println("              ");
  }
  else
  {
     display.setCursor(67, 4);
     if (power_mode == modeHigh) display.println("Hi"); else display.println("Lo");
     display.setCursor(87, 4);
     if (pake_Duplex) display.println(offsetTX); else display.println("direct");
  }
  display.display();
}

/*
  Digunakan untuk rubah langkah naik turun dial, terkecil 5 Khz, terbesar 1 MHz
*/
void ganti_langkah(){
switch (stepDial) {
        case 5:
          stepDial = 10;
        break;
        case 10:
          stepDial = 100;
        break;
        case 100:
          stepDial = 1000;
        break;
        case 1000:
          stepDial = 5;
        break;   
      }
      ubah_step  = SELESAI;
      // simpan data step ke avr
      if (mode_operasi == modeVFO) {
         eeprom_update_dword((uint32_t *)EE_SAVED_VFO_STEP, stepDial); 
         eeprom_update_dword((uint32_t *)EE_SAVED_VFO, freqKerja);
      } 
}

/*
  Digunakan untuk setting kondisi dan pindah ke mode adjust VFO
 */
void adjustDuplex() {
       // jika mode sekarang = mode trx, pindah ke mode set duplex,
       // jika mode sekarang = mode set duplex, maka balik ke mode normal, simpan duplex yg baru
        if (mode_operasi == modeVFO){
           mode_operasi = modeDupAdj; 
           stepSemula = stepDial;
           stepDial = 100;
        }
       else { // selesai update duplexO
         mode_operasi = modeVFO;  
         stepDial = stepSemula; 
         eeprom_update_dword((uint32_t *)EE_SAVED_VFO_DUPE, offsetTX);
         if (offsetTX == 0) pake_Duplex = TIDAK; else pake_Duplex = BENAR;
       }
       ubah_offset = SELESAI; 
}
/*
 * Digunakan untuk toggle VR power, mengatur pancaran High atau Low, default Low
*/
void gantiPower(){
    if (power_mode == modeLow){
         power_mode = modeHigh;
         digitalWrite(switchPower, HIGH);
    }
    else {
           power_mode = modeLow;
           digitalWrite(switchPower,LOW);
    }
     ubah_Power = SELESAI;      
}

/*
  Kirim data ke IC PLL
*/
void kirim_dataPLL(byte datanya) {
// lama waktu data = 0.3ms, lama waktu latch = 0.1ms

  for (byte packet = 0;packet<8;packet++){
      if (bitRead(datanya,packet)) digitalWrite(dataPort[packet],HIGH); 
      else digitalWrite(dataPort[packet],LOW);
}

// strobe high
digitalWrite(strobebit,HIGH);
delayMicroseconds(100);
digitalWrite(strobebit,LOW);
delayMicroseconds(50); 
}


/*
  Formula perhitungan data yang akan dikirim ke PLL
*/
void update_dataPLL(float temp) {
 // frekuensi untuk rx = frek kerja - 21.600 sedangkan tx =  frek kerja 
 // cara kirim perintah ke IC PLL MC145146
 // 8 byte dalam 8 x sequence kirim
 // perintah kirim low bit A     D0
 // Seq0 = &B00000000
 // perintah kirim high bit A    D1
 // Seq1 = &B00010000
 // perintah kirim lo bit N      D2
 // Seq2 = &B00100000
 // perintah kirim mid bit N     D3
 // Seq3 = &B00110000
 // perintah kirim high bit N    D4
 // Seq4 = &B01000000
 // perintah kirim ref data 5 khz (0 = 0000,C = 1100, 3 = 0011)
 // Seq5 = &B01010000            D5 ->  0
 // Seq6 = &B01101100            D6 -> 12
 // Seq7 = &B01110011            D7 ->  3

 // frekuensi PLL saat RX 
 // selisih 21600 (IF-nya);
 if (!sedang_TX) {
  temp = temp - 21600;
 }
 else {
  // transmit ? cek duplex apa enggak, pk duplex, hitung dengan offset
  if (pake_Duplex) temp = temp + offsetTX;
 }

 float desimal =0.0;
 
 // bagi dengan kelipatan 5Khz, 5 x 1024, ambil bagian integernya sebagai D4
 desimal = temp / 51200.00;
 Data4 = desimal;

// bagian pecahan dari D4 dikalikan 16, ambil integernya sebagai D3

 desimal = (desimal-Data4) * 16.0;

 Data3 = desimal;

 // bagian pecahan dari D3, kalikan 16, ambil integernya sebagai D2
 desimal = (desimal - Data3) * 16.0;

 Data2 = desimal;

 // bagian pecahan dari D2, kalikan 2.5, ambil integernya sebagai D1
 desimal = (desimal - Data2) * 2.5;

 if (desimal <= 1.94)
   Data1 = desimal;
  else Data1 = round(desimal);

 // bagian pecahan dari D1 , kalikan 16 dan bulatkan
 desimal = (desimal - Data1) * 16.0;
 Data0 = round(desimal);

 // kirim D0 s/d D7 
 data_PLL = Data0 | B00000000;
 kirim_dataPLL(data_PLL);
 data_PLL = Data1 | B00010000;
 kirim_dataPLL(data_PLL);
 data_PLL = Data2 | B00100000;
 kirim_dataPLL(data_PLL);
 data_PLL = Data3 | B00110000;
 kirim_dataPLL(data_PLL);
 data_PLL = Data4 | B01000000;
 kirim_dataPLL(data_PLL);
 data_PLL = Data5;
 kirim_dataPLL(data_PLL);
 data_PLL = Data6;
 kirim_dataPLL(data_PLL);
 data_PLL = Data7;
 kirim_dataPLL(data_PLL);
 
 for (int posisipin=0;posisipin<8;posisipin++) {
    digitalWrite(dataPort[posisipin],LOW);
  };
}

/**************************************/
/*            S E T U P               */
/**************************************/
void setup()
{
 // Serial.begin(115200);
  Wire.begin();
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
 
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds


  // Clear the buffer
  display.clearDisplay();
  //display.display();    //batas inisiasi SSD1306
  
  // posisi terakhir
  last_Mode = eeprom_read_dword((const uint32_t *)EE_SAVED_LASTMODE);
  // nomor memori terakhir dipakai
  nomor_Memori =  byte(eeprom_read_dword((const uint32_t *)EE_SAVED_LASTCH));

  if (last_Mode == modeVFO)  baca_dataVFO(); else baca_dataMEM();

       
  // data port setup
  for (int posisipin=0;posisipin<8;posisipin++) {
    pinMode(dataPort[posisipin],OUTPUT);
    digitalWrite(dataPort[posisipin],LOW);
  };
  // set up pll nya
  update_dataPLL(freqKerja);
  
  // Clear the buffer.
  display.clearDisplay();
  display.display();
  
  // text display tests
  display.setTextColor(WHITE);
  display.setCursor(0,5);
  display.setTextSize(1);
  display.println("Furincom Control v2.0");
  display.println("");
  display.println(" by Juhar / YC 3 TKM");
  display.setCursor(10,40);
  display.setTextSize(2);
  display.println(" SRL.1645");
  display.display();
  delay(1500);
  
  // Encoder setup
  tom_Dial.pin   = 4;
  tom_TX.pin     = 5;
  tom_MemVFO.pin = 6;
  tom_Pow.pin    = 7;

  //pinMode(dial_A, INPUT_PULLUP); // for encoder turn
  //pinMode(dial_B, INPUT_PULLUP); // for encoder turn 2
  pinMode(tom_Dial.pin, INPUT_PULLUP);
  pinMode(tom_TX.pin, INPUT_PULLUP);
  pinMode(tom_MemVFO.pin, INPUT_PULLUP);
  pinMode(tom_Pow.pin, INPUT_PULLUP);
  pinMode(switchPower, OUTPUT);
  if (power_mode == modeHigh) digitalWrite(switchPower, HIGH); else digitalWrite(switchPower,LOW);
 
  PCICR |= (1 << PCIE2);                     // Enable pin change interrupt for the encoder
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();

  // Display first time  
  display_frequency();  // Update the display   
  wdt_enable(WDTO_2S);
}

/**************************************/
/*             L O O P                */
/**************************************/
void loop(){
  // kalo ngehang, reset sendiri, watchdog timer
  wdt_reset();
  
  // jika terjadi perubahan frequency
  if (freq_berubah)  {
    if (mode_operasi == modeMEM) baca_dataMEM();
    update_dataPLL(freqKerja); 
    display_frequency();
    freq_berubah = SELESAI;
  }
  
  // cek tombol yang ditekan
  if (tombol_ditekan()) {
    // tombol dial ditekan sebentar
    if (ubah_TX) update_dataPLL(freqKerja);
    else
    if (ubah_step) ganti_langkah();
    else
    if (ubah_offset) adjustDuplex();
    else
    if (ubah_Power) gantiPower();
  }

  
  if ( (millis() % 30000 == 0) && (ada_perubahan)) {
    simpan_Status();
    ada_perubahan = TIDAK;
  }
  display_frequency();                             
}

//eof -- end of file --- commisioned by rudik wid, 14 Des 2022
