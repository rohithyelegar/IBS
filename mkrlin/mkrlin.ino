#define txPin 14
#define LIN_BREAK_DURATION    18    // Number of bits in the break.
#define LIN_TIMEOUT_IN_FRAMES 2

  int     serialSpd;           //  in bits/sec. Also called baud rate
  uint8_t serialOn;            //  whether the serial port is "begin"ed or "end"ed.  Optimization so we don't begin twice.
  unsigned long int timeout;
    uint8_t data1[] = { 0, 0, 0 };
    uint8_t data2[] = { 0, 0, 0 };
  uint8_t  bytereceived= 0;
  uint8_t  bytereceived1= 0;

void setup() {
  // put your setup code here, to run once:
 begint(19200);
 Serial.begin(9600);

 pinMode(0, OUTPUT);  
digitalWrite(0, HIGH); 
}
unsigned long t = 0;

void delay_until(unsigned long ms) {
  unsigned long end = t + (1000 * ms);
  unsigned long d = end - micros();

  // crazy long delay; probably negative wrap-around
  // just return
  if ( d > 1000000 ) {
    t = micros();
    return;
  }
  
  if (d > 15000) {
    unsigned long d2 = (d-15000)/1000;
    delay(d2);
    d = end - micros();
  }
  delayMicroseconds(d);
  t = end;
  
}


void loop() {
bytereceived= 0;
bytereceived1= 0;
  //serialBreak();
//  Serial1.write(0x55);
 // delay(100);
//  sendt(0x22, 0, 0, 2);
//  delay(20);
//  sendt(0x25,0,0,2);
//  delay(20);
//  sendt(0x26,0,0,2);
//  bytereceived = recv(0x22,data1,6,2);
//  delay(20);
//  for(int i =0;i<7;i++)
//  Serial.println(data1[i]);
//  Serial.print("number of bytes received");
//  Serial.println(bytereceived);
  delay(20);
    bytereceived1 = recv(0x25,data2,6,2);
  delay(20);
  for(int i =0;i<7;i++)
  Serial.println(data2[i]);
  Serial.print("number of bytes received");
  Serial.println(bytereceived1);
  /*sendt(0x22, 0, 0, 2);
  delay(10);
  sendt(0x25,0, 0, 2);
    delay(10);
  sendt(0x26,0, 0, 2);
  delay(10);
*/
}

void begint(int speed) 
{
  serialSpd = speed;
  Serial1.begin(serialSpd);
  serialOn  = 1;

  unsigned long int Tbit = 100000/serialSpd;  // Not quite in uSec, I'm saving an extra 10 to change a 1.4 (40%) to 14 below...
  unsigned long int nominalFrameTime = ((34*Tbit)+90*Tbit);  // 90 = 10*max # payload bytes + checksum (9). 
  timeout = LIN_TIMEOUT_IN_FRAMES * 14 * nominalFrameTime;  // 14 is the specced addtl 40% space above normal*10 -- the extra 10 is just pulled out of the 1000000 needed to convert to uSec (so that there are no decimal #s).
}

void serialBreak(void)
{
  if (serialOn) Serial1.end();

  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, LOW);  // Send BREAK
  unsigned long int brkend = (1000000UL/((unsigned long int)serialSpd));
  unsigned long int brkbegin = brkend*LIN_BREAK_DURATION;
  if (brkbegin > 16383) delay(brkbegin/1000);  // delayMicroseconds unreliable above 16383 see arduino man pages
  else delayMicroseconds(brkbegin);
  
  digitalWrite(txPin, HIGH);  // BREAK delimiter
 
  if (brkend > 16383) delay(brkend/1000);  // delayMicroseconds unreliable above 16383 see arduino man pages
  else delayMicroseconds(brkend);

  Serial1.begin(serialSpd);
  serialOn = 1;
}
  
uint8_t dataChecksum(const uint8_t* message, char nBytes,uint16_t sum)
{
    while (nBytes-- > 0) sum += *(message++);
    // Add the carry
    while(sum>>8)  // In case adding the carry causes another carry
      sum = (sum&255)+(sum>>8); 
    return (~sum);
}
#define BIT(data,shift) ((addr&(1<<shift))>>shift)
uint8_t addrParity(uint8_t addr)
{
  uint8_t p0 = BIT(addr,0) ^ BIT(addr,1) ^ BIT(addr,2) ^ BIT(addr,4);
  uint8_t p1 = ~(BIT(addr,1) ^ BIT(addr,3) ^ BIT(addr,4) ^ BIT(addr,5));
  return (p0 | (p1<<1))<<6;
}
void sendt(uint8_t addr, const uint8_t* message, uint8_t nBytes,uint8_t proto)
{
  uint8_t addrbyte = (addr&0x3f) | addrParity(addr);
  uint8_t cksum = dataChecksum(message,nBytes,(proto==1) ? 0:addrbyte);
  serialBreak();       // Generate the low signal that exceeds 1 char.
  //delayMicroseconds(22);
  Serial1.write(0x55);  // Sync byte
  Serial1.write(addrbyte);  // ID byte
  // Serial1.write(message,nBytes);  // data bytes
  //Serial1.write(cksum);  // checksum  
 // Serial.write(cksum);
  Serial1.flush();
}

uint8_t recv(uint8_t addr, uint8_t* message, uint8_t nBytes,uint8_t proto)
{
  uint8_t bytesRcvd=0;
  unsigned int timeoutCount=0;
  serialBreak();       // Generate the low signal that exceeds 1 char.
  Serial1.flush();
  Serial1.write(0x55);  // Sync byte
  uint8_t idByte = (addr&0x3f) | addrParity(addr);
  //p("ID byte %d", idByte);
  Serial1.write(idByte);  // ID byte
  //pinMode(txPin, INPUT);
  //digitalWrite(txPin, LOW);  // don't pull up
  do { // I hear myself
    while(!Serial1.available()) { delayMicroseconds(100); timeoutCount+= 100; if (timeoutCount>=timeout) goto done; }
  } while(Serial1.read() != 0x55);
  do {
    while(!Serial1.available()) { delayMicroseconds(100); timeoutCount+= 100; if (timeoutCount>=timeout) goto done; }
  } while(Serial1.read() != idByte);


  for (uint8_t i=0;i<nBytes;i++)
    {
      // This while loop strategy does not take into account the added time for the logic.  So the actual timeout will be slightly longer then written here.
      while(!Serial1.available()) { delayMicroseconds(100); timeoutCount+= 100; if (timeoutCount>=timeout) goto done; } 
      message[i] = Serial1.read();
      bytesRcvd++;
    }
  while(!Serial1.available()) { delayMicroseconds(100); timeoutCount+= 100; if (timeoutCount>=timeout) goto done; }
  if (Serial1.available())
    {
    uint8_t cksum = Serial1.read();
    bytesRcvd++;
    if (proto==1) idByte = 0;  // Don't cksum the ID byte in LIN 1.x
    if (dataChecksum(message,nBytes,idByte) == cksum) bytesRcvd = 0xff;
    //p("cksum byte %x, calculated %x %x\n",cksum,dataChecksum(message,nBytes,idByte),dataChecksum(message,nBytes,0));
    }

done:
  Serial1.flush();
  

  return bytesRcvd;
}
