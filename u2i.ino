// Author: Karl Rinne, karl.rinne@smartpaddy.ie
// (c) by kr64 Ltd. Proprietary. Confidential. Do not read or distribute unless authorized.

// History:
// 03/01/2014	v0.7	add salrt/ARA capabilities
// 02/01/2014	v0.6	add command t, statistical read
// 01/01/2014	v0.51	resolve issue 01: undesirable two-line read response if comms failure ("[]\nError: comms\n")
// 30/12/2013	v0.5	add group capability to write command
// 25/12/2013	v0.4	read/write status ('z')
// 24/12/2013	v0.3	set address range
// 20/12/2013	v0.2	I2C address scan
//			I2C read, write (non-grouped)
// 13/12/2013	v0.1	File created

// Module name:		u2i
// Module source:	u2i.ino
// Project:		generic
// Target Devices:	Arduino Micro, in a prototype board
// Tool versions used:	Arduino 1.5.5
// Tool OS:		Ubuntu 13.10

// Description:
// Gateway USB/serial <-> I2C, SMBus

// Commands:
// see help command

// includes
#include <Wire.h>
#include <string.h>
#include <ctype.h>

// the following define is the most memory-efficient way to store the version string (in flash, ensured when SW_VERSION is instantiated)
#define SW_VERSION "u2i v0.7 20140103 (c) kr64.com"

// ***********************************************************************************************
// u2i i/o definitions
// ***********************************************************************************************
const int led_red = 8;		// ATMEGA32U4 PB4 Arduino-micro-mapped to IO8
const int led_yellow = 9;	// ATMEGA32U4 PB5 Arduino-micro-mapped to IO9 (also PWM16b)
const int led_green = 10;	// ATMEGA32U4 PB6 Arduino-micro-mapped to IO10 (also PWM16b)

// interrupt-capable salrt
const int salrt = 0;		// ATMEGA32U4 INT2/PD2 Arduino-micro-mapped to D0/RX
				// ATMEGA32U4 INT3/PD3 Arduino-micro-mapped to D1/TX


// ***********************************************************************************************
// global defines and registers
// ***********************************************************************************************

int bus_speed=100;
int status=0;
  const int STATUS_READY=1, STATUS_BUSY=2, STATUS_ERROR=4;
const int MAX_SLAVES=24;
byte saddr[MAX_SLAVES];
byte nof_slaves;
const int MAX_RXTXBUFFER=8;
byte rxtxbuffer[MAX_RXTXBUFFER];

// command line and tokens 
const int MAX_TOKENS=MAX_SLAVES+1;
const int MAX_CHAR_PER_TOKENS=8;
const int COMMANDLINE_MAX_LENGTH=MAX_TOKENS*MAX_CHAR_PER_TOKENS;
char commandline[COMMANDLINE_MAX_LENGTH]="";
const char commandline_delimiter[]=" ,._";
char tokens[MAX_TOKENS][MAX_CHAR_PER_TOKENS];

volatile unsigned int salrt_events;

void setup() {
  int i;

  // configure serial interface
  Serial.begin(9600);
  Wire.begin();
  pinMode(salrt, INPUT);
  digitalWrite(salrt, HIGH);	// engage internal pull-up
  salrt_events=0;		// interrupt managed

  // configure ports for LED outputs
  pinMode(led_red, OUTPUT);     
  pinMode(led_yellow, OUTPUT);     
  pinMode(led_green, OUTPUT);     
  digitalWrite(led_red, LOW);
  digitalWrite(led_yellow, LOW);
  digitalWrite(led_green, LOW);

  // put your setup code here, to run once:
  status=STATUS_READY;
  status_show_led();
  
  kr_serial_clear_readbuffer();

  // set default slave address
  for (i=1; i<MAX_SLAVES; i++) saddr[i]=0;
  saddr[0]=0x10; nof_slaves=1;

  // enable interrupt service routine(s)
  attachInterrupt(2, isr_salrt, FALLING);

}

void loop() {
  // read and process command line
  kr_serial_read_str(commandline,COMMANDLINE_MAX_LENGTH);
  status_set(STATUS_BUSY); status_show_led();
  krstr_process_command_line(commandline);
  status_clr(STATUS_BUSY); status_show_led();
}

// ***********************************************************************************************
// set/clear/invert flags in status register
// ***********************************************************************************************
void status_set(const int flags) {
  status|=flags;
}
void status_clr(const int flags) {
  status&=(~flags);
}
void status_inv(const int flags) {
  status^=flags;
}
// ***********************************************************************************************
// send status register to LEDs
// ***********************************************************************************************
void status_show_led() {
  if (status&STATUS_READY) digitalWrite(led_green, HIGH); else digitalWrite(led_green, LOW);
  if (status&STATUS_BUSY) digitalWrite(led_yellow, HIGH); else digitalWrite(led_yellow, LOW);
  if (status&STATUS_ERROR) digitalWrite(led_red, HIGH); else digitalWrite(led_red, LOW);
}

// ***********************************************************************************************
// i2c functions
// ***********************************************************************************************
void kri2c_show_addresses(void) {
  byte i;
  Serial.print(F("["));
  for (i=0; i<nof_slaves; i++) {
    if (i) Serial.print(", ");
    Serial.print(saddr[i],HEX);
  }
  Serial.println(F("]"));
}

void kri2c_scan(byte addr0, byte addr1) {
  byte i, j, ack;
  for (i=0; i<MAX_SLAVES; i++) saddr[i]=0;
  nof_slaves=0;
  for (i=0, j=addr0; (j<=addr1 && i<MAX_SLAVES); j++) {
    Wire.beginTransmission(j); ack=Wire.endTransmission();	// start transmission (W access by default), end transmission, collect ack (0:success, 2:NACK_ADDR, 3: NACK_D)
    if (ack==0) { saddr[i++]=j; nof_slaves++; }
  }
  kri2c_show_addresses();
}

int kri2c_read(byte addr,byte command, byte nof_bytes, byte *rxtxbuffer) {
  byte ack, nof, i;
  Wire.beginTransmission(addr); Wire.write(command); ack=Wire.endTransmission((byte)false);	// send address and command. Sr (repeated start)
  nof=Wire.requestFrom(addr, nof_bytes, (byte)true);
  for (i=0;i<nof && i<MAX_RXTXBUFFER;i++) {
    if (Wire.available()) rxtxbuffer[i]= Wire.read();
  }
  if ((nof==nof_bytes) && (ack==0)) {
    Serial.print(F("["));
    for (i=0; i<nof_bytes; i++) {
      if (i) Serial.print(F(", "));
      Serial.print(rxtxbuffer[i],HEX);
    }
    Serial.println(F("]"));
    return 1;
  } else {
    // Serial.println(F("[]"));		this "[]" has turned out to be undesirable, as calling function produces an error message already
    return 0;
  }
}

int kri2c_write(byte *saddr, byte nof_slaves, byte nof_bytes, byte *rxtxbuffer) {
  byte ack, i;
  int nof;
  for (nof=0, ack=0, i=0;i<nof_slaves;i++) {
    if (i==nof_slaves-1) {
      // last slave to be written to in group fashion: do send stoP
      Wire.beginTransmission(saddr[i]); nof+=Wire.write(rxtxbuffer,nof_bytes); ack+=Wire.endTransmission();
    } else {
      // after command and byte(s) written: do not send stoP (instead, create a repeated start condition in order to group commands)
      Wire.beginTransmission(saddr[i]); nof+=Wire.write(rxtxbuffer,nof_bytes); ack+=Wire.endTransmission((byte)false);
    }
  }
  Serial.println(nof,HEX);
  return (nof==(nof_slaves*nof_bytes) && ack==0);
}

int kri2c_statr2s(byte addr, byte command, unsigned long nof_words) {
  byte nof, ack, ok;
  int value;
  unsigned long i_long;
  float sum;
  int value_min, value_max;
  sum=0.0; value_min=0x7FFF; value_max=0x8000;
  for (i_long=0, ok=1; i_long<nof_words && ok; i_long++) {
    Wire.beginTransmission(addr); Wire.write(command); ack=Wire.endTransmission((byte)false);	// send address and command. Sr (repeated start)
    delayMicroseconds(10);	// N.B.: this delay was found to be necessary otherwise highly sporadic errors occur.
				// when the error occurs, the Arduino I2C master fails to address the I2C slave correctly during the following Sr
    nof=Wire.requestFrom(addr, (byte)2, (byte)true);
    if (ack || nof!=2) { ok=0; break; }
    value=Wire.read(); value+=(Wire.read()<<8);
    // if (value==0xFFFF) { ok=0; Serial.print(addr,DEC); break; }
    if (value<value_min) value_min=value;
    if (value>value_max) value_max=value;
    sum+=(float)value;
  }
  sum=sum/(float)nof_words;
  if (ok) {
    Serial.print("["); Serial.print(value_min,HEX); Serial.print(", "); Serial.print(value_max,HEX); Serial.print(", "); Serial.print(sum,2); Serial.println("]");
  }
  return (ok);
}

void kri2c_salrt(byte ara) {
  byte nof_responses, nof_ara, slave_ara, slave_ara_prev;
  int salrt_poll;
  salrt_poll=digitalRead(salrt);
  switch (ara) {
    case 0:
      noInterrupts(); salrt_events=0; interrupts();
    case 1:
      Serial.print(F("[")); Serial.print(salrt_poll); Serial.print(F(", ")); Serial.print(salrt_events,HEX); Serial.println(F("]")); 
      break;
    default:
      nof_ara=0; slave_ara_prev=-1;
      Serial.print(F("["));
      while (nof_ara<MAX_SLAVES && salrt_poll==0) {
	nof_responses=Wire.requestFrom((byte)12, (byte)1, (byte)true);		// request ARP
	delayMicroseconds(10);
	if (nof_responses==1) {
	  delayMicroseconds(10); slave_ara=(Wire.read()>>1);
	  if (slave_ara==slave_ara_prev) break;		// if a single slave has a persistent fault, stop ARA-ing
	  slave_ara_prev=slave_ara;
	  if (nof_ara) Serial.print(F(", "));
	  Serial.print(slave_ara,HEX);
	  nof_ara++;
	} else break;
	salrt_poll=digitalRead(salrt);
      }
      Serial.println(F("]"));
      break;
  }
}

void isr_salrt() {
  if (salrt_events!=0xFFFF) salrt_events++;
}

// ***********************************************************************************************
// serial interface functions
// ***********************************************************************************************
void kr_serial_clear_readbuffer(void) {
  // read from input until available reports 0
  if (Serial.available() > 0) Serial.read();
}

void kr_serial_read_str(char *str, const int nof_char_max) {
  // wait for input char sequence terminated by \n
  // write input into str, keeping an eye on nof_char_max
  // input exceeding nof_char_max will over-write previously received char
  // str will be orderly null-terminated
  int rec_data, i=0;
  while (1) {
    if (Serial.available() > 0) {
      rec_data=Serial.read();
      if (rec_data=='\n' || rec_data=='\r') {
	str[i]='\0'; break;
      }
      else {
	if (isascii(rec_data)) str[i]=(char)rec_data;
      }
      if (i<(nof_char_max-1)) i++;
    }
  }
}

// ***********************************************************************************************
// string and command line functions
// ***********************************************************************************************
void krstr_show_in_brackets(const char *str) {
  Serial.print("["); Serial.print(str); Serial.println("]");
}

int krstr_to_hex(const char *str, unsigned long *hexvalue) {
  int i, valid;
  *hexvalue=0L; valid=0;
  for (i=0;i<9;i++) {
    if (str[i]=='\0') break;
    else if (isxdigit(str[i])) {
      valid=1;
      if (tolower(str[i])>='a') {
	*hexvalue=((*hexvalue)<<4)+(tolower(str[i])-'a'+10);
      } else {
	*hexvalue=((*hexvalue)<<4)+(tolower(str[i])-'0');
      }
    } else {
      valid=0; break;
    }
  }
  return valid;
}

// ***********************************************************************************************
// token functions
// ***********************************************************************************************
void krstr_show_tokens(void) {
  int i;
  unsigned long hexvalue;
  for (i=0; i<MAX_TOKENS; i++) {
    if (strlen(tokens[i])>0) {
      Serial.print(i,DEC);
      Serial.print(F(": ["));
      Serial.print(tokens[i]);
      Serial.print("]");
      if (krstr_to_hex(tokens[i],&hexvalue)) {
	Serial.print(" = 0d");
	Serial.print(hexvalue,DEC);
      }
      Serial.println("");
    }
  }
}

int krstr_tokenize(char *commandline) {
  int i;
  char *tok, *act, *temp_ptr;
  // firstly, erase all tokens
  for (i=0; i<MAX_TOKENS; i++) strcpy(tokens[i],"");

  for (i=0,tok=commandline,act=commandline;tok;act=NULL) {
    tok=strtok_r(act,commandline_delimiter,&temp_ptr);
    if (tok) {
      strncpy(tokens[i],tok,MAX_CHAR_PER_TOKENS-1);
      tokens[i][MAX_CHAR_PER_TOKENS-1]='\0';	// enforce that token is null-terminated
      i++;
      if (i==MAX_TOKENS) break;
    }
  }
  return i;
}

void krstr_cleanup(char *str) {
  // this function does not hesitate to modify your string str!
  // it converts all chars to lower case. converts all whitespace to simple space. and chops off leading/trailing spaces.
  int i, d, s, sc;
  strlwr(str);	// convert to lower case
  // convert all white space(s) to simple space(s)
  for (i=0; str[i]; i++) {
    if (isspace(str[i])) str[i]=' ';
  }
  // chop trailing white space
  for (i=strlen(str)-1;i>=0;i--) {
    if (!isspace(str[i])) break; else str[i]='\0';
  }
  // point to first non-space char
  for (i=0; str[i]; i++) {
    if (!isspace(str[i])) break;
  }
  // move str to 1st pos in memory, also skip multiple spaces
  for (d=0,s=i,sc=0;;) {
    if (isspace(str[s])) {
      if (!sc) {
	sc=1; str[d++]=str[s];
      }
    }
    else {
      sc=0; str[d++]=str[s];
    }
    if (!str[s++]) break;
  }
}
void krstr_error(const char *str) {
  status_set(STATUS_ERROR); status_show_led();
  Serial.print(F("Error: ")); Serial.println(str);
}
void krstr_ok(void) {
  status_set(STATUS_ERROR); status_show_led();
  Serial.println(F("ok"));
}
void krstr_line(int nr, char c) {
  int i;
  for (i=0;i<nr;i++) Serial.print(c);
  Serial.println();
}

void krstr_process_command_line(char *commandline) {
  byte value0, value1;
  int i;
  unsigned long temp_value, value_long0;
  krstr_cleanup(commandline);
  i=krstr_tokenize(commandline);
//   Serial.print("Reduced to ");
//   Serial.print(i,DEC);
//   Serial.println(" token(s)");
//   krstr_show_tokens();
  if (i) {
    switch (tokens[0][0]) {
      case 'r':
	if (i<3) {
	  krstr_error("syntax");
	} else {
	  if (krstr_to_hex(tokens[1],&temp_value)) {
	    value0=constrain((byte)temp_value,0,255);
	  } else {
	    krstr_error("value"); break;
	  }
	  if (krstr_to_hex(tokens[2],&temp_value)) {
	    value1=constrain((byte)temp_value,1,MAX_RXTXBUFFER);
	  } else {
	    krstr_error("value"); break;
	  }
	  if (kri2c_read(saddr[0],value0,value1,rxtxbuffer)==0) {
	    krstr_error("comms"); break;
	  }
	}
	break;
      case 'w':
	for (value0=0;value0<(i-1) && value0<MAX_RXTXBUFFER;value0++) {
	  if (krstr_to_hex(tokens[value0+1],&temp_value)) {
	    rxtxbuffer[value0]=constrain((byte)temp_value,0,255);
	  } else {
	    krstr_error("value"); break;
	  }
	}
	if (kri2c_write(saddr,nof_slaves,i-1,rxtxbuffer)==0) {
	  krstr_error("comms"); break;
	}
	break;
      case 'l':
	if (i<2) {
	  kri2c_salrt(1);	// just produce status
	} else {
	  if (tokens[1][0]=='0') {
	    kri2c_salrt(0);	// clear salrt events counter, then produce status
	  } else {
	    kri2c_salrt(2);	// carry out a full ARP
	  }
	}
	break;
      case 't':
	if (i<3) {
	  krstr_error("syntax");
	} else {
	  // token[1] holds the command
	  if (krstr_to_hex(tokens[1],&temp_value)) {
	    value0=constrain((byte)temp_value,0,255);
	  } else {
	    krstr_error("value"); break;
	  }
	  // token[2] holds the number of (word) reads. long
	  if (krstr_to_hex(tokens[2],&temp_value)) {
	    value_long0=temp_value;
	  } else {
	    krstr_error("value"); break;
	  }
	  if (kri2c_statr2s(saddr[0],value0,value_long0)==0) {
	    krstr_error("comms"); break;
	  }
	}
	break;
      case 'a':
	if (i>=2) {
	  nof_slaves=0;
	  for (value0=0;value0<(i-1) && value0<MAX_SLAVES;value0++) {
	    if (krstr_to_hex(tokens[value0+1],&temp_value)) {
	      saddr[value0]=constrain((byte)temp_value,0,127); nof_slaves++;
	    } else {
	      krstr_error("value"); break;
	    }
	  }
	}
	kri2c_show_addresses();
	break;
      case 's':
	value0=0x10; value1=0x7f;		// default scan range
	if (i>=3) {
	  if (krstr_to_hex(tokens[2],&temp_value)) {
	    value1=constrain((byte)temp_value,0,127);
	  } else {
	    krstr_error("value"); break;
	  }
	}
	if (i>=2) {
	  if (krstr_to_hex(tokens[1],&temp_value)) {
	    value0=constrain((byte)temp_value,0,value1);
	  } else {
	    krstr_error("value"); break;
	  }
	}
	kri2c_scan(value0,value1);
	break;
      case 'f':
	if (i>=2) {
	  if (krstr_to_hex(tokens[1],&temp_value)) {
	    bus_speed=(int)temp_value;
	    Serial.print(bus_speed,DEC);
	    Serial.println(F("kHz"));
	    temp_value=temp_value*1000L;
	    TWBR = ((F_CPU / temp_value) - 16) / 2;	// adopted from twi.h and twi.c (~/arduino-1.5.5/hardware/arduino/avr/libraries/Wire/utility)
	  } else {
	    krstr_error("value");
	  }
	} else {
	  Serial.print(bus_speed,DEC);
	  Serial.println(F("kHz"));
	}
	break;
      case 'v':
	Serial.println(F(SW_VERSION));
	break;
      case 'z':
	if (i>=2) {
	  if (krstr_to_hex(tokens[1],&temp_value)) {
	    status=constrain((byte)temp_value,0,255);
	  } else {
	    krstr_error("value"); break;
	  }
	}
	Serial.println(status,HEX);
	status_show_led();
	break;
      case '?':
      case 'h':
	// Serial.println("\x0B\x0C");		// vertical tab, form feed - doesn't work
	krstr_line(50,'=');
	Serial.println(F("u2i list of commands:"));
	krstr_line(50,'=');
	Serial.println(F("a [a..]     view/set i2c address(es)"));
	Serial.println(F("f [speed]   view/set bus frequency in kHz (kb/s)"));
	Serial.println(F("h           show help"));
	Serial.println(F("l [p]       view salrt status (no p). clears if p=0. arp if p=1"));
	Serial.println(F("r c b       smbus read byte/word c=command b=nof_bytes"));
	Serial.println(F("s [a0] [a1] scan i2c bus from address a0 to a1"));
	Serial.println(F("t c n       statistical read using command c reading n words"));
	Serial.println(F("v           show version"));
	Serial.println(F("w c b..     smbus write byte(s) c=command"));
	Serial.println(F("z [v]       view/set status byte"));
	krstr_line(20,'-');
	Serial.println(F("1. all values to u2i in hex w/o format specified"));
	Serial.println(F("2. smbus write commands will be grouped and sent to all set addresses"));
	Serial.println(F("3. smbus read commands issued to first set address (only)"));
	break;
      default:
	krstr_error("syntax");
	break;
    }
  }
}
