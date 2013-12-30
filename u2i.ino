// Author: Karl Rinne, karl.rinne@smartpaddy.ie
// (c) by kr64 Ltd. Proprietary. Confidential. Do not read or distribute unless authorized.

// History:
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

// global register definitions

// the following define is the most memory-efficient way to store the version string (in flash, ensured when SW_VERSION is instantiated)
#define SW_VERSION "u2i v0.50 20131230 (c) kr64.com"

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
const int MAX_CHAR_PER_TOKENS=6;
const int COMMANDLINE_MAX_LENGTH=MAX_TOKENS*MAX_CHAR_PER_TOKENS;
char commandline[COMMANDLINE_MAX_LENGTH]="";
const char commandline_delimiter[]=" ,._";
char tokens[MAX_TOKENS][MAX_CHAR_PER_TOKENS];

// u2i test system connections
const int led_red = 8;
const int led_yellow = 9;
const int led_green = 10;


void setup() {
  int i;

  // configure serial interface
  Serial.begin(9600);
  Wire.begin();

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
    Serial.println(F("[]")); return 0;
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
//   Serial.print("nof=");Serial.print(nof, DEC);Serial.print("   ack="); Serial.println(ack, DEC);
//   Serial.print("[");
//   for (i=0;i<nof_bytes;i++) {
//     if (i) Serial.print(", ");
//     Serial.print(rxtxbuffer[i],HEX);
//   }
//   Serial.println("]");
  Serial.println(nof,HEX);
  return (nof==(nof_slaves*nof_bytes) && ack==0);
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
  unsigned long temp_value;
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
	Serial.println(F("r [c] [b]   smbus read byte/word c=command b=nof_bytes"));
	Serial.println(F("s [a0] [a1] scan i2c bus from address a0 to a1"));
	Serial.println(F("v           show version"));
	Serial.println(F("w [c] [b..] smbus write byte(s) c=command"));
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
