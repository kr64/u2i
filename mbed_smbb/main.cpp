#include "mbed.h"
#include "string.h"
#include "ctype.h"
#include "pv3012_defs.h"

// History
// 22/08/2013 v0.50 statr2s like stat but handling signed words
// 02/08/2013 v0.48 statr2 (statistical ambareg read word) added
// 08/10/2012 v0.46 test 4 added (ADC offset calibration, offset-DAC linearity test)
// 20/09/2012 v0.44 test 3 slightly improved
// 19/09/2012 v0.44 test 3 added (sync_osc_trimf toggling between -10% and +10%)
// 16/08/2012 v0.40 test 2 added (alternating read VOUT_MARGIN_HIGH, VOUT_MARGIN_LOW)
// 16/08/2012 v0.38 complete (test 1 complete, stops on error)

DigitalOut led_ready(LED1);
DigitalOut led_transmit(LED2);
DigitalOut led_receive(LED3);
DigitalOut led_error(LED4);

AnalogOut dac(p18);     // 10b R-string DAC, 1us max update rate. VREFP=3.3V, hence DAC output range 0V..3.3V
                        // when using write_u16: 16-bit unsigned short representing the output voltage, normalised to a 16-bit value (0x0000 = 0v, 0xFFFF = 3.3v)

Serial pc(USBTX, USBRX); // virtual serial port to host PC (through mbed-if01 device
I2C i2c_host(p9, p10);

// I2C definitions
const int NOF_SLAVES_MAX=32;
const int NOF_BYTES_R=256;    // receive buffer
const int NOF_BYTES_W=16;    // transmit buffer
char i2c_buf[NOF_BYTES_R];
char i2c_cmd[NOF_BYTES_W];
double i2c_speed;
int slave_addresses[NOF_SLAVES_MAX];
enum kr_i2c_errors {I2C_OK=0, I2C_TNACK, I2C_RNACK};

// message types, error codes, etc
enum kr_msg_type {MSG_SUPPRESS=0, MSG_DEBUG, MSG_PLAINNOCR, MSG_PLAIN, MSG_INFO, MSG_ERROR};
enum kr_error_codes {ERR_CMD=1, ERR_PAR=2, ERR_ADDR=4};

// function prototypes
void kr_message(kr_msg_type msg_type, const char *message);
void kr_cmd_clear(void);
int kr_cmd_append(const char c);
int kr_cmd_tokenize(void);
void kr_process_tokens(void);
int kr_process_command(void);
void kr_i2c_clear_addresses(void);
void kr_i2c_addr_set(int addr_set);
void kr_i2c_scan(int addr_start, int addr_stop);
void kr_i2c_list_addresses(void);
void kr_i2c_r(int command, int nof_bytes);
void kr_i2c_rstat2(int command, unsigned long nof_reads, int treatwordassigned);
void kr_i2c_w(int command, int nof_bytes);
void kr_i2c_test(int test_number, int nof_loops);
kr_i2c_errors kr_i2c_raw_write_wb (int addr, int nof_bytes);
kr_i2c_errors kr_i2c_raw_read_wb (int addr, int nof_bytes);

// definition of constants
const char program_name[]="SMBus Bridge";
const char program_name_short[]="smbb";
const char program_creator[]="KR";
const char program_version[]="V0.50/20130822";
const char cmd_token_delimiters[]=" \t\r\n,";

// smbb commands, and command ID's
const char *smbb_commands[]={"r","w",";","status","ver","scan","addr","debug","echo","speed","test","dac","statr2","statr2s"};
enum {CMD_R, CMD_W, CMD_NOOP, CMD_STATUS, CMD_VER, CMD_SCAN, CMD_ADDR, CMD_DEBUG, CMD_ECHO, CMD_SPEED, CMD_TEST, CMD_DAC, CMD_STATR2, CMD_STATR2S};

int mode_debug=0;        // enable or disable DEBUG mode
int mode_echo=0;

const int CMD_BUFFER_TOKEN_SIZE=32;
const int CMD_BUFFER_TOKENS=16;
const int CMD_BUFFER_SIZE=CMD_BUFFER_TOKEN_SIZE*CMD_BUFFER_TOKENS;

char user_message[NOF_BYTES_R*2];
char cmd_buffer[CMD_BUFFER_SIZE];
int cmd_buffer_i;
const char *cmd_tokens[CMD_BUFFER_TOKENS];

int token_idx;
int status;

int main() {
    int i, c;
    while(1) {
        dac.write_u16(0x0000);                  // clear DAC output
        i2c_speed=400e3;                        // set default I2C speed to 400kb/s
        i2c_host.frequency(int(i2c_speed));
        kr_i2c_clear_addresses();
        kr_cmd_clear();
        status=0;

        led_ready = 1;
        while (1) {
            if (pc.readable()) {
                // there is a character ready to be read
                c=tolower(pc.getc());   // get character
                // echo, if desired
                if (mode_echo)
                    pc.putc(c);
                if (kr_cmd_append(c)) {
                    // we received a line terminated by '\n'. let's clean and interpret the line
                    led_receive=1; wait(0.2); led_receive=0;
                    sprintf(user_message,"%s command string: \'%s\'",program_name_short,cmd_buffer);
                    kr_message(MSG_DEBUG,user_message);
                    kr_cmd_tokenize();
                    i=0;
                    while (cmd_tokens[i]!=NULL) {
                        sprintf(user_message,"Token %02d: \'%s\'",i,cmd_tokens[i]);
                        kr_message(MSG_DEBUG,user_message);
                        i++;
                    }
                    token_idx=0; kr_process_tokens();
                    kr_cmd_clear();
                    if (mode_echo) {
                        // if in echo mode, produce a simple prompt
                        sprintf(user_message,"%s> ",program_name_short);
                        kr_message(MSG_PLAINNOCR,user_message);
                    }
                }
            }
        }
    }
}

void kr_cmd_clear(void) {
    cmd_buffer[0]=0;
    cmd_buffer_i=0;
}

void kr_message(kr_msg_type msg_type, const char *message) {
    switch (msg_type) {
        case MSG_DEBUG:
            if (mode_debug) {
                printf(" %s_debug: %s\r\n",program_name_short,message);
            }
            break;
        case MSG_PLAINNOCR:
            printf("%s",message);
            break;
        case MSG_PLAIN:
            printf("%s\r\n",message);
            break;
        case MSG_INFO:
            printf(" %s_info:  %s\r\n",program_name_short,message);
            break;
        case MSG_ERROR:
            printf(" %s_error: %s\r\n",program_name_short,message);
            break;
        default:
            break;
    }
}

int kr_cmd_append(const char c) {
    if (c=='\n' || c=='\r') {
        cmd_buffer[cmd_buffer_i]=0;
        return 1;
    }
    else {
        cmd_buffer[cmd_buffer_i]=c;
        if (cmd_buffer_i<(CMD_BUFFER_SIZE-1)) {
            cmd_buffer_i++;
        }
        return 0;
    }
}
int kr_cmd_tokenize(void) {
    // tokenize cmd_buffer
    int i=0;
    cmd_tokens[i]=strtok(cmd_buffer,cmd_token_delimiters);
    while (cmd_tokens[i]) {
        if (i<(CMD_BUFFER_TOKENS-1))
            i++;
        cmd_tokens[i]=strtok(0,cmd_token_delimiters);
    }
    return i;
}

void kr_process_tokens(void) {
    while (cmd_tokens[token_idx]) {
        if (kr_process_command()==0) {
        }
        else
        {
            return;
        }
    }
}

int kr_process_command(void) {
    int i, command, nof_bytes, temp;
    double nof_actions;

    for(i=0;i<(sizeof(smbb_commands)/sizeof(char *));i++) {
        if (strcmp(cmd_tokens[token_idx],smbb_commands[i])==0) {
            // command match
            switch(i) {
                case CMD_R:
                    // read byte. expect a command code
                    if (sscanf(cmd_tokens[token_idx+1],"%x",&command)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+1]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    if (sscanf(cmd_tokens[token_idx+2],"%x",&nof_bytes)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+2]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    kr_i2c_r(command,nof_bytes);
                    token_idx+=3;
                    return 0;
                case CMD_STATR2:
                    // read word n-times using command code provided. Return min, max, avg
                    // note: n is long int to support long-term stats
                    if (sscanf(cmd_tokens[token_idx+1],"%x",&command)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+1]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    if (sscanf(cmd_tokens[token_idx+2],"%lf",&nof_actions)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+2]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    kr_i2c_rstat2(command, long(nof_actions),0);
                    token_idx+=3;
                    return 0;
                case CMD_STATR2S:
                    // this is the signed version of CMD_STATR2
                    if (sscanf(cmd_tokens[token_idx+1],"%x",&command)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+1]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    if (sscanf(cmd_tokens[token_idx+2],"%lf",&nof_actions)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+2]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    kr_i2c_rstat2(command, long(nof_actions),1);
                    token_idx+=3;
                    return 0;
                case CMD_W:
                    // read byte. expect a command code, followed by n data bytes
                    if (sscanf(cmd_tokens[token_idx+1],"%x",&command)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+1]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    for (nof_bytes=0;nof_bytes<NOF_BYTES_W;nof_bytes++) {
                        if ( cmd_tokens[token_idx+2+nof_bytes]==0 || *cmd_tokens[token_idx+2+nof_bytes]==';' )
                            break;
                        if (sscanf(cmd_tokens[token_idx+2+nof_bytes],"%x",&temp)!=1) {
                            status|=ERR_PAR;
                            sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+1]);
                            kr_message(MSG_ERROR,user_message);
                            return -1;
                        }
                        i2c_cmd[1+nof_bytes]=(char)temp;
                    }
                    sprintf(user_message,"Writing command %02X and %d data byte(s)",command,nof_bytes);
                    kr_message(MSG_DEBUG,user_message);
                    kr_i2c_w(command,1+nof_bytes);
                    token_idx=token_idx+2+nof_bytes;
                    return 0;

                case CMD_STATUS:
                    if (cmd_tokens[token_idx+1]==0 || *cmd_tokens[token_idx+1]==';') {
                        sprintf(user_message,"%02X",status);
                        kr_message(MSG_PLAIN,user_message);
                        token_idx++;
                    }
                    else if (strcmp(cmd_tokens[token_idx+1],"clear")==0) {
                        kr_message(MSG_DEBUG,"status register cleared");
                        status=0;
                        sprintf(user_message,"%02X",status);
                        kr_message(MSG_PLAIN,user_message);
                        token_idx+=2;
                    }
                    else {
                        token_idx++;
                    }
                    return 0;
                case CMD_VER:
                    sprintf(user_message,"%s (%s) (c) by %s. Version: %s",program_name,program_name_short,program_creator,program_version);
                    kr_message(MSG_PLAIN,user_message);
                    token_idx++;
                    return 0;
                case CMD_SCAN:
                    // expect two parameters to follow: addr_start and addr_stop
                    int addr_start, addr_stop;
                    if (sscanf(cmd_tokens[token_idx+1],"%x",&addr_start)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+1]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    if (sscanf(cmd_tokens[token_idx+2],"%x",&addr_stop)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+2]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    sprintf(user_message,"SMBus address scan %02X..%02X",addr_start,addr_stop);
                    kr_message(MSG_DEBUG,user_message);
                    token_idx+=3;
                    kr_i2c_scan(addr_start,addr_stop);
                    return 0;
                case CMD_ADDR:
                    int addr_set;
                    if (sscanf(cmd_tokens[token_idx+1],"%x",&addr_set)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+1]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    sprintf(user_message,"Set address to 0x%02x",addr_set);
                    kr_message(MSG_DEBUG,user_message);
                    kr_i2c_addr_set(addr_set);
                    token_idx+=2;
                    return 0;
                case CMD_DEBUG:
                    if (mode_debug==0)
                        mode_debug=1;
                    else
                        mode_debug=0;
                    sprintf(user_message,"%1X",mode_debug);
                    kr_message(MSG_PLAIN,user_message);
                    token_idx++;
                    return 0;
                case CMD_ECHO:
                    if (mode_echo==0)
                        mode_echo=1;
                    else
                        mode_echo=0;
                    sprintf(user_message,"%1X",mode_echo);
                    kr_message(MSG_PLAIN,user_message);
                    token_idx++;
                    return 0;
                case CMD_SPEED:
                    if (cmd_tokens[token_idx+1]==0 || *cmd_tokens[token_idx+1]==';') {
                        token_idx++;
                    }
                    else if (sscanf(cmd_tokens[token_idx+1],"%lf",&i2c_speed)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+1]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    else {
                        i2c_host.frequency(int(i2c_speed));
                        token_idx+=2;
                    }
                    sprintf(user_message,"%.1fkb/s",i2c_speed/1e3);
                    kr_message(MSG_PLAIN,user_message);
                    return 0;
                case CMD_NOOP:
                    token_idx++;
                    return 0;
                case CMD_TEST:
                    // custom tests
                    int test_number, loops;
                    if (sscanf(cmd_tokens[token_idx+1],"%x",&test_number)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+1]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    if (sscanf(cmd_tokens[token_idx+2],"%x",&loops)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+2]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    kr_i2c_test(test_number,loops);
                    token_idx+=3;
                    return 0;
                case CMD_DAC:
                    // set DAC output voltage (unsigned 16b value expected, 0xFFFF representing 3.3V)
                    int dacvalue;
                    if (sscanf(cmd_tokens[token_idx+1],"%x",&dacvalue)!=1) {
                        status|=ERR_PAR;
                        sprintf(user_message,"Wrong parameter \'%s\'",cmd_tokens[token_idx+1]);
                        kr_message(MSG_ERROR,user_message);
                        return -1;
                    }
                    dac.write_u16((unsigned int) dacvalue);
                    sprintf(user_message,"%04X",dacvalue);
                    kr_message(MSG_PLAIN,user_message);
                    token_idx+=2;
                    return 0;
                default:
                    // we should never really arrive here (problem with command definition in this file)
                    return -1;
            }
        }
    }
    // unrecognised command
    sprintf(user_message,"Command \'%s\' not recognised",cmd_tokens[token_idx]);
    kr_message(MSG_ERROR,user_message);
    status|=ERR_CMD;
    return -1;
}

void kr_i2c_clear_addresses(void) {
    int i;
    for (i=0;i<NOF_SLAVES_MAX;i++) {
        slave_addresses[i]=-1;
    }
}

void kr_i2c_addr_set(int addr_set) {
    kr_i2c_clear_addresses();   // clear addresses
    if (addr_set>=0 && addr_set<128) {
        slave_addresses[0]=addr_set;
    }
    kr_i2c_list_addresses();
}

void kr_i2c_scan(int addr_start, int addr_stop) {
    int addr, i;
    kr_i2c_clear_addresses();   // clear addresses
    if (addr_start>addr_stop) {
        i=addr_stop;    // temp
        addr_stop=addr_start;
        addr_start=i;
    }
    i=0;
    for(addr=addr_start;addr<=addr_stop && i<NOF_SLAVES_MAX;addr++) {
        sprintf(user_message,"look for slave addr %2X",addr);
        kr_message(MSG_DEBUG,user_message);
        if (i2c_host.read(addr<<1,i2c_buf,1,0)==0) {
            slave_addresses[i++]=addr;
            sprintf(user_message,"slave addr %2X ACK received",addr);
            kr_message(MSG_DEBUG,user_message);
        }
    }
    kr_i2c_list_addresses();
}

void kr_i2c_list_addresses(void) {
    char addr_str[8];
    int i;
    i=0;
    strcpy(user_message,"");
    while (i<NOF_SLAVES_MAX && slave_addresses[i]>=0) {
        sprintf(addr_str,"%02x ",slave_addresses[i++]);
        strcat(user_message,addr_str);
    }
    kr_message(MSG_PLAIN,user_message);
}

void kr_i2c_r(int command, int nof_bytes) {
    char addr_str[8];
    int i,k,responded;
    responded=0;
    if (nof_bytes<1) {
        nof_bytes=1;
    }
    else if (nof_bytes>NOF_BYTES_R) {
        nof_bytes=NOF_BYTES_R;
    }
    i2c_cmd[0]=char(command);
    sprintf(user_message,"Read %d bytes command following command \'%hx\'",nof_bytes,i2c_cmd[0]);
    kr_message(MSG_DEBUG,user_message);

    for (i=0;i<NOF_SLAVES_MAX;i++) {
        if (slave_addresses[i]>=0) {
            strcpy(user_message,"");
            if (i2c_host.write(slave_addresses[i]<<1,i2c_cmd,1,1)==0) {
                // write command was ACK'd
                if (i2c_host.read(slave_addresses[i]<<1,i2c_buf,nof_bytes,0)==0) {
                    // repeated start for subsequent read was ACK'd. we've got presumably valid data in the buffer
                    for (k=0;k<nof_bytes;k++) {
                        sprintf(addr_str,"%02hx ",i2c_buf[k]);
                        strcat(user_message,addr_str);
                    }
                }
                else
                {
                    // repeated start for subsequent read was NACK'd
                    strcpy(user_message,"SrNACK");
                }
            }
            else
            {
                // write command was met by a NACK
                strcpy(user_message,"NACK");
            }
            kr_message(MSG_PLAIN,user_message); responded+=1;
        } else {
            if (responded==0) {
                kr_message(MSG_PLAIN,"");
            }
            return;
        }
    }
}

void kr_i2c_rstat2(int command, unsigned long nof_reads, int treatwordassigned) {
    int i,responded,slave_trouble;
    long int data_latest, data_min, data_max;
    double data_total;
    unsigned long longi;

    responded=0;
    i2c_cmd[0]=char(command);
    sprintf(user_message,"Read %ld words and produce stats (min/max/avg) following command \'%hx\'. Signed=%d",nof_reads,i2c_cmd[0],treatwordassigned);
    kr_message(MSG_DEBUG,user_message);
    for (i=0;i<NOF_SLAVES_MAX;i++) {
        if (slave_addresses[i]>=0) {
            strcpy(user_message,"");
            // reset stats
            data_total=0;
            if (treatwordassigned) {
                data_max=-32768; data_min=32767;
            } else {
                data_max=0; data_min=1<<16-1;
            }
            slave_trouble=0;
            for (longi=0;longi<nof_reads && slave_trouble==0;longi++) {
                strcpy(user_message,"");
                if (i2c_host.write(slave_addresses[i]<<1,i2c_cmd,1,1)==0) {
                    // write command was ACK'd
                    if (i2c_host.read(slave_addresses[i]<<1,i2c_buf,2,0)==0) {
                        // read was ACK'd. we've got a valid data word (LSB, MSB) in the buffer
                        data_latest=i2c_buf[1]*256+i2c_buf[0];
                        if (treatwordassigned) {
                            if (data_latest>0x7FFF) {
                                data_latest=data_latest-0x10000L;
                            }
                        }
                        if (data_latest>data_max) data_max=data_latest;
                        if (data_latest<data_min) data_min=data_latest;
                        data_total+=data_latest;
                        responded=1;
                    }
                    else
                    {
                        // repeated start for subsequent read was NACK'd
                        slave_trouble=1;
                    }
                }
                else
                {
                    // write command was met by a NACK
                    slave_trouble=1;
                }
            } // end of for longi
            if (slave_trouble==0) {
                sprintf(user_message,"%04hx %04hx %.3lf",data_min,data_max,data_total/double(nof_reads));
                kr_message(MSG_PLAIN,user_message);
            } else {
                sprintf(user_message,"Slave 0x%02x communication problems",slave_addresses[i]);
                kr_message(MSG_ERROR,user_message);
            }

        } else {    // we're finished with all slaves
            if (responded==0) {
                kr_message(MSG_PLAIN,"");
            }
            return;
        }
    }   // end of for slaves

}

void kr_i2c_w(int command, int nof_bytes) {
    int i,responded;
    responded=0;
    if (nof_bytes>NOF_BYTES_W) {
        nof_bytes=NOF_BYTES_W;
    }
    i2c_cmd[0]=char(command);
    sprintf(user_message,"Write %d bytes command following command \'%hx\'",nof_bytes,i2c_cmd[0]);
    kr_message(MSG_DEBUG,user_message);

    for (i=0;i<NOF_SLAVES_MAX;i++) {
        if (slave_addresses[i]>=0) {
            responded+=1;
            switch ( kr_i2c_raw_write_wb (slave_addresses[i], nof_bytes) ) {
                case I2C_OK:
                    strcpy(user_message,"");
                    break;
                case I2C_TNACK:
                default:
                    strcpy(user_message,"NACK");
                    break;
            }
            kr_message(MSG_PLAIN,user_message); responded+=1;
        } else {
            if (responded==0) {
                // if no slaves, respond anyway
                kr_message(MSG_PLAIN,"No SMBus device to communicate with");
            }
            return;
        }
    }
}

void kr_i2c_test(int test_number, int nof_loops) {
    int i, addr;
    int loop_number, cycles_per_loop, errors, errors_nack, err_total, iterations_total, show_errors;
    int run;
    int dac_offset_value;
    int dac_offset_direction;

    switch (test_number) {
        case 1:
            addr=slave_addresses[0];
            if (addr<0) {
                sprintf(user_message,"No SMBus slave address defined");
                kr_message(MSG_ERROR,user_message);
                status|=ERR_ADDR;
                return;
            }
            sprintf(user_message,"Run Custom Test #%d using SMBus slave addr 0x%02X",test_number,addr);
            kr_message(MSG_INFO,user_message);
            sprintf(user_message,"Write-Word Read-Word Cyclic Test (UT_WARN_LIMIT, alternate between +1degC and -2degC)");
            kr_message(MSG_INFO,user_message);
            err_total=0; iterations_total=0, show_errors=0;
            cycles_per_loop=1000000; run=1;

            dac.write_u16(0x0000);      // clear DAC trigger output

            for (loop_number=0;loop_number<nof_loops && run;loop_number++) {
                errors=0; errors_nack=0;
                for (i=0;i<cycles_per_loop && run;i++) {
                    // write UT_WARN_LIMIT +1degC
                    i2c_cmd[0]=0x52; i2c_cmd[1]=0x1; i2c_cmd[2]=0x0;
                    switch (kr_i2c_raw_write_wb(addr, 3)) {
                        case I2C_OK:
                            break;
                        default:
                            errors_nack+=1; run=0;
                            break;
                    }
                    // verify-read UT_WARN_LIMIT +1degC
                    i2c_cmd[0]=0x52;
                    switch (kr_i2c_raw_read_wb(addr, 2)) {
                        case I2C_OK:
                            if (i2c_buf[0]==1 && i2c_buf[1]==0) {
                            } else {
                                dac.write_u16( (unsigned int)((float)(1.0/3.3)*0xFFFF) );  // 2V
                                run=0;  // stop after error
                                if (show_errors<10) {
                                    sprintf(user_message,"expected read value 0x0001, but received 0x%02X%02X",i2c_buf[1],i2c_buf[0]);
                                    kr_message(MSG_ERROR,user_message);
                                    show_errors+=1;
                                    if (show_errors==10) {
                                        sprintf(user_message,"... further error messages will be suppressed (but counted)...");
                                        kr_message(MSG_ERROR,user_message);
                                    }
                                }
                                errors+=1;
                            }
                            break;
                        default:
                            errors_nack+=1; run=0;
                            break;
                    }
                    if (run==0) break;

                    // write UT_WARN_LIMIT -2degC
                    i2c_cmd[0]=0x52; i2c_cmd[1]=0xFE; i2c_cmd[2]=0xFF;
                    switch (kr_i2c_raw_write_wb(addr, 3)) {
                        case I2C_OK:
                            break;
                        default:
                            errors_nack+=1; run=0;
                            break;
                    }
                    // verify-read UT_WARN_LIMIT -2degC
                    i2c_cmd[0]=0x52;
                    switch (kr_i2c_raw_read_wb(addr, 2)) {
                        case I2C_OK:
                            if (i2c_buf[0]==0xFE && i2c_buf[1]==0xFF) {
                            } else {
                                dac.write_u16( (unsigned int)((float)(2.0/3.3)*0xFFFF) );  // 2V
                                run=0;  // stop after error
                                if (show_errors<10) {
                                    sprintf(user_message,"expected read value 0xFFFE, but received 0x%02X%02X",i2c_buf[1],i2c_buf[0]);
                                    kr_message(MSG_ERROR,user_message);
                                    show_errors+=1;
                                    if (show_errors==10) {
                                        sprintf(user_message,"... further error messages will be suppressed (but counted)...");
                                        kr_message(MSG_ERROR,user_message);
                                    }
                                }
                                errors+=1;
                            }
                            break;
                        default:
                            errors_nack+=1; run=0;
                            break;
                    }
                    if (run==0) break;
                }
                // end of single test loop
                iterations_total+=i;
                err_total=err_total+errors+errors_nack;
                sprintf(user_message,"Test loop #%d complete. %d cycles. %d errors in loop. %d NACK errors in loop",loop_number,i,errors,errors_nack);
                kr_message(MSG_INFO,user_message);
            }
            // all test loops finished
            sprintf(user_message,"Finished Custom Test #%d using SMBus slave addr 0x%02X. Total iterations: %d. Total number of errors: %d",test_number,addr,iterations_total,err_total);
            kr_message(MSG_INFO,user_message);
            break;
        case 2:
            addr=slave_addresses[0];
            if (addr<0) {
                sprintf(user_message,"No SMBus slave address defined");
                kr_message(MSG_ERROR,user_message);
                status|=ERR_ADDR;
                return;
            }
            sprintf(user_message,"Run Custom Test #%d using SMBus slave addr 0x%02X",test_number,addr);
            kr_message(MSG_INFO,user_message);
            sprintf(user_message,"Write VOUT_MARGIN_LOW=1V, VOUT_MARGIN_HIGH=2V, then alternating read from both registers");
            kr_message(MSG_INFO,user_message);
            err_total=0; iterations_total=0, show_errors=0;
            cycles_per_loop=1000000; run=1;
            errors_nack=0;

            dac.write_u16(0x0000);      // clear DAC trigger output

            // write initial values into registers
            i2c_cmd[0]=0x26; i2c_cmd[1]=0x01; i2c_cmd[2]=0x14;    // VOUT_MARGIN_LOW=1.0002V
            switch (kr_i2c_raw_write_wb(addr, 3)) {
                case I2C_OK:
                    break;
                default:
                    errors_nack+=1; run=0;
                    break;
            }
            i2c_cmd[0]=0x25; i2c_cmd[1]=0x02; i2c_cmd[2]=0x28;    // VOUT_MARGIN_HIGH=2.0004V
            switch (kr_i2c_raw_write_wb(addr, 3)) {
                case I2C_OK:
                    break;
                default:
                    errors_nack+=1; run=0;
                    break;
            }
            if (errors_nack) {
                sprintf(user_message,"Failed to write VOUT_MARGIN_HIGH/LOW");
                kr_message(MSG_ERROR,user_message);
            }

            for (loop_number=0;loop_number<nof_loops && run;loop_number++) {
                errors=0; errors_nack=0;
                
                for (i=0;i<cycles_per_loop && run;i++) {
                    // verify-read VOUT_MARGIN_LOW=1V
                    i2c_cmd[0]=0x26;
                    switch (kr_i2c_raw_read_wb(addr, 2)) {
                        case I2C_OK:
                            if (i2c_buf[0]==0x01 && i2c_buf[1]==0x14) {
                            } else {
                                dac.write_u16( (unsigned int)((float)(1.0/3.3)*0xFFFF) );  // 2V
                                run=0;  // stop after error
                                if (show_errors<10) {
                                    sprintf(user_message,"expected read value 0x1401 (1V), but received 0x%02X%02X",i2c_buf[1],i2c_buf[0]);
                                    kr_message(MSG_ERROR,user_message);
                                    show_errors+=1;
                                    if (show_errors==10) {
                                        sprintf(user_message,"... further error messages will be suppressed (but counted)...");
                                        kr_message(MSG_ERROR,user_message);
                                    }
                                }
                                errors+=1;
                            }
                            break;
                        default:
                            errors_nack+=1; run=0;
                            break;
                    }
                    if (run==0) break;

                    // verify-read VOUT_MARGIN_LOW=2V
                    i2c_cmd[0]=0x25;
                    switch (kr_i2c_raw_read_wb(addr, 2)) {
                        case I2C_OK:
                            if (i2c_buf[0]==0x02 && i2c_buf[1]==0x28) {
                            } else {
                                dac.write_u16( (unsigned int)((float)(2.0/3.3)*0xFFFF) );  // 2V
                                run=0;  // stop after error
                                if (show_errors<10) {
                                    sprintf(user_message,"expected read value 0x2802 (2V), but received 0x%02X%02X",i2c_buf[1],i2c_buf[0]);
                                    kr_message(MSG_ERROR,user_message);
                                    show_errors+=1;
                                    if (show_errors==10) {
                                        sprintf(user_message,"... further error messages will be suppressed (but counted)...");
                                        kr_message(MSG_ERROR,user_message);
                                    }
                                }
                                errors+=1;
                            }
                            break;
                        default:
                            errors_nack+=1; run=0;
                            break;
                    }
                    if (run==0) break;
                }
                // end of single test loop
                iterations_total+=i;
                err_total=err_total+errors+errors_nack;
                sprintf(user_message,"Test loop #%d complete. %d cycles. %d errors in loop. %d NACK errors in loop",loop_number,i,errors,errors_nack);
                kr_message(MSG_INFO,user_message);
            }
            // all test loops finished
            sprintf(user_message,"Finished Custom Test #%d using SMBus slave addr 0x%02X. Total iterations: %d. Total number of errors: %d",test_number,addr,iterations_total,err_total);
            kr_message(MSG_INFO,user_message);
            break;
        case 3:
            addr=slave_addresses[0];
            if (addr<0) {
                sprintf(user_message,"No SMBus slave address defined");
                kr_message(MSG_ERROR,user_message);
                status|=ERR_ADDR;
                return;
            }
            sprintf(user_message,"Run Custom Test #%d using SMBus slave addr 0x%02X",test_number,addr);
            kr_message(MSG_INFO,user_message);
            sprintf(user_message,"Write two distinct values into sync_osc_trimf");
            kr_message(MSG_INFO,user_message);
            cycles_per_loop=10; run=1;

            i2c_cmd[0]=0xD0; i2c_cmd[1]=0x02; i2c_cmd[2]=0x14;    // AMBAREG_ADDR 0x1402 (sync_osc_trimf)
            switch (kr_i2c_raw_write_wb(addr, 3)) {
                case I2C_OK:
                    break;
                default:
                    run=0;
                    break;
            }
            i2c_cmd[0]=0xD1; i2c_cmd[1]=0x00;                   // AMBAREG_INCR
            switch (kr_i2c_raw_write_wb(addr, 2)) {
                case I2C_OK:
                    break;
                default:
                    run=0;
                    break;
            }

            for (loop_number=0;loop_number<nof_loops && run;loop_number++) {
                for (i=0;i<cycles_per_loop && run;i++) {
                    i2c_cmd[0]=0xD3; i2c_cmd[1]=0x58; i2c_cmd[2]=0x00;    // AMBAREG_W sync_osc_trimf=0x58 (-10%)
                    switch (kr_i2c_raw_write_wb(addr, 3)) {
                        case I2C_OK:
                            break;
                        default:
                            run=0;
                            break;
                    }
                    dac.write_u16( (unsigned int)((float)(1.0/3.3)*0xFFFF) );  // 1V, AFTER we instructed device to trim osc down
                    wait_ms(1);
                    i2c_cmd[0]=0xD3; i2c_cmd[1]=0xA8; i2c_cmd[2]=0x00;    // AMBAREG_W sync_osc_trimf=0xA8 (+10%)
                    switch (kr_i2c_raw_write_wb(addr, 3)) {
                        case I2C_OK:
                            break;
                        default:
                            run=0;
                            break;
                    }
                    dac.write_u16( (unsigned int)((float)(2.0/3.3)*0xFFFF) );  // 1V, AFTER we instructed device to trim osc up
                    wait_ms(1);
                }
                sprintf(user_message,"Test loop #%d complete. %d toggle cycles",loop_number,i);
                kr_message(MSG_INFO,user_message);
            }
            sprintf(user_message,"Custom Test #%d using SMBus slave addr 0x%02X completed",test_number,addr);
            kr_message(MSG_INFO,user_message);
            i2c_cmd[0]=0xD3; i2c_cmd[1]=0x80; i2c_cmd[2]=0x00;    // AMBAREG_W sync_osc_trimf=0x80 (default)
            switch (kr_i2c_raw_write_wb(addr, 3)) {
                case I2C_OK:
                    break;
                default:
                    run=0;
                    break;
            }
            dac.write_u16( (unsigned int)((float)(0.0/3.3)*0xFFFF) );  // 0V
            wait_ms(1);
            break;
        case 4:
            addr=slave_addresses[0];
            if (addr<0) {
                sprintf(user_message,"No SMBus slave address defined");
                kr_message(MSG_ERROR,user_message);
                status|=ERR_ADDR;
                return;
            }
            sprintf(user_message,"Run Custom Test #%d using SMBus slave addr 0x%02X",test_number,addr);
            kr_message(MSG_INFO,user_message);
            sprintf(user_message,"ADC offset-DAC linearity test (linear ramp-up and ramp-down at a rate of 1lsb-step/1ms (acsr_afe_trim_cmp_stg1 addr 0x200E)");
            kr_message(MSG_INFO,user_message);
            sprintf(user_message,"This test assumes that FW-routine RT offset calibration has been disabled:");
            kr_message(MSG_INFO,user_message);
            sprintf(user_message,"(sariu_sample_update_irq_mask=0x00, sariu_adc_cal_ctrl=0x00)");
            kr_message(MSG_INFO,user_message);
            cycles_per_loop=1000; run=1;

            i2c_cmd[0]=0xD0; i2c_cmd[1]=0x0E; i2c_cmd[2]=0x20;    // AMBAREG_ADDR 0x200E (acsr_afe_trim_cmp_stg1)
            switch (kr_i2c_raw_write_wb(addr, 3)) {
                case I2C_OK:
                    break;
                default:
                    run=0;
                    break;
            }
            i2c_cmd[0]=0xD1; i2c_cmd[1]=0x00;                   // AMBAREG_INCR
            switch (kr_i2c_raw_write_wb(addr, 2)) {
                case I2C_OK:
                    break;
                default:
                    run=0;
                    break;
            }

            for (loop_number=0;loop_number<nof_loops && run;loop_number++) {
                for (i=0;i<cycles_per_loop && run;i++) {
                    dac_offset_value=0;
                    dac_offset_direction=0;
                    dac.write_u16( (unsigned int)((float)(3.0/3.3)*0xFFFF) );
                    wait_us(500);
                    dac.write_u16( (unsigned int)((float)(1.0/3.3)*0xFFFF) );
                    wait_us(500);
                    while (1) {
                        if (dac_offset_direction==0) {
                            // direction upwards, we're increasing offset DAC current
                            if (dac_offset_value<0x3f) {
                                dac_offset_value++;
                            }
                            else if (dac_offset_value>0x41) {
                                dac_offset_value--;
                            }
                            else if (dac_offset_value==0x3f) {
                                dac_offset_value--;
                                dac_offset_direction=1;
                            }
                            else {
                                // dac_offset_value has to be 0x40 or 0x41
                                dac_offset_value=0;
                            }
                        } else {
                            // direction downwards, we're reducing offset DAC current
                            if (dac_offset_value==0x0) {
                                dac_offset_value=0x41;
                            }
                            else if (dac_offset_value<=0x3f) {
                                dac_offset_value--;
                            }
                            else if (dac_offset_value==0x7f) {
                                dac_offset_value--;
                                dac_offset_direction=0;
                            }
                            else {
                                dac_offset_value++;
                            }
                        }
    
                        i2c_cmd[0]=0xD3; i2c_cmd[1]=(dac_offset_value&0xFF); i2c_cmd[2]=0x00;
                        switch (kr_i2c_raw_write_wb(addr, 3)) {
                            case I2C_OK:
                                break;
                            default:
                                run=0;
                                break;
                        }
                        dac.write_u16( (unsigned int)((float)(1.5/3.3)*0xFFFF) );  // 1V, AFTER we instructed device to trim osc down
                        wait_us(100);
                        dac.write_u16( (unsigned int)((float)(1.0/3.3)*0xFFFF) );  // 1V, AFTER we instructed device to trim osc up
                        wait_us(900);
    
                        //sprintf(user_message,"Writing value #0x%X into acsr_afe_trim_cmp_stg1, counting %s",dac_offset_value,(dac_offset_direction==0?"up":"down"));
                        //kr_message(MSG_INFO,user_message);
                        
                        if ( dac_offset_value==0x00 && dac_offset_direction==0) {
                            break;
                        }
                    }
                }
                sprintf(user_message,"Test loop #%d complete. %d toggle cycles",loop_number,i);
                kr_message(MSG_INFO,user_message);
            }
            sprintf(user_message,"Custom Test #%d using SMBus slave addr 0x%02X completed",test_number,addr);
            kr_message(MSG_INFO,user_message);
            break;

        case 9:
            sprintf(user_message,"Run Custom Test #%d: DAC Test (1V to 2V, 10us settle %d times",test_number,nof_loops);
            kr_message(MSG_INFO,user_message);
            err_total=0;
            for (iterations_total=0;iterations_total<nof_loops;iterations_total++) {
                dac.write_u16( (unsigned int)((float)(1.0/3.3)*0xFFFF) );  // 1V
                wait_us(10);
                dac.write_u16( (unsigned int)((float)(2.0/3.3)*0xFFFF) );  // 2V
                wait_us(10);
            }
            sprintf(user_message,"Finished Custom Test #%d. Total iterations: %d. Total number of errors: %d",test_number,iterations_total,err_total);
            kr_message(MSG_INFO,user_message);
            break;
        default:
            sprintf(user_message,"Custom Test #%d not implemented",test_number);
            kr_message(MSG_ERROR,user_message);
            break;
    }
}

// raw I2C communication routines
kr_i2c_errors kr_i2c_raw_write_wb (int addr, int nof_bytes) {
    // SMBus write word (or byte) protocol
    // expects a populated i2c_cmd
    if (i2c_host.write(addr<<1,i2c_cmd,nof_bytes,0)==0) {
        // all went fine
        return I2C_OK;
    } else {
        // oops, error
        return I2C_TNACK;
    }
}

// raw I2C communication routines
kr_i2c_errors kr_i2c_raw_read_wb (int addr, int nof_bytes) {
    // SMBus read word (or byte) protocol
    // expects a populated i2c_cmd
    if (i2c_host.write(addr<<1,i2c_cmd,1,1)==0) {
        // command has been written, Sr generated
    } else {
        // oops, error
        return I2C_TNACK;
    }
    if (i2c_host.read(addr<<1,i2c_buf,nof_bytes,0)==0) {
        // data has been received successfully
    } else {
        // oops, NACK during reception
        return I2C_RNACK;
    }
    return I2C_OK;
}
