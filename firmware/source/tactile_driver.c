#include "tactile_driver.h"
#include "uart.h"
#include "utils.h"
#include "settings.h"
#include "cmd.h"
#include "radio.h"
#include "sclock.h"
#include "timer.h"
#include "dfmem.h"
#include "pid-ip2.5.h"
#include <string.h>

#if defined(__TACTILE_OVER_UART)

        #define TACTILEUART             1

	// UART Configuration
	#define U2BAUD    		9  // For 1 Mbps (HIGH SPEED)
	#define U1BRGH  		1
#endif


static unsigned char tx_idx; //tx mode (to radio)
static unsigned char rx_idx; //rx mode (from radio)
static unsigned char TACTILE_ROWS; //number of rows in tactile grid
static unsigned char TACTILE_COLS; //number of columns in tactile grid
static int rx_count; //count received characters
static unsigned char buffer[LARGE_BUFFER]; //working buffer for bytes received from skinproc
static unsigned char fullbuffer[LARGE_BUFFER]; //full buffer for bytes received from skinproc
static unsigned int max_buffer_length; //maximum length of buffer pointer
static unsigned int buffer_length; //current length of buffer pointer
static unsigned int expected_length = 2; //length of current buffer
static unsigned char rxflag = 0;
static unsigned int tactile_src_addr = 0;
static unsigned int exp_len_loc = 1;
static tactileFrame_t fullFrame;
static char streaming = 0;
static float **N;
//static float N[6][3*0xFF*0xFF];
static DfmemGeometryStruct mem_geo;
static unsigned int zeroForces = 0;
static FORCEUNION forces;
static FORCEUNION forceOffsets;

//Initialize UART module and query skinproc tactile grid size
void tactileInit() {
    //unsigned char data[2] = {0};
    //radioSendData(0x3001, 0, CMD_TACTILE, 2, data, 0);
    if (TACTILEUART){
        /// UART2 for SkinProc, 1e5 Baud, 8bit, No parity, 1 stop bit
        unsigned int U2MODEvalue, U2STAvalue, U2BRGvalue;
        U2MODEvalue = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
                      UART_MODE_SIMPLEX & UART_UEN_00 & UART_DIS_WAKE &
                      UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE &
                      UART_BRGH_FOUR & UART_NO_PAR_8BIT & UART_1STOPBIT;
        U2STAvalue  = UART_INT_TX & UART_INT_RX_CHAR &UART_SYNC_BREAK_DISABLED &
                      UART_TX_ENABLE & UART_ADR_DETECT_DIS &
                      UART_IrDA_POL_INV_ZERO; // If not, whole output inverted.
        U2BRGvalue  = 86;//21; //9; // = (40e6 / (4 * 1e5)) - 1 so the baud rate = 100000
        //this value matches SkinProc

        // =3 for 2.5M Baud
        //U2BRGvalue  = 43; // =43 for 230500Baud (Fcy / ({16|4} * baudrate)) - 1
        //U2BRGvalue  = 86; // =86 for 115200 Baud
        //U2BRGvalue  = 1041; // =1041 for 9600 Baud

        OpenUART2(U2MODEvalue, U2STAvalue, U2BRGvalue);

        ConfigIntUART2(UART_TX_INT_EN & UART_TX_INT_PR4 & UART_RX_INT_EN & UART_RX_INT_PR4);
        //EnableIntU2TX;
        DisableIntU2TX;
        EnableIntU2RX;
    }

    tx_idx = TACTILE_TX_IDLE;
    rx_idx = TACTILE_RX_IDLE;

    rx_count = 0;
    TACTILE_ROWS = 0xFF;
    TACTILE_COLS = 0xFF;
    N = (float**)malloc(sizeof(float*) * 6);
    int i;
    for (i = 0; i < 6; i++) {
        N[i] = (float*)malloc(sizeof(float) * 3 * TACTILE_ROWS * TACTILE_COLS);
    }
    clearRXFlag();
    //checkFrameSize();
    max_buffer_length = LARGE_BUFFER;

}

//Query skinproc for size of frame
void checkFrameSize() {
    max_buffer_length = LARGE_BUFFER;
    buffer_length = 0;
    unsigned char length = 1;
    unsigned char test[1];
    rx_idx = TACTILE_MODE_G;
    test[0] = rx_idx;
    expected_length = 2;
    delay_ms(500); //waiting
    delay_ms(500); //for
    delay_ms(500); //skinproc
    delay_ms(500); //to startup
    sendTactileCommand(length, test);
    Nop();
    Nop();
    Nop();
    //blocking wait for skinproc to answer
    while (!checkRXFlag()) {
        Nop();
    }
    if (buffer[0] == rx_idx) {
        TACTILE_ROWS = buffer[2];
        TACTILE_COLS = buffer[3];
        max_buffer_length = TACTILE_ROWS*TACTILE_COLS*2+2;
        buffer_length = 0;

    }
    else {
        char test = buffer[0];
        TACTILE_ROWS = buffer[0];
        TACTILE_COLS = buffer[1];
        max_buffer_length = LARGE_BUFFER;
        buffer_length = 0;
    }

    Nop();
    Nop();
    rx_idx = TACTILE_RX_IDLE;
    clearRXFlag();
    sendCTS();
}


//Callback function when imageproc receives tactile command from radio
void handleSkinRequest(unsigned char length, unsigned char *frame, unsigned int src_addr) {
    tactile_src_addr = src_addr;
    rx_idx = frame[0];
    //unsigned char tempframe[TACTILE_ROWS * TACTILE_COLS * 2 + 1];
    //static unsigned char tempframe[100];
    //buffer = tempframe;
    buffer_length = 0;
    expected_length = 2;
    int i;
    switch (rx_idx) {
        case TACTILE_MODE_G: //query number of rows and columns
            sendTactileCommand(length, frame);
            /*
            //TACTILE_ROWS = 0x00;
            //TACTILE_COLS = 0x00;
            buffer_length = 4;
            //expected_length = 3;
            buffer[0] = rx_idx;
            buffer[1] = 0x02;
            buffer[2] = TACTILE_ROWS;
            buffer[3] = TACTILE_COLS;
            setRXFlag();
            //buffer = rowcol;
            //buffer = tempframe;*/
            break;
        case TACTILE_MODE_A: //sample individual pixel
            sendTactileCommand(length,frame);
            break;
        case TACTILE_MODE_B: //sample frame
            sendTactileCommand(length,frame);
            break;
        case TACTILE_MODE_C: //poll pixel
            sendTactileCommand(length,frame);
            break;
        case TACTILE_MODE_E: //start scan
            sendTactileCommand(length,frame);
            break;
        case TACTILE_MODE_F: //stop scan
            rx_idx = TACTILE_RX_IDLE;
            sendTactileCommand(length,frame);
            break;
        case TACTILE_MODE_S: //turn streaming on or off
            streaming = frame[1];
            unsigned char temp[3] = {rx_idx,1,streaming};
            radioSendData(tactile_src_addr, 0, CMD_TACTILE, sizeof(temp), temp, 0);
            rx_idx = TACTILE_RX_IDLE;
            break;
        case TACTILE_MODE_T: //test frame
            buffer_length = 0;
            expected_length = max_buffer_length;
            //buffer = tempframe;
            sendTactileCommand(length,frame);
            break;
        case TACTILE_MODE_L: //load force-torque calibration parameters
            Nop();
            float temp_f[6];
            memcpy(temp_f,&frame[2],6*sizeof(float));
            int n_ind;
            for (n_ind = 0; n_ind < 6; n_ind++) {
                N[n_ind][frame[1]] = temp_f[n_ind]; //frame[1] indicates index [0-ROWS*COLS*3)
            }
            //char* test3 = (char*)&temp_f;
            //unsigned char temp1[6] = {rx_idx,sizeof(float),test3[0],test3[1],test3[2],test3[3]};
            //radioSendData(tactile_src_addr, 0, CMD_TACTILE, sizeof(temp1), temp1, 0);
            
            
            rx_idx = TACTILE_RX_IDLE;
            break;
        case 'Y': //write N to dfmem
            dfmemGetGeometryParams(&mem_geo);
            int N_pages = (sizeof(N)/mem_geo.bytes_per_page) + 1; //how many pages needed to hold N
            //for (i = N_pages; i > 0; i--) {
            //    dfmemWrite(N + mem_geo.bytes_per_page*(N_pages-i),i > 1 ? mem_geo.bytes_per_page : sizeof(N)-mem_geo.bytes_per_page*(N_pages-1) ,mem_geo.max_pages-i,0,0);
            //}
            dfmemWrite(N,sizeof(N),mem_geo.max_pages-1,0,0);
            rx_idx = TACTILE_RX_IDLE;
            break;

        case 'Z': 
            /*//read N from dfmem
            dfmemGetGeometryParams(&mem_geo);
            int N_pages1 = (sizeof(N)/mem_geo.bytes_per_page) + 1; //how many pages needed to hold N
            //for (i = N_pages1; i > 0; i--) {
            //    dfmemRead(mem_geo.max_pages-i, 0, i > 1 ? mem_geo.bytes_per_page : sizeof(N)-mem_geo.bytes_per_page*(N_pages1-1), N + mem_geo.bytes_per_page*(N_pages1-i));
            //}
            dfmemRead(mem_geo.max_pages-1, 0, sizeof(N), N);
            */
            rx_idx = TACTILE_RX_IDLE;
            /*DisableIntT1;
            int pwm = frame[1]+(frame[2]<<8);
            tiHSetDC(1,pwm);*/
            
            //zero forces
            if (zeroForces == 0) {
                zeroForces = 1;
            }
            
            break;
        default:
            sendTactileCommand(length,frame);
            break;
    }
    //blocking wait for skinproc to answer
    /*while (buffer_length < expected_length) {
        Nop();
    }
    handleSkinData(buffer_length, buffer);
    rx_idx = TACTILE_RX_IDLE;*/
}

//Send command over UART to skinproc
unsigned char sendTactileCommand(unsigned char length, unsigned char *frame) {
    static int i;
    static unsigned char val;

    tx_idx = frame[0];
    for (i = 0; i < length; i++) {
        val = frame[i];
        if (TACTILEUART) {
            while(BusyUART2());
            WriteUART2(val);
        }
    }
    return 1;
}

//transmit skin data over radio, cap data length if over threshhold
void handleSkinData(unsigned int length, unsigned char *data){
    //Cannot handle any length over 114
    if (length > 114) {
        length = 114;
    }
    radioSendData(tactile_src_addr, 0, CMD_TACTILE, length, data, 0);
    //data = data + length/2 - 1;
    //data[0] = rx_idx;
    //radioSendData(RADIO_DST_ADDR, 0, CMD_TACTILE, length/2 +1, data, 0);
}

void checkTactileBuffer(){
    if (checkRXFlag()) {

        if (streaming || rx_idx != TACTILE_MODE_E) {
            if (rx_idx == TACTILE_MODE_E) {
                Nop();
                Nop();
            }
            handleSkinData(buffer_length, fullbuffer);
        }

        if (rx_idx == TACTILE_MODE_B || rx_idx == TACTILE_MODE_E) {
            int i = 0;
            for (i = 0; i < (fullbuffer[1]-4)/2; i++) {
                fullFrame.frame[i] = ((unsigned int)(fullbuffer[i*2+2])) + (((unsigned int)(fullbuffer[i*2+3]))<<8);
            }
            //calcForces(&fullFrame, &forces);
            //tactilePID(&forces);

            if (0) {
                //send back force data here
                unsigned char forcedata[2+6*sizeof(float)];
                forcedata[0] = 'F';
                forcedata[1] = 6*sizeof(float);

                for (i=0; i < 6; i++) {
                    memcpy(&forcedata[2+i*sizeof(float)],&forces.F[i],sizeof(float));
                }
                handleSkinData(sizeof(forcedata), forcedata);
            }
        }

        expected_length = 2;
        buffer_length = 0;
        if (rx_idx == TACTILE_MODE_G){
            TACTILE_ROWS = fullbuffer[2];
            TACTILE_COLS = fullbuffer[3];
            max_buffer_length = TACTILE_ROWS*TACTILE_COLS*2+2;
        }

        if (rx_idx != TACTILE_MODE_E){
            rx_idx = TACTILE_RX_IDLE;
        }
        clearRXFlag();
        sendCTS();
    }
}

void calcForces(tactileFrame_t* sensor, FORCEUNION* forces){

    Nop();
    int i,j;
    float A[3*TACTILE_ROWS*TACTILE_COLS];

    for (i = 0; i < TACTILE_ROWS*TACTILE_COLS; i++) {
        A[i*3] = (float) sensor->frame[i];
        A[i*3+1] = A[i*3]*A[i*3];
        A[i*3+2] = A[i*3+1]*A[i*3];
    }
    
    for (j = 0; j < 6; j++) {
        forces->F[j] = 0;
        for (i = 0; i < 3*TACTILE_ROWS*TACTILE_COLS; i++) {
            forces->F[j] += A[i]*N[j][i];
        }
    }
    Nop();
}

void tactilePID(FORCEUNION* forces) {
    Nop();
    float FxThreshForward = 0.6;
    float FxThreshBack = -0.6;
    int freq[2];
    if (forces->forces.Fx < FxThreshBack) {
        freq[0] = -1000;
        freq[1] = -1000;
        setLegFreqs(2,freq);
    } else if (forces->forces.Fx > FxThreshForward) {
        freq[0] = 1000;
        freq[1] = 1000;
        setLegFreqs(2,freq);
    }
}

void setRXFlag(){
    rxflag = 0x01;
}

void clearRXFlag(){
    rxflag = 0x00;
}

unsigned char checkRXFlag(){
    return rxflag;
}

void sendCTS(){
    unsigned char frame[1];
    frame[0] = CTS;
    sendTactileCommand(1,frame);
}

int tactileReturnFrame(tactileFrame_t* dst){
    if (dst != NULL) {
        memcpy(dst->frame, fullFrame.frame, TACTILE_ROWS*TACTILE_COLS*sizeof(unsigned int));
        return 0;
    } else {
        return 1;
    }
}

int setLegFreqs(int numfreqs, int* freq) {
    //freq is in milli-Hz
    int i, j;
    int interval[NUM_PIDS][NUM_VELS], delta[NUM_PIDS][NUM_VELS], vel[NUM_PIDS][NUM_VELS], period[NUM_PIDS];
    int setdelta = 0x4000;
    int onceFlag = 0;
    for (i = 0; i < numfreqs; i++) {
        //upper bound
        int upperbound = 8000; //don't want to go above 8Hz
        if (freq[i] > upperbound) {
            freq[i] = upperbound;
        } else if (freq[i] < -upperbound) {
            freq[i] = -upperbound;
        } else if (freq[i] < 31 && freq[i] > -31) { //min value 31 because 1000000/30 > 2^15
            if (freq[i] >= 0) {
                freq[i] = 31;
            } else {
                freq[i] = -31;
            }
        }
        period[i] = 1000000/freq[i]; //convert to ms
        for (j = 0; j < NUM_VELS; j++) {
            interval[i][j] = period[i]/NUM_VELS;
            delta[i][j] = setdelta;
            vel[i][j] = ((long)delta[i][j])*freq[i]*NUM_VELS/1000000;
            if (period[i] < 0) {
                //interval must always be positive
                //flip delta if negative frequency
                //vel will already be negative
                interval[i][j] = -interval[i][j];
                delta[i][j] = -delta[i][j];
            }
        }
        setPIDVelProfile(i, interval[i], delta[i], vel[i], onceFlag);
    }
    return 0;
}

//read data from the UART, and fill each byte into the buffer
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {
    unsigned char rx_byte;

    CRITICAL_SECTION_START
    LED_1 = ~LED_1;
    while(U2STAbits.URXDA) {
        rx_byte = U2RXREG;
        if (buffer_length == 0 && rx_byte != rx_idx) {
            Nop();  //first byte received isn't rx_idx
        } else {
            buffer[buffer_length] = rx_byte;
            if (buffer_length == exp_len_loc) {
                if (rx_byte == 0xFF) {
                    exp_len_loc += 1;
                }
                expected_length += (unsigned int) rx_byte;
            }
            ++buffer_length;
        }


    }
    if (buffer_length >= expected_length) { //captured a full packet
        memcpy(fullbuffer, buffer, expected_length);
        setRXFlag();
        exp_len_loc = 1;
    }

    if(U2STAbits.OERR) {
        U2STAbits.OERR = 0;
    }

    _U2RXIF = 0;
    //LED_1 = 0;
    CRITICAL_SECTION_END
}


void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
    //unsigned char tx_byte;
    CRITICAL_SECTION_START
    LED_3 = 1;
    
    _U2TXIF = 0;
    LED_3 = 0;
    CRITICAL_SECTION_END
}
