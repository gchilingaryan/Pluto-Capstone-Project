//MIT License
//
//Copyright (c) 2018 tvelliott
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#include <iio.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <complex.h>

#include "timers.h"

#include "liquid/liquid.h"

#include "ofdm_conf.h"
#include "ofdm_tx.h"
#include "pluto.h"
#include "ethernet.h"
#include "tap_device.h"

static unsigned int gmsk_M;
static unsigned int gmsk_cp_len;
static unsigned int gmsk_taper_len;
static unsigned int ofdm_payload_len;

static float complex gmsk_symbol_buffer[OFDM_M + CP_LEN];   // time-domain buffer
static unsigned char gmsk_header[8];            // header data
static unsigned char gmsk_payload[32*1024];
static unsigned char ofdm_p[OFDM_M];                 // subcarrier allocation (null/pilot/data)
static int gmsk_last_symbol;
static int ofdm_index=0;

crc_scheme check = LIQUID_CRC_32;
fec_scheme fec0  = LIQUID_FEC_NONE;
fec_scheme fec1  = LIQUID_FEC_NONE;

static int i;

static gmskframegen gmsk_fg;

static timer_obj *timer1;
static uint32_t out_pid;
const uint32_t charon_ack_magic = CHARON_ACK_MAGIC;
const int charon_added_length = 4 + sizeof(eth_hdr);//charon frame adds 1 eth_hdr to the batframe.
//then we add 4 pid bytes for detecting and dropping re-trans

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void init_ofdm_tx() {

    gmsk_fg = gmskframegen_create();

    timer1 = create_timer();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int do_ofdm_tx( uint8_t *buffer, int len, int is_retrans, int do_dump_rx, int is_broadcast, uint8_t *dst_gmsk0_mac, uint32_t pid ) {

    long long elapsed;
    charon_frame *charonframe = (charon_frame *) &gmsk_payload[0];
    eth_hdr *ethhdr_charon = (eth_hdr *) &(charonframe->ethhdr_charon);

    out_pid = pid;

    int tlen = len;
    tlen += charon_added_length;

    if( memcmp( dst_gmsk0_mac, ofdm0_mac, 6)==0) return 0; //don't send to ourself

    //fill-in the charon portion of the frame
    //first 4 bytes are either pid  or ack magic
    if(len>0) {
        memcpy( &(charonframe->first_four) , &out_pid, 4);  //normal frame
    }
    else {
        //charonframe->first_four = htonl( CHARON_ACK_MAGIC );  //ack frame
        memcpy( gmsk_payload, dst_gmsk0_mac, 6 );  //make ack 6-bytes
        tlen = 6;
        goto do_send;
    }

    ethhdr_charon->eth_type = htons( ETH_CHARON_TYPE );

    if( is_broadcast ) {
        memcpy( &(ethhdr_charon->dst_mac), mac_all_one, 6);
    }
    else {
        memcpy( &(ethhdr_charon->dst_mac), dst_gmsk0_mac, 6);
    }

    memcpy( &(ethhdr_charon->src_mac), ofdm0_mac, 6);

    //fill the rest of ofdm_payload with the buffer skipping past charon offset bytes
    //note that len==0 for ACKS, so we use tlen to tx
    for(i=0;i<len;i++) {
        gmsk_payload[charon_added_length+i] = buffer[i];
    }

    do_send:
    timer_reset(timer1);
    gmskframegen_assemble(gmsk_fg, gmsk_header, gmsk_payload, tlen, check, fec0, fec1);
    elapsed = timer_elapsed_usec(timer1);

    //fprintf(stderr, ", frame assembly time: %lld usec", elapsed);

    timer_reset(timer1);
    while(1) {

        gmsk_last_symbol = gmskframegen_write(gmsk_fg, gmsk_symbol_buffer, (OFDM_M+CP_LEN) );
        pluto_transmit(gmsk_symbol_buffer, (OFDM_M+CP_LEN), do_dump_rx, gmsk_last_symbol);

        if(gmsk_last_symbol) {
            elapsed = timer_elapsed_usec(timer1);
            //fprintf(stderr, ", frame tx time: %lld usec", elapsed);
            return 0;
        }
    }
}
