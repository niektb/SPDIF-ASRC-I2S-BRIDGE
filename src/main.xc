// Copyright (c) 2016, XMOS Ltd, All rights reserved
//Standard includes
#include <xs1.h>
#include <platform.h>
#include <string.h>
#include <xscope.h>

//Supporting libraries
#include "src.h"
#include "spdif.h"
#include "i2s.h"
#include "gpio.h"
#include "assert.h"

//Application specific includes
#include "main.h"
#include "block_serial.h"
#include "app_config.h"     //General settings

//Debug includes
#include "debug_print.h" //Enabled by -DDEBUG_PRINT_ENABLE=1 in Makefile

//These port assignments all correspond to XU216 multichannel audio board 2V0
//The port assignments can be changed for a different port map.

#define AUDIO_TILE                       0

port pled = XS1_PORT_4C;


//port port_i2s_mclk                       = on tile[AUDIO_TILE]: XS1_PORT_1F;
clock clk_mclk                           = on tile[AUDIO_TILE]: XS1_CLKBLK_2;
out buffered port:32 port_i2s_bclk       = on tile[AUDIO_TILE]: XS1_PORT_1M;
out buffered port:32 port_i2s_wclk       = on tile[AUDIO_TILE]: XS1_PORT_1N;
clock clk_i2s                            = on tile[AUDIO_TILE]: XS1_CLKBLK_1;
out buffered port:32 ports_i2s_dac[1]    = on tile[AUDIO_TILE]: {XS1_PORT_1O};

#define SPDIF_TILE                       0
port port_spdif_rx                       = on tile[SPDIF_TILE]: XS1_PORT_1D;
clock clk_spdif_rx                       = on tile[SPDIF_TILE]: XS1_CLKBLK_3;

//Application task prototypes. For functionality of these tasks, see comments in implementations below
[[combinable]] void spdif_handler(streaming chanend c_spdif_rx, client serial_transfer_push_if i_serial_in);
unsafe void asrc(server block_transfer_if i_serial2block, client block_transfer_if i_block2serial, client fs_ratio_enquiry_if i_fs_ratio);
[[distributable]] void i2s_handler(server i2s_callback_if i2s, client serial_transfer_pull_if i_serial_out);
[[combinable]]void rate_server(client sample_rate_enquiry_if i_spdif_rate, client sample_rate_enquiry_if i_output_rate, server fs_ratio_enquiry_if i_fs_ratio[ASRC_N_INSTANCES]);

int main(void){
    debug_printf("Hello World!\n");

    serial_transfer_push_if i_serial_in;
    block_transfer_if i_serial2block[ASRC_N_INSTANCES];
    block_transfer_if i_block2serial[ASRC_N_INSTANCES];
    serial_transfer_pull_if i_serial_out;
    sample_rate_enquiry_if i_sr_input, i_sr_output;
    fs_ratio_enquiry_if i_fs_ratio[ASRC_N_INSTANCES];
    interface i2s_callback_if i_i2s;
    streaming chan c_spdif_rx;

    par{
        spdif_rx(c_spdif_rx, port_spdif_rx, clk_spdif_rx, DEFAULT_FREQ_HZ_SPDIF);
        [[combine, ordered]] par{
            rate_server(i_sr_input, i_sr_output, i_fs_ratio);
            spdif_handler(c_spdif_rx, i_serial_in);
        }

        serial2block(i_serial_in, i_serial2block, i_sr_input);
        unsafe { par (int i=0; i<ASRC_N_INSTANCES; i++) asrc(i_serial2block[i], i_block2serial[i], i_fs_ratio[i]);}
        unsafe { par{[[distribute]] block2serial(i_block2serial, i_serial_out, i_sr_output);}}

        {
        	// 99999744Hz / 4069 = 24.576MHz
        	//configure_clock_ref(clk_mclk, 4069);
        	configure_clock_rate(clk_mclk, 24576, 1000);

            start_clock(clk_mclk);
            debug_printf("Starting I2S\n");
            i2s_master(i_i2s, ports_i2s_dac, 1, null, 0, port_i2s_bclk, port_i2s_wclk, clk_i2s, clk_mclk);
        }

        [[distribute]] i2s_handler(i_i2s, i_serial_out);
    }
    return 0;
}


//Shim task to handle setup and streaming of SPDIF samples from the streaming channel to the interface of serial2block
[[combinable]] //Run on same core as other tasks if possible
void spdif_handler(streaming chanend c_spdif_rx, client serial_transfer_push_if i_serial_in)
{
    unsigned index;                             //Channel index
    signed long sample;                         //Sample received from SPDIF

    delay_microseconds(10000);                   //Bug 17263 workaround (race condition in distributable task init)
    while (1) {
        select {
            case spdif_receive_sample(c_spdif_rx, sample, index):
                i_serial_in.push(sample, index);   //Push them into serial to block

            break;
        }

    }
}


//Helper function for converting sample to fs index value
static fs_code_t samp_rate_to_code(unsigned samp_rate){
    unsigned samp_code = 0xdead;
    switch (samp_rate){
    case 44100:
        samp_code = FS_CODE_44;
        break;
    case 48000:
        samp_code = FS_CODE_48;
        break;
    case 88200:
        samp_code = FS_CODE_88;
        break;
    case 96000:
        samp_code = FS_CODE_96;
        break;
    case 176400:
        samp_code = FS_CODE_176;
        break;
    case 192000:
        samp_code = FS_CODE_192;
        break;
    }
    return samp_code;
}

//The ASRC processing task - has it's own logical core to reserve processing MHz
unsafe void asrc(server block_transfer_if i_serial2block, client block_transfer_if i_block2serial, client fs_ratio_enquiry_if i_fs_ratio)
{
    int input_dbl_buf[2][ASRC_CHANNELS_PER_INSTANCE * ASRC_N_IN_SAMPLES];  //Double buffers for to block/serial tasks
    unsigned buff_idx = 0;
    int * unsafe asrc_input = input_dbl_buf[0]; //pointer for ASRC input buffer
    int * unsafe p_out_fifo;                    //C-style pointer for output FIFO

    p_out_fifo = i_block2serial.push(0);        //Get pointer to initial write buffer

    set_core_high_priority_on();                //Give me guarranteed 1/5 of the processor clock i.e. 100MHz

    fs_code_t in_fs_code = samp_rate_to_code(DEFAULT_FREQ_HZ_SPDIF);  //Sample rate code 0..5
    fs_code_t out_fs_code = samp_rate_to_code(DEFAULT_FREQ_HZ_I2S);

    asrc_state_t     asrc_state[ASRC_CHANNELS_PER_INSTANCE]; //ASRC state machine state
    int              asrc_stack[ASRC_CHANNELS_PER_INSTANCE][ASRC_STACK_LENGTH_MULT * ASRC_N_IN_SAMPLES]; //Buffer between filter stages
    asrc_ctrl_t      asrc_ctrl[ASRC_CHANNELS_PER_INSTANCE];  //Control structure
    asrc_adfir_coefs_t asrc_adfir_coefs;                     //Adaptive filter coefficients

    for(int ui = 0; ui < ASRC_CHANNELS_PER_INSTANCE; ui++)
    unsafe {
        //Set state, stack and coefs into ctrl structure
        asrc_ctrl[ui].psState                   = &asrc_state[ui];
        asrc_ctrl[ui].piStack                   = asrc_stack[ui];
        asrc_ctrl[ui].piADCoefs                 = asrc_adfir_coefs.iASRCADFIRCoefs;
    }

    //Initialise ASRC
    unsigned nominal_fs_ratio = asrc_init(in_fs_code, out_fs_code, asrc_ctrl, ASRC_CHANNELS_PER_INSTANCE, ASRC_N_IN_SAMPLES, ASRC_DITHER_SETTING);

    int do_dsp_flag = 0;                   //Flag to indiciate we are ready to process. Minimises blocking on push case below

    while(1){
        select{
            case i_serial2block.push(const unsigned n_samps) -> int * unsafe new_buff_ptr:
                asrc_input = input_dbl_buf[buff_idx];   //Grab address of freshly filled buffer
                do_dsp_flag = 1;                        //We have a fresh buffer to process
                buff_idx ^= 1;                          //Flip double buffer for filling
                new_buff_ptr = input_dbl_buf[buff_idx]; //Return pointer for serial2block to fill
            break;

            case i_fs_ratio.new_sr_notify():            //Notification from SR manager that we need to initialise ASRC
                in_fs_code = samp_rate_to_code(i_fs_ratio.get_in_fs());         //Get the new SRs
                out_fs_code = samp_rate_to_code(i_fs_ratio.get_out_fs());
                debug_printf("New rate in SRC in=%d, out=%d\n", in_fs_code, out_fs_code);
                nominal_fs_ratio = asrc_init(in_fs_code, out_fs_code, asrc_ctrl, ASRC_CHANNELS_PER_INSTANCE, ASRC_N_IN_SAMPLES, ASRC_DITHER_SETTING);
            break;

            do_dsp_flag => default:                    //Do the sample rate conversion
                //port_debug <: 1;                     //debug
                unsigned n_samps_out;
                fs_ratio_t fs_ratio = i_fs_ratio.get_ratio(nominal_fs_ratio); //Find out how many samples to produce

                //Run the ASRC and pass pointer of output to block2serial
                n_samps_out = asrc_process((int *)asrc_input, (int *)p_out_fifo, fs_ratio, asrc_ctrl);
                p_out_fifo = i_block2serial.push(n_samps_out);   //Get pointer to next write buffer

                do_dsp_flag = 0;                       //Clear flag and wait for next input block
                //port_debug <: 0;                     //debug
                break;
        }
    }//While 1
}//asrc

#define MUTE_MS_AFTER_SR_CHANGE   350    //350ms. Avoids incorrect rate playing momentarily while new rate is detected

//Shim task to handle setup and streaming of I2S samples from block2serial to the I2S module
[[distributable]]
#pragma unsafe arrays   //Performance optimisation of i2s_handler task
void i2s_handler(server i2s_callback_if i2s, client serial_transfer_pull_if i_serial_out)
    {
    unsigned sample_rate = DEFAULT_FREQ_HZ_I2S;
    unsigned mclk_rate = MCLK_FREQUENCY_48;
    unsigned restart_status = I2S_NO_RESTART;
    unsigned mute_counter; //Non zero indicates mute. Initialised on I2S init SR change

    while (1) {
        select {
            case i2s.init(i2s_config_t &?i2s_config, tdm_config_t &?tdm_config):
            	i2s_config.mode = I2S_MODE_I2S;
            	i2s_config.mclk_bclk_ratio = mclk_rate / (sample_rate << 6);
            break;

            //Start of I2S frame
            case i2s.restart_check() -> i2s_restart_t ret:
                ret = restart_status;
            break;

            //Get samples from ADC
            case i2s.receive(size_t index, int32_t sample):
            break;

            //Send samples to DAC
            case i2s.send(size_t index) -> int32_t sample:
                sample = i_serial_out.pull(index);
                if (mute_counter){
                    sample = 0;
                    mute_counter --;
                }
            break;
        }
    }
}


#define SR_TOLERANCE_PPM    5000    //How far the detect_frequency function will allow before declaring invalid in p.p.m.
#define LOWER_LIMIT(freq) (freq - (((long long) freq * SR_TOLERANCE_PPM) / 1000000))
#define UPPER_LIMIT(freq) (freq + (((long long) freq * SR_TOLERANCE_PPM) / 1000000))

static const unsigned sr_range[6 * 3] = {
        44100, LOWER_LIMIT(44100), UPPER_LIMIT(44100),
        48000, LOWER_LIMIT(48000), UPPER_LIMIT(48000),
        88200, LOWER_LIMIT(88200), UPPER_LIMIT(88200),
        96000, LOWER_LIMIT(96000), UPPER_LIMIT(96000),
        176400, LOWER_LIMIT(176400), UPPER_LIMIT(176400),
        192000, LOWER_LIMIT(192000), UPPER_LIMIT(192000) };

//Helper function for rate_server to check for validity of detected sample rate. Takes sample rate as integer
static sample_rate_status_t detect_frequency(unsigned sample_rate, unsigned &nominal_sample_rate)
{
    sample_rate_status_t result = INVALID;
    nominal_sample_rate = 0;
    for (int i = 0; i < 6 * 3; i+=3) {
        if ((sr_range[i + 1] < sample_rate) && (sample_rate < sr_range[i + 2])){
            nominal_sample_rate = sr_range[i];
            result = VALID;
        }
    }
    return result;
}


#define SR_CALC_PERIOD  2000000     //20ms The period over which we count samples to find the rate
                                    //Because we timestamp at 10ns resolution, we get 20000000/10 = 21bits of precision
#define REPORT_PERIOD   500100000   //5.001s. How often we print the rates to the screen for debug. Chosen to not clash with above
#define SR_FRAC_BITS    12          //Number of fractional bits used to store sample rate
                                    //Using 12 gives us 20 bits of integer - up to 1.048MHz SR before overflow
//Below is the multiplier is used to work out SR in 20.12 representation. There is enough headroom in a long long calc
//to support a measurement period of 1s at 192KHz with over 2 order of magnitude margin against overflow
#define SR_MULTIPLIER   ((1<<SR_FRAC_BITS) * (unsigned long long) XS1_TIMER_HZ)
#define SETTLE_CYCLES   3           //Number of measurement periods to skip after SR change (SR change blocks spdif momentarily so corrupts SR calc)

typedef struct rate_info_t{
    unsigned samp_count;            //Sample count over last period
    unsigned time_ticks;            //Time in ticks for last count
    unsigned current_rate;          //Current average rate in 20.12 fixed point format
    sample_rate_status_t status;    //Lock status
    unsigned nominal_rate;          //Snapped-to nominal rate as unsigned integer
} rate_info_t;

//Task that queires the de/serialisers periodically and calculates the number of samples for the SRC
//to produce to keep the output FIFO in block2serial rougly centered. Uses the timestamped sample counts
//requested from serial2block and block2serial and FIFO level as P and I terms
[[combinable]]
#pragma unsafe arrays   //Performance optimisation
void rate_server(client sample_rate_enquiry_if i_spdif_rate, client sample_rate_enquiry_if i_i2s_rate,
        server fs_ratio_enquiry_if i_fs_ratio[ASRC_N_INSTANCES])
{
    rate_info_t spdif_info = {  //Initialise to nominal values for default frequency
            ((DEFAULT_FREQ_HZ_SPDIF * 10000000ULL) / XS1_TIMER_HZ),
            SR_CALC_PERIOD,
            DEFAULT_FREQ_HZ_SPDIF << SR_FRAC_BITS,
            INVALID,
            DEFAULT_FREQ_HZ_SPDIF};

    rate_info_t i2s_info = {    //Initialise to nominal values for default frequency
            ((DEFAULT_FREQ_HZ_I2S * 10000000ULL) / XS1_TIMER_HZ),
            SR_CALC_PERIOD,
            DEFAULT_FREQ_HZ_I2S << SR_FRAC_BITS,
            INVALID,
            DEFAULT_FREQ_HZ_I2S};

    unsigned i2s_buff_level = 0;                //Buffer fill level. Initialise to empty.
    unsigned i2s_buff_size = OUT_FIFO_SIZE;
    unsigned skip_validity = 0;                 //Do SR validity check - need this to allow SR to settle after SR change

    timer t_print;                              //Debug print timers
    int t_print_trigger;

    t_print :> t_print_trigger;
    t_print_trigger += REPORT_PERIOD;

    fs_ratio_t fs_ratio;                        //4.28 fixed point value of how many samples we want SRC to produce
                                                //input fs/output fs. ie. below 1 means inoput faster than output
    fs_ratio_t fs_ratio_old;                    //Last time round value for filtering
    fs_ratio_t fs_ratio_nominal;                //Nominal fs ratio reported by SRC
    timer t_period_calc;                        //Timer to govern sample count periods
    int t_calc_trigger;                         //Trigger comparison for above

    int sample_time_spdif;                      //Used for passing to get_sample_count method by refrence
    int sample_time_i2s;                        //Used for passing to get_sample_count method by refrence

    t_period_calc :> t_calc_trigger;            //Get current time and set trigger for the future
    t_calc_trigger += SR_CALC_PERIOD;

    while(1){
        select{
            //Serve up latest sample count value when required. Note selects over array of interfaces
            case i_fs_ratio[int if_index].get_ratio(unsigned nominal_fs_ratio) -> fs_ratio_t fs_ratio_ret:
                fs_ratio_nominal =  nominal_fs_ratio;   //Allow use outside of this case
                if ((spdif_info.status == VALID) && (i2s_info.status == VALID)){
                    fs_ratio_ret = fs_ratio;    //Pass back calculated value
                }
                else {
                    fs_ratio = nominal_fs_ratio; //Pass back nominal until we have valid rate data
                }

            break;

            //Serve up the input sample rate
            case i_fs_ratio[int if_index].get_in_fs(void) -> unsigned fs:
                fs = spdif_info.nominal_rate;
            break;

            //Serve up the output sample rate
            case i_fs_ratio[int if_index].get_out_fs(void) -> unsigned fs:
                fs = i2s_info.nominal_rate;
            break;

            //Timeout to trigger calculation of new fs_ratio
            case t_period_calc when timerafter(t_calc_trigger) :> int _:
                t_calc_trigger += SR_CALC_PERIOD;

                unsigned samp_count_spdif = i_spdif_rate.get_sample_count(sample_time_spdif); //get spdif sample count;
                unsigned samp_count_i2s   = i_i2s_rate.get_sample_count(sample_time_i2s);     //And I2S
                {i2s_buff_size, i2s_buff_level} = i_i2s_rate.get_buffer_level();

                if (sample_time_spdif){ //If time is non-zero - avoids divide by zero if no input
                    spdif_info.current_rate = (((unsigned long long)samp_count_spdif * SR_MULTIPLIER) / sample_time_spdif);
                }
                else spdif_info.current_rate = 0;
                if (sample_time_i2s){
                    i2s_info.current_rate   = (((unsigned long long)samp_count_i2s * SR_MULTIPLIER) / sample_time_i2s);
                }
                else i2s_info.current_rate = 0;

                //Find lock status of input/output sample rates
                sample_rate_status_t spdif_status_new = detect_frequency(spdif_info.current_rate >> SR_FRAC_BITS, spdif_info.nominal_rate);
                sample_rate_status_t i2s_status_new  = detect_frequency(i2s_info.current_rate >> SR_FRAC_BITS, i2s_info.nominal_rate);

                //If either has changed from invalid to valid, send message to SRC to initialise
                if ((spdif_status_new == VALID && i2s_status_new == VALID) && ((spdif_info.status == INVALID || i2s_info.status == INVALID)) && !skip_validity){
                    for(int i = 0; i < ASRC_N_INSTANCES; i++){
                        i_fs_ratio[i].new_sr_notify();
                    }
                    skip_validity =  SETTLE_CYCLES;  //Don't check on validity for a few cycles as will be corrupted by SR change and SRC init
                    fs_ratio = (unsigned) ((spdif_info.nominal_rate * 0x10000000ULL) / i2s_info.nominal_rate); //Initialise rate to nominal
                }

                if (skip_validity) skip_validity--;

                //Update current sample rate status flags for input and output
                spdif_info.status = spdif_status_new;
                i2s_info.status   = i2s_status_new;

#define BUFFER_LEVEL_TERM   20000   //How much to apply the buffer level feedback term (effectively 1/I term)
#define OLD_VAL_WEIGHTING   5       //Simple low pass filter. Set proportion of old value to carry over


                //Calculate fs_ratio to tell asrc how many samples to produce in 4.28 fixed point format
                int i2s_buffer_level_from_half = (signed)i2s_buff_level - (i2s_buff_size / 2);    //Level w.r.t. half full
                if (spdif_info.status == VALID && i2s_info.status == VALID) {
                    fs_ratio_old = fs_ratio;        //Save old value
                    fs_ratio = (unsigned) ((spdif_info.current_rate * 0x10000000ULL) / i2s_info.current_rate);

                    //If buffer is negative, we need to produce more samples so fs_ratio needs to be < 1
                    //If positive, we need to back off a bit so fs_ratio needs to be over unity to get more samples from asrc
                    fs_ratio = (unsigned) (((BUFFER_LEVEL_TERM + i2s_buffer_level_from_half) * (unsigned long long)fs_ratio) / BUFFER_LEVEL_TERM);
                    //debug_debug_printf("sp=%d\ti2s=%d\tbuff=%d\tfs_raw=0x%x\tfs_av=0x%x\n", spdif_info.current_rate, i2s_info.current_rate, i2s_buffer_level_from_half, fs_ratio, fs_ratio_old);
                    //Apply simple low pass filter
                    fs_ratio = (unsigned) (((unsigned long long)(fs_ratio_old) * OLD_VAL_WEIGHTING + (unsigned long long)(fs_ratio) ) /
                            (1 + OLD_VAL_WEIGHTING));
                }

                //Set Sample rate LEDs
                unsigned spdif_fs_code = samp_rate_to_code(spdif_info.nominal_rate) + 1;
                if (spdif_info.status == INVALID) spdif_fs_code = 0;
                pled <: spdif_fs_code;




            break;

            case t_print when timerafter(t_print_trigger) :> int _:
                t_print_trigger += REPORT_PERIOD;
                //Calculate sample rates in Hz for human readability
#if 1
                debug_printf("spdif rate ave=%d, valid=%d, i2s rate=%d, valid=%d, i2s_buff=%d, fs_ratio=0x%x, nom_fs=0x%x\n",
                        spdif_info.current_rate >> SR_FRAC_BITS, spdif_info.status,
                        i2s_info.current_rate >> SR_FRAC_BITS, i2s_info.status,
                        (signed)i2s_buff_level - (i2s_buff_size / 2), fs_ratio, fs_ratio_nominal);
#endif
            break;
        }
    }
}
