
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <iio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/mman.h>
#include <inttypes.h>
#include <assert.h>
#include <chrono>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <liquid/liquid.h>
#include <string>
#include <vector>

/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

#define IIO_ENSURE(expr) { \
        if (!(expr)) { \
                (void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
                (void) abort(); \
        } \
}

/* RX is input, TX is output */
enum iodev { RX, TX };

/* common RX and TX streaming params */
struct stream_cfg
{
    // Analog banwidth in Hz
    long long bw_hz;
    // Baseband sample rate in Hz
    long long fs_hz;
    // Local oscillator frequency in Hz
    long long lo_hz;
    // Port name
    const char* rfport;
};

struct fsk_config_params
{
    int fsk_k;
    int fsk_m;
    double fsk_bandwidth;
} fsk_config;

modulation_scheme psk = LIQUID_MODEM_PSK2;
struct psk_config_params
{
    int psk_k;
    int psk_m;
    double psk_bandwidth;
} psk_config;

struct gmsk_config_params
{
    int gmsk_k;
    int gmsk_m;
    float gmsk_bandwidth;
} gmsk_config;

/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer *rxbuf = NULL;
static struct iio_buffer *txbuf = NULL;
// Streaming devices
struct iio_device *tx = NULL;
struct iio_device *rx = NULL;

unsigned int tx_buffer_size = 10 * 640000;
unsigned int rx_buffer_size = 10 * 640000;
double iteration_count = 0;
double modul_type = 0;

struct received_data
{
    uint16_t* data;
    int size;
} rx_data;

bool rx_enabled = false;

/* cleanup and exit */
static void shutdown()
{
    //printf("* Destroying buffers\n");
    if (rxbuf) { iio_buffer_destroy(rxbuf); }
    if (txbuf) { iio_buffer_destroy(txbuf); }

    //printf("* Disabling streaming channels\n");
    if (rx0_i) { iio_channel_disable(rx0_i); }
    if (rx0_q) { iio_channel_disable(rx0_q); }
    if (tx0_i) { iio_channel_disable(tx0_i); }
    if (tx0_q) { iio_channel_disable(tx0_q); }

    //printf("* Destroying context\n");
    if (ctx) { iio_context_destroy(ctx); }
    exit(0);
}

int16_t convert_float_to_int(float f)
{
    float scalar = 32767.0;
    float a;
    a = scalar * f;
    if (a < SHRT_MIN) {
        a = SHRT_MIN;
    } else if (a > SHRT_MAX) {
        a = SHRT_MAX;
    }
    return std::rintf(a);
}


/* check return value of attr_write function */
static void errchk(int v, const char* what)
{
    if (v < 0) {
        fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what);
        shutdown();
    }
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
        errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
        errchk(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)
{
        snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
        return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(struct iio_context *contex)
{
    struct iio_device *dev =  iio_context_find_device(contex, "ad9361-phy");
    IIO_ENSURE(dev && "No ad9361-phy found");
    return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(struct iio_context *contex, enum iodev d, struct iio_device **dev)
{
    switch (d) {
        case TX: *dev = iio_context_find_device(contex, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
        case RX: *dev = iio_context_find_device(contex, "cf-ad9361-lpc");  return *dev != NULL;
        default: IIO_ENSURE(0); return false;
    }
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(__notused struct iio_context *contex,
                                 enum iodev d, struct iio_device *dev,
                                 int chid, struct iio_channel **chn)
{
        *chn = iio_device_find_channel(dev,
                                       get_ch_name("voltage", chid), d == TX);
        if (!*chn)
                *chn = iio_device_find_channel(dev,
                       get_ch_name("altvoltage", chid), d == TX);
        return *chn != NULL;
}

static bool get_phy_chan(struct iio_context *contex, enum iodev d,
                         int chid, struct iio_channel **chn)
{
    switch (d) {
        case RX: {
            *chn = iio_device_find_channel(get_ad9361_phy(contex),
                                           get_ch_name("voltage", chid), false);
            return *chn != NULL;
        }
        case TX: {
            *chn = iio_device_find_channel(get_ad9361_phy(contex),
                                           get_ch_name("voltage", chid), true);
            return *chn != NULL;
        }
        default: {
            IIO_ENSURE(0);
        return false;
    }
    }
}

static bool get_lo_chan(struct iio_context *contex,
                        enum iodev d, struct iio_channel **chn)
{
    switch (d) {
        // LO chan is always output, i.e. true
        case RX: {
            *chn = iio_device_find_channel(get_ad9361_phy(contex), get_ch_name("altvoltage", 0), true);
            return *chn != NULL;
        }
        case TX: {
            *chn = iio_device_find_channel(get_ad9361_phy(contex), get_ch_name("altvoltage", 1), true);
            return *chn != NULL;
        }
        default: {
            IIO_ENSURE(0);
            return false;
        }
    }
}

bool cfg_ad9361_streaming_ch(struct iio_context *contex,
                             struct stream_cfg *cfg, enum iodev type, int chid)
{
    struct iio_channel *chn = NULL;
    // Configure phy and lo channels
    printf("* Acquiring AD9361 phy channel %d\n", chid);
    if (!get_phy_chan(contex, type, chid, &chn)) {     return false; }
    wr_ch_str(chn, "rf_port_select",     cfg->rfport);
    wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
    wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

    // Configure LO channel
    //printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
    if (!get_lo_chan(contex, type, &chn)) {
        return false;
    }
    wr_ch_lli(chn, "frequency", cfg->lo_hz);
    return true;
}

void modulate_data(liquid_float_complex x_out[], int data, fskmod mod)
{
    //std::cout << "data " << data << std::endl;
    //std::cout << "fsk_pointer " << x_out << std::endl;
    fskmod_modulate(mod, data, x_out);
    //std::cout << "modulated " << std::endl;
}

unsigned int demodulate_signal(liquid_float_complex frame[], fskdem dem)
{
    unsigned int x = fskdem_demodulate(dem, frame);
    return x;
}

void init(long long rx_bw_hz = 20, long long rx_fs_hz = 15,
          double rx_lo_hz = 2.5,
          long long tx_bw_hz = 20, long long tx_fs_hz = 15,
          double tx_lo_hz = 2.5)
{
    // Stream configurations
    struct stream_cfg rxcfg;
    struct stream_cfg txcfg;

    // RX stream config
    rxcfg.bw_hz = MHZ(rx_bw_hz);   // 2 MHz rf bandwidth
    rxcfg.fs_hz = MHZ(rx_fs_hz);   // 31 MS/s rx sample rate
    rxcfg.lo_hz = GHZ(rx_lo_hz); // 2.5 GHz rf frequency
    rxcfg.rfport = "A_BALANCED"; // port A (select for rf freq.)

    // TX stream config
    txcfg.bw_hz = MHZ(tx_bw_hz); // 1.5 MHz rf bandwidth
    txcfg.fs_hz = MHZ(tx_fs_hz);   // 31 MS/s tx sample rate
    txcfg.lo_hz = GHZ(tx_lo_hz); // 2.5 GHz rf frequency
    txcfg.rfport = "A"; // port A (select for rf freq.)

    //printf("* Acquiring IIO context\n");
    IIO_ENSURE((ctx = iio_create_default_context()) && "No context");
    IIO_ENSURE(iio_context_get_devices_count(ctx) > 0 && "No devices");

    //printf("* Acquiring AD9361 streaming devices\n");
    IIO_ENSURE(get_ad9361_stream_dev(ctx, TX, &tx) && "No tx dev found");
    IIO_ENSURE(get_ad9361_stream_dev(ctx, RX, &rx) && "No rx dev found");

    //printf("* Configuring AD9361 for streaming\n");
    IIO_ENSURE(cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0) && "TX port 0 not found");
    IIO_ENSURE(cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0) && "RX port 0 not found");

    //printf("* Initializing AD9361 IIO streaming channels\n");
    IIO_ENSURE(get_ad9361_stream_ch(ctx, RX, rx, 0, &rx0_i) && "RX chan i not found");
    IIO_ENSURE(get_ad9361_stream_ch(ctx, RX, rx, 1, &rx0_q) && "RX chan q not found");
    IIO_ENSURE(get_ad9361_stream_ch(ctx, TX, tx, 0, &tx0_i) && "TX chan i not found");
    IIO_ENSURE(get_ad9361_stream_ch(ctx, TX, tx, 1, &tx0_q) && "TX chan q not found");

    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);

    txbuf = iio_device_create_buffer(tx, tx_buffer_size, false);
    iio_buffer_set_blocking_mode(txbuf, true);
    if (!txbuf) {
        shutdown();
    }

    iio_channel_enable(rx0_i);
    iio_channel_enable(rx0_q);

    rxbuf = iio_device_create_buffer(rx, rx_buffer_size, false);
    iio_buffer_set_blocking_mode(rxbuf, true);
    if (!rxbuf) {
        shutdown();
    }

}

void send(int* data, ssize_t size)
{
    if (size > 64) {
        return;
    }
    std::cout << __func__ << std::endl;

    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);
    iio_channel_disable(rx0_i);
    iio_channel_disable(rx0_q);

    ssize_t nbytes_tx;
    int16_t* p_dat;
    int16_t* p_end;
    ptrdiff_t p_inc;
    p_inc = iio_buffer_step(txbuf);
    p_end = (int16_t*)iio_buffer_end(txbuf);
    liquid_float_complex fsk_out[fsk_config.fsk_k];
    int idx = 0;
    int data_idx = 0;
    int count = 0;
    static fskmod mod = fskmod_create(fsk_config.fsk_m, fsk_config.fsk_k,
                               fsk_config.fsk_bandwidth);
    auto start = std::chrono::system_clock::now();
    for (p_dat = (int16_t *)iio_buffer_first(txbuf, tx0_i);
         p_dat < p_end; p_dat += p_inc) {
        ++count;
        if (0 == (idx % fsk_config.fsk_k)) {
            //modulate_data(fsk_out, (int)data[data_idx], mod);
            fskmod_modulate(mod, (int)data[data_idx], fsk_out);
            idx = 0;
            if (data_idx < (size - 1)) {
                ++data_idx;
            } else {
                data_idx = 0;
            }
        }
        ((int16_t*)p_dat)[0] = convert_float_to_int(fsk_out[idx].real);
        ((int16_t*)p_dat)[1] = convert_float_to_int(fsk_out[idx].imag);
        ++idx;
    }
    nbytes_tx = iio_buffer_push(txbuf);
    if (nbytes_tx < 0) {
        std::cout << "Buffer push failed. " << std::endl;
        shutdown();
        return;
    }
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<
        std::chrono::seconds>(end - start).count();
    std::cout << "Duration: " << duration << " seconds"<< std::endl;
}


void receive(int rx_idx, liquid_float_complex fsk_out[])
{
    ssize_t nbytes_rx;
    std::cout << __func__ << std::endl;

    iio_channel_disable(tx0_i);
    iio_channel_disable(tx0_q);
    iio_channel_enable(rx0_i);
    iio_channel_enable(rx0_q);

    if (! rxbuf) {
        std::cout << "RX buffer is null" << std::endl;
        shutdown();
    }
    int id = rx_idx * rx_buffer_size / (2 * fsk_config.fsk_k);
    int16_t* p_dat;
    int16_t* p_end;
    ptrdiff_t p_inc;
    nbytes_rx = iio_buffer_refill(rxbuf);
    if (nbytes_rx < 0) {
        std::cout << "test" << std::endl;
        shutdown();
    }
    p_inc = iio_buffer_step(rxbuf);
    p_end = (int16_t*)iio_buffer_end(rxbuf);
    p_dat = (int16_t*)iio_buffer_first(rxbuf, rx0_i);
    int idx = 0;
    int count = 0;
    static fskdem dem = fskdem_create(fsk_config.fsk_m, fsk_config.fsk_k,
                               fsk_config.fsk_bandwidth);
    auto start = std::chrono::system_clock::now();
    for (p_dat = (int16_t *)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end;
         p_dat += p_inc) {
        if ((idx % fsk_config.fsk_k) == 0) {
            //unsigned int x = fskdem_demodulate(dem, frame);
            rx_data.data[id] = fskdem_demodulate(dem, fsk_out);// (unsigned int)demodulate_signal(fsk_out, dem);
            ++id;
            idx = 0;
	    ++count;
        }
        fsk_out[idx].real = p_dat[0];
        fsk_out[idx].imag = p_dat[1];
        ++idx;
    }
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<
        std::chrono::seconds>(end - start).count();
    std::cout << "Duration: " << duration << " seconds"<< std::endl;
}

void send_psk(int* data, ssize_t size)
{
    if (size > 64) {
        return;
    }
    std::cout << __func__ << std::endl;

    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);
    iio_channel_disable(rx0_i);
    iio_channel_disable(rx0_q);

    ssize_t nbytes_tx;
    int16_t* p_dat;
    int16_t* p_end;
    ptrdiff_t p_inc;
    p_inc = iio_buffer_step(txbuf);
    p_end = (int16_t*)iio_buffer_end(txbuf);
    liquid_float_complex psk_out[psk_config.psk_k];
    int idx = 0;
    int data_idx = 0;
    int count = 0;
    static modem psk_mod = modem_create(psk);
    auto start = std::chrono::system_clock::now();
    for (p_dat = (int16_t *)iio_buffer_first(txbuf, tx0_i);
         p_dat < p_end; p_dat += p_inc) {
        ++count;
        if (0 == (idx % psk_config.psk_k)) {
            modem_modulate(psk_mod, (int)data[data_idx], psk_out);
            idx = 0;
            if (data_idx < (size - 1)) {
                ++data_idx;
            } else {
                data_idx = 0;
            }
        }
        ((int16_t*)p_dat)[0] = convert_float_to_int(psk_out[idx].real);
        ((int16_t*)p_dat)[1] = convert_float_to_int(psk_out[idx].imag);
        ++idx;
    }
    nbytes_tx = iio_buffer_push(txbuf);
    if (nbytes_tx < 0) {
        std::cout << "Buffer push failed. " << std::endl;
        shutdown();
        return;
    }
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<
            std::chrono::seconds>(end - start).count();
    std::cout << "Duration: " << duration << " seconds"<< std::endl;
}

void receive_psk(int rx_idx)
{
    ssize_t nbytes_rx;
    std::cout << __func__ << std::endl;

    iio_channel_disable(tx0_i);
    iio_channel_disable(tx0_q);
    iio_channel_enable(rx0_i);
    iio_channel_enable(rx0_q);

    if (! rxbuf) {
        std::cout << "RX buffer is null" << std::endl;
        shutdown();
    }
    int id = rx_idx * rx_buffer_size / (2 * psk_config.psk_k);
    int16_t* p_dat;
    int16_t* p_end;
    ptrdiff_t p_inc;
    nbytes_rx = iio_buffer_refill(rxbuf);
    if (nbytes_rx < 0) {
        shutdown();
    }
    p_inc = iio_buffer_step(rxbuf);
    p_end = (int16_t*)iio_buffer_end(rxbuf);
    p_dat = (int16_t*)iio_buffer_first(rxbuf, rx0_i);
    int idx = 0;
    int count = 0;
    unsigned int simb = 0;
    liquid_float_complex psk_out[psk_config.psk_k];
    static modem psk_dem = modem_create(psk);
    auto start = std::chrono::system_clock::now();
    for (p_dat = (int16_t *)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end;
         p_dat += p_inc) {
        psk_out[idx].real = p_dat[0];
        psk_out[idx].imag = p_dat[1];
        if ((idx % psk_config.psk_k) == 0) {
            modem_demodulate(psk_dem, psk_out[idx], &simb);
            ++id;
            idx = 0;
            ++count;
            rx_data.data[id] = simb;
        }

        ++idx;
    }
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<
            std::chrono::seconds>(end - start).count();
    std::cout << "Duration: " << duration << " seconds"<< std::endl;
}

void send_gmsk(int* data, ssize_t size)
{
    if (size > 64) {
        return;
    }
    std::cout << __func__ << std::endl;

    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);
    iio_channel_disable(rx0_i);
    iio_channel_disable(rx0_q);

    ssize_t nbytes_tx;
    int16_t* p_dat;
    int16_t* p_end;
    ptrdiff_t p_inc;
    p_inc = iio_buffer_step(txbuf);
    p_end = (int16_t*)iio_buffer_end(txbuf);
    liquid_float_complex gmsk_out[gmsk_config.gmsk_k];
    int idx = 0;
    int data_idx = 0;
    int count = 0;
    static gmskmod gmsk_mod = gmskmod_create(gmsk_config.gmsk_k, gmsk_config.gmsk_m, gmsk_config.gmsk_bandwidth);
    auto start = std::chrono::system_clock::now();
    for (p_dat = (int16_t *)iio_buffer_first(txbuf, tx0_i);
         p_dat < p_end; p_dat += p_inc) {
        ++count;
        if (0 == (idx % gmsk_config.gmsk_k)) {
            gmskmod_modulate(gmsk_mod, (int)data[data_idx], gmsk_out);
            idx = 0;
            if (data_idx < (size - 1)) {
                ++data_idx;
            } else {
                data_idx = 0;
            }
        }
        ((int16_t*)p_dat)[0] = convert_float_to_int(gmsk_out[idx].real);
        ((int16_t*)p_dat)[1] = convert_float_to_int(gmsk_out[idx].imag);
        ++idx;
    }
    nbytes_tx = iio_buffer_push(txbuf);
    if (nbytes_tx < 0) {
        std::cout << "Buffer push failed. " << std::endl;
        shutdown();
        return;
    }
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<
            std::chrono::seconds>(end - start).count();
    std::cout << "Duration: " << duration << " seconds"<< std::endl;
}

void receive_gmsk(int rx_idx)
{
    ssize_t nbytes_rx;
    std::cout << __func__ << std::endl;

    iio_channel_disable(tx0_i);
    iio_channel_disable(tx0_q);
    iio_channel_enable(rx0_i);
    iio_channel_enable(rx0_q);

    if (! rxbuf) {
        std::cout << "RX buffer is null" << std::endl;
        shutdown();
    }
    int id = rx_idx * rx_buffer_size / (2 * gmsk_config.gmsk_k);
    int16_t* p_dat;
    int16_t* p_end;
    ptrdiff_t p_inc;
    nbytes_rx = iio_buffer_refill(rxbuf);
    if (nbytes_rx < 0) {
        shutdown();
    }
    p_inc = iio_buffer_step(rxbuf);
    p_end = (int16_t*)iio_buffer_end(rxbuf);
    p_dat = (int16_t*)iio_buffer_first(rxbuf, rx0_i);
    int idx = 0;
    int count = 0;
    unsigned int simb = 0;
    liquid_float_complex gmsk_out[gmsk_config.gmsk_k];
    static gmskdem gmsk_dem = gmskdem_create(gmsk_config.gmsk_k, gmsk_config.gmsk_m, gmsk_config.gmsk_bandwidth);
    auto start = std::chrono::system_clock::now();
    for (p_dat = (int16_t *)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end;
         p_dat += p_inc) {

        if ((idx % gmsk_config.gmsk_k) == 0) {
            gmskdem_demodulate(gmsk_dem, gmsk_out, &simb);
            ++id;
            idx = 0;
            ++count;
            rx_data.data[id] = simb;
        }
        gmsk_out[idx].real = p_dat[0];
        gmsk_out[idx].imag = p_dat[1];
        ++idx;
    }
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<
            std::chrono::seconds>(end - start).count();
    std::cout << "Duration: " << duration << " seconds"<< std::endl;
}

int count_ones(int data)
{
    int c = 0;
    for (int i = data; i != 0; i >>= 1) {
        c += i & 1;
    }
    return c;
}

double parse_value(const std::vector<std::string>& tokens,
                   const std::string& argument, double value)
{
    assert(! argument.empty());
    auto rx_freq_p = std::find(tokens.begin(), tokens.end(), argument);
    if (tokens.end() != rx_freq_p) {
        if (tokens.end() != (rx_freq_p + 1)) {
            double r = std::stod(*(rx_freq_p + 1));
            return r;
        } else {
            std::cerr << argument << " does not have value " << std::endl;
        }
    } else {
        std::cerr << "mandatory argument " << argument << " is missing. " <<
            "The default value will be used: " << value << "\n";
    }
    return value;
}

void parse_command_line(int argc, char** argv)
{
    std::vector<std::string> tokens;
    for (int i = 1; i < argc; ++i) {
        tokens.push_back(std::string(argv[i]));
    }
    double rx_freq = parse_value(tokens, "-rx_freq", 2.5);
    double tx_freq = parse_value(tokens, "-tx_freq", 2.5);
    double rx_bandwidth = parse_value(tokens, "-rx_bandwidth", 20);
    double tx_bandwidth = parse_value(tokens, "-tx_bandwidth", 20);
    double rx_sample_rate = parse_value(tokens, "-rx_sample_rate", 30);
    double tx_sample_rate = parse_value(tokens, "-tx_sample_rate", 30);
    init(rx_bandwidth, rx_sample_rate, rx_freq,
         tx_bandwidth, tx_sample_rate, tx_freq);
    fsk_config.fsk_bandwidth =
        parse_value(tokens, "-frequency_spacing", 0.25);
    iteration_count = parse_value(tokens, "-iterations", 20);
    modul_type= parse_value(tokens, "-modul_type", 0);

}

void configure_fsk()
{
    fsk_config.fsk_m = 1;
    fsk_config.fsk_k = 2;
    std::cout << "fsk settings ..." << std::endl;
    std::cout << "bits per symbol " << fsk_config.fsk_m << std::endl;
    std::cout << "samples per symbol " << fsk_config.fsk_k << std::endl;
    std::cout << "bandwidth " << fsk_config.fsk_bandwidth << std::endl;
}

void configure_psk()
{
    psk_config.psk_m = 1;
    psk_config.psk_k = 2;
    std::cout << "psk settings ..." << std::endl;
    std::cout << "bits per symbol " << psk_config.psk_m << std::endl;
    std::cout << "samples per symbol " << psk_config.psk_k << std::endl;
    std::cout << "bandwidth " << psk_config.psk_bandwidth << std::endl;
}

void configure_gmsk()
{
    gmsk_config.gmsk_m = 1;
    gmsk_config.gmsk_k = 2;
    gmsk_config.gmsk_bandwidth = 0.25f;
    std::cout << "gmsk settings ..." << std::endl;
    std::cout << "bits per symbol " << gmsk_config.gmsk_m << std::endl;
    std::cout << "samples per symbol " << gmsk_config.gmsk_k << std::endl;
    std::cout << "bandwidth " << gmsk_config.gmsk_bandwidth << std::endl;
}

int main(int argc, char** argv) {
    parse_command_line(argc, argv);
    if (modul_type == 0) {
        configure_fsk();
        int a[4] = {0x0, 0x1, 0x0, 0x1};
        rx_data.size = iteration_count * rx_buffer_size / (2 * fsk_config.fsk_k);
        rx_data.data = new uint16_t[rx_data.size];
        liquid_float_complex fsk_out[fsk_config.fsk_k];
        std::cout << "(INFO) sending 800000 ones per iteration." << std::endl;
        for (int i = 0; i < iteration_count; ++i) {
            send(a, 4);

            receive(i, fsk_out);
        }
        int ones_count = 0;
        for (int i = 0; i < rx_data.size; ++i) {
            ones_count += count_ones(rx_data.data[i]);
        }
        delete rx_data.data;
        float total_one_count = iteration_count * rx_buffer_size / (2 * fsk_config.fsk_k) / 2;
        std::cout << "Received ones count " << int(ones_count) << std::endl;
        std::cout << "Invalid bits rate " << float(abs(total_one_count - ones_count) / total_one_count) << std::endl;
        return 0;
    }

    else if (modul_type == 1) {
        configure_psk();
        int a[4] = {0x0, 0x1, 0x0, 0x1};
        rx_data.size = iteration_count * rx_buffer_size / (2 * psk_config.psk_k);
        rx_data.data = new uint16_t[rx_data.size];
        std::cout << "(INFO) sending 800000 ones per iteration." << std::endl;
        for (int i = 0; i < iteration_count; ++i) {
            send_psk(a, 4);

            receive_psk(i);
        }
        int ones_count = 0;
        for (int i = 0; i < rx_data.size; ++i) {
            ones_count += count_ones(rx_data.data[i]);
        }
        delete rx_data.data;
        float total_one_count = iteration_count * rx_buffer_size / (2 * psk_config.psk_k) / 2;
        std::cout << "Received ones count " << int(ones_count) << std::endl;
        std::cout << "Invalid bits rate " << float(abs(total_one_count - ones_count) / total_one_count) << std::endl;
        return 0;
    }

    else if (modul_type == 2) {
        configure_gmsk();
        int a[4] = {0x0, 0x1, 0x0, 0x1};
        rx_data.size = iteration_count * rx_buffer_size / (2 * gmsk_config.gmsk_k);
        rx_data.data = new uint16_t[rx_data.size];
        std::cout << "(INFO) sending 800000 ones per iteration." << std::endl;
        for (int i = 0; i < iteration_count; ++i) {
            send_gmsk(a, 4);

            receive_gmsk(i);
        }
        int ones_count = 0;
        for (int i = 0; i < rx_data.size; ++i) {
            ones_count += count_ones(rx_data.data[i]);
        }
        delete rx_data.data;
        float total_one_count = iteration_count * rx_buffer_size / (2 * gmsk_config.gmsk_k) / 2;
        std::cout << "Received ones count " << int(ones_count) << std::endl;
        std::cout << "Invalid bits rate " << float(abs(total_one_count - ones_count) / total_one_count) << std::endl;
        return 0;
    }
}

