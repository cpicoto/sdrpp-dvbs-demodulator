#pragma once
#include <dsp/block.h>
#include <dsp/stream.h>
#include <dsp/types.h>
#include <dsp/math/conjugate.h>
#include <dsp/math/multiply.h>
#include <dsp/math/add.h>
#include <dsp/math/subtract.h>
#include <dsp/math/delay.h>
#include <dsp/loop/fast_agc.h>
#include <dsp/sink/handler_sink.h>
#include <utils/flog.h>
#include <string>
#include <vector>
#include <cmath>
#include <complex>
#include <fftw3.h>
extern "C" {
#include "common/correct/correct.h"
}

namespace dsp::dvbt {

    // DVB-T transmission parameters
    enum DVBTMode {
        DVBT_MODE_2K = 0,
        DVBT_MODE_8K = 1
    };

    enum DVBTModulation {
        DVBT_MOD_QPSK = 0,
        DVBT_MOD_16QAM = 1,
        DVBT_MOD_64QAM = 2
    };

    enum DVBTCodeRate {
        DVBT_CR_1_2 = 0,
        DVBT_CR_2_3 = 1,
        DVBT_CR_3_4 = 2,
        DVBT_CR_5_6 = 3,
        DVBT_CR_7_8 = 4
    };

    enum DVBTGuardInterval {
        DVBT_GI_1_32 = 0,
        DVBT_GI_1_16 = 1,
        DVBT_GI_1_8 = 2,
        DVBT_GI_1_4 = 3
    };

    enum DVBTHierarchy {
        DVBT_HIER_NONE = 0,
        DVBT_HIER_ALPHA_1 = 1,
        DVBT_HIER_ALPHA_2 = 2,
        DVBT_HIER_ALPHA_4 = 3
    };

    // TPS (Transmission Parameter Signalling) structure
    struct TPSInfo {
        bool locked = false;
        DVBTMode mode = DVBT_MODE_2K;
        DVBTModulation modulation = DVBT_MOD_QPSK;
        DVBTCodeRate code_rate_hp = DVBT_CR_1_2;
        DVBTCodeRate code_rate_lp = DVBT_CR_1_2;
        DVBTGuardInterval guard_interval = DVBT_GI_1_32;
        DVBTHierarchy hierarchy = DVBT_HIER_NONE;
        int frame_number = 0;
        bool cell_id_valid = false;
        int cell_id = 0;
        float snr_estimate = 0.0f;
        float frequency_offset = 0.0f;
        float timing_offset = 0.0f;
    };

    // DVB-T specific convolutional deinterleaver
    class DVBTConvolutionalDeinterleaver {
    private:
        std::vector<std::vector<uint8_t>> delay_lines;
        std::vector<int> delays;
        std::vector<int> positions;
        int sequence_length;
        
    public:
        DVBTConvolutionalDeinterleaver() {
            // DVB-T uses I=12, M=17 for convolutional interleaving
            sequence_length = 12;
            delays.resize(sequence_length);
            positions.resize(sequence_length);
            delay_lines.resize(sequence_length);
            
            // Initialize delay lines
            for (int i = 0; i < sequence_length; i++) {
                delays[i] = i * 17;  // M = 17
                positions[i] = 0;
                delay_lines[i].resize(delays[i], 0);
            }
        }
        
        void deinterleave(const uint8_t* input, uint8_t* output, int length) {
            for (int i = 0; i < length; i++) {
                int branch = i % sequence_length;
                
                if (delays[branch] == 0) {
                    output[i] = input[i];
                } else {
                    output[i] = delay_lines[branch][positions[branch]];
                    delay_lines[branch][positions[branch]] = input[i];
                    positions[branch] = (positions[branch] + 1) % delays[branch];
                }
            }
        }
    };

    // DVB-T Reed-Solomon decoder
    class DVBTReedSolomon {
    private:
        correct_reed_solomon* rs;
        
    public:
        DVBTReedSolomon() {
            // DVB-T uses RS(204,188) shortened from RS(255,239)
            rs = correct_reed_solomon_create(correct_rs_primitive_polynomial_8_4_3_2_0, 0, 1, 16);
        }
        
        ~DVBTReedSolomon() {
            correct_reed_solomon_destroy(rs);
        }
        
        int decode(uint8_t* data) {
            uint8_t rs_buffer[255];
            uint8_t out_buffer[255];
            
            // Pad with zeros (51 bytes) to make it 255 bytes
            memset(rs_buffer, 0, 51);
            memcpy(rs_buffer + 51, data, 188);
            memcpy(rs_buffer + 239, data + 188, 16);
            
            ssize_t result = correct_reed_solomon_decode(rs, rs_buffer, 255, out_buffer);
            
            if (result == -1) {
                return -1;  // Uncorrectable error
            }
            
            // Copy back the corrected data
            memcpy(data, out_buffer + 51, 188);
            
            // Count errors corrected
            int errors = 0;
            for (int i = 51; i < 239; i++) {
                if (rs_buffer[i] != out_buffer[i]) {
                    errors++;
                }
            }
            
            return errors;
        }
    };

    class DVBTDemod : public Processor<complex_t, uint8_t> {
        using base_type = Processor<complex_t, uint8_t>;
        
    public:
        DVBTDemod() {}
        DVBTDemod(stream<complex_t>* in, int bandwidth_mhz);

        void init(stream<complex_t>* in, int bandwidth_mhz);
        void setBandwidth(int bandwidth_mhz);
        void setSamplerate(double samplerate);
        void reset();
        void start();
        void stop();

        // Statistics and status
        TPSInfo getTPS() const { return tps_info; }
        bool isTPSLocked() const { return tps_info.locked; }
        float getSNR() const { return tps_info.snr_estimate; }
        float getFrequencyOffset() const { return tps_info.frequency_offset; }
        
        // Constellation data for display
        void setConstellationHandler(void (*handler)(complex_t* data, int count, void* ctx), void* ctx);

        // Debug output control
        void setDebugOutput(bool enabled) { debug_output = enabled; }

        // Implementation of pure virtual method from Processor
        int run();

    private:
        void worker();
        void processOFDMFrame(complex_t* samples, int count);
        void extractTPS(complex_t* ofdm_symbols);
        void demodulateData(complex_t* ofdm_symbols, uint8_t* output);
        void updateTPSLock();
        void calculateSNR(complex_t* pilots);
        void estimateFrequencyOffset(complex_t* pilots);
        
        // OFDM processing
        void performFFT(complex_t* time_domain, complex_t* freq_domain);
        void removeCyclicPrefix(complex_t* input, complex_t* output);
        void channelCorrection(complex_t* symbols);
        void extractPilots(complex_t* symbols, complex_t* pilots);
        void updateChannelEstimate(complex_t* pilots);
        
        // DVB-T specific pilot pattern
        bool isPilotCarrier(int carrier_index, int symbol_index);
        bool isTpsCarrier(int carrier_index);
        complex_t getPilotReference(int carrier_index, int symbol_index);
        
        // Transport stream processing
        void processTransportStream(uint8_t* raw_data, int length);
        bool findTSSync(uint8_t* data, int length, int& sync_pos);
        
        // Internal state
        int bandwidth_mhz = 8;
        double samplerate = 8000000.0;
        bool debug_output = true;
        
        // OFDM parameters
        int fft_size = 8192;
        int guard_samples = 256;
        int useful_carriers = 6817;
        int pilot_carriers = 177;
        int tps_carriers = 68;
        
        // FFTW plans
        fftwf_plan fft_plan;
        fftwf_complex* fft_in;
        fftwf_complex* fft_out;
        
        // AGC
        dsp::loop::FastAGC<complex_t> agc;
        
        // Processing buffers
        std::vector<complex_t> fft_buffer;
        std::vector<complex_t> ofdm_buffer;
        std::vector<complex_t> pilot_buffer;
        std::vector<complex_t> data_buffer;
        std::vector<complex_t> channel_estimate;
        std::vector<complex_t> pilot_estimates;
        std::vector<uint8_t> ts_packet_buffer;
        
        // TPS processing
        TPSInfo tps_info;
        int tps_sync_count = 0;
        int tps_error_count = 0;
        std::vector<uint8_t> tps_buffer;
        
        // Symbol and frame counters
        int symbol_counter = 0;
        int frame_counter = 0;
        bool frame_sync = false;
        
        // Error correction
        DVBTReedSolomon reed_solomon;
        DVBTConvolutionalDeinterleaver deinterleaver;
        
        // Constellation handler
        void (*constellation_handler)(complex_t* data, int count, void* ctx) = nullptr;
        void* constellation_ctx = nullptr;
        
        // TS packet sync
        std::vector<uint8_t> ts_sync_buffer;
        int ts_sync_state = 0;
        int ts_packet_count = 0;
    };
} 