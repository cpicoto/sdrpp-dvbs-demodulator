#pragma once
#include <dsp/stream.h>
#include <dsp/types.h>
#include <dsp/processor.h>
#include <dsp/buffer/reshaper.h>
#include <dsp/taps/windowed_sinc.h>
#include <dsp/filter/fir.h>
#include <dsp/math/conjugate.h>
#include <dsp/convert/complex_to_real.h>
#include <dsp/routing/splitter.h>
#include <dsp/math/multiply.h>
#include <dsp/math/add.h>
#include <dsp/math/subtract.h>
#include <dsp/loop/fast_agc.h>
#include <dsp/clock_recovery/mm.h>
#include <dsp/taps/root_raised_cosine.h>
#include <dsp/filter/fir.h>
#include <dsp/math/conjugate.h>
#include <dsp/convert/complex_to_real.h>
#include <dsp/math/fast_atan2.h>
#include <dsp/correction/dc_blocker.h>
#include <fftw3.h>
#include <cstring>
#include <algorithm>
#include <stdexcept>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace dsp::dvbt {
    
    enum dvbt_constellation_t {
        DVBT_QPSK = 0,
        DVBT_QAM16,
        DVBT_QAM64
    };
    
    enum dvbt_transmission_mode_t {
        DVBT_2K = 0,
        DVBT_8K
    };
    
    enum dvbt_guard_interval_t {
        DVBT_GI_1_32 = 0,
        DVBT_GI_1_16,
        DVBT_GI_1_8,
        DVBT_GI_1_4
    };
    
    enum dvbt_code_rate_t {
        DVBT_CR_1_2 = 0,
        DVBT_CR_2_3,
        DVBT_CR_3_4,
        DVBT_CR_5_6,
        DVBT_CR_7_8
    };
    
    class DVBTConfig {
    public:
        dvbt_constellation_t constellation;
        dvbt_transmission_mode_t transmission_mode;
        dvbt_guard_interval_t guard_interval;
        dvbt_code_rate_t code_rate_HP;
        dvbt_code_rate_t code_rate_LP;
        
        int fft_length;
        int guard_length;
        int symbol_length;
        int payload_length;
        int constellation_size;
        
        DVBTConfig(dvbt_constellation_t const_type = DVBT_QAM16,
                   dvbt_transmission_mode_t trans_mode = DVBT_2K,
                   dvbt_guard_interval_t guard_int = DVBT_GI_1_32,
                   dvbt_code_rate_t code_rate_hp = DVBT_CR_1_2,
                   dvbt_code_rate_t code_rate_lp = DVBT_CR_1_2) {
            
            constellation = const_type;
            transmission_mode = trans_mode;
            guard_interval = guard_int;
            code_rate_HP = code_rate_hp;
            code_rate_LP = code_rate_lp;
            
            // Set FFT parameters based on transmission mode
            switch (transmission_mode) {
                case DVBT_2K:
                    fft_length = 2048;
                    payload_length = 1512;
                    break;
                case DVBT_8K:
                    fft_length = 8192;
                    payload_length = 6048;
                    break;
            }
            
            // Set guard interval length
            switch (guard_interval) {
                case DVBT_GI_1_32:
                    guard_length = fft_length / 32;
                    break;
                case DVBT_GI_1_16:
                    guard_length = fft_length / 16;
                    break;
                case DVBT_GI_1_8:
                    guard_length = fft_length / 8;
                    break;
                case DVBT_GI_1_4:
                    guard_length = fft_length / 4;
                    break;
            }
            
            symbol_length = fft_length + guard_length;
            
            // Set constellation size
            switch (constellation) {
                case DVBT_QPSK:
                    constellation_size = 4;
                    break;
                case DVBT_QAM16:
                    constellation_size = 16;
                    break;
                case DVBT_QAM64:
                    constellation_size = 64;
                    break;
            }
        }
    };
    
    class OFDMSymbolAcquisition {
    private:
        DVBTConfig config;
        fftwf_complex* fft_in;
        fftwf_complex* fft_out;
        fftwf_plan fft_plan;
        
        dsp::stream<dsp::complex_t>* input;
        dsp::stream<dsp::complex_t>* output;
        
        float* window;
        int symbol_count;
        bool sync_found;
        
    public:
        OFDMSymbolAcquisition(DVBTConfig cfg) : config(cfg), fft_in(nullptr), fft_out(nullptr), window(nullptr) {
            printf("[DVB-T] Initializing OFDM Symbol Acquisition...\n");
            printf("[DVB-T] FFT length: %d, Guard length: %d, Symbol length: %d\n", 
                   config.fft_length, config.guard_length, config.symbol_length);
            try {
                // Allocate FFT buffers
                printf("[DVB-T] Allocating FFTW buffers (%d complex samples each)...\n", config.fft_length);
                fft_in = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * config.fft_length);
                fft_out = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * config.fft_length);
                
                if (!fft_in || !fft_out) {
                    printf("[DVB-T] ERROR: Failed to allocate FFTW buffers\n");
                    throw std::runtime_error("Failed to allocate FFTW buffers");
                }
                printf("[DVB-T] FFTW buffers allocated successfully\n");
                
                printf("[DVB-T] Creating FFTW plan for %d-point FFT...\n", config.fft_length);
                fft_plan = fftwf_plan_dft_1d(config.fft_length, fft_in, fft_out, FFTW_FORWARD, FFTW_ESTIMATE);
                
                if (!fft_plan) {
                    printf("[DVB-T] ERROR: Failed to create FFTW plan\n");
                    throw std::runtime_error("Failed to create FFTW plan");
                }
                printf("[DVB-T] FFTW plan created successfully\n");
                
                // Create window function
                printf("[DVB-T] Creating Hanning window function...\n");
                window = new float[config.fft_length];
                for (int i = 0; i < config.fft_length; i++) {
                    window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (config.fft_length - 1)));
                }
                
                symbol_count = 0;
                sync_found = false;
                printf("[DVB-T] OFDM Symbol Acquisition initialized successfully\n");
            } catch (...) {
                printf("[DVB-T] ERROR: OFDM Symbol Acquisition initialization failed\n");
                cleanup();
                throw;
            }
        }
        
        ~OFDMSymbolAcquisition() {
            cleanup();
        }
        
        void cleanup() {
            if (fft_plan) {
                fftwf_destroy_plan(fft_plan);
                fft_plan = nullptr;
            }
            if (fft_in) {
                fftwf_free(fft_in);
                fft_in = nullptr;
            }
            if (fft_out) {
                fftwf_free(fft_out);
                fft_out = nullptr;
            }
            if (window) {
                delete[] window;
                window = nullptr;
            }
        }
        
        void init(dsp::stream<dsp::complex_t>* in, dsp::stream<dsp::complex_t>* out) {
            input = in;
            output = out;
        }
        
        void process(int count) {
            if (!input || !output || !fft_in || !fft_out || !window) return;
            
            for (int i = 0; i < count; i++) {
                if (!input->swap(config.symbol_length)) { return; }

                // Copy input to FFT buffer (skip guard interval)
                dsp::complex_t* data_ptr = input->readBuf;
                for (int j = 0; j < config.fft_length; j++) {
                    dsp::complex_t sample = data_ptr[config.guard_length + j];
                    fft_in[j][0] = sample.re * window[j];
                    fft_in[j][1] = sample.im * window[j];
                }

                // Execute FFT
                fftwf_execute(fft_plan);

                // Copy output
                if (!output->swap(config.fft_length)) { return; }
                dsp::complex_t* out_ptr = output->writeBuf;
                for (int j = 0; j < config.fft_length; j++) {
                    out_ptr[j].re = fft_out[j][0];
                    out_ptr[j].im = fft_out[j][1];
                }

                input->flush();
                output->flush();
            }
        }
    };
    
    class DVBTDemap {
    private:
        DVBTConfig config;
        dsp::stream<dsp::complex_t>* input;
        dsp::stream<uint8_t>* output;
        
        int bits_per_symbol;
        
    public:
        DVBTDemap(DVBTConfig cfg) : config(cfg) {
            switch (config.constellation) {
                case DVBT_QPSK:
                    bits_per_symbol = 2;
                    break;
                case DVBT_QAM16:
                    bits_per_symbol = 4;
                    break;
                case DVBT_QAM64:
                    bits_per_symbol = 6;
                    break;
            }
        }
        
        void init(dsp::stream<dsp::complex_t>* in, dsp::stream<uint8_t>* out) {
            input = in;
            output = out;
        }
        
        uint8_t demap_qpsk(dsp::complex_t sample) {
            uint8_t bits = 0;
            bits |= (sample.re > 0.0f) ? 0 : 1;
            bits |= (sample.im > 0.0f) ? 0 : 2;
            return bits;
        }
        
        uint8_t demap_qam16(dsp::complex_t sample) {
            uint8_t bits = 0;
            float scale = 0.7071f; // 1/sqrt(2)
            
            // I channel
            if (sample.re > 0.0f) {
                bits |= (sample.re > scale) ? 0 : 2;
            } else {
                bits |= (sample.re < -scale) ? 1 : 3;
            }
            
            // Q channel  
            if (sample.im > 0.0f) {
                bits |= (sample.im > scale) ? 0 : 8;
            } else {
                bits |= (sample.im < -scale) ? 4 : 12;
            }
            
            return bits;
        }
        
        uint8_t demap_qam64(dsp::complex_t sample) {
            // Simplified QAM64 demapping - would need full constellation map
            uint8_t bits = 0;
            
            // Quantize to 8 levels per axis
            int i_level = (int)((sample.re + 1.0f) * 4.0f);
            int q_level = (int)((sample.im + 1.0f) * 4.0f);
            
            i_level = std::max(0, std::min(7, i_level));
            q_level = std::max(0, std::min(7, q_level));
            
            bits = (i_level & 0x7) | ((q_level & 0x7) << 3);
            return bits;
        }
        
        void process(int count) {
            for (int i = 0; i < count; i++) {
                if (!input->swap(1)) { return; }
                if (!output->swap(1)) { return; }

                dsp::complex_t sample = input->readBuf[0];
                uint8_t bits;

                switch (config.constellation) {
                    case DVBT_QPSK:
                        bits = demap_qpsk(sample);
                        break;
                    case DVBT_QAM16:
                        bits = demap_qam16(sample);
                        break;
                    case DVBT_QAM64:
                        bits = demap_qam64(sample);
                        break;
                }

                output->writeBuf[0] = bits;

                input->flush();
                output->flush();
            }
        }
    };
    
    class DVBTViterbiDecoder {
    private:
        DVBTConfig config;
        dsp::stream<uint8_t>* input;
        dsp::stream<uint8_t>* output;
        
        // Viterbi decoder state
        uint8_t* decoded_bits;
        int decoded_size;
        
    public:
        DVBTViterbiDecoder(DVBTConfig cfg) : config(cfg) {
            decoded_size = 1024; // Adjust based on needs
            decoded_bits = new uint8_t[decoded_size];
        }
        
        ~DVBTViterbiDecoder() {
            delete[] decoded_bits;
        }
        
        void init(dsp::stream<uint8_t>* in, dsp::stream<uint8_t>* out) {
            input = in;
            output = out;
        }
        
        void process(int count) {
            for (int i = 0; i < count; i++) {
                if (!input->swap(1)) { return; }
                if (!output->swap(1)) { return; }

                uint8_t bit = input->readBuf[0];
                output->writeBuf[0] = bit;

                input->flush();
                output->flush();
            }
        }
    };
    
    // TPS (Transport Parameter Signaling) decoder for automatic parameter detection
    class TPSDecoder {
    private:
        DVBTConfig* config;
        
        // TPS carrier positions for 2K mode (subset of pilot carriers)
        static const int TPS_CARRIERS_2K[];
        static const int TPS_CARRIERS_8K[];
        
        uint8_t tps_bits[68]; // TPS data is 68 bits
        int tps_bit_count;
        bool tps_sync;
        
    public:
        TPSDecoder(DVBTConfig* cfg) : config(cfg), tps_bit_count(0), tps_sync(false) {}
        
        bool decodeTPS(dsp::complex_t* ofdm_symbols, int symbol_count) {
            // Extract TPS carriers from OFDM symbols
            const int* tps_carriers = (config->transmission_mode == DVBT_2K) ? 
                                     TPS_CARRIERS_2K : TPS_CARRIERS_8K;
            int num_tps_carriers = (config->transmission_mode == DVBT_2K) ? 17 : 68;
            
            for (int sym = 0; sym < symbol_count && sym < 68; sym++) {
                for (int i = 0; i < num_tps_carriers && tps_bit_count < 68; i++) {
                    int carrier_idx = tps_carriers[i];
                    if (carrier_idx < config->fft_length) {
                        // Simple BPSK demodulation of TPS carriers
                        dsp::complex_t tps_symbol = ofdm_symbols[sym * config->fft_length + carrier_idx];
                        tps_bits[tps_bit_count++] = (tps_symbol.re > 0.0f) ? 0 : 1;
                    }
                }
            }
            
            // Decode TPS when we have enough bits
            if (tps_bit_count >= 68) {
                return decodeTPSParameters();
            }
            
            return false;
        }
        
        bool decodeTPSParameters() {
            // TPS bit allocation (simplified):
            // Bits 0-1: Constellation (00=QPSK, 01=16QAM, 10=64QAM)
            // Bits 2-3: Hierarchy (not used in non-hierarchical)
            // Bits 4-6: Code rate HP (000=1/2, 001=2/3, 010=3/4, 011=5/6, 100=7/8)
            // Bits 7-9: Code rate LP
            // Bits 10-11: Guard interval (00=1/32, 01=1/16, 10=1/8, 11=1/4)
            // Bits 12-13: Transmission mode (00=2K, 01=8K)
            
            // Extract constellation
            int const_bits = (tps_bits[0] << 1) | tps_bits[1];
            switch (const_bits) {
                case 0: config->constellation = DVBT_QPSK; break;
                case 1: config->constellation = DVBT_QAM16; break;
                case 2: config->constellation = DVBT_QAM64; break;
                default: return false;
            }
            
            // Extract code rate HP
            int cr_bits = (tps_bits[4] << 2) | (tps_bits[5] << 1) | tps_bits[6];
            switch (cr_bits) {
                case 0: config->code_rate_HP = DVBT_CR_1_2; break;
                case 1: config->code_rate_HP = DVBT_CR_2_3; break;
                case 2: config->code_rate_HP = DVBT_CR_3_4; break;
                case 3: config->code_rate_HP = DVBT_CR_5_6; break;
                case 4: config->code_rate_HP = DVBT_CR_7_8; break;
                default: return false;
            }
            
            // Extract guard interval
            int gi_bits = (tps_bits[10] << 1) | tps_bits[11];
            switch (gi_bits) {
                case 0: config->guard_interval = DVBT_GI_1_32; break;
                case 1: config->guard_interval = DVBT_GI_1_16; break;
                case 2: config->guard_interval = DVBT_GI_1_8; break;
                case 3: config->guard_interval = DVBT_GI_1_4; break;
            }
            
            // Extract transmission mode
            int tm_bits = (tps_bits[12] << 1) | tps_bits[13];
            switch (tm_bits) {
                case 0: config->transmission_mode = DVBT_2K; break;
                case 1: config->transmission_mode = DVBT_8K; break;
                default: return false;
            }
            
            // Recalculate derived parameters
            *config = DVBTConfig(config->constellation, config->transmission_mode, 
                               config->guard_interval, config->code_rate_HP, config->code_rate_LP);
            
            tps_sync = true;
            return true;
        }
        
        bool hasTPS() const { return tps_sync; }
        void reset() { tps_bit_count = 0; tps_sync = false; }
    };
    
    // TPS carrier positions (subset for demonstration)
    const int TPSDecoder::TPS_CARRIERS_2K[] = {
        34, 50, 209, 346, 413, 569, 595, 688, 790, 901, 1073, 1219, 1262, 1286, 1469, 1594, 1687
    };
    
    const int TPSDecoder::TPS_CARRIERS_8K[] = {
        // Simplified - would need full 68 TPS carrier positions for 8K mode
        34, 50, 209, 346, 413, 569, 595, 688, 790, 901, 1073, 1219, 1262, 1286, 1469, 1594, 1687,
        1738, 1754, 1913, 2050, 2117, 2273, 2299, 2392, 2494, 2605, 2777, 2923, 2966, 2990, 3173,
        3298, 3391, 3442, 3458, 3617, 3754, 3821, 3977, 4003, 4096, 4198, 4309, 4481, 4627, 4670,
        4694, 4877, 5002, 5095, 5146, 5162, 5321, 5458, 5525, 5681, 5707, 5800, 5902, 6013, 6185,
        6331, 6374, 6398, 6581, 6706, 6799
    };
    
    class DVBTDemodulator : public dsp::Processor<dsp::complex_t, uint8_t> {
        using base_type = dsp::Processor<dsp::complex_t, uint8_t>;
    private:
        DVBTConfig config;
        
        // Processing stages
        OFDMSymbolAcquisition* ofdm_acq;
        DVBTDemap* demapper;
        DVBTViterbiDecoder* viterbi;
        TPSDecoder* tps_decoder;
        
        // Auto-detection state
        float bandwidth_mhz;
        bool auto_detect_enabled;
        bool parameters_locked;
        int symbol_count_for_tps;
        
        // Buffer size
        int buffer_size;

    public:
        DVBTDemodulator() : base_type(), bandwidth_mhz(8.0f), auto_detect_enabled(true), 
                           parameters_locked(false), symbol_count_for_tps(0) {
            printf("[DVB-T] Initializing DVB-T demodulator with default parameters\n");
            printf("[DVB-T] Bandwidth: %.1f MHz, Auto-detect: enabled\n", bandwidth_mhz);
            // Start with default 2K mode for initial processing
            config = DVBTConfig(DVBT_QPSK, DVBT_2K, DVBT_GI_1_32, DVBT_CR_1_2, DVBT_CR_1_2);
            printf("[DVB-T] Default config: 2K mode, QPSK, Guard 1/32, Code Rate 1/2\n");
            
            // Set buffer size first
            buffer_size = config.symbol_length;
            printf("[DVB-T] Buffer size calculated: %d samples\n", buffer_size);
            
            // Initialize processing chain but don't call base_type::init yet
            printf("[DVB-T] Initializing processing chain...\n");
            
            initProcessingChain();
            printf("[DVB-T] Initialization complete\n");
        }

        DVBTDemodulator(float bandwidth_mhz) : base_type(), bandwidth_mhz(bandwidth_mhz), auto_detect_enabled(true),
                                              parameters_locked(false), symbol_count_for_tps(0) {
            printf("[DVB-T] Initializing DVB-T demodulator with bandwidth: %.1f MHz\n", bandwidth_mhz);
            // Start with default 2K mode for initial processing  
            config = DVBTConfig(DVBT_QPSK, DVBT_2K, DVBT_GI_1_32, DVBT_CR_1_2, DVBT_CR_1_2);
            printf("[DVB-T] Config: 2K mode, QPSK, Guard 1/32, Code Rate 1/2\n");
            
            // Set buffer size first
            buffer_size = config.symbol_length;
            printf("[DVB-T] Buffer size calculated: %d samples\n", buffer_size);
            
            // Initialize processing chain but don't call base_type::init yet
            printf("[DVB-T] Initializing processing chain...\n");
            
            initProcessingChain();
            printf("[DVB-T] DVB-T demodulator ready for auto-detection\n");
        }

        // Legacy constructor for manual parameter setting
        DVBTDemodulator(dsp::dvbt::dvbt_constellation_t constellation,
                        dsp::dvbt::dvbt_transmission_mode_t transmission_mode,
                        dsp::dvbt::dvbt_guard_interval_t guard_interval,
                        dsp::dvbt::dvbt_code_rate_t code_rate_hp,
                        dsp::dvbt::dvbt_code_rate_t code_rate_lp) : 
            base_type(), config(constellation, transmission_mode, guard_interval, code_rate_hp, code_rate_lp),
            bandwidth_mhz(8.0f), auto_detect_enabled(false), parameters_locked(true), symbol_count_for_tps(0) {
            initProcessingChain();
        }
        
        ~DVBTDemodulator() {
            if (base_type::_block_init) { 
                base_type::stop();
            }
            cleanup();
        }

        void init(dsp::stream<dsp::complex_t>* in) {
            printf("[DVB-T] init() called - initializing base processor with input stream\n");
            base_type::init(in);
            printf("[DVB-T] Base processor initialized successfully\n");
        }

        void initProcessingChain() {
            printf("[DVB-T] Starting processing chain initialization...\n");
            // Initialize processing stages safely
            ofdm_acq = nullptr;
            demapper = nullptr;
            viterbi = nullptr;
            tps_decoder = nullptr;
            
            try {
                printf("[DVB-T] Creating OFDM Symbol Acquisition (FFT length: %d)...\n", config.fft_length);
                ofdm_acq = new OFDMSymbolAcquisition(config);
                
                printf("[DVB-T] Creating DVB-T Demapper (%s constellation)...\n", 
                       (config.constellation == DVBT_QPSK) ? "QPSK" : 
                       (config.constellation == DVBT_QAM16) ? "16QAM" : "64QAM");
                demapper = new DVBTDemap(config);
                
                printf("[DVB-T] Creating Viterbi Decoder...\n");
                viterbi = new DVBTViterbiDecoder(config);
                
                printf("[DVB-T] Creating TPS Decoder for auto-detection...\n");
                tps_decoder = new TPSDecoder(&config);
                
                // Set buffer size based on config
                buffer_size = config.symbol_length;
                printf("[DVB-T] Buffer size set to %d samples (FFT: %d + Guard: %d)\n", 
                       buffer_size, config.fft_length, config.guard_length);
                printf("[DVB-T] Processing chain initialization successful!\n");
            } catch (...) {
                printf("[DVB-T] ERROR: Processing chain initialization failed!\n");
                cleanup();
                throw;
            }
        }
        
        void cleanup() {
            if (ofdm_acq) {
                delete ofdm_acq;
                ofdm_acq = nullptr;
            }
            if (demapper) {
                delete demapper;
                demapper = nullptr;
            }
            if (viterbi) {
                delete viterbi;
                viterbi = nullptr;
            }
            if (tps_decoder) {
                delete tps_decoder;
                tps_decoder = nullptr;
            }
        }

        inline int process(int count, const dsp::complex_t* in, uint8_t* out) {
            printf("[DVB-T] CRASH DEBUG: process() called with count=%d, in=%p, out=%p\n", count, in, out);
            
            if (!in || !out || count <= 0) {
                printf("[DVB-T] CRASH DEBUG: Invalid parameters - in=%p, out=%p, count=%d\n", in, out, count);
                return 0;
            }
            
            printf("[DVB-T] CRASH DEBUG: Parameters validated, proceeding with processing\n");
            
            // Simple pass-through for now - copy count bytes  
            int output_count = std::min(count/4, count);
            printf("[DVB-T] CRASH DEBUG: Calculated output_count=%d (count/4=%d)\n", output_count, count/4);
            
            try {
                printf("[DVB-T] CRASH DEBUG: Starting data conversion loop\n");
                for (int i = 0; i < output_count; i++) {
                    if (i < count) {
                        float magnitude = std::sqrt(in[i].re * in[i].re + in[i].im * in[i].im);
                        out[i] = (uint8_t)std::min(255.0f, magnitude * 255.0f);
                    } else {
                        out[i] = 0;
                    }
                    
                    // Add periodic checks during loop
                    if (i > 0 && i % 1000 == 0) {
                        printf("[DVB-T] CRASH DEBUG: Processed %d/%d samples\n", i, output_count);
                    }
                }
                printf("[DVB-T] CRASH DEBUG: Data conversion loop completed successfully\n");
            } catch (const std::exception& e) {
                printf("[DVB-T] CRASH DEBUG: Exception in data conversion: %s\n", e.what());
                return 0;
            } catch (...) {
                printf("[DVB-T] CRASH DEBUG: Unknown exception in data conversion\n");
                return 0;
            }
            
            printf("[DVB-T] CRASH DEBUG: process() returning %d\n", output_count);
            return output_count;
        }

        int run() {
            printf("[DVB-T] CRASH DEBUG: Starting run() function\n");
            
            printf("[DVB-T] CRASH DEBUG: About to call _in->read()\n");
            int count = 0;
            try {
                count = base_type::_in->read();
                printf("[DVB-T] CRASH DEBUG: _in->read() returned %d\n", count);
            } catch (const std::exception& e) {
                printf("[DVB-T] CRASH DEBUG: Exception in _in->read(): %s\n", e.what());
                return -1;
            } catch (...) {
                printf("[DVB-T] CRASH DEBUG: Unknown exception in _in->read()\n");
                return -1;
            }
            
            if (count < 0) { 
                printf("[DVB-T] CRASH DEBUG: Read returned negative count: %d\n", count);
                return -1; 
            }

            static int debug_counter = 0;
            if (debug_counter % 1000 == 0) { // Debug every 1000 calls
                printf("[DVB-T] Processing %d samples, auto-detect: %s, locked: %s\n", 
                       count, auto_detect_enabled ? "ON" : "OFF", parameters_locked ? "YES" : "NO");
            }

            printf("[DVB-T] CRASH DEBUG: Checking auto-detect conditions\n");
            printf("[DVB-T] CRASH DEBUG: auto_detect_enabled=%s, parameters_locked=%s, count=%d, symbol_length=%d\n",
                   auto_detect_enabled ? "true" : "false", 
                   parameters_locked ? "true" : "false", 
                   count, config.symbol_length);

            // Auto-detect parameters if enabled and not locked
            if (auto_detect_enabled && !parameters_locked && count >= config.symbol_length) {
                printf("[DVB-T] CRASH DEBUG: Entering auto-detect block\n");
                
                // Try to decode TPS from incoming symbols
                if (symbol_count_for_tps < 68) { // Need enough symbols for TPS
                    printf("[DVB-T] CRASH DEBUG: symbol_count_for_tps=%d, continuing TPS detection\n", symbol_count_for_tps);
                    
                    if (debug_counter % 500 == 0) { // Less frequent debug for TPS
                        printf("[DVB-T] TPS Detection: Collected %d/%d symbols\n", symbol_count_for_tps, 68);
                    }
                    
                    printf("[DVB-T] CRASH DEBUG: About to check tps_decoder pointer: %p\n", tps_decoder);
                    if (!tps_decoder) {
                        printf("[DVB-T] CRASH DEBUG: ERROR - tps_decoder is null!\n");
                    } else {
                        printf("[DVB-T] CRASH DEBUG: tps_decoder is valid, checking readBuf\n");
                        
                        printf("[DVB-T] CRASH DEBUG: base_type::_in = %p\n", base_type::_in);
                        if (!base_type::_in) {
                            printf("[DVB-T] CRASH DEBUG: ERROR - base_type::_in is null!\n");
                        } else {
                            printf("[DVB-T] CRASH DEBUG: About to access readBuf\n");
                            try {
                                auto* readBuf = base_type::_in->readBuf;
                                printf("[DVB-T] CRASH DEBUG: readBuf = %p\n", readBuf);
                                
                                if (!readBuf) {
                                    printf("[DVB-T] CRASH DEBUG: ERROR - readBuf is null!\n");
                                } else {
                                    printf("[DVB-T] CRASH DEBUG: About to call tps_decoder->decodeTPS()\n");
                                    
                                    int symbols_to_process = count / config.symbol_length;
                                    printf("[DVB-T] CRASH DEBUG: symbols_to_process = %d\n", symbols_to_process);
                                    
                                    if (symbols_to_process > 0) {
                                        bool tps_result = false;
                                        try {
                                            tps_result = tps_decoder->decodeTPS(readBuf, symbols_to_process);
                                            printf("[DVB-T] CRASH DEBUG: tps_decoder->decodeTPS() returned %s\n", 
                                                   tps_result ? "true" : "false");
                                        } catch (const std::exception& e) {
                                            printf("[DVB-T] CRASH DEBUG: Exception in decodeTPS: %s\n", e.what());
                                        } catch (...) {
                                            printf("[DVB-T] CRASH DEBUG: Unknown exception in decodeTPS\n");
                                        }
                                        
                                        if (tps_result) {
                                            parameters_locked = true;
                                            
                                            printf("[DVB-T] *** TPS LOCK ACHIEVED! ***\n");
                                            printf("[DVB-T] Auto-detected parameters:\n");
                                            printf("[DVB-T]   Constellation: %s\n", 
                                                   (config.constellation == DVBT_QPSK) ? "QPSK" : 
                                                   (config.constellation == DVBT_QAM16) ? "16QAM" : "64QAM");
                                            printf("[DVB-T]   Mode: %s\n", (config.transmission_mode == DVBT_2K) ? "2K" : "8K");
                                            printf("[DVB-T]   Guard: %s\n", 
                                                   (config.guard_interval == DVBT_GI_1_32) ? "1/32" :
                                                   (config.guard_interval == DVBT_GI_1_16) ? "1/16" :
                                                   (config.guard_interval == DVBT_GI_1_8) ? "1/8" : "1/4");
                                            printf("[DVB-T]   Code Rate: %s\n",
                                                   (config.code_rate_HP == DVBT_CR_1_2) ? "1/2" :
                                                   (config.code_rate_HP == DVBT_CR_2_3) ? "2/3" :
                                                   (config.code_rate_HP == DVBT_CR_3_4) ? "3/4" :
                                                   (config.code_rate_HP == DVBT_CR_5_6) ? "5/6" : "7/8");
                                            
                                            // Reinitialize processing chain with detected parameters
                                            printf("[DVB-T] Reinitializing processing chain with detected parameters...\n");
                                            try {
                                                printf("[DVB-T] CRASH DEBUG: About to call cleanup()\n");
                                                cleanup();
                                                printf("[DVB-T] CRASH DEBUG: cleanup() completed\n");
                                                
                                                printf("[DVB-T] CRASH DEBUG: About to call initProcessingChain()\n");
                                                initProcessingChain();
                                                printf("[DVB-T] CRASH DEBUG: initProcessingChain() completed\n");
                                                
                                                printf("[DVB-T] Ready for signal processing!\n");
                                            } catch (const std::exception& e) {
                                                printf("[DVB-T] CRASH DEBUG: Exception during reinit: %s\n", e.what());
                                            } catch (...) {
                                                printf("[DVB-T] CRASH DEBUG: Unknown exception during reinit\n");
                                            }
                                        }
                                    }
                                    symbol_count_for_tps += symbols_to_process;
                                    printf("[DVB-T] CRASH DEBUG: symbol_count_for_tps updated to %d\n", symbol_count_for_tps);
                                }
                            } catch (const std::exception& e) {
                                printf("[DVB-T] CRASH DEBUG: Exception accessing readBuf: %s\n", e.what());
                            } catch (...) {
                                printf("[DVB-T] CRASH DEBUG: Unknown exception accessing readBuf\n");
                            }
                        }
                    }
                } else {
                    printf("[DVB-T] CRASH DEBUG: Already collected enough TPS symbols (%d)\n", symbol_count_for_tps);
                }
            } else {
                printf("[DVB-T] CRASH DEBUG: Skipping auto-detect (conditions not met)\n");
            }

            printf("[DVB-T] CRASH DEBUG: About to call process() function\n");
            int outCount = 0;
            try {
                printf("[DVB-T] CRASH DEBUG: Calling process with count=%d\n", count);
                outCount = process(count, base_type::_in->readBuf, base_type::out.writeBuf);
                printf("[DVB-T] CRASH DEBUG: process() returned %d\n", outCount);
            } catch (const std::exception& e) {
                printf("[DVB-T] CRASH DEBUG: Exception in process(): %s\n", e.what());
                return -1;
            } catch (...) {
                printf("[DVB-T] CRASH DEBUG: Unknown exception in process()\n");
                return -1;
            }
            
            if (debug_counter % 2000 == 0) { // Debug output processing
                printf("[DVB-T] Processed %d samples -> %d output bytes\n", count, outCount);
            }
            
            printf("[DVB-T] CRASH DEBUG: About to call _in->flush()\n");
            try {
                base_type::_in->flush();
                printf("[DVB-T] CRASH DEBUG: _in->flush() completed\n");
            } catch (const std::exception& e) {
                printf("[DVB-T] CRASH DEBUG: Exception in flush(): %s\n", e.what());
            } catch (...) {
                printf("[DVB-T] CRASH DEBUG: Unknown exception in flush()\n");
            }
            
            if (outCount > 0) {
                printf("[DVB-T] CRASH DEBUG: About to call out.swap(%d)\n", outCount);
                try {
                    if (!base_type::out.swap(outCount)) { 
                        printf("[DVB-T] CRASH DEBUG: out.swap() returned false\n");
                        return -1; 
                    }
                    printf("[DVB-T] CRASH DEBUG: out.swap() completed successfully\n");
                } catch (const std::exception& e) {
                    printf("[DVB-T] CRASH DEBUG: Exception in out.swap(): %s\n", e.what());
                    return -1;
                } catch (...) {
                    printf("[DVB-T] CRASH DEBUG: Unknown exception in out.swap()\n");
                    return -1;
                }
            }
            
            debug_counter++;
            printf("[DVB-T] CRASH DEBUG: run() function completed successfully, returning %d\n", outCount);
            return outCount;
        }
        
        // Set bandwidth and enable auto-detection
        void setBandwidth(float bw_mhz) {
            printf("[DVB-T] Setting bandwidth to %.1f MHz\n", bw_mhz);
            bandwidth_mhz = bw_mhz;
            auto_detect_enabled = true;
            parameters_locked = false;
            symbol_count_for_tps = 0;
            if (tps_decoder) {
                printf("[DVB-T] Resetting TPS decoder for new signal\n");
                tps_decoder->reset();
            }
            printf("[DVB-T] Ready for auto-detection on new bandwidth\n");
        }
        
        // Get current detected parameters
        DVBTConfig getDetectedConfig() const {
            return config;
        }
        
        bool isParametersLocked() const {
            return parameters_locked;
        }
        
        // Configuration methods to match DVB-S interface
        void setConfig(dsp::dvbt::dvbt_constellation_t constellation,
                      dsp::dvbt::dvbt_transmission_mode_t transmission_mode,
                      dsp::dvbt::dvbt_guard_interval_t guard_interval,
                      dsp::dvbt::dvbt_code_rate_t code_rate_hp,
                      dsp::dvbt::dvbt_code_rate_t code_rate_lp) {
            config = DVBTConfig(constellation, transmission_mode, guard_interval, code_rate_hp, code_rate_lp);
            if (base_type::_block_init) {
                cleanup();
                initProcessingChain();
            }
        }
        
        // Add setInput method with debug logging
        void setInput(dsp::stream<dsp::complex_t>* input) {
            printf("[DVB-T] ==> setInput() method entered\n");
            printf("[DVB-T] ==> Input parameter address: %p\n", input);
            printf("[DVB-T] ==> About to validate input parameter...\n");
            
            if (!input) {
                printf("[DVB-T] ==> ERROR: Input stream is null!\n");
                throw std::invalid_argument("Input stream cannot be null");
            }
            printf("[DVB-T] ==> Input parameter is valid\n");
            printf("[DVB-T] ==> About to call init()...\n");
            try {
                printf("[DVB-T] ==> Calling init method...\n");
                init(input);
                printf("[DVB-T] ==> init() completed successfully\n");
                printf("[DVB-T] ==> Block is now initialized: _block_init = %s\n", base_type::_block_init ? "true" : "false");
            } catch (const std::exception& e) {
                printf("[DVB-T] ==> ERROR in init(): %s\n", e.what());
                throw;
            } catch (...) {
                printf("[DVB-T] ==> ERROR: Unknown exception in init()\n");
                throw;
            }
            printf("[DVB-T] ==> setInput() method completed successfully\n");
        }

        // Add start method with debug logging and _block_init check
        void start() {
            printf("[DVB-T] ==> start() method entered\n");
            printf("[DVB-T] ==> Checking if block is initialized: _block_init = %s\n", base_type::_block_init ? "true" : "false");
            if (!base_type::_block_init) {
                printf("[DVB-T] ==> ERROR: Block not initialized - call setInput first!\n");
                return; // Prevent assertion failure
            }
            
            printf("[DVB-T] ==> About to validate demodulator state...\n");
            
            if (!ofdm_acq || !demapper || !viterbi || !tps_decoder) {
                printf("[DVB-T] ==> ERROR: Processing chain not properly initialized!\n");
                printf("[DVB-T] ==> ofdm_acq: %p, demapper: %p, viterbi: %p, tps_decoder: %p\n", 
                       ofdm_acq, demapper, viterbi, tps_decoder);
                throw std::runtime_error("Processing chain not initialized");
            }
            printf("[DVB-T] ==> Processing chain validation passed\n");
            
            printf("[DVB-T] ==> About to call base_type::start()...\n");
            try {
                printf("[DVB-T] ==> Calling base class start method...\n");
                base_type::start();
                printf("[DVB-T] ==> base_type::start() completed successfully\n");
            } catch (const std::exception& e) {
                printf("[DVB-T] ==> ERROR in base_type::start(): %s\n", e.what());
                throw;
            } catch (...) {
                printf("[DVB-T] ==> ERROR: Unknown exception in base_type::start()\n");
                throw;
            }
            printf("[DVB-T] ==> start() method completed successfully\n");
        }
    };
}
