#include "module_dvbt_demod.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace dsp::dvbt {

    DVBTDemod::DVBTDemod(stream<complex_t>* in, int bandwidth_mhz) {
        init(in, bandwidth_mhz);
    }

    void DVBTDemod::init(stream<complex_t>* in, int bandwidth_mhz) {
        if (debug_output) {
            flog::info("DVB-T Demodulator: Starting initialization with bandwidth {} MHz", bandwidth_mhz);
        }
        
        this->bandwidth_mhz = bandwidth_mhz;
        this->samplerate = bandwidth_mhz * 1000000.0;
        
        // Initialize AGC - use a dummy stream initially
        if (debug_output) {
            flog::info("DVB-T Demodulator: Initializing AGC");
        }
        try {
            agc.init(nullptr, 1.0f, 65536);
        } catch (...) {
            flog::error("DVB-T Demodulator: AGC initialization failed");
            throw;
        }
        
        // Set up OFDM parameters based on bandwidth
        // For now, assume 8K mode - will be updated from TPS
        fft_size = 8192;
        guard_samples = fft_size / 32;  // 1/32 guard interval default
        useful_carriers = 6817;
        pilot_carriers = 177;
        tps_carriers = 68;
        
        if (debug_output) {
            flog::info("DVB-T Demodulator: OFDM parameters - FFT size: {}, guard: {}", fft_size, guard_samples);
        }
        
        // Allocate FFTW buffers with error checking
        if (debug_output) {
            flog::info("DVB-T Demodulator: Allocating FFTW buffers");
        }
        
        fft_in = (fftwf_complex*)fftwf_malloc(fft_size * sizeof(fftwf_complex));
        if (!fft_in) {
            flog::error("DVB-T Demodulator: Failed to allocate FFTW input buffer");
            throw std::runtime_error("Failed to allocate FFTW input buffer");
        }
        
        fft_out = (fftwf_complex*)fftwf_malloc(fft_size * sizeof(fftwf_complex));
        if (!fft_out) {
            flog::error("DVB-T Demodulator: Failed to allocate FFTW output buffer");
            fftwf_free(fft_in);
            fft_in = nullptr;
            throw std::runtime_error("Failed to allocate FFTW output buffer");
        }
        
        if (debug_output) {
            flog::info("DVB-T Demodulator: Creating FFTW plan");
        }
        
        fft_plan = fftwf_plan_dft_1d(fft_size, fft_in, fft_out, FFTW_FORWARD, FFTW_ESTIMATE);
        if (!fft_plan) {
            flog::error("DVB-T Demodulator: Failed to create FFTW plan");
            fftwf_free(fft_in);
            fftwf_free(fft_out);
            fft_in = nullptr;
            fft_out = nullptr;
            throw std::runtime_error("Failed to create FFTW plan");
        }
        
        if (debug_output) {
            flog::info("DVB-T Demodulator: Allocating processing buffers");
        }
        
        // Allocate buffers with exception handling
        try {
            fft_buffer.resize(fft_size);
            ofdm_buffer.resize(fft_size + guard_samples);
            pilot_buffer.resize(pilot_carriers);
            data_buffer.resize(useful_carriers);
            channel_estimate.resize(fft_size);
            pilot_estimates.resize(pilot_carriers);
            ts_packet_buffer.resize(204 * 8);  // Multiple RS-coded packets
            ts_sync_buffer.resize(188 * 16);   // Buffer for TS sync
        } catch (const std::exception& e) {
            flog::error("DVB-T Demodulator: Buffer allocation failed: {}", e.what());
            // Clean up FFTW resources
            if (fft_plan) { fftwf_destroy_plan(fft_plan); fft_plan = nullptr; }
            if (fft_in) { fftwf_free(fft_in); fft_in = nullptr; }
            if (fft_out) { fftwf_free(fft_out); fft_out = nullptr; }
            throw;
        }
        
        // Initialize TPS info
        tps_info = TPSInfo();
        tps_buffer.resize(68);  // TPS is 68 bits
        
        // Initialize channel estimate with unity gain
        std::fill(channel_estimate.begin(), channel_estimate.end(), complex_t{1.0f, 0.0f});
        
        if (debug_output) {
            flog::info("DVB-T Demodulator: Initializing processor chain");
        }
        
        // Initialize processing chain
        try {
            Processor<complex_t, uint8_t>::init(in);
        } catch (const std::exception& e) {
            flog::error("DVB-T Demodulator: Processor initialization failed: {}", e.what());
            // Clean up resources
            if (fft_plan) { fftwf_destroy_plan(fft_plan); fft_plan = nullptr; }
            if (fft_in) { fftwf_free(fft_in); fft_in = nullptr; }
            if (fft_out) { fftwf_free(fft_out); fft_out = nullptr; }
            throw;
        }
        
        if (debug_output) {
            flog::info("DVB-T Demodulator: Initialization complete - Bandwidth: {} MHz", bandwidth_mhz);
        }
    }

    void DVBTDemod::setBandwidth(int bandwidth_mhz) {
        if (this->bandwidth_mhz != bandwidth_mhz) {
            this->bandwidth_mhz = bandwidth_mhz;
            this->samplerate = bandwidth_mhz * 1000000.0;
            
            if (debug_output) {
                flog::info("DVB-T Bandwidth changed to {} MHz", bandwidth_mhz);
            }
            
            reset();
        }
    }

    void DVBTDemod::setSamplerate(double samplerate) {
        this->samplerate = samplerate;
    }

    void DVBTDemod::reset() {
        std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
        
        // Reset synchronization state
        symbol_counter = 0;
        frame_counter = 0;
        frame_sync = false;
        tps_sync_count = 0;
        tps_error_count = 0;
        ts_sync_state = 0;
        ts_packet_count = 0;
        
        // Reset TPS info
        tps_info = TPSInfo();
        
        // Clear buffers
        std::fill(fft_buffer.begin(), fft_buffer.end(), complex_t{0, 0});
        std::fill(ofdm_buffer.begin(), ofdm_buffer.end(), complex_t{0, 0});
        std::fill(pilot_buffer.begin(), pilot_buffer.end(), complex_t{0, 0});
        std::fill(data_buffer.begin(), data_buffer.end(), complex_t{0, 0});
        std::fill(channel_estimate.begin(), channel_estimate.end(), complex_t{1.0f, 0.0f});
        
        if (debug_output) {
            flog::info("DVB-T Demodulator reset");
        }
    }

    void DVBTDemod::start() {
        std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
        if (base_type::running) { return; }
        
        agc.start();
        base_type::start();
        
        if (debug_output) {
            flog::info("DVB-T Demodulator started");
        }
    }

    void DVBTDemod::stop() {
        std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
        if (!base_type::running) { return; }
        
        base_type::stop();
        agc.stop();
        
        // Clean up FFTW resources
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
        
        if (debug_output) {
            flog::info("DVB-T Demodulator stopped");
        }
    }

    void DVBTDemod::setConstellationHandler(void (*handler)(complex_t* data, int count, void* ctx), void* ctx) {
        constellation_handler = handler;
        constellation_ctx = ctx;
    }

    int DVBTDemod::run() {
        worker();
        return 0;
    }

    void DVBTDemod::worker() {
        int count = base_type::_in->read();
        if (count < 0) { return; }

        // Apply AGC
        agc.setInput(base_type::_in);
        agc.process(count);
        
        // Process OFDM frames
        processOFDMFrame(agc.out.readBuf, count);
        
        base_type::_in->flush();
        agc.out.flush();
    }

    void DVBTDemod::processOFDMFrame(complex_t* samples, int count) {
        static int buffer_pos = 0;
        static std::vector<complex_t> frame_buffer(fft_size + guard_samples);
        
        // Collect samples for OFDM symbol processing
        for (int i = 0; i < count; i++) {
            frame_buffer[buffer_pos++] = samples[i];
            
            // When we have enough samples for one OFDM symbol
            if (buffer_pos >= fft_size + guard_samples) {
                // Remove cyclic prefix
                removeCyclicPrefix(frame_buffer.data(), fft_buffer.data());
                
                // Perform FFT
                performFFT(fft_buffer.data(), fft_buffer.data());
                
                // Extract pilots for channel estimation and synchronization
                extractPilots(fft_buffer.data(), pilot_buffer.data());
                
                // Update channel estimate
                updateChannelEstimate(pilot_buffer.data());
                
                // Calculate SNR and frequency offset
                calculateSNR(pilot_buffer.data());
                estimateFrequencyOffset(pilot_buffer.data());
                
                // Try to extract TPS information
                extractTPS(fft_buffer.data());
                
                // If TPS is locked, demodulate data
                if (tps_info.locked) {
                    // Apply channel correction
                    channelCorrection(fft_buffer.data());
                    
                    // Send constellation data for display
                    if (constellation_handler) {
                        constellation_handler(fft_buffer.data(), std::min(1024, fft_size), constellation_ctx);
                    }
                    
                    // Demodulate data carriers
                    static std::vector<uint8_t> raw_data(204 * 8);  // RS-coded data
                    demodulateData(fft_buffer.data(), raw_data.data());
                    
                    // Process through error correction chain
                    processTransportStream(raw_data.data(), raw_data.size());
                }
                
                buffer_pos = 0;
                symbol_counter++;
            }
        }
    }

    void DVBTDemod::removeCyclicPrefix(complex_t* input, complex_t* output) {
        // Skip guard interval samples
        memcpy(output, input + guard_samples, fft_size * sizeof(complex_t));
    }

    void DVBTDemod::performFFT(complex_t* time_domain, complex_t* freq_domain) {
        // Copy input to FFTW buffer
        for (int i = 0; i < fft_size; i++) {
            fft_in[i][0] = time_domain[i].re;
            fft_in[i][1] = time_domain[i].im;
        }
        
        // Execute FFT
        fftwf_execute(fft_plan);
        
        // Copy output from FFTW buffer
        for (int i = 0; i < fft_size; i++) {
            freq_domain[i].re = fft_out[i][0];
            freq_domain[i].im = fft_out[i][1];
        }
    }

    bool DVBTDemod::isPilotCarrier(int carrier_index, int symbol_index) {
        // DVB-T continual pilot pattern (simplified for 8K mode)
        // Continual pilots are at positions: 0, 48, 54, 87, 141, 156, 192, 201, 255, 279, 282, 333, 432, 450, 483, 525, 531, 618, 636, 714, 759, 765, 780, 804, 873, 888, 918, 939, 942, 969, 984, 1050, 1101, 1107, 1110, 1137, 1140, 1146, 1206, 1269, 1323, 1377, 1491, 1683, 1704, 1752, 1758, 1791, 1845, 1860, 1896, 1905, 1959, 1983, 1986, 2037, 2136, 2154, 2187, 2229, 2235, 2322, 2340, 2418, 2463, 2469, 2484, 2508, 2577, 2592, 2622, 2643, 2646, 2673, 2688, 2754, 2805, 2811, 2814, 2841, 2844, 2850, 2910, 2973, 3027, 3081, 3195, 3387, 3408, 3456, 3462, 3495, 3549, 3564, 3600, 3609, 3663, 3687, 3690, 3741, 3840, 3858, 3891, 3933, 3939, 4026, 4044, 4122, 4167, 4173, 4188, 4212, 4281, 4296, 4326, 4347, 4350, 4377, 4392, 4458, 4509, 4515, 4518, 4545, 4548, 4554, 4614, 4677, 4731, 4785, 4899, 5091, 5112, 5160, 5166, 5199, 5253, 5268, 5304, 5313, 5367, 5391, 5394, 5445, 5544, 5562, 5595, 5637, 5643, 5730, 5748, 5826, 5871, 5877, 5892, 5916, 5985, 6000, 6030, 6051, 6054, 6081, 6096, 6162, 6213, 6219, 6222, 6249, 6252, 6258, 6318, 6381, 6435, 6489, 6603, 6795, 6816
        
        // For simplicity, use a basic pattern - every 12th carrier starting from 0
        int k = carrier_index - (fft_size / 2);  // Convert to centered index
        return (k % 12 == 0) && (k >= -3409) && (k <= 3408);
    }

    bool DVBTDemod::isTpsCarrier(int carrier_index) {
        // TPS carriers are at specific positions in the DVB-T spectrum
        // Simplified implementation - in practice these are defined in the standard
        int k = carrier_index - (fft_size / 2);  // Convert to centered index
        return (k % 68 == 34) && (k >= -3409) && (k <= 3408);
    }

    complex_t DVBTDemod::getPilotReference(int carrier_index, int symbol_index) {
        // DVB-T pilot reference sequence
        // Simplified - in practice this follows a specific PRBS pattern
        int k = carrier_index - (fft_size / 2);
        float phase = (k * symbol_index * 2.0f * M_PI) / 1024.0f;
        return complex_t{cosf(phase), sinf(phase)};
    }

    void DVBTDemod::extractPilots(complex_t* symbols, complex_t* pilots) {
        // Extract pilot carriers from OFDM symbol using proper DVB-T pattern
        int pilot_idx = 0;
        for (int k = 0; k < fft_size && pilot_idx < pilot_carriers; k++) {
            if (isPilotCarrier(k, symbol_counter)) {
                pilots[pilot_idx++] = symbols[k];
            }
        }
    }

    void DVBTDemod::updateChannelEstimate(complex_t* pilots) {
        // Update channel estimate using pilots
        int pilot_idx = 0;
        for (int k = 0; k < fft_size; k++) {
            if (isPilotCarrier(k, symbol_counter) && pilot_idx < pilot_carriers) {
                complex_t reference = getPilotReference(k, symbol_counter);
                complex_t estimate = pilots[pilot_idx] / reference;
                
                // Simple low-pass filter for channel estimate
                float alpha = 0.1f;  // Smoothing factor
                channel_estimate[k] = channel_estimate[k] * (1.0f - alpha) + estimate * alpha;
                
                pilot_idx++;
            }
        }
        
        // Interpolate channel estimate for data carriers
        for (int k = 0; k < fft_size; k++) {
            if (!isPilotCarrier(k, symbol_counter)) {
                // Find nearest pilot carriers for interpolation
                int prev_pilot = k - 1;
                int next_pilot = k + 1;
                
                while (prev_pilot >= 0 && !isPilotCarrier(prev_pilot, symbol_counter)) {
                    prev_pilot--;
                }
                while (next_pilot < fft_size && !isPilotCarrier(next_pilot, symbol_counter)) {
                    next_pilot++;
                }
                
                if (prev_pilot >= 0 && next_pilot < fft_size) {
                    // Linear interpolation
                    float w = (float)(k - prev_pilot) / (next_pilot - prev_pilot);
                    channel_estimate[k] = channel_estimate[prev_pilot] * (1.0f - w) + 
                                         channel_estimate[next_pilot] * w;
                } else if (prev_pilot >= 0) {
                    channel_estimate[k] = channel_estimate[prev_pilot];
                } else if (next_pilot < fft_size) {
                    channel_estimate[k] = channel_estimate[next_pilot];
                }
            }
        }
    }

    void DVBTDemod::calculateSNR(complex_t* pilots) {
        if (pilot_carriers == 0) return;
        
        float signal_power = 0.0f;
        float noise_power = 0.0f;
        
        for (int i = 0; i < pilot_carriers; i++) {
            float magnitude = std::abs(pilots[i]);
            signal_power += magnitude * magnitude;
            
            // Estimate noise from pilot deviation
            float expected_magnitude = 1.0f;  // Pilots are typically normalized
            float error = magnitude - expected_magnitude;
            noise_power += error * error;
        }
        
        signal_power /= pilot_carriers;
        noise_power /= pilot_carriers;
        
        if (noise_power > 0) {
            tps_info.snr_estimate = 10.0f * log10f(signal_power / noise_power);
        }
    }

    void DVBTDemod::estimateFrequencyOffset(complex_t* pilots) {
        // Simplified frequency offset estimation using pilot phase
        static complex_t prev_pilots[177];
        static bool first_run = true;
        
        if (first_run) {
            memcpy(prev_pilots, pilots, pilot_carriers * sizeof(complex_t));
            first_run = false;
            return;
        }
        
        float phase_diff_sum = 0.0f;
        int valid_pilots = 0;
        
        for (int i = 0; i < pilot_carriers; i++) {
            if (std::abs(pilots[i]) > 0.1f && std::abs(prev_pilots[i]) > 0.1f) {
                complex_t correlation = pilots[i] * std::conj(prev_pilots[i]);
                phase_diff_sum += std::arg(correlation);
                valid_pilots++;
            }
        }
        
        if (valid_pilots > 0) {
            float avg_phase_diff = phase_diff_sum / valid_pilots;
            tps_info.frequency_offset = avg_phase_diff * samplerate / (2.0f * M_PI * fft_size);
        }
        
        memcpy(prev_pilots, pilots, pilot_carriers * sizeof(complex_t));
    }

    void DVBTDemod::extractTPS(complex_t* ofdm_symbols) {
        // TPS extraction from specific carriers
        static int tps_bit_counter = 0;
        static std::vector<uint8_t> tps_bits(68);
        
        // Extract TPS bits from TPS carriers
        int tps_idx = 0;
        for (int k = 0; k < fft_size && tps_idx < 68; k++) {
            if (isTpsCarrier(k)) {
                // DBPSK demodulation for TPS
                float phase = std::arg(ofdm_symbols[k]);
                tps_bits[tps_bit_counter++] = (phase > 0) ? 1 : 0;
                tps_idx++;
            }
        }
        
        // When we have collected all TPS bits
        if (tps_bit_counter >= 68) {
            // Decode TPS information
            // Bits 0-1: Mode (2K/8K)
            int mode_bits = (tps_bits[0] << 1) | tps_bits[1];
            tps_info.mode = (mode_bits == 0) ? DVBT_MODE_2K : DVBT_MODE_8K;
            
            // Bits 2-4: Modulation
            int mod_bits = (tps_bits[2] << 2) | (tps_bits[3] << 1) | tps_bits[4];
            switch (mod_bits) {
                case 0: tps_info.modulation = DVBT_MOD_QPSK; break;
                case 1: tps_info.modulation = DVBT_MOD_16QAM; break;
                case 2: tps_info.modulation = DVBT_MOD_64QAM; break;
                default: tps_info.modulation = DVBT_MOD_QPSK; break;
            }
            
            // Bits 5-7: Code rate HP
            int cr_hp_bits = (tps_bits[5] << 2) | (tps_bits[6] << 1) | tps_bits[7];
            switch (cr_hp_bits) {
                case 0: tps_info.code_rate_hp = DVBT_CR_1_2; break;
                case 1: tps_info.code_rate_hp = DVBT_CR_2_3; break;
                case 2: tps_info.code_rate_hp = DVBT_CR_3_4; break;
                case 3: tps_info.code_rate_hp = DVBT_CR_5_6; break;
                case 4: tps_info.code_rate_hp = DVBT_CR_7_8; break;
                default: tps_info.code_rate_hp = DVBT_CR_1_2; break;
            }
            
            // Bits 8-10: Guard interval
            int gi_bits = (tps_bits[8] << 2) | (tps_bits[9] << 1) | tps_bits[10];
            switch (gi_bits) {
                case 0: tps_info.guard_interval = DVBT_GI_1_32; break;
                case 1: tps_info.guard_interval = DVBT_GI_1_16; break;
                case 2: tps_info.guard_interval = DVBT_GI_1_8; break;
                case 3: tps_info.guard_interval = DVBT_GI_1_4; break;
                default: tps_info.guard_interval = DVBT_GI_1_32; break;
            }
            
            updateTPSLock();
            tps_bit_counter = 0;
            
            if (debug_output && tps_info.locked) {
                flog::info("DVB-T TPS Locked - Mode: {}, Modulation: {}, Code Rate: {}, Guard: {}", 
                    (tps_info.mode == DVBT_MODE_2K) ? "2K" : "8K",
                    (tps_info.modulation == DVBT_MOD_QPSK) ? "QPSK" : 
                    (tps_info.modulation == DVBT_MOD_16QAM) ? "16QAM" : "64QAM",
                    (tps_info.code_rate_hp == DVBT_CR_1_2) ? "1/2" :
                    (tps_info.code_rate_hp == DVBT_CR_2_3) ? "2/3" :
                    (tps_info.code_rate_hp == DVBT_CR_3_4) ? "3/4" :
                    (tps_info.code_rate_hp == DVBT_CR_5_6) ? "5/6" : "7/8",
                    (tps_info.guard_interval == DVBT_GI_1_32) ? "1/32" :
                    (tps_info.guard_interval == DVBT_GI_1_16) ? "1/16" :
                    (tps_info.guard_interval == DVBT_GI_1_8) ? "1/8" : "1/4");
            }
        }
    }

    void DVBTDemod::updateTPSLock() {
        // Simple TPS lock detection based on consistency
        static DVBTMode last_mode = DVBT_MODE_2K;
        static DVBTModulation last_modulation = DVBT_MOD_QPSK;
        static int consistent_count = 0;
        
        if (tps_info.mode == last_mode && tps_info.modulation == last_modulation) {
            consistent_count++;
            if (consistent_count >= 5) {  // Require 5 consistent readings
                tps_info.locked = true;
                tps_sync_count++;
            }
        } else {
            consistent_count = 0;
            tps_info.locked = false;
            tps_error_count++;
        }
        
        last_mode = tps_info.mode;
        last_modulation = tps_info.modulation;
    }

    void DVBTDemod::channelCorrection(complex_t* symbols) {
        // Apply channel correction using pilot-based channel estimation
        for (int i = 0; i < fft_size; i++) {
            if (std::abs(channel_estimate[i]) > 0.1f) {
                symbols[i] = symbols[i] / channel_estimate[i];
            }
        }
    }

    void DVBTDemod::demodulateData(complex_t* ofdm_symbols, uint8_t* output) {
        // Demodulate data carriers to produce raw bits
        static int byte_counter = 0;
        static uint8_t current_byte = 0;
        static int bit_counter = 0;
        
        // Extract data from non-pilot, non-TPS carriers
        for (int k = 0; k < fft_size && byte_counter < 204 * 8; k++) {
            // Skip pilot and TPS carriers
            if (!isPilotCarrier(k, symbol_counter) && !isTpsCarrier(k)) {
                complex_t symbol = ofdm_symbols[k];
                
                // Demodulate based on TPS modulation (QPSK only for now)
                if (tps_info.modulation == DVBT_MOD_QPSK) {
                    uint8_t bits = ((symbol.re > 0) ? 1 : 0) << 1;
                    bits |= (symbol.im > 0) ? 1 : 0;
                    
                    // Pack bits into bytes
                    current_byte = (current_byte << 2) | bits;
                    bit_counter += 2;
                    
                    if (bit_counter >= 8) {
                        output[byte_counter++] = current_byte;
                        current_byte = 0;
                        bit_counter = 0;
                        
                        if (byte_counter >= 204 * 8) break;  // Frame complete
                    }
                }
            }
        }
        
        // Reset for next frame
        if (byte_counter >= 204 * 8) {
            byte_counter = 0;
        }
    }

    void DVBTDemod::processTransportStream(uint8_t* raw_data, int length) {
        // Process through convolutional deinterleaver
        static std::vector<uint8_t> deinterleaved_data(204 * 8);
        deinterleaver.deinterleave(raw_data, deinterleaved_data.data(), length);
        
        // Process each RS-coded packet
        for (int i = 0; i < 8; i++) {
            uint8_t* packet = &deinterleaved_data[i * 204];
            
            // Reed-Solomon decode
            int rs_errors = reed_solomon.decode(packet);
            
            if (rs_errors >= 0) {  // Successfully corrected or no errors
                // Extract 188-byte TS packet
                uint8_t ts_packet[188];
                memcpy(ts_packet, packet, 188);
                
                // Find TS sync and output packet
                int sync_pos;
                if (findTSSync(ts_packet, 188, sync_pos)) {
                    // Output valid TS packet
                    if (base_type::out.writeBuf) {
                        memcpy(base_type::out.writeBuf, ts_packet, 188);
                        base_type::out.swap(188);
                        ts_packet_count++;
                    }
                }
            }
        }
    }

    bool DVBTDemod::findTSSync(uint8_t* data, int length, int& sync_pos) {
        // Look for TS sync byte (0x47)
        for (int i = 0; i < length; i++) {
            if (data[i] == 0x47) {
                sync_pos = i;
                return true;
            }
        }
        return false;
    }
} 