#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#include <imgui.h>
#include <config.h>
#include <core.h>
#include <gui/style.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <module.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <fstream>
#include "dvbs/module_dvbs_demod.h"
#include "dvbs2/module_dvbs2_demod.h"
#include "dvbs2/bbframe_ts_parser.h"
#include "dvbt/module_dvbt_demod.h"

#include <gui/widgets/constellation_diagram.h>
#include <gui/widgets/file_select.h>
#include <gui/widgets/volume_meter.h>

#include "gui_widgets.h"

SDRPP_MOD_INFO{
    /* Name:            */ "dvbs_demodulator",
    /* Description:     */ "DVB-S/DVB-S2 satellite decoder",
    /* Author:          */ "cpicoto",
    /* Version:         */ 1, 0, 0,
    /* Max instances    */ -1
};

#ifdef ENABLE_NNG_NETWORKING
#include <utils/net.h>
#endif
#include <utils/flog.h>

#define CONCAT(a, b)    ((std::string(a) + b).c_str())

ConfigManager config;

const char* s2ConstellationsTxt[] = {
    "QPSK",
    "8PSK",
    "16APSK",
    "32APSK",
};
const char* s2CoderatesTxt[] = {
    "1/4",
    "1/3",
    "2/5",
    "1/2",
    "3/5",
    "2/3",
    "3/4",
    "4/5",
    "5/6",
    "7/8",
    "8/9",
    "9/10",
};
const char* s2FramesizesTxt[] = {
    "NORM",
    "SHORT",
};
const char* s2PilotsTxt[] = {
    "-",
    "+",
};

#define DVBS2_DEMOD_SOF_THRES 0.6f
#define DVBS2_DEMOD_LDPC_RETRIES 16
#define CLOCK_RECOVERY_BW 0.00628f
#define CLOCK_RECOVERY_DAMPN_F 0.707f
#define CLOCK_RECOVERY_REL_LIM 0.02f
#define RRC_TAP_COUNT 65
#define RRC_ALPHA 0.35f
#define AGC_RATE 0.0001f
#define COSTAS_LOOP_BANDWIDTH 0.00628f
#define FLL_LOOP_BANDWIDTH 0.006f

class DVBSDemodulatorModule : public ModuleManager::Instance {
public:
    DVBSDemodulatorModule(std::string name) {
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Constructor starting for '{}'", name);
        
        try {
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Setting name");
            this->name = name;
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Loading configuration");
            // Load config
            config.acquire();
            
            // Set default configuration if not exists
            if (!config.conf.contains(name) || !config.conf[name].contains("hostname")) {
                flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Creating default configuration");
                config.conf[name]["hostname"] = "localhost";
                config.conf[name]["port"] = 8355;
                config.conf[name]["sending"] = false;
                config.conf[name]["dvbs_version"] = 0;
                config.conf[name]["dvbs_symrate"] = 250000;
                config.conf[name]["dvbs2_symrate"] = 250000;
                config.conf[name]["dvbs2_constellation"] = dsp::dvbs2::MOD_QPSK;
                config.conf[name]["dvbs2_coderate"] = dsp::dvbs2::C1_2;
                config.conf[name]["dvbs2_framesize"] = dsp::dvbs2::FECFRAME_SHORT;
                config.conf[name]["dvbs2_pilots"] = false;
                config.conf[name]["dvbs2_automodcod"] = false;
                config.conf[name]["dvbs_bandwidth"] = 500000.0f;
                config.conf[name]["dvbs2_bandwidth"] = 500000.0f;
                config.conf[name]["dvb_mode"] = 0; // DVB-S
                config.conf[name]["dvbt_bandwidth"] = 2; // MHz, default 2
            }
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Reading configuration values");
            strcpy(hostname, std::string(config.conf[name]["hostname"]).c_str());
            port = config.conf[name]["port"];
            bool startNow = config.conf[name]["sending"];
            dvbs_ver_selected = config.conf[name]["dvbs_version"];
            dvbs_sym_rate_disp = dvbs_sym_rate = config.conf[name]["dvbs_symrate"];
            dvbs2_sym_rate_disp = dvbs2_sym_rate = config.conf[name]["dvbs2_symrate"];
            dvbs2_cfg.constellation = config.conf[name]["dvbs2_constellation"];
            dvbs2_cfg.coderate = config.conf[name]["dvbs2_coderate"];
            dvbs2_cfg.framesize = config.conf[name]["dvbs2_framesize"];
            dvbs2_cfg.pilots = true; // init later
            auto_modcod = config.conf[name]["dvbs2_automodcod"];
            dvbs_bw = config.conf[name]["dvbs_bandwidth"];
            dvbs2_bw = config.conf[name]["dvbs2_bandwidth"];
            dvb_mode = config.conf[name]["dvb_mode"];
            dvbt_bandwidth = config.conf[name]["dvbt_bandwidth"];
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Config values - DVB mode: {}, DVB-T BW: {} MHz, Network: {}:{}", 
                      dvb_mode, dvbt_bandwidth, hostname, port);
            
            config.release(true);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Creating VFO");
            vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, dvbs_bw, dvbs_sym_rate*2.0f, 1000.0f, dvbs_sym_rate*2.5f, false);
            onUserChangedBandwidthHandler.handler = vfoUserChangedBandwidthHandler;
            onUserChangedBandwidthHandler.ctx = this;
            vfo->wtfVFO->onUserChangedBandwidth.bindHandler(&onUserChangedBandwidthHandler);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Calculating clock recovery coefficients");
            // Clock recovery coefficients
            float recov_bandwidth = CLOCK_RECOVERY_BW;
            float recov_dampningFactor = CLOCK_RECOVERY_DAMPN_F;
            float recov_denominator = (1.0f + 2.0*recov_dampningFactor*recov_bandwidth + recov_bandwidth*recov_bandwidth);
            float recov_mu = (4.0f * recov_dampningFactor * recov_bandwidth) / recov_denominator;
            float recov_omega = (4.0f * recov_bandwidth * recov_bandwidth) / recov_denominator;
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Creating constellation lists");
            // Create lists
            s2ConstellationsListTxt = "";
            for (int i = 0; i < 4; i++) {
                s2ConstellationsListTxt += s2ConstellationsTxt[i];
                if (i < 3) { s2ConstellationsListTxt += '\0'; }
            }
            
            s2CoderatesListTxt = "";
            for (int i = 0; i < 12; i++) {
                s2CoderatesListTxt += s2CoderatesTxt[i];
                if (i < 11) { s2CoderatesListTxt += '\0'; }
            }
            
            s2FramesizesListTxt = "";
            for (int i = 0; i < 2; i++) {
                s2FramesizesListTxt += s2FramesizesTxt[i];
                if (i < 1) { s2FramesizesListTxt += '\0'; }
            }
            
            s2PilotsListTxt = "";
            for (int i = 0; i < 2; i++) {
                s2PilotsListTxt += s2PilotsTxt[i];
                if (i < 1) { s2PilotsListTxt += '\0'; }
            }
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Initializing constellation diagram");
            constDiag.init();
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Initializing DVB-S demodulator");
            dvbsDemod.init(vfo->output, dvbs_sym_rate, dvbs_sym_rate*2, AGC_RATE, RRC_ALPHA, RRC_TAP_COUNT, COSTAS_LOOP_BANDWIDTH, FLL_LOOP_BANDWIDTH, recov_omega, recov_mu, _constDiagHandler, this, CLOCK_RECOVERY_REL_LIM);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Initializing DVB-S2 demodulator");
            dvbs2Demod.init(vfo->output, dvbs2_sym_rate, dvbs2_sym_rate*2, AGC_RATE, RRC_ALPHA, RRC_TAP_COUNT, COSTAS_LOOP_BANDWIDTH, FLL_LOOP_BANDWIDTH, recov_omega, recov_mu, _constDiagHandler, this, dsp::dvbs2::get_dvbs2_modcod(dvbs2_cfg), (dvbs2_cfg.framesize == dsp::dvbs2::FECFRAME_SHORT), (dvbs2_cfg.pilots), DVBS2_DEMOD_SOF_THRES, DVBS2_DEMOD_LDPC_RETRIES, CLOCK_RECOVERY_REL_LIM);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Reading DVB-S2 pilots config");
            config.acquire();
            dvbs2_cfg.pilots = config.conf[name]["dvbs2_pilots"];
            config.release();
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Updating S2 demodulator");
            updateS2Demod();
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: DVB-T demodulator will be initialized in enable()");
            // NOTE: DVB-T demodulator will be initialized in enable() after VFO is created
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Initializing DVB-T averaging arrays");
            // Initialize DVB-T averaging arrays
            for (int i = 0; i < 30; i++) {
                dvbt_snr_avg[i] = 0.0f;
                dvbt_freq_offset_avg[i] = 0.0f;
            }
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Initializing demod sink");
            demodSink.init(&dvbsDemod.out, _demodSinkHandler, this);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Setting initial mode");
            setMode();
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Checking if should start network");
            if(startNow) {
                flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Starting network");
                startNetwork();
            }
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Registering menu entry");
            gui::menu.registerEntry(name, menuHandler, this, this);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Constructor completed successfully for '{}'", name);
        } catch (const std::exception& e) {
            flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Constructor failed for '{}': {}", name, e.what());
            throw;
        } catch (...) {
            flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Constructor failed for '{}' with unknown error", name);
            throw;
        }
    }

    ~DVBSDemodulatorModule() {
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Destructor called for '{}'", name);
        
        try {
            if(isEnabled()) {
                flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Disabling module during destruction");
                disable();
            }
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Unregistering menu entry");
            gui::menu.removeEntry(name);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Unregistering sink stream");
            sigpath::sinkManager.unregisterStream(name);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Destructor completed for '{}'", name);
        } catch (const std::exception& e) {
            flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Destructor failed for '{}': {}", name, e.what());
        } catch (...) {
            flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Destructor failed for '{}' with unknown error", name);
        }
    }

    void postInit() {}

    void enable() {
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: enable() called for module '{}'", name);
        
        try {
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Creating VFO for module '{}'", name);
            vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, dvbs_sym_rate*2.0f, dvbs_sym_rate*2.0f, 1000.0f, dvbs_sym_rate*2.5f, false);
            vfo->wtfVFO->onUserChangedBandwidth.bindHandler(&onUserChangedBandwidthHandler);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: VFO created successfully");
            
            // Now initialize DVB-T demodulator with valid VFO
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Initializing DVB-T demodulator with bandwidth {} MHz", dvbt_bandwidth);
            dvbtDemod.init(vfo->output, dvbt_bandwidth);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Setting DVB-T constellation handler");
            dvbtDemod.setConstellationHandler(_constDiagHandler, this);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Enabling DVB-T debug output");
            dvbtDemod.setDebugOutput(true);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: DVB-T demodulator initialization successful");
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Setting initial mode");
            setMode();
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Module '{}' enabled successfully", name);
            enabled = true;
        } catch (const std::exception& e) {
            flog::error("DVB-S/DVB-S2/DVB-T Demodulator: enable() failed for '{}': {}", name, e.what());
            throw;
        } catch (...) {
            flog::error("DVB-S/DVB-S2/DVB-T Demodulator: enable() failed for '{}' with unknown error", name);
            throw;
        }
    }

    void disable() {
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: disable() called for module '{}'", name);
        
        try {
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Stopping demodulators");
            // Stop DSP here
            demodSink.stop();
            dvbsDemod.stop();
            dvbs2Demod.stop();
            dvbtDemod.stop();
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Deleting VFO");
            sigpath::vfoManager.deleteVFO(vfo);
            
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Module '{}' disabled successfully", name);
            enabled = false;
        } catch (const std::exception& e) {
            flog::error("DVB-S/DVB-S2/DVB-T Demodulator: disable() failed for '{}': {}", name, e.what());
        } catch (...) {
            flog::error("DVB-S/DVB-S2/DVB-T Demodulator: disable() failed for '{}' with unknown error", name);
        }
    }

    bool isEnabled() {
        return enabled;
    }

private:

    void startNetwork() {
        stopNetwork();
#ifdef ENABLE_NNG_NETWORKING
        try {
            conn = net::openudp(hostname, port);
        } catch (std::runtime_error& e) {
            flog::error("Network error: %s\n", e.what());
        }
#endif
    }

    void stopNetwork() {
#ifdef ENABLE_NNG_NETWORKING
        if (conn) { conn->close(); }
#endif
    }

    void setMode() {
        demodSink.stop();
        dvbsDemod.stop();
        dvbs2Demod.stop();
        dvbtDemod.stop();
        
        if(dvb_mode == 0) {  // DVB-S
            dvbs_ver_selected = 0;
            dvbsDemod.setInput(vfo->output);
            demodSink.setInput(&dvbsDemod.out);
            dvbsDemod.start();
            demodSink.start();
            setSymRate();
        } else if(dvb_mode == 1) {  // DVB-S2
            dvbs_ver_selected = 1;
            dvbs2Demod.setInput(vfo->output);
            demodSink.setInput(&dvbs2Demod.out);
            dvbs2Demod.start();
            demodSink.start();
            setSymRate();
        } else if(dvb_mode == 2) {  // DVB-T
            dvbtDemod.setInput(vfo->output);
            dvbtDemod.setBandwidth(dvbt_bandwidth);
            demodSink.setInput(&dvbtDemod.out);
            dvbtDemod.start();
            demodSink.start();
            setDVBTSampleRate();
        }
    }

    void setSymRate() {
        if(dvbs_ver_selected == 0) {
            if(dvbs_bw >= dvbs_sym_rate*2.0f) {
                dvbs_bw = dvbs_sym_rate*2.0f;
            }
            if(dvbs_bw <= dvbs_sym_rate/2.0f) {
                dvbs_bw = dvbs_sym_rate/2.0f;
            }
            vfo->setSampleRate(dvbs_sym_rate*2.0f, dvbs_bw);
            vfo->setBandwidthLimits(dvbs_sym_rate/2.0f, dvbs_sym_rate*2.5f, false);
            dvbsDemod.setSamplerate(dvbs_sym_rate*2);
            dvbsDemod.setSymbolrate(dvbs_sym_rate);
            dvbsDemod.reset();
        } else {
            if(dvbs2_bw >= dvbs2_sym_rate*2.0f) {
                dvbs2_bw = dvbs2_sym_rate*2.0f;
            }
            if(dvbs2_bw <= dvbs2_sym_rate/2.0f) {
                dvbs2_bw = dvbs2_sym_rate/2.0f;
            }
            vfo->setSampleRate(dvbs2_sym_rate*2.0f, dvbs2_bw);
            vfo->setBandwidthLimits(dvbs2_sym_rate/2.0f, dvbs2_sym_rate*2.5f, false);
            dvbs2Demod.setSamplerate(dvbs2_sym_rate*2);
            dvbs2Demod.setSymbolrate(dvbs2_sym_rate);
            dvbs2Demod.reset();
        }
    }

    void updateS2Demod() {
        dvbs2_cfg = dsp::dvbs2::get_dvbs2_cfg(dsp::dvbs2::get_dvbs2_modcod(dvbs2_cfg), (dvbs2_cfg.framesize == dsp::dvbs2::FECFRAME_SHORT), (dvbs2_cfg.pilots));
        dvbs2Demod.setDemodParams(dsp::dvbs2::get_dvbs2_modcod(dvbs2_cfg), (dvbs2_cfg.framesize == dsp::dvbs2::FECFRAME_SHORT), (dvbs2_cfg.pilots), DVBS2_DEMOD_SOF_THRES, DVBS2_DEMOD_LDPC_RETRIES);
        dvbs2bbparser.setFrameSize(dvbs2Demod.getKBCH());
    }

    void setDVBTSampleRate() {
        double samplerate = dvbt_bandwidth * 1000000.0;  // Convert MHz to Hz
        double bandwidth = samplerate * 0.8;  // 80% of sample rate for bandwidth
        
        vfo->setSampleRate(samplerate, bandwidth);
        vfo->setBandwidthLimits(samplerate * 0.1, samplerate * 1.2, false);
        dvbtDemod.setSamplerate(samplerate);
        dvbtDemod.reset();
    }

    static void menuHandler(void* ctx) {
        DVBSDemodulatorModule* _this = (DVBSDemodulatorModule*)ctx;
        float menuWidth = ImGui::GetContentRegionAvail().x;

        if(!_this->enabled) {
            style::beginDisabled();
        }

        ImGui::BeginGroup();
        ImGui::Columns(3, CONCAT("DVBSModeColumns##_", _this->name), false);
        if (ImGui::RadioButton(CONCAT("DVB-S##_", _this->name), _this->dvb_mode == 0) && _this->dvb_mode != 0) {
            _this->dvb_mode = 0;
            _this->dvbs_ver_selected = 0;
            _this->setMode();
            config.acquire();
            config.conf[_this->name]["dvb_mode"] = _this->dvb_mode;
            config.release(true);
        }
        ImGui::NextColumn();
        if (ImGui::RadioButton(CONCAT("DVB-S2##_", _this->name), _this->dvb_mode == 1) && _this->dvb_mode != 1) {
            _this->dvb_mode = 1;
            _this->dvbs_ver_selected = 1;
            _this->setMode();
            config.acquire();
            config.conf[_this->name]["dvb_mode"] = _this->dvb_mode;
            config.release(true);
        }
        ImGui::NextColumn();
        if (ImGui::RadioButton(CONCAT("DVB-T##_", _this->name), _this->dvb_mode == 2) && _this->dvb_mode != 2) {
            _this->dvb_mode = 2;
            _this->setMode();
            config.acquire();
            config.conf[_this->name]["dvb_mode"] = _this->dvb_mode;
            config.release(true);
        }
        ImGui::Columns(1, CONCAT("EndDVBSModeColumns##_", _this->name), false);
        ImGui::EndGroup();

        if (_this->dvb_mode == 2) {
            ImGui::Text("Bandwidth (MHz):");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            static const char* bw_options[] = {"1", "2", "3", "4", "5", "6", "7", "8"};
            int bw_idx = _this->dvbt_bandwidth - 1;
            if (ImGui::Combo(CONCAT("##_dvbt_bandwidth_", _this->name), &bw_idx, bw_options, 8)) {
                _this->dvbt_bandwidth = bw_idx + 1;
                _this->setDVBTSampleRate();
                config.acquire();
                config.conf[_this->name]["dvbt_bandwidth"] = _this->dvbt_bandwidth;
                config.release(true);
            }
        }

#ifdef ENABLE_NNG_NETWORKING
        bool netActive = (_this->conn && _this->conn->isOpen());
#else
        bool netActive = false;
#endif
        if(netActive) { style::beginDisabled(); }
        if (ImGui::InputText(CONCAT("UDP ##_dvbsdemod_host_", _this->name), _this->hostname, 1023)) {
            config.acquire();
            config.conf[_this->name]["hostname"] = _this->hostname;
            config.release(true);
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::InputInt(CONCAT("##_dvbsdemod_port_", _this->name), &(_this->port), 0, 0)) {
            config.acquire();
            config.conf[_this->name]["port"] = _this->port;
            config.release(true);
        }
        if(netActive) { style::endDisabled(); }

        if (netActive && ImGui::Button(CONCAT("Net stop##_dvbsdemod_net_stop_", _this->name), ImVec2(menuWidth, 0))) {
            _this->stopNetwork();
            config.acquire();
            config.conf[_this->name]["sending"] = false;
            config.release(true);
        } else if (!netActive && ImGui::Button(CONCAT("Net start##_dvbsdemod_net_stop_", _this->name), ImVec2(menuWidth, 0))) {
            _this->startNetwork();
            config.acquire();
            config.conf[_this->name]["sending"] = true;
            config.release(true);
        }

        ImGui::TextUnformatted("Net status:");
        ImGui::SameLine();
        if (netActive) {
            if(_this->dvbs_ver_selected == 0) {
                if(_this->dvbsDemod.stats_viterbi_lock) {
                    ImGui::TextColored(ImVec4(0.0, 1.0, 0.0, 1.0), "Sending");
                } else {
                    ImGui::TextColored(ImVec4(1.0, 1.0, 0.0, 1.0), "Ready");
                }
            } else {
                if(_this->dvbs2bbparser.last_bb_proc > _this->dvbs2bbparser.last_bb_cnt/2) {
                    ImGui::TextColored(ImVec4(0.0, 1.0, 0.0, 1.0), "Sending");
                } else {
                    ImGui::TextColored(ImVec4(1.0, 1.0, 0.0, 1.0), "Ready");
                }
            }
        } else {
            ImGui::TextUnformatted("Idle");
        }
        if(_this->dvb_mode == 0) {  // DVB-S
            ImGui::Text("Symbol rate: ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            ImGui::InputInt(CONCAT("##_dvbsdemod_rate_", _this->name), &(_this->dvbs_sym_rate_disp), 1, 10);
            if (ImGui::Button(CONCAT("Apply##_dvbsdemod_rate_a_", _this->name))) {
                _this->dvbs_sym_rate = _this->dvbs_sym_rate_disp;
                if(_this->dvbs_sym_rate > 0) {
                    _this->setSymRate();
                }
                config.acquire();
                config.conf[_this->name]["dvbs_symrate"] = _this->dvbs_sym_rate;
                config.release(true);
            }
            _this->dvbs_viterbi_err_avg[_this->dvbs_viterbi_err_avg_ptr] = _this->dvbsDemod.stats_viterbi_ber;
            _this->dvbs_viterbi_err_avg_ptr++;
            if(_this->dvbs_viterbi_err_avg_ptr >= 30) _this->dvbs_viterbi_err_avg_ptr = 0;
            float avg_viterbi_err = 0;
            for(int i = 0; i < 30; i++) {
                avg_viterbi_err += _this->dvbs_viterbi_err_avg[i];
            }
            avg_viterbi_err /= 0.3f;
            avg_viterbi_err = 100.0f - avg_viterbi_err;
            ImGui::Text("Viterbi sig lvl: ");
            ImGui::SameLine();
            ImGui::SigQualityMeter(avg_viterbi_err, 60.0f, 100.0f);
            if(!_this->dvbsDemod.stats_viterbi_lock) {
                style::beginDisabled();
            }
            ImGui::Text("Viterbi detected rate: %s", _this->dvbsDemod.stats_viterbi_rate.c_str());
            ImGui::Text("Deframer errors: %d", _this->dvbsDemod.stats_deframer_err);
            ImGui::Text("Reed-solomon avg errors: %f", _this->dvbsDemod.stats_rs_avg);
            if(!_this->dvbsDemod.stats_viterbi_lock) {
                style::endDisabled();
            }
        } else if(_this->dvb_mode == 1) {  // DVB-S2
            ImGui::Text("Symbol rate: ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            ImGui::InputInt(CONCAT("##_dvbs2demod_rate_", _this->name), &(_this->dvbs2_sym_rate_disp), 1, 10);
            if (ImGui::Button(CONCAT("Apply##_dvbs2demod_rate_a_", _this->name))) {
                _this->dvbs2_sym_rate = _this->dvbs2_sym_rate_disp;
                if(_this->dvbs2_sym_rate > 0) {
                    _this->setSymRate();
                }
                config.acquire();
                config.conf[_this->name]["dvbs2_symrate"] = _this->dvbs2_sym_rate;
                config.release(true);
            }
            dsp::dvbs2::dvb_cgf_holder modcod_det;
            modcod_det.constellation = dsp::dvbs2::MOD_QPSK;
            modcod_det.coderate = dsp::dvbs2::C1_4;
            modcod_det.pilots = 0;
            modcod_det.framesize = dsp::dvbs2::FECFRAME_NORMAL;
            if(_this->dvbs2Demod.detected_modcod > 0 && _this->dvbs2Demod.detected_modcod < 29) {
                modcod_det = dsp::dvbs2::get_dvbs2_cfg(_this->dvbs2Demod.detected_modcod, _this->dvbs2Demod.detected_shortframes, _this->dvbs2Demod.detected_pilots);
            }
            _this->dvbs2_modcod_checkbuff[_this->dvbs2_modcod_checkbuff_ptr] = _this->dvbs2Demod.detected_modcod;
            _this->dvbs2_modcod_checkbuff_ptr++;
            if(_this->dvbs2_modcod_checkbuff_ptr >= 50) {
                _this->dvbs2_modcod_checkbuff_ptr = 0;
            }
            bool modcod_consistent = true;
            int modcod_check = _this->dvbs2_modcod_checkbuff[0];
            for(int i = 0; i < 50; i++) {
                if(_this->dvbs2_modcod_checkbuff[i] != modcod_check || _this->dvbs2_modcod_checkbuff[i] <= 0 || _this->dvbs2_modcod_checkbuff[i] >= 29) {
                    modcod_consistent = false;
                    break;
                }
            }
            if(_this->auto_modcod && modcod_consistent) {
                if(dsp::dvbs2::get_dvbs2_modcod(_this->dvbs2_cfg) != _this->dvbs2Demod.detected_modcod ||
                _this->dvbs2_cfg.pilots != _this->dvbs2Demod.detected_pilots) {
                    _this->dvbs2_cfg = dsp::dvbs2::get_dvbs2_cfg(dsp::dvbs2::get_dvbs2_modcod(modcod_det), (_this->dvbs2_cfg.framesize == dsp::dvbs2::FECFRAME_SHORT), (modcod_det.pilots));
                    _this->updateS2Demod();
                    config.acquire();
                    config.conf[_this->name]["dvbs2_constellation"] = _this->dvbs2_cfg.constellation;
                    config.conf[_this->name]["dvbs2_coderate"] = _this->dvbs2_cfg.coderate;
                    config.conf[_this->name]["dvbs2_framesize"] = _this->dvbs2_cfg.framesize;
                    config.conf[_this->name]["dvbs2_pilots"] = _this->dvbs2_cfg.pilots;
                    config.release(true);
                }
            }
            ImGui::Columns(2, CONCAT("DVBS2CFGCols##_", _this->name), false);
            ImGui::Text("Demod params");ImGui::NextColumn();ImGui::Text("Modcod detected");ImGui::NextColumn();
            ImGui::Text("Constellation:");ImGui::SameLine();
            if(_this->dvbs2_cfg.constellation >= 0 && _this->dvbs2_cfg.constellation < 4 && ImGui::Combo(CONCAT("##_dvbs2_constellation_sel_", _this->name), (int*)&_this->dvbs2_cfg.constellation, _this->s2ConstellationsListTxt.c_str())) {
                _this->updateS2Demod();
                config.acquire();
                config.conf[_this->name]["dvbs2_constellation"] = _this->dvbs2_cfg.constellation;
                config.release(true);
            }
            ImGui::NextColumn();ImGui::TextColored((modcod_consistent ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 0.0, 0.0, 1.0)), "%s", s2ConstellationsTxt[modcod_det.constellation]);ImGui::NextColumn();
            ImGui::Text("Coderate:");ImGui::SameLine();
            if(_this->dvbs2_cfg.coderate >= 0 && _this->dvbs2_cfg.coderate < 12 && ImGui::Combo(CONCAT("##_dvbs2_coderate_sel_", _this->name), (int*)&_this->dvbs2_cfg.coderate, _this->s2CoderatesListTxt.c_str())) {
                _this->updateS2Demod();
                config.acquire();
                config.conf[_this->name]["dvbs2_coderate"] = _this->dvbs2_cfg.coderate;
                config.release(true);
            }
            ImGui::NextColumn();ImGui::TextColored((modcod_consistent ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 0.0, 0.0, 1.0)),"%s", s2CoderatesTxt[modcod_det.coderate]);ImGui::NextColumn();
            ImGui::Text("Pilots:");ImGui::SameLine();
            if(ImGui::Checkbox(CONCAT("##_dvbs2_pilots_sel_", _this->name), &_this->dvbs2_cfg.pilots)) {
                _this->updateS2Demod();
                config.acquire();
                config.conf[_this->name]["dvbs2_pilots"] = _this->dvbs2_cfg.pilots;
                config.release(true);
            }
            ImGui::NextColumn();ImGui::TextColored((modcod_consistent ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 0.0, 0.0, 1.0)), "%s", s2PilotsTxt[modcod_det.pilots]);ImGui::NextColumn();
            ImGui::Text("Frames:");ImGui::SameLine();
            if(_this->dvbs2_cfg.framesize >= 0 && _this->dvbs2_cfg.framesize < 2 && ImGui::Combo(CONCAT("##_dvbs2_frames_sel_", _this->name), (int*)&_this->dvbs2_cfg.framesize, _this->s2FramesizesListTxt.c_str())) {
                _this->updateS2Demod();
                config.acquire();
                config.conf[_this->name]["dvbs2_framesize"] = _this->dvbs2_cfg.framesize;
                config.release(true);
            }
            ImGui::NextColumn();ImGui::TextColored((modcod_consistent ? ImVec4(0.0, 1.0, 0.0, 1.0) : ImVec4(1.0, 0.0, 0.0, 1.0)), "%s", s2FramesizesTxt[modcod_det.framesize]);ImGui::NextColumn();
            ImGui::Columns(1, CONCAT("EndDVBS2CFGCols##_", _this->name), false);
            ImGui::Text("AUTO modcod:");ImGui::SameLine();
            if(ImGui::Checkbox(CONCAT("##_dvbs2_automodcod_", _this->name), &_this->auto_modcod)) {
                config.acquire();
                config.conf[_this->name]["dvbs2_automodcod"] = _this->auto_modcod;
                config.release(true);
            }

            _this->dvbs2_pl_best_avg[_this->dvbs2_pl_best_avg_ptr] = _this->dvbs2Demod.pl_sync_best_match;
            _this->dvbs2_pl_best_avg_ptr++;
            if(_this->dvbs2_pl_best_avg_ptr >= 30) _this->dvbs2_pl_best_avg_ptr = 0;
            float avg_bestmatch = 0;
            for(int i = 0; i < 30; i++) {
                avg_bestmatch += _this->dvbs2_pl_best_avg[i];
            }
            avg_bestmatch /= 0.3f;
            // float avg_bestmatch = _this->dvbs2Demod.pl_sync_best_match * 100.0f;
            ImGui::Text("PLSync best match: %d", int(avg_bestmatch));
            ImGui::SameLine();
            ImGui::SigQualityMeter(avg_bestmatch, 20.0f, 100.0f);
            if(!(avg_bestmatch >= DVBS2_DEMOD_SOF_THRES*100.0f)) {
                style::beginDisabled();
            }
            ImGui::Text("LDPC trials: %d / %d", int(_this->dvbs2Demod.ldpc_trials), DVBS2_DEMOD_LDPC_RETRIES);

            _this->dvbs2_bch_corr_avg[_this->dvbs2_bch_corr_avg_ptr] = _this->dvbs2Demod.bch_corrections;
            _this->dvbs2_bch_corr_avg_ptr++;
            if(_this->dvbs2_bch_corr_avg_ptr >= 30) _this->dvbs2_bch_corr_avg_ptr = 0;
            _this->dvbs2_avg_bchcorr = 0;
            for(int i = 0; i < 30; i++) {
                _this->dvbs2_avg_bchcorr += _this->dvbs2_bch_corr_avg[i];
            }
            _this->dvbs2_avg_bchcorr /= 0.3f;
            _this->dvbs2_avg_bchcorr = 100.0f - _this->dvbs2_avg_bchcorr*0.1;
            if(_this->dvbs2bbparser.last_bb_proc < _this->dvbs2bbparser.last_bb_cnt/2) {
                ImGui::Text("BCH sig quality: 0");
                ImGui::SameLine();
                ImGui::SigQualityMeter(0, 20.0f, 100.0f);
                ImGui::Text("BBF procesed: %d/%d", _this->dvbs2bbparser.last_bb_proc, _this->dvbs2bbparser.last_bb_cnt);
                if(_this->dvbs2bbparser.last_bb_cnt != 0) {
                    ImGui::SameLine();
                    ImGui::SigQualityMeter(100.0f*((float)_this->dvbs2bbparser.last_bb_proc)/((float)_this->dvbs2bbparser.last_bb_cnt), 0.0f, 100.0f);
                }
            } else {
                ImGui::Text("BCH sig quality: %d", int(_this->dvbs2_avg_bchcorr));
                ImGui::SameLine();
                ImGui::SigQualityMeter(_this->dvbs2_avg_bchcorr, 20.0f, 100.0f);
                ImGui::Text("BBF processed: %d/%d", _this->dvbs2bbparser.last_bb_proc, _this->dvbs2bbparser.last_bb_cnt);
                if(_this->dvbs2bbparser.last_bb_cnt != 0) {
                    ImGui::SameLine();
                    ImGui::SigQualityMeter(100.0f*((float)_this->dvbs2bbparser.last_bb_proc)/((float)_this->dvbs2bbparser.last_bb_cnt), 0.0f, 100.0f);
                }
                ImGui::Text("BBFrame content: %s", (_this->dvbs2bbparser.last_header.ts_gs == 0b11 ? "MPEGTS" : (_this->dvbs2bbparser.last_header.ts_gs == 0b01 ? "GSE" : "UNK")));
                ImGui::Text("Input stream: %s", (_this->dvbs2bbparser.last_header.sis_mis ? "Single" : "Multiple"));
                ImGui::Text("Cod&Mod: %s", (_this->dvbs2bbparser.last_header.ccm_acm ? "CCM" : "ACM"));
                ImGui::Text("ISSY: %s", (_this->dvbs2bbparser.last_header.issyi ? "Y" : "N"));
                ImGui::Text("NPD: %s", (_this->dvbs2bbparser.last_header.npd ? "Y" : "N"));
                ImGui::Text("RO: %s", (_this->dvbs2bbparser.last_header.ro == 0b00 ? "0.35" : (_this->dvbs2bbparser.last_header.ro == 0b01 ? "0.25" : "0.20")));
            }
            if(!(avg_bestmatch >= DVBS2_DEMOD_SOF_THRES*100.0f)) {
                style::endDisabled();
            }
        } else if(_this->dvb_mode == 2) {  // DVB-T mode
            // Update DVB-T TPS info
            _this->dvbt_tps_info = _this->dvbtDemod.getTPS();
            
            // Display TPS lock status
            ImGui::Text("TPS Lock: ");
            ImGui::SameLine();
            if(_this->dvbt_tps_info.locked) {
                ImGui::TextColored(ImVec4(0.0, 1.0, 0.0, 1.0), "LOCKED");
            } else {
                ImGui::TextColored(ImVec4(1.0, 0.0, 0.0, 1.0), "UNLOCKED");
            }
            
            if(_this->dvbt_tps_info.locked) {
                // Display TPS parameters
                ImGui::Text("Mode: %s", (_this->dvbt_tps_info.mode == dsp::dvbt::DVBT_MODE_2K) ? "2K" : "8K");
                ImGui::Text("Modulation: %s", 
                    (_this->dvbt_tps_info.modulation == dsp::dvbt::DVBT_MOD_QPSK) ? "QPSK" :
                    (_this->dvbt_tps_info.modulation == dsp::dvbt::DVBT_MOD_16QAM) ? "16QAM" : "64QAM");
                ImGui::Text("Code Rate: %s",
                    (_this->dvbt_tps_info.code_rate_hp == dsp::dvbt::DVBT_CR_1_2) ? "1/2" :
                    (_this->dvbt_tps_info.code_rate_hp == dsp::dvbt::DVBT_CR_2_3) ? "2/3" :
                    (_this->dvbt_tps_info.code_rate_hp == dsp::dvbt::DVBT_CR_3_4) ? "3/4" :
                    (_this->dvbt_tps_info.code_rate_hp == dsp::dvbt::DVBT_CR_5_6) ? "5/6" : "7/8");
                ImGui::Text("Guard Interval: %s",
                    (_this->dvbt_tps_info.guard_interval == dsp::dvbt::DVBT_GI_1_32) ? "1/32" :
                    (_this->dvbt_tps_info.guard_interval == dsp::dvbt::DVBT_GI_1_16) ? "1/16" :
                    (_this->dvbt_tps_info.guard_interval == dsp::dvbt::DVBT_GI_1_8) ? "1/8" : "1/4");
                
                // Calculate and display averaged SNR
                _this->dvbt_snr_avg[_this->dvbt_snr_avg_ptr] = _this->dvbt_tps_info.snr_estimate;
                _this->dvbt_snr_avg_ptr++;
                if(_this->dvbt_snr_avg_ptr >= 30) _this->dvbt_snr_avg_ptr = 0;
                
                float avg_snr = 0;
                for(int i = 0; i < 30; i++) {
                    avg_snr += _this->dvbt_snr_avg[i];
                }
                avg_snr /= 30.0f;
                
                ImGui::Text("SNR: %.1f dB", avg_snr);
                ImGui::SameLine();
                ImGui::SigQualityMeter(avg_snr, 0.0f, 30.0f);
                
                // Calculate and display averaged frequency offset
                _this->dvbt_freq_offset_avg[_this->dvbt_freq_offset_avg_ptr] = _this->dvbt_tps_info.frequency_offset;
                _this->dvbt_freq_offset_avg_ptr++;
                if(_this->dvbt_freq_offset_avg_ptr >= 30) _this->dvbt_freq_offset_avg_ptr = 0;
                
                float avg_freq_offset = 0;
                for(int i = 0; i < 30; i++) {
                    avg_freq_offset += _this->dvbt_freq_offset_avg[i];
                }
                avg_freq_offset /= 30.0f;
                
                ImGui::Text("Freq Offset: %.1f Hz", avg_freq_offset);
            } else {
                ImGui::Text("Searching for TPS...");
            }
        }
        ImGui::Text("Signal+PLSCode constellation: ");
        ImGui::SetNextItemWidth(menuWidth);
        _this->constDiag.draw();

        if(!_this->enabled) {
            style::endDisabled();
        }
    }

    static void _constDiagHandler(dsp::complex_t* data, int count, void* ctx) {
        DVBSDemodulatorModule* _this = (DVBSDemodulatorModule*)ctx;
        int curidx = 0;
        if(_this->dvbs_ver_selected == 0) {
            dsp::complex_t* cdBuff = _this->constDiag.acquireBuffer();
            memcpy(cdBuff, data, std::min((uint32_t)count, CONSTDIAG_SIZE) * sizeof(dsp::complex_t));
            _this->constDiag.releaseBuffer(std::min((uint32_t)count, CONSTDIAG_SIZE));
        } else {
            dsp::complex_t* cdBuff = _this->constDiag.acquireBuffer();
            memcpy(cdBuff, &data[90], std::min((uint32_t)count-90, CONSTDIAG_SIZE) * sizeof(dsp::complex_t));
            _this->constDiag.releaseBuffer(std::min((uint32_t)count-90, CONSTDIAG_SIZE));
            cdBuff = _this->constDiag.acquireRedBuffer();
            memcpy(cdBuff, &data[0], 90 * sizeof(dsp::complex_t));
            _this->constDiag.releaseRedBuffer(90);
        }
    }

    static void _demodSinkHandler(uint8_t* data, int count, void* ctx) {
        DVBSDemodulatorModule* _this = (DVBSDemodulatorModule*)ctx;
        if(_this->dvbs_ver_selected == 0) {
#ifdef ENABLE_NNG_NETWORKING
            if(_this->conn && _this->conn->isOpen())
                _this->conn->send(data, count);
#endif
        } else {
            int cnt = _this->dvbs2bbparser.work(data, count/(_this->dvbs2Demod.getKBCH()/8), _this->packetbuff, 65536*10);
            if(_this->dvbs2bbparser.last_header.ts_gs == 0b11) {
                //MPEGTS
                if(cnt > 0 && (cnt%188)==0) {
#ifdef ENABLE_NNG_NETWORKING
                    for(int k = 0; k < cnt/1880; k++) {
                        if(_this->conn && _this->conn->isOpen())
                            _this->conn->send(&_this->packetbuff[1880*k], 1880);
                    }
                    int rem = cnt%1880;
                    if(rem != 0 && _this->conn && _this->conn->isOpen())
                            _this->conn->send(&_this->packetbuff[cnt-rem], rem);
#endif
                }
            } else {
                //GSE/OTHER
                if(cnt > 0) {
#ifdef ENABLE_NNG_NETWORKING
                    if(_this->conn && _this->conn->isOpen()) 
                        _this->conn->send(_this->packetbuff, cnt);
#endif
                }
            }
        }
    }

    static void vfoUserChangedBandwidthHandler(double newBw, void* ctx) {
        DVBSDemodulatorModule* _this = (DVBSDemodulatorModule*)ctx;
        _this->vfo->setBandwidth(newBw);
        if(_this->dvb_mode == 0) {  // DVB-S
            _this->dvbs_bw = newBw;
            config.acquire();
            config.conf[_this->name]["dvbs_bandwidth"] = _this->dvbs_bw;
            config.release(true);
        } else if(_this->dvb_mode == 1) {  // DVB-S2
            _this->dvbs2_bw = newBw;
            config.acquire();
            config.conf[_this->name]["dvbs2_bandwidth"] = _this->dvbs2_bw;
            config.release(true);
        } else if(_this->dvb_mode == 2) {  // DVB-T
            // DVB-T bandwidth is controlled by the bandwidth dropdown, not VFO
            // So we don't need to do anything here for DVB-T
        }
    }

    std::string name;
    bool enabled = true;

    int dvbs_ver_selected = 0;
    int dvb_mode = 0; // 0 = DVB-S, 1 = DVB-S2, 2 = DVB-T
    int dvbt_bandwidth = 2; // MHz, default 2

    VFOManager::VFO* vfo;
    float dvbs_bw = 250000*2.0f;
    float dvbs2_bw = 250000*2.0f;
    ImGui::DVBS_ConstellationDiagram constDiag;

    int dvbs_sym_rate = 250000;
    int dvbs_sym_rate_disp = 250000;
    dsp::dvbs::DVBSDemod dvbsDemod;
    float dvbs_viterbi_err_avg[30];
    int dvbs_viterbi_err_avg_ptr = 0;

    int dvbs2_sym_rate = 250000;
    int dvbs2_sym_rate_disp = 250000;
    dsp::dvbs2::dvb_cgf_holder dvbs2_cfg;
    dsp::dvbs2::DVBS2Demod dvbs2Demod;
    float dvbs2_pl_best_avg[30];
    int dvbs2_pl_best_avg_ptr = 0;
    float dvbs2_bch_corr_avg[30];
    int dvbs2_bch_corr_avg_ptr = 0;
    float dvbs2_avg_bchcorr = 0;
    int dvbs2_modcod_checkbuff[50];
    int dvbs2_modcod_checkbuff_ptr = 0;
    bool auto_modcod = false;
    dsp::dvbs2::BBFrameTSParser dvbs2bbparser;
    uint8_t packetbuff[65536*10];

    // DVB-T demodulator
    dsp::dvbt::DVBTDemod dvbtDemod;
    dsp::dvbt::TPSInfo dvbt_tps_info;
    float dvbt_snr_avg[30];
    int dvbt_snr_avg_ptr = 0;
    float dvbt_freq_offset_avg[30];
    int dvbt_freq_offset_avg_ptr = 0;

    dsp::sink::Handler<uint8_t> demodSink;

    EventHandler<double> onUserChangedBandwidthHandler;

    std::string s2ConstellationsListTxt;
    std::string s2CoderatesListTxt;
    std::string s2FramesizesListTxt;
    std::string s2PilotsListTxt;

    char hostname[1024] = "127.0.0.1";
    int port = 4754;

#ifdef ENABLE_NNG_NETWORKING
    std::shared_ptr<net::Socket> conn;
#endif

};

MOD_EXPORT void _INIT_() {
    flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Module _INIT_ called");
    try {
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Getting root path from core args");
        std::string root = (std::string)core::args["root"];
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Root path: '{}'", root);
        
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Creating default config");
        json def = json({});
        
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Setting config path");
        config.setPath(root + "/dvbs_demodulator_config.json");
        
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Loading config");
        config.load(def);
        
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Enabling auto save");
        config.enableAutoSave();
        
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Module _INIT_ completed successfully");
    } catch (const std::exception& e) {
        flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Module _INIT_ failed: {}", e.what());
        throw;
    } catch (...) {
        flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Module _INIT_ failed with unknown error");
        throw;
    }
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Creating instance '{}'", name);
    try {
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Calling constructor for '{}'", name);
        auto* instance = new DVBSDemodulatorModule(name);
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Instance '{}' created successfully", name);
        return instance;
    } catch (const std::exception& e) {
        flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Failed to create instance '{}': {}", name, e.what());
        throw;
    } catch (...) {
        flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Failed to create instance '{}' with unknown error", name);
        throw;
    }
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Deleting instance");
    try {
        if (instance) {
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Calling destructor");
            delete (DVBSDemodulatorModule*)instance;
            flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Instance deleted successfully");
        } else {
            flog::warn("DVB-S/DVB-S2/DVB-T Demodulator: Instance pointer is null");
        }
    } catch (const std::exception& e) {
        flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Failed to delete instance: {}", e.what());
    } catch (...) {
        flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Failed to delete instance with unknown error");
    }
}

MOD_EXPORT void _END_() {
    flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Module _END_ called");
    try {
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Disabling auto save");
        config.disableAutoSave();
        
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Saving config");
        config.save();
        
        flog::info("DVB-S/DVB-S2/DVB-T Demodulator: Module _END_ completed successfully");
    } catch (const std::exception& e) {
        flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Module _END_ failed: {}", e.what());
    } catch (...) {
        flog::error("DVB-S/DVB-S2/DVB-T Demodulator: Module _END_ failed with unknown error");
    }
}
