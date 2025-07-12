#pragma once
#include <imgui.h>

namespace ImGui {
    ImVec2 operator+(ImVec2 a, ImVec2 b) {
        return ImVec2(a.x + b.x, a.y + b.y);
    }

    #define CONSTDIAG_SIZE (uint32_t)70000
    class DVBS_ConstellationDiagram {
    public:
        DVBS_ConstellationDiagram() {
            memset(buffer, 0, 1024 * sizeof(dsp::complex_t));
        }

        void draw(const ImVec2& size_arg = ImVec2(0, 0)) {
            std::lock_guard<std::mutex> lck(bufferMtx);
            
            // Use public ImGui API
            ImGuiStyle& style = GetStyle();
            ImVec2 size = size_arg;
            if (size.x <= 0.0f) size.x = CalcItemWidth();
            if (size.y <= 0.0f) size.y = CalcItemWidth();
            
            // Create an invisible button to reserve space and get draw list
            InvisibleButton("##constellation_diagram", size);
            ImDrawList* draw_list = GetWindowDrawList();
            ImVec2 min = GetItemRectMin();
            ImVec2 max = GetItemRectMax();

            draw_list->AddRectFilled(min, max, IM_COL32(0, 0, 0, 255));
            ImU32 col = ImGui::GetColorU32(ImGuiCol_CheckMark, 0.7f);
            ImU32 col_red = ImGui::ColorConvertFloat4ToU32(ImVec4(0.9f,0.01f,0.01f,1.f));
            for (int i = 0; i < used_syms; i++) {
                if (buffer[i].re > 1.5f || buffer[i].re < -1.5f) { continue; }
                if (buffer[i].im > 1.5f || buffer[i].im < -1.5f) { continue; }
                draw_list->AddCircleFilled(ImVec2((((buffer[i].re / 1.5f) + 1) * (size.x * 0.5f)) + min.x, (((buffer[i].im / 1.5f) + 1) * (size.y * 0.5f)) + min.y), 2, col);
            }
            for (int i = 0; i < used_red_syms; i++) {
                if (red_buffer[i].re > 1.5f || red_buffer[i].re < -1.5f) { continue; }
                if (red_buffer[i].im > 1.5f || red_buffer[i].im < -1.5f) { continue; }
                draw_list->AddCircleFilled(ImVec2((((red_buffer[i].re / 1.5f) + 1) * (size.x * 0.5f)) + min.x, (((red_buffer[i].im / 1.5f) + 1) * (size.y * 0.5f)) + min.y), 2, col_red);
            }
        }

        dsp::complex_t* acquireBuffer() {
            bufferMtx.lock();
            return buffer;
        }

        void releaseBuffer(uint32_t usedSyms) {
            used_syms = std::min(usedSyms, CONSTDIAG_SIZE);
            bufferMtx.unlock();
        }

        dsp::complex_t* acquireRedBuffer() {
            bufferMtx.lock();
            return red_buffer;
        }

        void releaseRedBuffer(uint32_t usedSyms) {
            used_red_syms = std::min(usedSyms, CONSTDIAG_SIZE);
            bufferMtx.unlock();
        }

    private:
        std::mutex bufferMtx;
        dsp::complex_t buffer[CONSTDIAG_SIZE];
        dsp::complex_t red_buffer[CONSTDIAG_SIZE];
        uint32_t used_syms = 0;
        uint32_t used_red_syms = 0;
    };

    void BoxIndicator(float menuWidth, ImU32 color, const ImVec2& size_arg = ImVec2(0, 0)) {
        // Use public ImGui API instead of internal API
        ImVec2 cursor_pos = GetCursorPos();
        float fontSize = GetFontSize();
        ImVec2 size = size_arg;
        if (size.x <= 0.0f) size.x = fontSize;
        if (size.y <= 0.0f) size.y = fontSize;
        
        // Position the indicator on the right side
        SetCursorPosX(menuWidth - fontSize);
        
        // Create an invisible button to reserve space
        InvisibleButton("##box_indicator", size);
        
        // Draw the colored rectangle
        ImDrawList* draw_list = GetWindowDrawList();
        ImVec2 p_min = GetItemRectMin();
        ImVec2 p_max = GetItemRectMax();
        draw_list->AddRectFilled(p_min, p_max, color);
        
        // Restore cursor position
        SetCursorPos(cursor_pos);
    }

    void SigQualityMeter(float avg, float val_min, float val_max, const ImVec2& size_arg = ImVec2(0, 0)) {
        // Use public ImGui API instead of internal API
        avg = std::clamp<float>(avg, val_min, val_max);
        
        ImGuiStyle& style = GetStyle();
        float fontSize = GetFontSize();
        ImVec2 size = size_arg;
        if (size.x <= 0.0f) size.x = CalcItemWidth();
        if (size.y <= 0.0f) size.y = (fontSize / 2) + style.FramePadding.y;
        
        // Create an invisible button to reserve space
        InvisibleButton("##sig_quality_meter", size);
        
        // Draw the meter
        ImDrawList* draw_list = GetWindowDrawList();
        ImVec2 p_min = GetItemRectMin();
        ImVec2 p_max = GetItemRectMax();
        
        float badSig = roundf(0.3f * size.x);
        
        // Background: red for bad signal range, green for good signal range
        draw_list->AddRectFilled(p_min, ImVec2(p_min.x + badSig, p_max.y), IM_COL32(136, 9, 9, 255));
        draw_list->AddRectFilled(ImVec2(p_min.x + badSig, p_min.y), p_max, IM_COL32(9, 136, 9, 255));
        
        // Current signal level indicator
        float end = roundf(((avg - val_min) / (val_max - val_min)) * size.x);
        
        if (avg <= (val_min+(val_max-val_min)*0.3f)) {
            // Signal is in bad range - red indicator
            draw_list->AddRectFilled(p_min, ImVec2(p_min.x + end, p_max.y), IM_COL32(230, 5, 5, 255));
        }
        else {
            // Signal is in good range - red for bad part, green for good part
            draw_list->AddRectFilled(p_min, ImVec2(p_min.x + badSig, p_max.y), IM_COL32(230, 5, 5, 255));
            draw_list->AddRectFilled(ImVec2(p_min.x + badSig, p_min.y), ImVec2(p_min.x + end, p_max.y), IM_COL32(5, 230, 5, 255));
        }
    }
} 
