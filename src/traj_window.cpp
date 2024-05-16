#include "traj_window.h"

#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "implot_internal.h"

extern "C" {
#include "mc_traj.h"
}

#include <cmath>
#include <vector>
#include <algorithm>


void ShowStpWindow()
{
    static mc_traj traj = {
        .acc_max = 2660,    // ((0.38 newton*meter) / (0.000143 kg*m^2)) in rad/s^2
        .dec_max = 1330,

        .vel_min = -418.9,  // 4000 RPM in rad/s
        .vel_max = 418.9,

        .pos_min = -10000,
        .pos_max = 10000
    };

    static float dt = 1.0 / 100;

    static float vel_init = -500;
    static float pos_init = -100;
    static float pos_target = 1000;

    if (!ImGui::Begin("Simple Trajectory Planner"))
    {
        // Early out if the window is collapsed, as an optimization.
        ImGui::End();
        return;
    }

    ImGui::PushMultiItemsWidths(2, ImGui::CalcItemWidth());
    ImGui::DragFloat("acc_max", &traj.acc_max, 0.1, 0, 10000);
    ImGui::SameLine();
    ImGui::PopItemWidth();
    ImGui::DragFloat("dec_max", &traj.dec_max, 0.1, 0, 10000);
    ImGui::PopItemWidth();

    ImGui::PushMultiItemsWidths(2, ImGui::CalcItemWidth());
    ImGui::DragFloat("vel_min", &traj.vel_min, 0.1, -10000, 0);
    ImGui::SameLine();
    ImGui::PopItemWidth();
    ImGui::DragFloat("vel_max", &traj.vel_max, 0.1, 0, 10000);
    ImGui::PopItemWidth();

    ImGui::PushMultiItemsWidths(2, ImGui::CalcItemWidth());
    ImGui::DragFloat("pos_min", &traj.pos_min, 0.1, -10000, 10000);
    ImGui::SameLine();
    ImGui::PopItemWidth();
    ImGui::DragFloat("pos_max", &traj.pos_max, 0.1, -10000, 10000);
    ImGui::PopItemWidth();

    ImGui::PushMultiItemsWidths(2, ImGui::CalcItemWidth());
    ImGui::DragFloat("vel_init", &vel_init, 0.1, -10000, 10000);
    ImGui::SameLine();
    ImGui::PopItemWidth();
    ImGui::DragFloat("pos_init", &pos_init, 0.1, -10000, 10000);
    ImGui::PopItemWidth();

    ImGui::DragFloat("dt", &dt, 10e-6, 50e-6, 100000e-6, "%.6f", ImGuiSliderFlags_AlwaysClamp);
    ImGui::DragFloat("pos_target", &pos_target, 0.1, -10000, 10000);

    int N = roundf(5.0 / dt);

    std::vector<float> data_t(N);
    std::vector<float> data_acc(N);
    std::vector<float> data_vel(N);
    std::vector<float> data_pos(N);

    ImGui::Text("N=%d", N);

    traj.acc = 0;
    traj.vel = vel_init;
    traj.pos = pos_init;

    for (int i = 0; i < N; i++) {
        mc_traj_step(&traj, pos_target, dt);
        data_t[i] = i * dt;
        data_acc[i] = traj.acc;
        data_vel[i] = traj.vel;
        data_pos[i] = traj.pos;
    }

    if (ImPlot::BeginPlot("Line Plots", ImVec2(800, 600))) {
        ImPlot::SetupAxes("x","y");

        auto limits = ImPlot::GetPlotLimits();

        // Only show acc markers when zoomed in
        //
        if (limits.X.Size() < 100 * dt)
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 2);
        
        ImPlot::PlotLine("acc", data_t.data(), data_acc.data(), data_t.size());
        ImPlot::PlotLine("vel", data_t.data(), data_vel.data(), data_t.size());
        ImPlot::PlotLine("pos", data_t.data(), data_pos.data(), data_t.size());

        // Plot tooltip
        //
        if (ImPlot::IsPlotHovered()) {
            ImPlotPoint mouse = ImPlot::GetPlotMousePos();
            float mx = mouse.x - dt / 2;
            auto it = std::lower_bound(data_t.begin(), data_t.end(), mx);

            if (it != data_t.end() && mx >= data_t[0] - dt) {
                int idx = it - data_t.begin();

                double vals[] = { data_t[idx] };
                ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1, 1, 1, 0.5));
                ImPlot::PlotInfLines("Vertical", vals, 1, ImPlotItemFlags_NoLegend);
                ImPlot::PopStyleColor();

                ImGui::BeginTooltip();
                ImGui::Text("idx: %d", idx);
                ImGui::Text("t  : %f", data_t[idx]);
                ImGui::Text("acc: %f", data_acc[idx]);
                ImGui::Text("vel: %f", data_vel[idx]);
                ImGui::Text("pos: %f", data_pos[idx]);
                ImGui::EndTooltip();
            }
        }

        ImPlot::EndPlot();
    }

    ImGui::End();
}
