#include "prx/libSceAgcDriver/Execution/include/ProfileOutput.hpp"
#include "prx/libSceAgcDriver/Execution/include/Driver/Driver.hpp"
#include "prx/libSceAgcDriver/Execution/include/Driver/Draw/DrawTiming.hpp"
#include <cstdlib>

namespace AgcDriver::DriverDetail {

PendingDrawPhases& Driver::pendingDrawPhases() {
    static thread_local PendingDrawPhases pending;
    return pending;
}

void Driver::addDrawPhases(const std::array<double, DrawDriverPhaseCount>& ms, bool drawn, std::uint64_t captures) {
    std::lock_guard lock(drawPhasesMutex);
    auto& totals = drawPhaseTotals;
    for (std::size_t i = 0; i < DrawDriverPhaseCount; ++i) totals.ms[i] += ms[i];
    ++totals.packets;
    if (drawn) ++totals.drawn;
    totals.captures += captures;
    const auto now = std::chrono::steady_clock::now();
    if (now - totals.lastReport < std::chrono::seconds(10)) return;
    totals.lastReport = now;
    std::string report;
    double total = 0;
    for (std::size_t i = 0; i < DrawDriverPhaseCount; ++i) {
        char text[64];
        std::snprintf(text, sizeof(text), " %s %.1f", DrawDriverPhaseNames[i], totals.ms[i] * 1000 / static_cast<double>(totals.packets));
        report += text;
        total += totals.ms[i];
    }
    AgcDriver::ProfilePrint_nid_no_patch( "[draw] driver phases (10 s, %llu draw packets, %llu drawn, %llu captures), us per packet:%s, total %.1f (%.1f ms; Graphics::Draw %.1f ms, skipped %.1f ms)\n", static_cast<unsigned long long>(totals.packets), static_cast<unsigned long long>(totals.drawn), static_cast<unsigned long long>(totals.captures), report.c_str(), total * 1000 / static_cast<double>(totals.packets), total, totals.ms[DrawRowGraphics], totals.ms[DrawRowSkipped]);
    totals.ms = {};
    totals.packets = totals.drawn = totals.captures = 0;
}

void addIndirectDrawPhases(bool cpuPath, double validateMs, double materializeMs, double recordMs) {
    static const bool enabled = std::getenv("APS5_PROFILE_INDIRECT") != nullptr;
    if (!enabled) return;
    static std::mutex totalsMutex;
    static IndirectDrawTotals totals;
    std::lock_guard lock(totalsMutex);
    const auto index = cpuPath ? 0u : 1u;
    ++totals.draws[index];
    totals.validateMs[index] += validateMs;
    totals.materializeMs[index] += materializeMs;
    totals.recordMs[index] += recordMs;
    static const bool print = std::getenv("APS5_PROFILE_DRAW") != nullptr;
    if (!print) return;
    const auto now = std::chrono::steady_clock::now();
    if (now - totals.lastReport < std::chrono::seconds(10)) return;
    AgcDriver::ProfilePrint_nid_no_patch("[draw-indirect] (10 s): cpu %llu draws validate %.1f materialize %.1f record %.1f ms, gpu %llu draws validate %.1f materialize %.1f record %.1f ms, total %.1f ms\n", static_cast<unsigned long long>(totals.draws[0]), totals.validateMs[0], totals.materializeMs[0], totals.recordMs[0], static_cast<unsigned long long>(totals.draws[1]), totals.validateMs[1], totals.materializeMs[1], totals.recordMs[1], totals.validateMs[0] + totals.validateMs[1] + totals.materializeMs[0] + totals.materializeMs[1] + totals.recordMs[0] + totals.recordMs[1]);
    totals = IndirectDrawTotals{};
    totals.lastReport = now;
}

void DrawPhaseTiming::Phase(DrawDriverPhase which) {
    if (!profile) return;
    const auto now = std::chrono::steady_clock::now();
    phaseMs[which] += std::chrono::duration<double, std::milli>(now - phaseLap).count();
    phaseLap = now;
}

}
