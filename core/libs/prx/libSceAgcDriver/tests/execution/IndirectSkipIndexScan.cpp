// B99 (10-24): a vertex shader that fetches its own vertices (runtime fetch: no Vulkan vertex attributes) draws with 32-bit
// indices past 64Ki. ZeroFetchVertex used to call an empty attribute list "every vertex past record 0 reads zeros", so the
// zero-fetch clamp (a3cba2d1f) moved every index to 0 and the triangle collapsed; with APS5_ZERO_FETCH_NEEDS_ATTRIBUTE=1 the
// clamp only runs when a per-vertex attribute reads zeros. With APS5_INDIRECT_SKIP_INDEX_SCAN=1 the GPU-path indexed
// indirect draw of the same indices (imported in place) skips the index scan and draws the same triangle.
#include "prx/libSceAgcDriver/Execution/include/VulkanDevice.hpp"
#include "prx/libSceAgcDriver/Execution/include/Driver/Driver.hpp"
#include "prx/libSceAgcDriver/Graphics/include/Draw.hpp"
#include "prx/libc/include/GuestAllocations.hpp"
#include "Recompiler.hpp"
#include "VulkanTestDevice.hpp"
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <new>
#include <optional>
#include <string>

namespace {

using AgcDriver::Graphics::Require;
using ShaderRecompiler::ShaderStage;

constexpr std::uint32_t Extent = 64u;
constexpr std::uint32_t FirstIndex = 70000u;
constexpr std::uint32_t ControlIndex = 1000u;
constexpr std::uint32_t Records = FirstIndex + 3u;
constexpr std::uint32_t NoLocation = 0x280u;
constexpr std::size_t BlockBytes = 65536;
alignas(256) std::array<std::byte, Extent * Extent * 4u> pixels{};
alignas(256) std::array<std::array<float, 4>, Records> vertices{};
constexpr std::array<std::array<float, 4>, 3> positions{{{-1.0f, -1.0f, 0.0f, 1.0f}, {3.0f, -1.0f, 0.0f, 1.0f}, {-1.0f, 3.0f, 0.0f, 1.0f}}};
// From VertexFetch.cpp: the record is v5 (vertex index) + s2 + v8 (instance index) + s3, read with buffer_load_format_xyzw.
constexpr std::array<std::uint32_t, 11> vertexCode{0xf4080100u, 0xfa000000u, 0x4a0a0a02u, 0x4a0a0b08u, 0x4a0a0a03u, 0xe00c2000u, 0x80010005u, 0xbf8c3f70u, 0xf80008cfu, 0x03020100u, 0xbf810000u};
constexpr std::array<std::uint32_t, 5> pixelCode{0x7e0002f2u, 0x7e020280u, 0xf800180fu, 0x00010100u, 0xbf810000u};

std::size_t Red() {
    std::size_t red = 0;
    for (std::size_t index = 0; index < pixels.size(); index += 4u) {
        if (pixels[index] == std::byte{255} && pixels[index + 1u] == std::byte{0} && pixels[index + 2u] == std::byte{0} && pixels[index + 3u] == std::byte{255}) ++red;
    }
    return red;
}

std::string Run(const std::shared_ptr<AgcDriver::VulkanDevice>& device) {
    // Indices and the indirect record live in one registered guest block, so the draw reads the indices in place.
    auto* block = static_cast<std::byte*>(::operator new(BlockBytes, std::align_val_t{BlockBytes}));
    std::memset(block, 0, BlockBytes);
    {
        GuestAllocations::Mutation mutation;
        mutation.Add(block, BlockBytes, true, true, true);
    }
    auto* indices = reinterpret_cast<std::uint32_t*>(block);
    auto* record = reinterpret_cast<std::uint32_t*>(block + 256);
    for (std::uint32_t i = 0; i < 3u; ++i) indices[i] = FirstIndex + i;
    const std::array<std::uint32_t, 5> indexedRecord{3u, 1u, 0u, 0u, 0u};
    std::memcpy(record, indexedRecord.data(), sizeof(indexedRecord));
    for (std::uint32_t i = 0; i < 3u; ++i) vertices[FirstIndex + i] = positions[i];
    for (std::uint32_t i = 0; i < 3u; ++i) vertices[ControlIndex + i] = positions[i];
    auto* controlIndices = reinterpret_cast<std::uint32_t*>(block + 512);
    for (std::uint32_t i = 0; i < 3u; ++i) controlIndices[i] = ControlIndex + i;

    const auto address = reinterpret_cast<std::uintptr_t>(vertices.data());
    std::array<std::uint32_t, 4> descriptor{static_cast<std::uint32_t>(address), static_cast<std::uint32_t>(address >> 32u) | (16u << 16u), Records, 0x01000facu | (77u << 12u)};
    const auto tableAddress = reinterpret_cast<std::uintptr_t>(descriptor.data());
    const std::vector<std::uint32_t> userData{static_cast<std::uint32_t>(tableAddress), static_cast<std::uint32_t>(tableAddress >> 32u), 0u, 0u};
    const std::array<ShaderRecompiler::MemoryRegion, 1> memory{{{tableAddress, std::as_bytes(std::span(descriptor))}}};
    ShaderRecompiler::ShaderVertexStageInfo vertexInfo{};
    vertexInfo.fetchEmbedded = true;
    vertexInfo.resourcesNum = 1u;
    vertexInfo.resourcesDst[0] = {0, 4, 0, 0};
    vertexInfo.resources[0].fields = descriptor;
    ShaderRecompiler::RecompileRequest request{{ShaderStage::Vertex, reinterpret_cast<std::uintptr_t>(vertexCode.data()), vertexCode, 0, {}}, {32u, 0u, userData, std::nullopt, std::nullopt, vertexInfo, memory}, device->Target(), {0u, 0u, 0u, 96u}};
    request.useCache = false;
    const auto vertex = ShaderRecompiler::Recompile(request);
    Require(vertex.vertexAttributes.empty(), "the runtime-fetch vertex shader has Vulkan vertex attributes; the case needs none");
    const auto pushBytes = static_cast<std::uint32_t>(vertex.pushConstants.size());
    ShaderRecompiler::ShaderPixelStageInfo pixelInfo{};
    pixelInfo.wave32 = true;
    pixelInfo.targetOutputMode[0] = 9u;
    pixelInfo.targetExportMapping.fill(0xe4u);
    ShaderRecompiler::RecompileRequest pixelRequest{{ShaderStage::Fragment, reinterpret_cast<std::uintptr_t>(pixelCode.data()), pixelCode, 0, {}}, {32u, 0u, {}, std::nullopt, pixelInfo, std::nullopt, {}}, device->Target(), PixelPushLayout(pushBytes, device->Target())};
    pixelRequest.useCache = false;
    const auto pixel = ShaderRecompiler::Recompile(pixelRequest);
    const std::array<AgcDriver::Graphics::CompiledShader, 2> shaders{{{ShaderStage::Vertex, &vertex, 0u}, {ShaderStage::Fragment, &pixel, PixelPushOffset(pushBytes, device->Target())}}};

    AgcDriver::Graphics::State state{};
    state.stages = {AgcDriver::Graphics::ShaderPath::Vertex, 0u, 32u, 32u, std::nullopt, std::nullopt};
    state.color = {reinterpret_cast<std::uintptr_t>(pixels.data()), {Extent, Extent}, VK_FORMAT_R8G8B8A8_UNORM, pixels.size(), 0xe4u};
    state.colors = {state.color};
    state.hasColorTarget = true;
    state.renderExtent = {Extent, Extent};
    state.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    state.viewport = {0.0f, static_cast<float>(Extent), static_cast<float>(Extent), -static_cast<float>(Extent), 0.0f, 1.0f};
    state.scissor = {{0, 0}, {Extent, Extent}};
    state.cullMode = VK_CULL_MODE_NONE;
    state.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    state.blend.colorWriteMask = 15u;
    state.blends = {state.blend};

    std::string failures;
    const auto expectFull = [&](const std::string& what) {
        const auto red = Red();
        std::cout << what << ": " << red << " of " << Extent * Extent << " pixels red\n";
        if (red != Extent * Extent) failures += what + ": " + std::to_string(red) + " of " + std::to_string(Extent * Extent) + " pixels red (indices past 64Ki were moved)\n";
    };
    {
        // Control: indices under 64Ki past record 0 are never clamped, so this case draws with or without the fix.
        AgcDriver::Pm4::DrawParameters draw{reinterpret_cast<std::uintptr_t>(controlIndices), 3u, 4u, 1u, 0u, true};
        pixels.fill(std::byte{0});
        device->Draw(state, draw, shaders);
        device->WaitIdle();
        expectFull("control: direct indexed draw, 32-bit indices 1000..1002, runtime vertex fetch");
    }
    {
        AgcDriver::Pm4::DrawParameters draw{reinterpret_cast<std::uintptr_t>(indices), 3u, 4u, 1u, 0u, true};
        pixels.fill(std::byte{0});
        device->Draw(state, draw, shaders);
        device->WaitIdle();
        expectFull("direct indexed draw, 32-bit indices 70000..70002, runtime vertex fetch");
    }
    {
        AgcDriver::Pm4::DrawParameters draw{reinterpret_cast<std::uintptr_t>(indices), 3u, 4u, 0u, 0u, true};
        draw.indirect = AgcDriver::Pm4::DrawParameters::IndirectDraw{reinterpret_cast<std::uintptr_t>(record), 0x25u, 20u, 20u, 1u, false, 0u, NoLocation, NoLocation, NoLocation, false, 0u};
        draw.indirect->baseVertexSgpr = -1;
        draw.indirect->startInstanceSgpr = -1;
        AgcDriver::DriverDetail::DrawProgram program{};
        program.firstUserSgpr = 0;
        program.userData = userData;
        const auto path = AgcDriver::DriverDetail::Driver::ClassifyIndirectDraw(vertex, state, program, device, draw, false);
        if (path) failures += std::string("the indexed indirect record was classified to the CPU path (") + AgcDriver::Graphics::IndirectDrawPathName(*path) + "), the case needs the GPU path\n";
        pixels.fill(std::byte{0});
        device->Draw(state, draw, shaders);
        device->WaitIdle();
        expectFull("GPU-path indexed indirect draw, in-place 32-bit indices 70000..70002, runtime vertex fetch");
    }
    return failures;
}

}

int main() {
#ifdef _WIN32
    _putenv_s("APS5_INDIRECT_SKIP_INDEX_SCAN", "1");
    _putenv_s("APS5_ZERO_FETCH_NEEDS_ATTRIBUTE", "1");
#else
    setenv("APS5_INDIRECT_SKIP_INDEX_SCAN", "1", 1);
    setenv("APS5_ZERO_FETCH_NEEDS_ATTRIBUTE", "1", 1);
#endif
    try {
        std::shared_ptr<AgcDriver::VulkanDevice> device = OpenVulkanTestDevice();
        if (!device) return VulkanTestSkipped;
        const auto failures = Run(device);
        if (!failures.empty()) {
            std::cerr << "runtime-fetch draws with indices past 64Ki:\n" << failures;
            return 1;
        }
        std::cout << "indirect skip index scan tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
