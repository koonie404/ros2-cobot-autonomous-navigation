#pragma once

#include <cstdint>
#include <array>

// ── 상수 ──────────────────────────────────────────────────────────────────────
constexpr int SCAN_BUFFER_SIZE = 667;   // 0.54° 해상도 (≈ 667 / 360)

// ── 공개 API (Python __all__ 대응) ────────────────────────────────────────────
bool AnalysisOne(uint8_t byte);
void Parse();

const std::array<float, SCAN_BUFFER_SIZE>& GetRanges();
const std::array<float, SCAN_BUFFER_SIZE>& GetIntensities();

int  GetScanBufferSize();

struct ScanStats {
    int correctCRC;
    int wrongVerLen;
    int wrongCRC;
};
ScanStats get_stats();
