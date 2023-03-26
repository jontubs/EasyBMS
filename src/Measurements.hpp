#pragma once

#include <array>
#include <bitset>

struct Measurements {
    std::array<float, 12> cell_voltages;
    std::array<float, 12> cell_diffs_to_avg;
    float cell_diff;
    float cell_diff_trend; // "Current cell diff" difference to "cell diff 1h ago"
    float min_cell_voltage;
    float max_cell_voltage;
    float avg_cell_voltage;
    float module_voltage;
    float module_temp_1;
    float module_temp_2; 
    float chip_temp;
    float soc; // 0.00% to 100.00%
};