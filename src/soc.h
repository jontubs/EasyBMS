#include <vector>
#include <tuple>

auto soc_data_points = std::vector<std::pair<float,float>>();

void initialize_soc_data_points() {
    soc_data_points.push_back({0.000, -0.2});
    soc_data_points.push_back({3.420, 0.0});
    soc_data_points.push_back({3.499, 0.1});
    soc_data_points.push_back({3.579, 0.2});
    soc_data_points.push_back({3.615, 0.3});
    soc_data_points.push_back({3.641, 0.4});
    soc_data_points.push_back({3.678, 0.5});
    soc_data_points.push_back({3.756, 0.6});
    soc_data_points.push_back({3.825, 0.7});
    soc_data_points.push_back({3.913, 0.8});
    soc_data_points.push_back({4.016, 0.9});
    soc_data_points.push_back({4.136, 1.0});
    soc_data_points.push_back({5.000, 1.2});

}

float voltage_to_soc(float cell_voltage) {
    if (soc_data_points.empty()) {
        initialize_soc_data_points();
    }

    for(size_t i = 0; i < soc_data_points.size() - 1; i++) {
        float lower_voltage = soc_data_points[i].first;
        float upper_voltage = soc_data_points[i+1].first;

        float lower_soc = soc_data_points[i].second;
        float upper_soc = soc_data_points[i+1].second;

        if (lower_voltage <= cell_voltage && cell_voltage <= upper_voltage) {
            float d = (upper_voltage - cell_voltage) / (upper_voltage - lower_voltage);
            float soc = (1 - d) * upper_soc + d * lower_soc;
            return soc*100.0f;
        }
    }

    return -1;
}