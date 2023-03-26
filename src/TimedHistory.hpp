#include <queue>


template <typename T>
class TimedHistory {
public:
    struct Element {
        Element(T v, long t) {
            value = v;
            timestamp = t;
        }

        T value;
        long timestamp;
    };

private:
    std::queue<Element> _values;
    long _retention_period;
    long _granularity;

    void clean() {
        long current_time = millis();

        while (!_values.empty() && current_time - _values.front().timestamp > _retention_period) {
            _values.pop();
        }
    }

public:
    TimedHistory(long retention_period_millis, long granularity_millis) {
        _retention_period = retention_period_millis;
        _granularity = granularity_millis;
    }

    void insert(T value) {
        long current_time = millis();
        Element new_element(value, current_time);

        if (_values.empty()) {
            _values.push(new_element);
        }
        else {
            long last_insert_time = _values.back().timestamp;

            if (current_time - last_insert_time > _granularity) {
                _values.push(new_element);
            }

            clean();
        }
    }

    std::optional<Element> oldest_element() {
        clean();

        if(_values.empty()) {
            return {};
        }
        else {
            return _values.front();
        }

        
    }
};