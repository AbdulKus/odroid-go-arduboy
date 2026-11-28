#pragma once

#include <algorithm>
#include <cstdint>

namespace go {

class DacConverter {
public:
    uint8_t convert(int32_t sample, unsigned volume) {
        const int64_t target = int64_t(sample) * 65536;
        meanQ16_ += (target - meanQ16_) / 512;
        const int32_t ac = sample - int32_t(meanQ16_ / 65536);
        if (volume == 0 || (ac >= -1 && ac <= 1)) {
            errorQ8_ = 0;
            return 128;
        }
        const int64_t level = std::min(volume, 100u);
        // A full-scale Arduboy signal is +/-2000. Leave headroom for
        // transients while preserving the relative levels of mixed voices.
        const int32_t signalQ8 = int32_t(int64_t(ac) * level * level * 6144 / 10000000);
        const int32_t valueQ8 = signalQ8 + errorQ8_;
        const int32_t rounded = valueQ8 >= 0 ? (valueQ8 + 128) / 256 : -((-valueQ8 + 128) / 256);
        const int32_t output = std::clamp<int32_t>(rounded, -128, 127);
        errorQ8_ = output == rounded ? valueQ8 - output * 256 : 0;
        return uint8_t(128 + output);
    }

    void reset() { meanQ16_ = errorQ8_ = 0; }

private:
    int64_t meanQ16_ = 0;
    int32_t errorQ8_ = 0;
};

}
