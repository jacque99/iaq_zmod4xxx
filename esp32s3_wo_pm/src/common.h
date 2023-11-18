
#ifndef common_h_
#define common_h_

typedef struct {
    float temperature;
    float humidity;
    // int presense;
    float etoh;
    float tvoc;
    float eco2;
    float iaq;
} sensor_reading_t;

typedef struct
{
    uint32_t red;
    uint32_t green;
    uint32_t blue;
} rgb_color_t;

#endif