#include <stdint.h>
#include <stdbool.h>

/* Therapy parameter limits */
#define STRENGTH_MAX            100
#define STRENGTH_MIN            10
#define FREQ_MIN                15
#define FREQ_MAX                350
#define FREQ_DEF                60
#define INTENSITY_MIN           1
#define INTENSITY_MAX           8
#define Z_MIN                   10  // Inter-pulse gap
#define Z_DEF                   20
#define Z_MAX                   80
#define MODULATION_MIN          0
#define MODULATION_MAX          5
#define MOD_FREQ_MAX            120
#define MOD_FREQ_MIN            30

typedef struct{
    uint16_t strength;
    uint8_t freq_cycling;
    uint16_t base_freq;
    uint8_t intensity;
    uint8_t interval_gap;
    uint8_t filter;
    uint8_t modulation;
} parameter_block_t;

