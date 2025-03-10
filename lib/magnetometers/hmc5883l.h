#define HMC_DEVADDR 0x1E
#define HMC_RA_CFGA 0x00
#define HMC_RA_CFGB 0x01
#define HMC_RA_MODE 0x02
#define HMC_RA_DATA 0x03

#define HMC_CFGA_DATA_RATE_0_75 0b000 << 2
#define HMC_CFGA_DATA_RATE_1_5  0b001 << 2
#define HMC_CFGA_DATA_RATE_3    0b010 << 2
#define HMC_CFGA_DATA_RATE_7_5  0b011 << 2
#define HMC_CFGA_DATA_RATE_15   0b100 << 2
#define HMC_CFGA_DATA_RATE_30   0b101 << 2
#define HMC_CFGA_DATA_RATE_75   0b110 << 2
#define HMC_CFGA_AVG_SAMPLES_1  0b00 << 5
#define HMC_CFGA_AVG_SAMPLES_2  0b01 << 5
#define HMC_CFGA_AVG_SAMPLES_4  0b10 << 5
#define HMC_CFGA_AVG_SAMPLES_8  0b11 << 5
#define HMC_CFGA_BIAS_NORMAL    0b00
#define HMC_CFGA_BIAS_POS       0b01
#define HMC_CFGA_BIAS_NEG       0b10

#define HMC_CFGB_GAIN_0_88 0
#define HMC_CFGB_GAIN_1_30 1 << 5
#define HMC_CFGB_GAIN_1_90 2 << 5
#define HMC_CFGB_GAIN_2_50 3 << 5
#define HMC_CFGB_GAIN_4_00 4 << 5
#define HMC_CFGB_GAIN_4_70 5 << 5
#define HMC_CFGB_GAIN_5_60 6 << 5
#define HMC_CFGB_GAIN_8_10 7 << 5

#define HMC_MODE_HIGHSPEED 1 << 7
#define HMC_MODE_READ_CONTINUOUS 0b00
#define HMC_MODE_READ_SINGLEMEAS 0b01
