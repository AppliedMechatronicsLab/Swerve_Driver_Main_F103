
#define AS5600_ADDR 0x36 

#define ZMCO 0x00
#define ZPOS_H 0x01 
#define ZPOS_L 0x02
#define MPOS_H 0x03 
#define MPOS_L 0x04
#define MANG_H 0x05
#define MANG_L 0x06
#define CONF_L 0x07
#define CONF_H 0x08

#define RAWANG_H 0x0C 
#define RAWANG_L 0x0D
#define ANGLE_H 0x0E 
#define ANGLE_L 0x0F

#define STATUS 0x0B
#define AGC 0x1A
#define MAGN_H 0x1B
#define MAGN_L 0x1C

#define BURN 0xFF

#define CONF_L_PM 0x03  
#define CONF_H_SF 0x03  
#define CONF_H_FTH 0x1C 
#define CONF_H_WD 0x20  

#define MAGNET_LOW 0x10
#define MAGNET_HIGH 0x20
#define MAGNET_NORM 0x30

#define HYST_MASK 0x0C
#define HYST_OFF 0x00
#define HYST_1LSB 0x04
#define HYST_2LSB 0x08
#define HYST_3LSB 0x0C

#define OUT_STG_MASK 0x30
#define OUT_STG_ANALOG 0x00
#define OUT_STG_ANALOG_RED 0x10
#define OUT_STG_PWM 0x20

#define PWMF_MASK 0xC0
#define PWMF_115HZ 0x00
#define PWMF_230HZ 0x40
#define PWMF_460HZ 0x80
#define PWMF_920HZ 0xC0

void AS5600_WriteReg(uint8_t Reg, uint8_t Data);
uint8_t AS5600_ReadReg(uint8_t Reg);
float AS5600_GetAngle(void);
uint16_t AS5600_GetRawAngle(void);
uint8_t AS5600_GetStatus(void);
void AS5600_SetHystheresis(uint8_t Hyst);
void AS5600_SetOutputStage(uint8_t OutStage);
void AS5600_SetPWMFreq(uint8_t Freq);