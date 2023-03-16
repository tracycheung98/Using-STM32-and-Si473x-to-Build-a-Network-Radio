/*****************************************************************************/
/*!
    @file       FMTX.cpp
    @author     www.elechouse.com
    @version    V1.0
    @date       2012-11-1
    @brief      FMTX demo header file

    @section  HISTORY
    
    V1.0    Initial version.

    Copyright (c) 2012 www.elechouse.com  All right reserved.
*/
/*****************************************************************************/
#include "FMTX.h"
#include <Wire.h>

void i2c_init(void)
{
    Wire.begin();
    Wire.setClock(((F_CPU / I2C_FREQ) - 16) / 2);
}

void i2c_start(void)
{
    Wire.beginTransmission(0x52);
#ifdef __FM_DEBUG
    Serial.print("I2C Start\r\n");
#endif
}

void i2c_stop(void)
{
    Wire.endTransmission();
#ifdef __FM_DEBUG
    Serial.print("I2C Stop\r\n");
#endif
}

void i2c_write(u8 dt)
{
    Wire.write(dt);
#ifdef __FM_DEBUG
    Serial.print("I2C Write\r\n");
#endif
}
u8 i2c_read_nack(void)
{
    Wire.read();
#ifdef __FM_DEBUG
    Serial.print("I2C Read Nack\n");
#endif
}

u8 fmtx_reg[18]={
    0x00, 0x01, 0x02, 0x04, 0x0B,
    0x0C, 0x0E, 0x0F, 0x10, 0x12,
    0x13, 0x14, 0x15, 0x16, 0x17,
    0x1E, 0x26, 0x27,
};

u8 fmtx_reg_val[18]={

};

void fmtx_write_reg(u8 reg, u8 dt)
{
    i2c_start();
    i2c_write(FMTX_CMD_WRITE);
    i2c_write(reg);
    i2c_write(dt);
    i2c_stop();
}

u8 fmtx_read_reg(u8 reg)
{
    u8 dt;
    i2c_start();
    i2c_write(FMTX_CMD_WRITE);
    i2c_write(reg);
    i2c_start();
    i2c_write(FMTX_CMD_READ);
    dt = i2c_read_nack();
    i2c_stop();

    return dt;
}

void fmtx_read_all(u8 *buf)
{
    u8 i;
    for(i=0; i<18; i++){
        buf[i] = fmtx_read_reg(fmtx_reg[i]);
    }
}

void fmtx_set_freq(float freq)
{
    u16 f;
    u8 reg0, reg1_8, reg9_11;

    reg0 = fmtx_read_reg(0x02);
    reg9_11 = fmtx_read_reg(0x01);

    freq *= 20;
    f = (u16)freq;
    f &= 0x0FFF;

    if(f&0x01){
        reg0 |= 0x80;
    }else{
        reg0 &= ~0x80;
    }

    reg1_8 = (u8)(f>>1);
    reg9_11 = (reg9_11&0xF8) | (u8)(f>>9);

    fmtx_write_reg(0x02, reg0);
    fmtx_write_reg(0x01, reg9_11);
    fmtx_write_reg(0x00, reg1_8);
}

void fmtx_set_pga(fmtx_pga_type pga)
{
    u8 reg;
    u8 pga_val;
    reg = fmtx_read_reg(0x01);
    pga_val = (u8)pga;
    pga_val &= ~0xC7;
    reg = (reg&0xC7) | pga_val;
    fmtx_write_reg(0x01, reg);
}

void fmtx_set_rfgain(u8 rfgain)
{
    u8 reg3, reg0_1, reg2;

    reg0_1 = fmtx_read_reg(0x01);
    reg2 = fmtx_read_reg(0x13);
    reg3 = fmtx_read_reg(0x02);

    rfgain &= 0x0F;
    reg0_1 = (reg0_1&0x3F) | (rfgain<<6);
    if(rfgain & 0x04){
        reg2 |= 0x80;
    }else{
        reg2 &= ~0x80;
    }

    if(rfgain & 0x08){
        reg3 |= 0x40;
    }else{
        reg3 &= ~0x40;
    }

    fmtx_write_reg(0x01, reg0_1);
    fmtx_write_reg(0x13, reg2);
    fmtx_write_reg(0x02, reg3);
}

void fmtx_set_alc(u8 sta)
{
    u8 reg;
    reg = fmtx_read_reg(0x04);
    if(!sta){
        reg &= ~0x80;
    }else{
        reg |= 0x80;
    }
    fmtx_write_reg(0x04, reg);
}

void fmtx_pa_external()
{
    u8 reg;

    reg = fmtx_read_reg(0x13);
    while( !(fmtx_read_reg(0x0F)&0x10));

    reg |= 0x04;
    fmtx_write_reg(0x13, reg);
}

void fmtx_set_sl(void)
{
    u8 reg;
    reg = fmtx_read_reg(0x12);
    fmtx_write_reg(0x12, (reg&0x81) | 0x7E);

    reg = fmtx_read_reg(0x14);
    fmtx_write_reg(0x14, (reg&0x02) | 0xDD);

    reg = fmtx_read_reg(0x16);
    fmtx_write_reg(0x16, (reg&0xF8) | 0x07);

    reg = fmtx_read_reg(0x0B);
    fmtx_write_reg(0x1B, reg | 0x04);

    reg = fmtx_read_reg(0x12);
    fmtx_write_reg(0x12, reg&0x7F);
}

void fmtx_set_au_enhance(void)
{
    u8 reg;
    reg = fmtx_read_reg(0x17);

    fmtx_write_reg(0x17, reg |= 0x20);
}

void fmtx_set_xtal(void)
{
    u8 reg;
    reg = fmtx_read_reg(0x1E);
    fmtx_write_reg(0x1E, reg | 0x40);
}

void fmtx_init(float freq)
{
    i2c_init();
    fmtx_set_freq(freq);
    fmtx_set_rfgain(4);
    fmtx_set_pga(PGA_0DB);
    fmtx_set_xtal();
}
