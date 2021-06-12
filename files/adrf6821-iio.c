#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/version.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>

#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "adrf6821_defs.h"
#include "adrf6821-iio.h"

void update_shadowregs(struct adrf6821_rf_phy_state *st, u8 *shadowregs){
	//ADRF6821_ADI_SPI_CONFIG
    shadowregs[0] = (st->SOFTRESET_ ? ADRF6821_FLAG_SOFTRESET_ : 0) |
                    (st->LSB_FIRST_ ? ADRF6821_FLAG_LSB_FIRST_ : 0) |
                    (st->ENDIAN_ ? ADRF6821_FLAG_ENDIAN_ : 0) |
                    (st->SDOACTIVE_ ? ADRF6821_FLAG_SDOACTIVE_ : 0) |
                    (st->SDOACTIVE ? ADRF6821_FLAG_SDOACTIVE : 0) |
                    (st->ENDIAN ? ADRF6821_FLAG_ENDIAN : 0) |
                    (st->LSB_FIRST ? ADRF6821_FLAG_LSB_FIRST : 0) |
					(st->SOFTRESET ? ADRF6821_FLAG_SOFTRESET : 0);
    //ADRF6821_SPI_CONFIG_B
    shadowregs[1] =	(st->SINGLE_INSTRUCTION ? ADRF6821_FLAG_SINGLE_INSTRUCTION : 0) |
                    (st->CSB_STALL ? ADRF6821_FLAG_CSB_STALL : 0) |
                    (st->MASTER_SLAVE_RB ? ADRF6821_FLAG_MASTER_SLAVE_RB : 0) |
                    ADRF6821_BITS_SOFT_RESET(st->SOFT_RESET) |
                    (st->MASTER_SLAVE_TRANSFER ? ADRF6821_FLAG_MASTER_SLAVE_TRANSFER : 0);
    //ADRF6821_SCRATCHPAD
    shadowregs[5] = st->SCRATCHPAD;
    //ADRF6821_MASTER_CONFIG
    shadowregs[9] = (st->EN_ANALOG_MASTER ? ADRF6821_FLAG_EN_ANALOG_MASTER : 0) |
                    (st->SPI_18_33_SEL ? ADRF6821_FLAG_SPI_18_33_SEL : 0);
    //ADRF6821_RF_SWITCH
    shadowregs[10] =(st->SEL_RFSW_SPI_CONTROL ? ADRF6821_FLAG_SEL_RFSW_SPI_CONTROL : 0) |
                    (st->RFSW_SEL1 ? ADRF6821_FLAG_RFSW_SEL1 : 0) |
                    (st->RFSW_SEL0 ? ADRF6821_FLAG_RFSW_SEL0 : 0) |
                    (st->ENB_SW_1P8_GEN ? ADRF6821_FLAG_ENB_SW_1P8_GEN : 0) |
					(st->EN_SW ? ADRF6821_FLAG_EN_SW : 0);
    //ADRF6821_DSA_CONTROL
    shadowregs[11] =ADRF6821_BITS_ATTEN_DSA(st->ATTEN_DSA) |
                    (st->ENB_DSA_1P8_GEN ? ADRF6821_FLAG_ENB_DSA_1P8_GEN : 0) |
                    (st->EN_DSA ? ADRF6821_FLAG_EN_DSA : 0);
    //ADRF6821_DEMOD_ENABLES
    shadowregs[12] =(st->EN_IMXBIAS_Q ? ADRF6821_FLAG_EN_IMXBIAS_Q : 0) |
                    (st->EN_IMXBIAS_I ? ADRF6821_FLAG_EN_IMXBIAS_I : 0) |
                    (st->EN_MIXIBIASGEN ? ADRF6821_FLAG_EN_MIXIBIASGEN : 0) |
                    (st->EN_MIX_Q ? ADRF6821_FLAG_EN_MIX_Q : 0) |
                    (st->EN_MIX_I ? ADRF6821_FLAG_EN_MIX_I : 0) |
                    (st->ENB_MIX_1P8_GEN ? ADRF6821_FLAG_ENB_MIX_1P8_GEN : 0);
    //ADRF6821_DEMOD_LO_COM_CTRL
    shadowregs[13] = st->CODE_MIXER_DRVR;

    //ADRF6821_DEMOD_OUT_COM_CTRL
    shadowregs[14] = st->CODE_MIXER_OCM;

    //ADRF6821_DEMOD_SPARES
    shadowregs[15] =ADRF6821_BITS_Q_MIXER_GAIN_ADJ(st->Q_MIXER_GAIN_ADJ) |
                    ADRF6821_BITS_I_MIXER_GAIN_ADJ(st->I_MIXER_GAIN_ADJ) |
                    ADRF6821_BITS_MIXER_GAIN_PEAK(st->MIXER_GAIN_PEAK);
    //ADRF6821_DEMOD_DRIVER_COM_CTRL
    shadowregs[16] =(st->EN_ICMLOBIAS_Q ? ADRF6821_FLAG_EN_ICMLOBIAS_Q : 0) |
                    (st->EN_ICMLOBIAS_I ? ADRF6821_FLAG_EN_ICMLOBIAS_I : 0) |
                    (st->EN_ICMOBIAS_Q ? ADRF6821_FLAG_EN_ICMOBIAS_Q : 0) |
                    (st->EN_ICMOBIAS_I ? ADRF6821_FLAG_EN_ICMOBIAS_I : 0);
    //ADRF6821_DC_CTRL
    shadowregs[17] =(st->EN_DC_DAC_Q ? ADRF6821_FLAG_EN_DC_DAC_Q : 0) |
                    (st->EN_DC_DAC_I ? ADRF6821_FLAG_EN_DC_DAC_I : 0) |
                    (st->ENB_DCCOMP_1P8_GEN ? ADRF6821_FLAG_ENB_DCCOMP_1P8_GEN : 0);
    //ADRF6821_DC_COMP_I_CHAN_RF0
    shadowregs[18] = st->CODE_DC_IDAC_RF0;
    //ADRF6821_DC_COMP_Q_CHAN_RF0
    shadowregs[19] = st->CODE_DC_QDAC_RF0;
    //ADRF6821_DC_COMP_I_CHAN_RF1
    shadowregs[20] = st->CODE_DC_IDAC_RF1;
    //ADRF6821_DC_COMP_Q_CHAN_RF1
    shadowregs[21] = st->CODE_DC_QDAC_RF1;
    //ADRF6821_LPF_BW_SEL
    shadowregs[22] =(st->SEL_LPF_BW_LSB ? ADRF6821_FLAG_SEL_LPF_BW_LSB : 0) |
                    (st->SEL_LPF_BW_MSB ? ADRF6821_FLAG_SEL_LPF_BW_MSB : 0);
    //ADRF6821_IF_AMP_CTRL
    shadowregs[23] =(st->EN_IFAMP_Q ? ADRF6821_FLAG_EN_IFAMP_Q : 0) |
                    (st->EN_IFAMP_I ? ADRF6821_FLAG_EN_IFAMP_I : 0);
    //ADRF6821_LO_CTRL
    shadowregs[24] =(st->SEL_LODRV_PREDRVQ_POL ? ADRF6821_FLAG_SEL_LODRV_PREDRVQ_POL : 0) |
                    (st->SEL_LODRV_PREDRVI_POL ? ADRF6821_FLAG_SEL_LODRV_PREDRVI_POL : 0);
    //ADRF6821_EN_LO_DIVIDER_CTRL
    shadowregs[25] =(st->EN_LODRV_PREDRVQ ? ADRF6821_FLAG_EN_LODRV_PREDRVQ : 0) |
                    (st->EN_LODRV_PREDRVI ? ADRF6821_FLAG_EN_LODRV_PREDRVI : 0) |
                    (st->EN_LODRV_DRVQ ? ADRF6821_FLAG_EN_LODRV_DRVQ : 0) |
                    (st->EN_LODRV_DRVI ? ADRF6821_FLAG_EN_LODRV_DRVI : 0) |
                    (st->EN_DIVPATH_QUADDIV ? ADRF6821_FLAG_EN_DIVPATH_QUADDIV : 0) |
                    (st->EN_DIVPATH_BUF ? ADRF6821_FLAG_EN_DIVPATH_BUF : 0) |
                    (st->EN_IBIASGEN ? ADRF6821_FLAG_EN_IBIASGEN : 0);
    //ADRF6821_LO_PHASE_ADJ
    shadowregs[26] =ADRF6821_BITS_TRM_LODRV_CAPQ(st->TRM_LODRV_CAPQ) |
                    ADRF6821_BITS_TRM_LODRV_CAPI(st->TRM_LODRV_CAPI);
    //ADRF6821_BLOCK_RESETS
    shadowregs[27] =0x80 |
                    (st->ARSTB_BLOCK_LKD ? ADRF6821_FLAG_ARSTB_BLOCK_LKD : 0) |
                    (st->ARSTB_BLOCK_AUTOCAL ? ADRF6821_FLAG_ARSTB_BLOCK_AUTOCAL : 0) |
                    (st->ARSTB_BLOCK_NDIV ? ADRF6821_FLAG_ARSTB_BLOCK_NDIV : 0) |
                    (st->ARSTB_BLOCK_RDIV ? ADRF6821_FLAG_ARSTB_BLOCK_RDIV : 0) |
                    (st->ARSTB_BLOCK_DSMOSTG ? ADRF6821_FLAG_ARSTB_BLOCK_DSMOSTG : 0) |
                    (st->ARSTB_BLOCK_DSMCORE ? ADRF6821_FLAG_ARSTB_BLOCK_DSMCORE : 0) |
                    (st->ARSTB_BLOCK_DSMALL ? ADRF6821_FLAG_ARSTB_BLOCK_DSMALL : 0);
    //ADRF6821_SIG_PATH_9_NORMAL
    shadowregs[28] = ADRF6821_BITS_TRM_MIXLODRV_DRV_POUT(st->TRM_MIXLODRV_DRV_POUT) |
                    ADRF6821_BITS_TRM_XLODRV_DRV_POUT(st->TRM_XLODRV_DRV_POUT);
    //ADRF6821_INT_L
    shadowregs[29] =(st->INT_DIV) & 0xFF;
    //ADRF6821_INT_H
    shadowregs[30] =(st->INT_DIV >> 8) & 0xFF;
    //ADRF6821_FRAC1_L
    shadowregs[31] =(st->FRAC) & 0xFF;
    //ADRF6821_FRAC1_M
    shadowregs[32] =(st->FRAC >> 8) & 0xFF;
    //ADRF6821_FRAC1_H
    shadowregs[33] =(st->FRAC >> 16) & 0xFF;
    //ADRF6821_SD_PHASE_L_0
    shadowregs[34] =(st->PHASE) & 0xFF;
    //ADRF6821_SD_PHASE_M_0
    shadowregs[35] =(st->PHASE >> 8) & 0xFF;
    //ADRF6821_SD_PHASE_H_0
    shadowregs[36] =(st->PHASE >> 16) & 0xFF;
    //ADRF6821_MOD_L
    shadowregs[37] =(st->MOD2) & 0xFF;
    //ADRF6821_MOD_H
    shadowregs[38] =(st->MOD2 >> 8) & 0x3F;
    //ADRF6821_SYNTH
    shadowregs[39] =(st->PRE_SEL ? ADRF6821_FLAG_PRE_SEL : 0) |
                    (st->EN_FBDIV ? ADRF6821_FLAG_EN_FBDIV : 0);
    //ADRF6821_R_DIV
    shadowregs[40] =(st->R_DIV) & 0x7F;
    //ADRF6821_SYNTH_0
    shadowregs[41] =(st->DOUBLER_EN ? ADRF6821_FLAG_DOUBLER_EN : 0) |
                    0x04 |
                    (st->RDIV2_SEL ? ADRF6821_FLAG_RDIV2_SEL : 0);
    //ADRF6821_MULTI_FUNC_SYNTH_CTRL_0214
    shadowregs[42] =ADRF6821_BITS_LD_BIAS(st->LD_BIAS) |
                    ADRF6821_BITS_LDP(st->LDP);
    //ADRF6821_SI_BAND_0
    shadowregs[43] =st->SI_BAND_SEL;
    //ADRF6821_SI_VCO_SEL
    shadowregs[44] =(st->SI_VCO_SEL) & 0x0f;
    //ADRF6821_VCO_FSM
    shadowregs[48] =(st->DISABLE_CAL ? ADRF6821_FLAG_DISABLE_CAL : 0);
    //ADRF6821_SD_CTRL
    shadowregs[49] =(st->SD_EN_FRAC0 ? ADRF6821_FLAG_SD_EN_FRAC0 : 0) |
                    (st->SD_EN_OUT_OFF ? ADRF6821_FLAG_SD_EN_OUT_OFF : 0) |
                    (st->SD_SM_2 ? ADRF6821_FLAG_SD_SM_2 : 0);
    //ADRF6821_MULTI_FUNC_SYNTH_CTRL_022C
    shadowregs[50] = (st->CP_HIZ) & 0x03;
    //ADRF6821_MULTI_FUNC_SYNTH_CTRL_022D
    shadowregs[51] =(st->EN_PFD_CP ? ADRF6821_FLAG_EN_PFD_CP : 0) |
                    (st->BLEED_POL ? ADRF6821_FLAG_BLEED_POL : 0) |
                    (st->INT_ABP ? ADRF6821_FLAG_INT_ABP : 0) |
                    (st->BLEED_EN ? ADRF6821_FLAG_BLEED_EN : 0);
    //ADRF6821_CP_CURR
    shadowregs[52] =(st->CP_CURRENT) & 0x0f;
    //ADRF6821_BICP
    shadowregs[53] =st->BICP;
    //ADRF6821_FRAC2_L
    shadowregs[54] =(st->FRAC2) & 0xFF;
    //ADRF6821_FRAC2_H
    shadowregs[55] =(st->FRAC2 >> 8) & 0x3F;
    //ADRF6821_MULTI_FUNC_SYNTH_CTRL_0235
    shadowregs[56] =(st->PHASE_ADJ_EN ? ADRF6821_FLAG_PHASE_ADJ_EN : 0);
    //ADRF6821_VCO_LUT_CTRL
    shadowregs[57] =(st->SI_VCO_FORCE_CAPSVCOI ? ADRF6821_FLAG_SI_VCO_FORCE_CAPSVCOI : 0) |
                    (st->SI_VCO_FORCE_VCO ? ADRF6821_FLAG_SI_VCO_FORCE_VCO : 0) |
                    (st->SI_VCO_FORCE_CAPS ? ADRF6821_FLAG_SI_VCO_FORCE_CAPS : 0);
    //ADRF6821_MULTI_FUNC_CTRL
    shadowregs[59] =(st->SPI_1P8_3P3_CTRL ? ADRF6821_FLAG_SPI_1P8_3P3_CTRL : 0);
    //ADRF6821_LO_CNTRL2
    shadowregs[60] =(st->EN_BIAS_R ? ADRF6821_FLAG_EN_BIAS_R : 0) |
                    (st->REFBUF_EN ? ADRF6821_FLAG_REFBUF_EN : 0) |
                    0x13;
    //ADRF6821_LO_CNTRL8
    shadowregs[61] =(st->MIX_OE ? ADRF6821_FLAG_MIX_OE : 0) |
                    (st->LO_OE ? ADRF6821_FLAG_LO_OE : 0) |
                    (st->USEEXT_LOI ? ADRF6821_FLAG_USEEXT_LOI : 0) |
                    ADRF6821_BITS_OUT_DIVRATIO(st->OUT_DIVRATIO);
    //ADRF6821_DISABLE_CFG
    shadowregs[74] =ADRF6821_BITS_DSM_LAUNCH_DLY(st->DSM_LAUNCH_DLY) |
                    (st->DISABLE_FREQHOP ? ADRF6821_FLAG_DISABLE_FREQHOP : 0) |
                    (st->DISABLE_DBLBUFFERING ? ADRF6821_FLAG_DISABLE_DBLBUFFERING : 0) |
                    (st->DISABLE_PHASEADJ ? ADRF6821_FLAG_DISABLE_PHASEADJ : 0);
}

u32 adrf6821_tune(struct adrf6821_rf_phy_state *st, u32 tune_freq){
	struct adrf6821_rf_phy *phy = st->phy;
	struct spi_device *spi;
	u64 ideal_vco_freq, tmp, ideal_divider_times_2exp24, difference_times_2exp24;
	u8 out_div_shift;
    uint8_t shadowregs[NUM_REGS];
    int ret, regctr;

	if (phy==NULL)
		return 0;

	spi = phy->spi;

	if (spi==NULL){
		dev_err(&st->phy->indio_dev->dev, "couldn't find SPI device\n");
		return 0;
	}

	if (tune_freq < 450000000UL)
	{
		dev_err(&st->phy->indio_dev->dev, "frequency %d too low\n", tune_freq);
		return 0;
	}
	if ((tune_freq >= 450000000UL) && (tune_freq < 500000000UL))
	{
		st->OUT_DIVRATIO = 8;
		out_div_shift = 4;
	}
	if ((tune_freq >= 500000000UL) && (tune_freq < 1000000000UL))
	{
		st->OUT_DIVRATIO = 4;
		out_div_shift = 3;
	}
	if ((tune_freq >= 1000000000UL) && (tune_freq < 2000000000UL))
	{
		st->OUT_DIVRATIO = 2;
		out_div_shift = 2;
	}
	if ((tune_freq >= 2000000000UL) && (tune_freq <= 2800000000UL))
	{
		st->OUT_DIVRATIO = 1;
		out_div_shift = 1;
	}
	if (tune_freq > 2800000000UL)
	{
		dev_err(&st->phy->indio_dev->dev, "frequency %d too high\n", tune_freq);
		return 0;
	}

	ideal_vco_freq = (u64)tune_freq * 2 * st->OUT_DIVRATIO;

	dev_dbg(&spi->dev, "choosing LO divider %d\n", st->OUT_DIVRATIO);

	tmp = ideal_vco_freq << 24;
	do_div(tmp, st->pfd_freq_Hz);
	ideal_divider_times_2exp24 = tmp;

	difference_times_2exp24 = (ideal_vco_freq << 24) - (ideal_divider_times_2exp24 * st->pfd_freq_Hz);

	st->INT_DIV = (u32)(ideal_divider_times_2exp24 >> 24);
	st->FRAC = (u32)(ideal_divider_times_2exp24 & 0xFFFFFF);

	st->MOD2 = 10000;
	st->FRAC2 = (difference_times_2exp24 * 10000) >> 24;

	dev_dbg(&spi->dev, "found divider: INT_DIV=%d, FRAC=%d, MOD=%d, FRAC2=%d\n",st->INT_DIV, st->FRAC, st->MOD2, st->FRAC2);

	st->vco_freq_Hz = (u64)st->INT_DIV * (u64)st->pfd_freq_Hz;
	st->vco_freq_Hz += ((u64)st->FRAC * (u64)st->pfd_freq_Hz) >> 24;
	tmp = st->pfd_freq_Hz;
	tmp *= st->FRAC2;
	tmp = tmp >> 24;
	do_div(tmp, st->MOD2);
	st->vco_freq_Hz += tmp;

	st->tune_freq_Hz = st->vco_freq_Hz >> out_div_shift;

	dev_dbg(&spi->dev, "f_VCO=%llu, f_LO=%u\n",st->vco_freq_Hz, st->tune_freq_Hz);

    update_shadowregs(st, shadowregs);

    for (regctr=0; regctr < WRITE_LENGTH_PLL;regctr++)
    {
        int regidx = writeorder_pll[regctr];
        adrf6821_spi_write(spi, regsnames[regidx].reg, shadowregs[regidx]);
    }

	msleep_interruptible(1);
    ret=adrf6821_spi_read(spi, ADRF6821_LOCK_DETECT);

    if (ret!=0x01){
    	dev_err(&st->phy->indio_dev->dev, "did not obtain PLL lock\n");
    }

	return st->tune_freq_Hz;
}

int adrf6821_spi_write(struct spi_device *spi, u16 reg, u8 val)
{
	u8 buf[3];
	int ret;

	buf[0] = (reg >> 8) & 0x7F;
	buf[1] = reg & 0xFF;
	buf[2] = val & 0xFF;

	ret = spi_write_then_read(spi, buf, 3, NULL, 0);
	if (ret < 0) {
		dev_err(&spi->dev, "Write Error %d", ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: reg 0x%X val 0x%X\n", __func__, reg, val);

	return 0;
}

int adrf6821_spi_read(struct spi_device *spi, u16 reg)
{
	u8 buf[2];
	u8 val;
	int ret;

	buf[0] = ((reg >> 8) & 0x7F) | 0x80;
	buf[1] = reg & 0xFF;

	ret = spi_write_then_read(spi, buf, 2, &val, 1);
	if (ret < 0) {
		dev_err(&spi->dev, "Read Error %d", ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: reg 0x%X val 0x%X\n", __func__, reg, val);

	ret = val;

	return ret;
}

static int __adrf6821_of_get_u32(struct iio_dev *indio_dev,
			     struct device_node *np, const char *propname,
			     u32 defval, void *out_value, u32 size)
{
	u32 tmp = defval;
	int ret;

	ret = of_property_read_u32(np, propname, &tmp);

	if (out_value) {
		switch (size){
		case 1:
			*(u8*)out_value = tmp;
			break;
		case 2:
			*(u16*)out_value = tmp;
			break;
		case 4:
			*(u32*)out_value = tmp;
			break;
		default:
			ret = -EINVAL;
		}
	}
	return ret;
}

static void adrf6821_of_get_bool(struct iio_dev *indio_dev, struct device_node *np,
			       const char *propname, bool *out_value)
{
	*out_value = of_property_read_bool(np, propname);

	return;
}

static struct adrf6821_phy_platform_data
	*adrf6821_phy_parse_dt(struct iio_dev *iodev, struct device *dev)
{
	struct adrf6821_phy_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	return pdata;
}


struct adrf6821_rf_phy* adrf6821_spi_to_phy(struct spi_device *spi)
{
	return spi_get_drvdata(spi);
}

static void adrf6821_init_state(struct adrf6821_rf_phy *phy)
{
	struct adrf6821_rf_phy_state *st = phy->state;

	st->phy = phy;

	st->tune_freq_Hz=2440000000UL;
    st->ARSTB_BLOCK_AUTOCAL = 1;
    st->ARSTB_BLOCK_DSMALL = 1;
    st->ARSTB_BLOCK_DSMCORE = 1;
    st->ARSTB_BLOCK_DSMOSTG = 1;
    st->ARSTB_BLOCK_LKD = 1;
    st->ARSTB_BLOCK_NDIV = 1;
    st->ARSTB_BLOCK_RDIV = 1;
    st->ATTEN_DSA = 0;
    st->BICP = 64;
    st->BLEED_EN = 1;
    st->BLEED_POL = 0;
    st->CODE_DC_IDAC_RF0 = 0;
    st->CODE_DC_IDAC_RF1 = 0;
    st->CODE_DC_QDAC_RF0 = 0;
    st->CODE_DC_QDAC_RF1 = 0;
    st->CODE_MIXER_DRVR = 0;
    st->CODE_MIXER_OCM = 0;
    st->CP_CURRENT = 7;
    st->CP_HIZ = 0b11;
    st->CSB_STALL = 0;
    st->DISABLE_CAL = 0;
    st->DISABLE_DBLBUFFERING = 0;
    st->DISABLE_FREQHOP = 0;
    st->DISABLE_PHASEADJ = 0;
    st->DOUBLER_EN = 0;
    st->DSM_LAUNCH_DLY = 0;
    st->ENB_DCCOMP_1P8_GEN = 0;
    st->ENB_DSA_1P8_GEN = 0;
    st->ENB_MIX_1P8_GEN = 0;
    st->ENB_SW_1P8_GEN = 0;
    st->ENDIAN = 0;
    st->ENDIAN_ = 0;
    st->EN_ANALOG_MASTER = 0;
    st->EN_BIAS_R = 1;
    st->EN_DC_DAC_I = 0;
    st->EN_DC_DAC_Q = 0;
    st->EN_DIVPATH_BUF = 0;
    st->EN_DIVPATH_QUADDIV = 0;
    st->EN_DSA = 0;
    st->EN_FBDIV = 1;
    st->EN_IBIASGEN = 0;
    st->EN_ICMLOBIAS_I = 0;
    st->EN_ICMLOBIAS_Q = 0;
    st->EN_ICMOBIAS_I = 0;
    st->EN_ICMOBIAS_Q = 0;
    st->EN_IFAMP_I = 0;
    st->EN_IFAMP_Q = 0;
    st->EN_IMXBIAS_I = 0;
    st->EN_IMXBIAS_Q = 0;
    st->EN_LODRV_DRVI = 0;
    st->EN_LODRV_DRVQ = 0;
    st->EN_LODRV_PREDRVI = 0;
    st->EN_LODRV_PREDRVQ = 0;
    st->EN_MIXIBIASGEN = 0;
    st->EN_MIX_I = 0;
    st->EN_MIX_Q = 0;
    st->EN_PFD_CP = 1;
    st->EN_SW = 0;
    st->FRAC = 0;
    st->FRAC2 = 0;
    st->GPO1_BLK_SEL = 0;
    st->GPO1_ENABLE = 0;
    st->GPO1_SGNL_SEL = 0;
    st->INT_ABP = 0;
    st->INT_DIV = 80;
    st->I_MIXER_GAIN_ADJ = 0;
    st->LDP = 1;
    st->LD_BIAS = 1;
    st->LD_HIZ = 0;
    st->LO_OE = 0;
    st->LSB_FIRST = 0;
    st->LSB_FIRST_ = 0;
    st->MASTER_SLAVE_RB = 0;
    st->MASTER_SLAVE_TRANSFER = 0;
    st->MIXER_GAIN_PEAK = 0;
    st->MIX_OE = 0;
    st->MOD2 = 0;
    st->OUT_DIVRATIO = 8;
    st->PHASE = 0;
    st->PHASE_ADJ_EN = 0;
    st->PRE_SEL = 0;
    st->Q_MIXER_GAIN_ADJ = 0;
    st->RDIV2_SEL = 0;
    st->REFBUF_EN = 1;
    st->RFSW_SEL0 = 0;
    st->RFSW_SEL1 = 0;
    st->R_DIV = 1;
    st->SCRATCHPAD = 0;
    st->SDOACTIVE = 1;
    st->SDOACTIVE_ = 1;
    st->SD_EN_FRAC0 = 0;
    st->SD_EN_OUT_OFF = 0;
    st->SD_SM_2 = 1;
    st->SEL_IDAC_RANGE = 0;
    st->SEL_LODRV_PREDRVI_POL = 0;
    st->SEL_LODRV_PREDRVQ_POL = 0;
    st->SEL_LPF_BW_LSB = 0;
    st->SEL_LPF_BW_MSB = 0;
    st->SEL_QDAC_RANGE = 0;
    st->SEL_RFSW_SPI_CONTROL = 0;
    st->SINGLE_INSTRUCTION = 0;
    st->SI_BAND_SEL = 0;
    st->SI_VCO_FORCE_CAPS = 0;
    st->SI_VCO_FORCE_CAPSVCOI = 0;
    st->SI_VCO_FORCE_VCO = 0;
    st->SI_VCO_SEL = 0;
    st->SOFTRESET = 0;
    st->SOFTRESET_ = 0;
    st->SOFT_RESET = 0;
    st->SPI_18_33_SEL = 1;
    st->SPI_1P8_3P3_CTRL = 1;
    st->TRM_LODRV_CAPI = 0;
    st->TRM_LODRV_CAPQ = 0;
    st->TRM_MIXLODRV_DRV_POUT = 1;
    st->TRM_XLODRV_DRV_POUT = 1;
    st->USEEXT_LOI = 0;

	st->pll_ref_in_Hz=50000000UL;

	st->pll_ref_div = (st->DOUBLER_EN==1) ? ((1+st->RDIV2_SEL) * st->R_DIV) >> 2 : ((1+st->RDIV2_SEL) * st->R_DIV);

	st->pfd_freq_Hz = st->pll_ref_in_Hz / st->pll_ref_div;

	return;
}

static IIO_CONST_ATTR(in_voltage_rf_bandwidth_available, "[1000000 1 2000000]");

static struct attribute *adrf6821_phy_attributes[] = {
		&iio_const_attr_in_voltage_rf_bandwidth_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group adrf6821_phy_attribute_group = {
	.attrs = adrf6821_phy_attributes,
};

static int adrf6821_phy_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
	case IIO_CHAN_INFO_SAMP_FREQ:
	case IIO_CHAN_INFO_PROCESSED:
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_OFFSET:
	case IIO_CHAN_INFO_SCALE:
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int adrf6821_phy_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct adrf6821_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
	case IIO_CHAN_INFO_SAMP_FREQ:
	case IIO_CHAN_INFO_RAW:
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int adrf6821_phy_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
	case IIO_CHAN_INFO_SAMP_FREQ: {
		break;
	}
	}

	return -EINVAL;
}

//this is referenced in adrf6821_probe(), reporting general information (not channel specific)

static const struct iio_info adrf6821_phy_info = {
	.read_raw = &adrf6821_phy_read_raw,
	.write_raw = &adrf6821_phy_write_raw,
	.read_avail = adrf6821_phy_read_avail,
	.attrs = &adrf6821_phy_attribute_group,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,0,0)
	.driver_module = THIS_MODULE,
#endif	
};

//here starts stuff dealing with the extended info

static ssize_t adrf6821_phy_lo_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct adrf6821_rf_phy *phy = iio_priv(indio_dev);
	struct adrf6821_rf_phy_state *st = phy->state;
	u64 readin;
	int ret = 0;
	u32 u32_ret = 0;

	switch (private) {
		case LOEXT_FREQ:
				ret = kstrtoull(buf, 10, &readin);
			if (ret)
				return ret;
			break;
	}

	mutex_lock(&indio_dev->mlock);
	switch (private) {
	case LOEXT_FREQ:
		st->tune_freq_Hz = (u32)readin;
		u32_ret = adrf6821_tune(st, st->tune_freq_Hz);
		if (u32_ret==0)	//tuning failed, because out of range
			ret = -EINVAL;
		else
			ret = 0;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t adrf6821_phy_lo_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct adrf6821_rf_phy *phy = iio_priv(indio_dev);
	struct adrf6821_rf_phy_state *st = phy->state;
	u64 val = 0;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch (private) {
	case LOEXT_FREQ:
		val = st->tune_freq_Hz;
		break;

	default:
		ret = 0;

	}
	mutex_unlock(&indio_dev->mlock);

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}


#define _ADRF6821_EXT_LO_INFO(_name, _ident) { \
	.name = _name, \
	.read = adrf6821_phy_lo_read, \
	.write = adrf6821_phy_lo_write, \
	.private = _ident, \
}

#define _ADRF6821_EXT_LO_INFO_RO(_name, _ident) { \
	.name = _name, \
	.read = adrf6821_phy_lo_read, \
	.private = _ident, \
}

static const struct iio_chan_spec_ext_info adrf6821_phy_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRF6821_EXT_LO_INFO("frequency", LOEXT_FREQ),
	_ADRF6821_EXT_LO_INFO_RO("frequency_available", LOEXT_FREQ_AVAILABLE),
	{ },
};


//this struct is referenced in adrf6821_probe()
//it reports one channel with extended information

static const struct iio_chan_spec adrf6821_phy_chan[] = {
{	/* RX LO */
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 0,
	.extend_name = "RX_LO",
	.ext_info = adrf6821_phy_ext_info,
}
};

static int adrf6821_setup(struct adrf6821_rf_phy *phy)
{
	struct adrf6821_rf_phy_state *st = phy->state;
	struct device *dev = &phy->spi->dev;
	struct spi_device *spi = phy->spi;
	int ret, regctr;
    uint8_t shadowregs[NUM_REGS];

    //set to 2440MHz as initial value
    st->OUT_DIVRATIO = 1;
    st->tune_freq_Hz = 2440000000UL;
    st->vco_freq_Hz = 2440000000ULL * 2;
    st->INT_DIV = 48;
    st->FRAC = 0x00666666;
    st->MOD2 = 10000;
    st->FRAC2 = 0;

    update_shadowregs(st, shadowregs);

    adrf6821_spi_write(spi, ADRF6821_ADI_SPI_CONFIG,
			ADRF6821_FLAG_SOFTRESET_ |
            (st->SDOACTIVE_ ? ADRF6821_FLAG_SDOACTIVE_ : 0) |
            (st->SDOACTIVE ? ADRF6821_FLAG_SDOACTIVE : 0) |
			ADRF6821_FLAG_SOFTRESET
			);
	msleep_interruptible(100);

    adrf6821_spi_write(spi, ADRF6821_ADI_SPI_CONFIG,
            (st->SDOACTIVE_ ? ADRF6821_FLAG_SDOACTIVE_ : 0) |
            (st->SDOACTIVE ? ADRF6821_FLAG_SDOACTIVE : 0)
    		);

    //write all shadow registers
    for (regctr=0; regctr < WRITE_LENGTH_ALL;regctr++)
    {
        int regidx = writeorder_all[regctr];
        adrf6821_spi_write(spi, regsnames[regidx].reg, shadowregs[regidx]);
    }

	msleep_interruptible(1);
    ret=adrf6821_spi_read(spi, ADRF6821_LOCK_DETECT);

    if (ret!=0x01){
    	dev_err(&st->phy->indio_dev->dev, "did not obtain initial PLL lock\n");
    }

    st->EN_ANALOG_MASTER = 1;

    st->SEL_RFSW_SPI_CONTROL = 1;
    st->RFSW_SEL0 = 1;
    st->EN_SW = 1;

    st->EN_DSA = 1;

    st->EN_IMXBIAS_I = 1;
    st->EN_IMXBIAS_Q = 1;
    st->EN_MIXIBIASGEN = 1;
    st->EN_MIX_I = 1;
    st->EN_MIX_Q = 1;

    st->CODE_MIXER_DRVR = 0x2d;
    st->CODE_MIXER_OCM = 0x2d;

    st->I_MIXER_GAIN_ADJ = 1;
    st->Q_MIXER_GAIN_ADJ = 1;

    st->EN_ICMLOBIAS_I = 1;
    st->EN_ICMLOBIAS_Q = 1;
    st->EN_ICMOBIAS_I = 1;
    st->EN_ICMOBIAS_Q = 1;

    st->EN_DC_DAC_I = 1;
    st->EN_DC_DAC_Q = 1;

    st->SEL_LPF_BW_LSB = 1;
    st->SEL_LPF_BW_MSB = 1;

    st->EN_IFAMP_I = 1;
    st->EN_IFAMP_Q = 1;

    st->EN_LODRV_DRVI = 1;
    st->EN_LODRV_DRVQ = 1;
    st->EN_LODRV_PREDRVI = 1;
    st->EN_LODRV_PREDRVQ = 1;
    st->EN_DIVPATH_BUF = 1;
    st->EN_DIVPATH_QUADDIV = 1;
    st->EN_IBIASGEN = 1;

    st->MIX_OE = 1;

    update_shadowregs(st, shadowregs);

    //write shadow registers not related to the PLL
    for (regctr=0; regctr < WRITE_LENGTH_NOTPLL;regctr++)
    {
        int regidx = writeorder_notpll[regctr];
        adrf6821_spi_write(spi, regsnames[regidx].reg, shadowregs[regidx]);
    }

	return 0;

}



static int adrf6821_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adrf6821_rf_phy_state *st;
	struct adrf6821_rf_phy *phy;
	int ret;

	dev_info(&spi->dev, "%s : enter (%s)", __func__,
		 spi_get_device_id(spi)->name);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->state = st;
	phy->indio_dev = indio_dev;
	phy->spi = spi;

	adrf6821_init_state(phy);

	phy->pdata = adrf6821_phy_parse_dt(indio_dev, &spi->dev);
	if (phy->pdata == NULL)
		return -EINVAL;

	//adrf6821_reset(phy);

	ret = adrf6821_setup(phy);

	indio_dev->dev.parent = &spi->dev;

	if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = "adrf6821";

	indio_dev->info = &adrf6821_phy_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adrf6821_phy_chan;
	indio_dev->num_channels = 1;

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto out_iio_device_unregister;

	dev_info(&spi->dev, "%s : ADRF6821 successfully initialized", __func__);

	return 0;

out_iio_device_unregister:
	iio_device_unregister(indio_dev);

	return ret;
}

static int adrf6821_remove(struct spi_device *spi)
{
	struct adrf6821_rf_phy *phy = adrf6821_spi_to_phy(spi);

	//do something

	return 0;
}

static const struct spi_device_id adrf6821_id[] = {
	{"adrf6821", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, adrf6821_id);

static struct spi_driver adrf6821_driver = {
	.driver = {
		.name	= "adrf6821",
		.owner	= THIS_MODULE,
	},
	.probe		= adrf6821_probe,
	.remove		= adrf6821_remove,
	.id_table	= adrf6821_id,
};
module_spi_driver(adrf6821_driver);

MODULE_AUTHOR("Henning Paul <hnch@gmx.net>");
MODULE_DESCRIPTION("Analog Devices ADRF6821 Receiver");
MODULE_LICENSE("GPL v2");
