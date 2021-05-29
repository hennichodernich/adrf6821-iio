#ifndef IIO_FREQUENCY_ADRF6821_H_
#define IIO_FREQUENCY_ADRF6821_H_

#include "adrf6821_defs.h"

#define NUM_REGS 75
#define WRITE_LENGTH_ALL 50
#define WRITE_LENGTH_PLL 11
#define WRITE_LENGTH_NOTPLL 24

typedef struct{
    u16 reg;
    char *name;
} t_regnamepair;

static const t_regnamepair regsnames[NUM_REGS] = {
	{ADRF6821_ADI_SPI_CONFIG, "ADI_SPI_CONFIG"},
	{ADRF6821_SPI_CONFIG_B, "SPI_CONFIG_B"},
	{ADRF6821_CHIPTYPE, "CHIPTYPE"},
	{ADRF6821_PRODUCT_ID_L, "PRODUCT_ID_L"},
	{ADRF6821_PRODUCT_ID_H, "PRODUCT_ID_H"},
	{ADRF6821_SCRATCHPAD, "SCRATCHPAD"},
	{ADRF6821_SPI_REV, "SPI_REV"},
	{ADRF6821_VENDOR_ID_L, "VENDOR_ID_L"},
	{ADRF6821_VENDOR_ID_H, "VENDOR_ID_H"},
	{ADRF6821_MASTER_CONFIG, "MASTER_CONFIG"},
	{ADRF6821_RF_SWITCH, "RF_SWITCH"},
	{ADRF6821_DSA_CONTROL, "DSA_CONTROL"},
	{ADRF6821_DEMOD_ENABLES, "DEMOD_ENABLES"},
	{ADRF6821_DEMOD_LO_COM_CTRL, "DEMOD_LO_COM_CTRL"},
	{ADRF6821_DEMOD_OUT_COM_CTRL, "DEMOD_OUT_COM_CTRL"},
	{ADRF6821_DEMOD_SPARES, "DEMOD_SPARES"},
	{ADRF6821_DEMOD_DRIVER_COM_CTRL, "DEMOD_DRIVER_COM_CTRL"},
	{ADRF6821_DC_CTRL, "DC_CTRL"},
	{ADRF6821_DC_COMP_I_CHAN_RF0, "DC_COMP_I_CHAN_RF0"},
	{ADRF6821_DC_COMP_Q_CHAN_RF0, "DC_COMP_Q_CHAN_RF0"},
	{ADRF6821_DC_COMP_I_CHAN_RF1, "DC_COMP_I_CHAN_RF1"},
	{ADRF6821_DC_COMP_Q_CHAN_RF1, "DC_COMP_Q_CHAN_RF1"},
	{ADRF6821_LPF_BW_SEL, "LPF_BW_SEL"},
	{ADRF6821_IF_AMP_CTRL, "IF_AMP_CTRL"},
	{ADRF6821_LO_CTRL, "LO_CTRL"},
	{ADRF6821_EN_LO_DIVIDER_CTRL, "EN_LO_DIVIDER_CTRL"},
	{ADRF6821_LO_PHASE_ADJ, "LO_PHASE_ADJ"},
	{ADRF6821_BLOCK_RESETS, "BLOCK_RESETS"},
	{ADRF6821_SIG_PATH_9_NORMAL, "SIG_PATH_9_NORMAL"},
	{ADRF6821_INT_L, "INT_L"},
	{ADRF6821_INT_H, "INT_H"},
	{ADRF6821_FRAC1_L, "FRAC1_L"},
	{ADRF6821_FRAC1_M, "FRAC1_M"},
	{ADRF6821_FRAC1_H, "FRAC1_H"},
	{ADRF6821_SD_PHASE_L_0, "SD_PHASE_L_0"},
	{ADRF6821_SD_PHASE_M_0, "SD_PHASE_M_0"},
	{ADRF6821_SD_PHASE_H_0, "SD_PHASE_H_0"},
	{ADRF6821_MOD_L, "MOD_L"},
	{ADRF6821_MOD_H, "MOD_H"},
	{ADRF6821_SYNTH, "SYNTH"},
	{ADRF6821_R_DIV, "R_DIV"},
	{ADRF6821_SYNTH_0, "SYNTH_0"},
	{ADRF6821_MULTI_FUNC_SYNTH_CTRL_0214, "MULTI_FUNC_SYNTH_CTRL_0214"},
	{ADRF6821_SI_BAND_0, "SI_BAND_0"},
	{ADRF6821_SI_VCO_SEL, "SI_VCO_SEL"},
	{ADRF6821_VCO_TIMEOUT_L, "VCO_TIMEOUT_L"},
	{ADRF6821_VCO_TIMEOUT_H, "VCO_TIMEOUT_H"},
	{ADRF6821_VCO_BAND_DIV, "VCO_BAND_DIV"},
	{ADRF6821_VCO_FSM, "VCO_FSM"},
	{ADRF6821_SD_CTRL, "SD_CTRL"},
	{ADRF6821_MULTI_FUNC_SYNTH_CTRL_022C, "MULTI_FUNC_SYNTH_CTRL_022C"},
	{ADRF6821_MULTI_FUNC_SYNTH_CTRL_022D, "MULTI_FUNC_SYNTH_CTRL_022D"},
	{ADRF6821_CP_CURR, "CP_CURR"},
	{ADRF6821_BICP, "BICP"},
	{ADRF6821_FRAC2_L, "FRAC2_L"},
	{ADRF6821_FRAC2_H, "FRAC2_H"},
	{ADRF6821_MULTI_FUNC_SYNTH_CTRL_0235, "MULTI_FUNC_SYNTH_CTRL_0235"},
	{ADRF6821_VCO_LUT_CTRL, "VCO_LUT_CTRL"},
	{ADRF6821_LOCK_DETECT, "LOCK_DETECT"},
	{ADRF6821_MULTI_FUNC_CTRL, "MULTI_FUNC_CTRL"},
	{ADRF6821_LO_CNTRL2, "LO_CNTRL2"},
	{ADRF6821_LO_CNTRL8, "LO_CNTRL8"},
	{ADRF6821_FRAC2_L_SLAVE, "FRAC2_L_SLAVE"},
	{ADRF6821_FRAC2_H_SLAVE, "FRAC2_H_SLAVE"},
	{ADRF6821_FRAC_L_SLAVE, "FRAC_L_SLAVE"},
	{ADRF6821_FRAC_M_SLAVE, "FRAC_M_SLAVE"},
	{ADRF6821_FRAC_H_SLAVE2, "FRAC_H_SLAVE2"},
	{ADRF6821_PHASE_L_SLAVE, "PHASE_L_SLAVE"},
	{ADRF6821_PHASE_M_SLAVE2, "PHASE_M_SLAVE2"},
	{ADRF6821_PHASE_H_SLAVE3, "PHASE_H_SLAVE3"},
	{ADRF6821_INT_DIV_L_SLAVE, "INT_DIV_L_SLAVE"},
	{ADRF6821_INT_DIV_H_SLAVE, "INT_DIV_H_SLAVE"},
	{ADRF6821_R_DIV_SLAVE, "R_DIV_SLAVE"},
	{ADRF6821_RDIV2_SEL_SLAVE, "RDIV2_SEL_SLAVE"},
	{ADRF6821_DISABLE_CFG, "DISABLE_CFG"}
};


static const unsigned int writeorder_all[WRITE_LENGTH_ALL] = {
9, // MASTER_CONFIG
10, // RF_SWITCH
11, // DSA_CONTROL
12, // DEMOD_ENABLES
13, // DEMOD_LO_COM_CTRL
14, // DEMOD_OUT_COM_CTRL
15, // DEMOD_SPARES
16, // DEMOD_DRIVER_COM_CTRL
17, // DC_CTRL
18, // DC_COMP_I_CHAN_RF0
19, // DC_COMP_Q_CHAN_RF0
20, // DC_COMP_I_CHAN_RF1
21, // DC_COMP_Q_CHAN_RF1
22, // LPF_BW_SEL
23, // IF_AMP_CTRL
24, // LO_CTRL
25, // EN_LO_DIVIDER_CTRL
26, // LO_PHASE_ADJ
27, // BLOCK_RESETS
28, // SIG_PATH_9_NORMAL
30, // INT_H
31, // FRAC1_L
32, // FRAC1_M
33, // FRAC1_H
34, // SD_PHASE_L_0
35, // SD_PHASE_M_0
36, // SD_PHASE_H_0
37, // MOD_L
38, // MOD_H
39, // SYNTH
40, // R_DIV
41, // SYNTH_0
42, // MULTI_FUNC_SYNTH_CTRL_0214
43, // SI_BAND_0
44, // SI_VCO_SEL
//45, // VCO_TIMEOUT_L
//46, // VCO_TIMEOUT_H
//47, // VCO_BAND_DIV
48, // VCO_FSM
49, // SD_CTRL
50, // MULTI_FUNC_SYNTH_CTRL_022C
51, // MULTI_FUNC_SYNTH_CTRL_022D
52, // CP_CURR
53, // BICP
54, // FRAC2_L
55, // FRAC2_H
56, // MULTI_FUNC_SYNTH_CTRL_0235
57, // VCO_LUT_CTRL
59, // MULTI_FUNC_CTRL
//60, // LO_CNTRL2
61, // LO_CNTRL8
74, // DISABLE_CFG
29, // INT_L
5, // SCRATCHPAD
};

static const unsigned int writeorder_pll[WRITE_LENGTH_PLL] = {
61, // LO_CNTRL8
30, // INT_H
31, // FRAC1_L
32, // FRAC1_M
33, // FRAC1_H
37, // MOD_L
38, // MOD_H
54, // FRAC2_L
55, // FRAC2_H
29, // INT_L
5, // SCRATCHPAD
};

static const unsigned int writeorder_notpll[WRITE_LENGTH_NOTPLL] = {
9, // MASTER_CONFIG
10, // RF_SWITCH
11, // DSA_CONTROL
12, // DEMOD_ENABLES
13, // DEMOD_LO_COM_CTRL
14, // DEMOD_OUT_COM_CTRL
15, // DEMOD_SPARES
16, // DEMOD_DRIVER_COM_CTRL
17, // DC_CTRL
18, // DC_COMP_I_CHAN_RF0
19, // DC_COMP_Q_CHAN_RF0
20, // DC_COMP_I_CHAN_RF1
21, // DC_COMP_Q_CHAN_RF1
22, // LPF_BW_SEL
23, // IF_AMP_CTRL
24, // LO_CTRL
25, // EN_LO_DIVIDER_CTRL
26, // LO_PHASE_ADJ
27, // BLOCK_RESETS
28, // SIG_PATH_9_NORMAL
42, // MULTI_FUNC_SYNTH_CTRL_0214
59, // MULTI_FUNC_CTRL
//60, // LO_CNTRL2
61, // LO_CNTRL8
74, // DISABLE_CFG
};


struct adrf6821_phy_platform_data {
	int dummy;
};

struct adrf6821_rf_phy_state {
	    struct adrf6821_rf_phy *phy;
	    u8 ARSTB_BLOCK_AUTOCAL;
	    u8 ARSTB_BLOCK_DSMALL;
	    u8 ARSTB_BLOCK_DSMCORE;
	    u8 ARSTB_BLOCK_DSMOSTG;
	    u8 ARSTB_BLOCK_LKD;
	    u8 ARSTB_BLOCK_NDIV;
	    u8 ARSTB_BLOCK_RDIV;
	    u8 ATTEN_DSA;
	    u8 BICP;
	    u8 BLEED_EN;
	    u8 BLEED_POL;
	    u8 CODE_DC_IDAC_RF0;
	    u8 CODE_DC_IDAC_RF1;
	    u8 CODE_DC_QDAC_RF0;
	    u8 CODE_DC_QDAC_RF1;
	    u8 CODE_MIXER_DRVR;
	    u8 CODE_MIXER_OCM;
	    u8 CP_CURRENT;
	    u8 CP_HIZ;
	    u8 CSB_STALL;
	    u8 DISABLE_CAL;
	    u8 DISABLE_DBLBUFFERING;
	    u8 DISABLE_FREQHOP;
	    u8 DISABLE_PHASEADJ;
	    u8 DOUBLER_EN;
	    u8 DSM_LAUNCH_DLY;
	    u8 ENB_DCCOMP_1P8_GEN;
	    u8 ENB_DSA_1P8_GEN;
	    u8 ENB_MIX_1P8_GEN;
	    u8 ENB_SW_1P8_GEN;
	    u8 ENDIAN;
	    u8 ENDIAN_;
	    u8 EN_ANALOG_MASTER;
	    u8 EN_BIAS_R;
	    u8 EN_DC_DAC_I;
	    u8 EN_DC_DAC_Q;
	    u8 EN_DIVPATH_BUF;
	    u8 EN_DIVPATH_QUADDIV;
	    u8 EN_DSA;
	    u8 EN_FBDIV;
	    u8 EN_IBIASGEN;
	    u8 EN_ICMLOBIAS_I;
	    u8 EN_ICMLOBIAS_Q;
	    u8 EN_ICMOBIAS_I;
	    u8 EN_ICMOBIAS_Q;
	    u8 EN_IFAMP_I;
	    u8 EN_IFAMP_Q;
	    u8 EN_IMXBIAS_I;
	    u8 EN_IMXBIAS_Q;
	    u8 EN_LODRV_DRVI;
	    u8 EN_LODRV_DRVQ;
	    u8 EN_LODRV_PREDRVI;
	    u8 EN_LODRV_PREDRVQ;
	    u8 EN_MIXIBIASGEN;
	    u8 EN_MIX_I;
	    u8 EN_MIX_Q;
	    u8 EN_PFD_CP;
	    u8 EN_SW;
	    u32 FRAC;
	    u32 FRAC2;
	    u8 GPO1_BLK_SEL;
	    u8 GPO1_ENABLE;
	    u8 GPO1_SGNL_SEL;
	    u8 INT_ABP;
	    u32 INT_DIV;
	    u8 I_MIXER_GAIN_ADJ;
	    u8 LDP;
	    u8 LD_BIAS;
	    u8 LD_HIZ;
	    u8 LO_OE;
	    u8 LSB_FIRST;
	    u8 LSB_FIRST_;
	    u8 MASTER_SLAVE_RB;
	    u8 MASTER_SLAVE_TRANSFER;
	    u8 MIXER_GAIN_PEAK;
	    u8 MIX_OE;
	    u32 MOD2;
	    u8 OUT_DIVRATIO;
	    u32 PHASE;
	    u8 PHASE_ADJ_EN;
	    u8 PRE_SEL;
	    u8 Q_MIXER_GAIN_ADJ;
	    u8 RDIV2_SEL;
	    u8 REFBUF_EN;
	    u8 RFSW_SEL0;
	    u8 RFSW_SEL1;
	    u32 R_DIV;
	    u8 SCRATCHPAD;
	    u8 SDOACTIVE;
	    u8 SDOACTIVE_;
	    u8 SD_EN_FRAC0;
	    u8 SD_EN_OUT_OFF;
	    u8 SD_SM_2;
	    u8 SEL_IDAC_RANGE;
	    u8 SEL_LODRV_PREDRVI_POL;
	    u8 SEL_LODRV_PREDRVQ_POL;
	    u8 SEL_LPF_BW_LSB;
	    u8 SEL_LPF_BW_MSB;
	    u8 SEL_QDAC_RANGE;
	    u8 SEL_RFSW_SPI_CONTROL;
	    u8 SINGLE_INSTRUCTION;
	    u8 SI_BAND_SEL;
	    u8 SI_VCO_FORCE_CAPS;
	    u8 SI_VCO_FORCE_CAPSVCOI;
	    u8 SI_VCO_FORCE_VCO;
	    u8 SI_VCO_SEL;
	    u8 SOFTRESET;
	    u8 SOFTRESET_;
	    u8 SOFT_RESET;
	    u8 SPI_18_33_SEL;
	    u8 SPI_1P8_3P3_CTRL;
	    u8 TRM_LODRV_CAPI;
	    u8 TRM_LODRV_CAPQ;
	    u8 TRM_MIXLODRV_DRV_POUT;
	    u8 TRM_XLODRV_DRV_POUT;
	    u8 USEEXT_LOI;
	    u32 pll_ref_in_Hz;
	    u8 pll_ref_div;
	    u32 pfd_freq_Hz;
	    u64 lo_double_freq_Hz;
	    u64 vco_freq_Hz;
	    u32 tune_freq_Hz;
};


struct adrf6821_rf_phy {
	struct spi_device 	*spi;
	struct iio_dev 		*indio_dev;
	struct work_struct 	work;
	struct completion       complete;
	struct adrf6821_rf_phy_state	*state;
	struct adrf6821_phy_platform_data *pdata;
};

enum lo_ext_info {
	LOEXT_FREQ,
	LOEXT_FREQ_AVAILABLE,
};

void update_shadowregs(struct adrf6821_rf_phy_state *st, u8 *shadowregs);

int adrf6821_spi_write(struct spi_device *spi, u16 reg, u8 val);
int adrf6821_spi_read(struct spi_device *spi, u16 reg);
static int __adrf6821_of_get_u32(struct iio_dev *indio_dev,
			     struct device_node *np, const char *propname,
			     u32 defval, void *out_value, u32 size);

#define adrf6821_of_get_u32(iodev, dnp, name, def, outp) \
	__adrf6821_of_get_u32(iodev, dnp, name, def, outp, sizeof(*outp))

static void adrf6821_of_get_bool(struct iio_dev *indio_dev, struct device_node *np,
			       const char *propname, bool *out_value);

static struct adrf6821_phy_platform_data
	*adrf6821_phy_parse_dt(struct iio_dev *iodev, struct device *dev);

struct adrf6821_rf_phy* adrf6821_spi_to_phy(struct spi_device *spi);

static void adrf6821_init_state(struct adrf6821_rf_phy *phy);
static int adrf6821_phy_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m);
static int adrf6821_phy_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask);

static int adrf6821_phy_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask);
static ssize_t adrf6821_phy_lo_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len);

static ssize_t adrf6821_phy_lo_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf);
static int adrf6821_setup(struct adrf6821_rf_phy *phy);
static int adrf6821_probe(struct spi_device *spi);
static int adrf6821_remove(struct spi_device *spi);

#endif
