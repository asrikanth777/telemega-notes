/*
 * Copyright © 2012 Keith Packard <keithp@keithp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#ifndef _AO_CC1120_H_
#define _AO_CC1120_H_

#define CC1120_READ	(7)
#define CC1120_BURST	(6)

/* Register space */
#define CC1120_IOCFG3		0x00
#define  CC1120_IOCFG_GPIO_ATRAN	7
#define  CC1120_IOCFG_GPIO_INV		6
#define  CC1120_IOCFG_GPIO_CFG		0
#define  CC1120_IOCFG_GPIO_CFG_RXFIFO_THR	0
#define  CC1120_IOCFG_GPIO_CFG_RXFIFO_THR_PKT	1	
#define  CC1120_IOCFG_GPIO_CFG_TXFIFO_THR	2
#define  CC1120_IOCFG_GPIO_CFG_TXFIFO_THR_PKT	3
#define  CC1120_IOCFG_GPIO_CFG_RXFIFO_OVERFLOW	4
#define  CC1120_IOCFG_GPIO_CFG_TXFIFO_UNDERFLOW	5
#define  CC1120_IOCFG_GPIO_CFG_PKT_SYNC_RXTX	6
#define  CC1120_IOCFG_GPIO_CFG_CRC_OK		7
#define  CC1120_IOCFG_GPIO_CFG_SERIAL_CLK	8
#define  CC1120_IOCFG_GPIO_CFG_SERIAL_RX	9
#define  CC1120_IOCFG_GPIO_CFG_PQT_REACHED	11
#define  CC1120_IOCFG_GPIO_CFG_PQT_VALID	12
#define  CC1120_IOCFG_GPIO_CFG_RSSI_VALID	13
#define  CC1120_IOCFG_GPIO3_CFG_RSSI_UPDATE	14
#define  CC1120_IOCFG_GPIO2_CFG_RSSI_UPDATE	14
#define  CC1120_IOCFG_GPIO1_CFG_AGC_HOLD	14
#define  CC1120_IOCFG_GPIO0_CFG_AGC_UPDATE	14
#define  CC1120_IOCFG_GPIO3_CFG_CGA_STATUS	15
#define  CC1120_IOCFG_GPIO2_CFG_TXONCCA_DONE	15
#define  CC1120_IOCFG_GPIO1_CFG_CCA_STATUS	15
#define  CC1120_IOCFG_GPIO0_CFG_TXONCCA_FAILED	15
#define  CC1120_IOCFG_GPIO_CFG_CARRIER_SENSE_VALID	16
#define  CC1120_IOCFG_GPIO_CFG_CARRIER_SENSE	17
#define  CC1120_IOCFG_GPIO3_CFG_DSSS_CLK	18
#define  CC1120_IOCFG_GPIO2_CFG_DSSS_DATA0	18
#define  CC1120_IOCFG_GPIO1_CFG_DSSS_CLK	18
#define  CC1120_IOCFG_GPIO0_CFG_DSSS_DATA1	18
#define  CC1120_IOCFG_GPIO_CFG_PKT_CRC_OK	19
#define  CC1120_IOCFG_GPIO_CFG_MARC_MCU_WAKEUP	20
#define  CC1120_IOCFG_GPIO_CFG_SYNC_LOW0_HIGH1	21
#define  CC1120_IOCFG_GPIO_CFG_LNA_PA_REG_PD	23
#define  CC1120_IOCFG_GPIO_CFG_LNA_PD		24
#define  CC1120_IOCFG_GPIO_CFG_PA_RD		25
#define  CC1120_IOCFG_GPIO_CFG_RX0TX1_CFG	26
#define  CC1120_IOCFG_GPIO_CFG_IMAGE_FOUND	28
#define  CC1120_IOCFG_GPIO_CFG_CLKEN_SOFT	29
#define  CC1120_IOCFG_GPIO_CFG_SOFT_TX_DATA_CLK	30
#define  CC1120_IOCFG_GPIO_CFG_RSSI_STEP_FOUND	33
#define  CC1120_IOCFG_GPIO_CFG_RSSI_STEP_EVENT	34
#define  CC1120_IOCFG_GPIO_CFG_ANTENNA_SELECT	36
#define  CC1120_IOCFG_GPIO_CFG_MARC_2PIN_STATUS1	37
#define  CC1120_IOCFG_GPIO_CFG_MARC_2PIN_STATUS0	38
#define  CC1120_IOCFG_GPIO2_CFG_TXFIFO_OVERFLOW		39
#define  CC1120_IOCFG_GPIO0_CFG_RXFIFO_UNDERFLOW	39
#define  CC1120_IOCFG_GPIO3_CFG_MAGN_VALID	40
#define  CC1120_IOCFG_GPIO2_CFG_CHFILT_VALID	40
#define  CC1120_IOCFG_GPIO1_CFG_RCC_CAL_VALID	40
#define  CC1120_IOCFG_GPIO0_CFG_CHFILTER_STARTUP_VALID	40
#define  CC1120_IOCFG_GPIO3_CFG_COLLISION_FOUND		41
#define  CC1120_IOCFG_GPIO2_CFG_SYNC_EVENT		41
#define  CC1120_IOCFG_GPIO1_CFG_COLLISION_FOUND		41
#define  CC1120_IOCFG_GPIO0_CFG_COLLISION_EVENT		41
#define  CC1120_IOCFG_GPIO_CFG_PA_RAMP_UP		42
#define  CC1120_IOCFG_GPIO3_CFG_CRC_FAILED		43
#define  CC1120_IOCFG_GPIO2_CFG_LENGTH_FAILED		43
#define  CC1120_IOCFG_GPIO1_CFG_ADDR_FAILED		43
#define  CC1120_IOCFG_GPIO0_CFG_UART_FRAMING_ERROR	43
#define  CC1120_IOCFG_GPIO_CFG_AGC_STABLE_GAIN		44
#define  CC1120_IOCFG_GPIO_CFG_AGC_UPDATE		45
#define  CC1120_IOCFG_GPIO3_CFG_ADC_CLOCK		46
#define  CC1120_IOCFG_GPIO2_CFG_ADC_Q_DATA_SAMPLE	46
#define  CC1120_IOCFG_GPIO1_CFG_ADC_CLOCK		46
#define  CC1120_IOCFG_GPIO0_CFG_ADC_I_DATA_SAMPLE	46
#define  CC1120_IOCFG_GPIO_CFG_HIGHZ			48
#define  CC1120_IOCFG_GPIO_CFG_EXT_CLOCK		49
#define  CC1120_IOCFG_GPIO_CFG_CHIP_RDY			50
#define  CC1120_IOCFG_GPIO_CFG_HW0			51
#define  CC1120_IOCFG_GPIO_CFG_CLOCK_32K		54
#define  CC1120_IOCFG_GPIO_CFG_WOR_EVENT0		55
#define  CC1120_IOCFG_GPIO_CFG_WOR_EVENT1		56
#define  CC1120_IOCFG_GPIO_CFG_WOR_EVENT2		57
#define  CC1120_IOCFG_GPIO_CFG_XOSC_STABLE		59
#define  CC1120_IOCFG_GPIO_CFG_EXT_OSC_EN		60
#define  CC1120_IOCFG_GPIO_CFG_MASK	0x3f

#define CC1120_IOCFG3		0x00
#define CC1120_IOCFG2		0x01
#define CC1120_IOCFG1		0x02
#define CC1120_IOCFG0		0x03
#define CC1120_SYNC3		0x04
#define CC1120_SYNC2		0x05
#define CC1120_SYNC1		0x06
#define CC1120_SYNC0		0x07
#define CC1120_SYNC_CFG1	0x08
#define  CC1120_SYNC_CFG1_DEM_CFG	5
#define  CC1120_SYNC_CFG1_DEM_CFG_PQT_GATING_DISABLED	0
#define  CC1120_SYNC_CFG1_DEM_CFG_PQT_GATING_ENABLED	2
#define  CC1120_SYNC_CFG1_DEM_CFG_MASK			0x7

#define  CC1120_SYNC_CFG1_SYNC_THR	0
#define  CC1120_SYNC_CFG1_SYNC_MASK			0x1f

#define CC1120_SYNC_CFG0	0x09
#define  CC1120_SYNC_CFG0_SYNC_MODE	2
#define  CC1120_SYNC_CFG0_SYNC_MODE_NONE		0
#define  CC1120_SYNC_CFG0_SYNC_MODE_11_BITS		1
#define  CC1120_SYNC_CFG0_SYNC_MODE_16_BITS		2
#define  CC1120_SYNC_CFG0_SYNC_MODE_18_BITS		3
#define  CC1120_SYNC_CFG0_SYNC_MODE_24_BITS		4
#define  CC1120_SYNC_CFG0_SYNC_MODE_32_BITS		5
#define  CC1120_SYNC_CFG0_SYNC_MODE_16H_BITS		6
#define  CC1120_SYNC_CFG0_SYNC_MODE_16D_BITS		7
#define  CC1120_SYNC_CFG0_SYNC_MODE_MASK		7
#define  CC1120_SYNC_CFG0_SYNC_NUM_ERROR	0
#define  CC1120_SYNC_CFG0_SYNC_NUM_ERROR_0		0
#define  CC1120_SYNC_CFG0_SYNC_NUM_ERROR_2		1
#define  CC1120_SYNC_CFG0_SYNC_NUM_ERROR_DISABLED	3
#define  CC1120_SYNC_CFG0_SYNC_NUM_ERROR_MASK		3

#define CC1120_DEVIATION_M	0x0a
#define CC1120_MODCFG_DEV_E	0x0b
#define CC1120_MODCFG_DEV_E_MODEM_MODE		6
#define  CC1120_MODCFG_DEV_E_MODEM_MODE_NORMAL		0
#define  CC1120_MODCFG_DEV_E_MODEM_MODE_DSSS_REPEAT	1
#define  CC1120_MODCFG_DEV_E_MODEM_MODE_DSSS_PN		2
#define  CC1120_MODCFG_DEV_E_MODEM_MODE_MASK		3
#define CC1120_MODCFG_DEV_E_MOD_FORMAT		3
#define CC1120_MODCFG_DEV_E_MOD_FORMAT_2_FSK		0
#define CC1120_MODCFG_DEV_E_MOD_FORMAT_2_GFSK		1
#define CC1120_MODCFG_DEV_E_MOD_FORMAT_ASK_OOK		3
#define CC1120_MODCFG_DEV_E_MOD_FORMAT_4_FSK		4
#define CC1120_MODCFG_DEV_E_MOD_FORMAT_4_GFSK		5
#define CC1120_MODCFG_DEV_E_MOD_FORMAT_SC_MSK_UNSHAPED	6
#define CC1120_MODCFG_DEV_E_MOD_FORMAT_SC_MSK_SHAPED	7
#define CC1120_MODCFG_DEV_E_MOD_FORMAT_MASK		7
#define CC1120_MODCFG_DEV_E_DEV_E		0
#define CC1120_MODCFG_DEV_E_DEV_E_MASK		7

#define CC1120_DCFILT_CFG	0x0c
#define CC1120_PREAMBLE_CFG1	0x0d
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE	2
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_NONE		0
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_0_5_BYTE	1
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_1_BYTE	2
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_1_5_BYTE	3
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_2_BYTES	4
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_3_BYTES	5
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_4_BYTES	6
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_5_BYTES	7
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_6_BYTES	8
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_7_BYTES	9
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_8_BYTES	10
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_12_BYTES	11
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_24_BYTES	12
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_30_BYTES	13
#define  CC1120_PREAMBLE_CFG1_NUM_PREAMBLE_MASK		0xf

#define  CC1120_PREAMBLE_CFG1_PREAMBLE_WORD	0
#define  CC1120_PREAMBLE_CFG1_PREAMBLE_WORD_AA		0
#define  CC1120_PREAMBLE_CFG1_PREAMBLE_WORD_55		1
#define  CC1120_PREAMBLE_CFG1_PREAMBLE_WORD_33		2
#define  CC1120_PREAMBLE_CFG1_PREAMBLE_WORD_CC		3
#define  CC1120_PREAMBLE_CFG1_PREAMBLE_WORD_MASK	3

#define CC1120_PREAMBLE_CFG0	0x0e
#define CC1120_FREQ_IF_CFG	0x0f
#define CC1120_IQIC		0x10
#define CC1120_CHAN_BW		0x11
#define CC1120_MDMCFG1		0x12
#define  CC1120_MDMCFG1_CARRIER_SENSE_GATE	7
#define  CC1120_MDMCFG1_FIFO_EN			6
#define  CC1120_MDMCFG1_MANCHESTER_EN		5
#define  CC1120_MDMCFG1_INVERT_DATA_EN		4
#define  CC1120_MDMCFG1_COLLISION_DETECT_EN	3
#define  CC1120_MDMCFG1_DVGA_GAIN		1
#define  CC1120_MDMCFG1_DVGA_GAIN_0			0
#define  CC1120_MDMCFG1_DVGA_GAIN_3			1
#define  CC1120_MDMCFG1_DVGA_GAIN_6			2
#define  CC1120_MDMCFG1_DVGA_GAIN_9			3
#define  CC1120_MDMCFG1_DVGA_GAIN_MASK			3
#define  CC1120_MDMCFG1_SINGLE_ADC_EN		0

#define CC1120_MDMCFG0		0x13
#define CC1120_DRATE2		0x14
#define CC1120_DRATE2_DATARATE_E		4
#define CC1120_DRATE2_DATARATE_E_MASK		0xf
#define CC1120_DRATE2_DATARATE_M_19_16		0
#define CC1120_DRATE2_DATARATE_M_19_16_MASK	0xf

#define CC1120_DRATE1		0x15
#define CC1120_DRATE0		0x16
#define CC1120_AGC_REF		0x17
#define CC1120_AGC_CS_THR	0x18
#define CC1120_AGC_GAIN_ADJUST	0x19
#define CC1120_AGC_CFG3		0x1a
#define CC1120_AGC_CFG2		0x1b
#define CC1120_AGC_CFG1		0x1c
#define CC1120_AGC_CFG0		0x1d
#define CC1120_FIFO_CFG		0x1e
#define CC1120_DEV_ADDR		0x1f
#define CC1120_SETTLING_CFG	0x20
#define  CC1120_SETTLING_CFG_FS_AUTOCAL		3
#define  CC1120_SETTLING_CFG_FS_AUTOCAL_NEVER		0
#define  CC1120_SETTLING_CFG_FS_AUTOCAL_IDLE_TO_ON	1
#define  CC1120_SETTLING_CFG_FS_AUTOCAL_ON_TO_IDLE	2
#define  CC1120_SETTLING_CFG_FS_AUTOCAL_EVERY_4TH_TIME	3
#define  CC1120_SETTLING_CFG_FS_AUTOCAL_MASK		3
#define  CC1120_SETTLING_CFG_LOCK_TIME		1
#define  CC1120_SETTLING_CFG_LOCK_TIME_50_20		0
#define  CC1120_SETTLING_CFG_LOCK_TIME_70_30		1
#define  CC1120_SETTLING_CFG_LOCK_TIME_100_40		2
#define  CC1120_SETTLING_CFG_LOCK_TIME_150_60		3
#define  CC1120_SETTLING_CFG_LOCK_TIME_MASK		3
#define  CC1120_SETTLING_CFG_FSREG_TIME		0
#define  CC1120_SETTLING_CFG_FSREG_TIME_30		0
#define  CC1120_SETTLING_CFG_FSREG_TIME_60		1
#define  CC1120_SETTLING_CFG_FSREG_TIME_MASK		1

#define CC1120_FS_CFG		0x21
#define  CC1120_FS_CFG_LOCK_EN			4
#define  CC1120_FS_CFG_FSD_BANDSELECT		0
#define  CC1120_FS_CFG_FSD_BANDSELECT_820_960		2
#define  CC1120_FS_CFG_FSD_BANDSELECT_410_480		4
#define  CC1120_FS_CFG_FSD_BANDSELECT_273_320		6
#define  CC1120_FS_CFG_FSD_BANDSELECT_205_240		8
#define  CC1120_FS_CFG_FSD_BANDSELECT_164_192		10
#define  CC1120_FS_CFG_FSD_BANDSELECT_136_160		11
#define  CC1120_FS_CFG_FSD_BANDSELECT_MASK		0xf

#define CC1120_WOR_CFG1		0x22
#define CC1120_WOR_CFG0		0x23
#define CC1120_WOR_EVENT0_MSB	0x24
#define CC1120_WOR_EVENT0_LSB	0x25
#define CC1120_PKT_CFG2		0x26
#define  CC1120_PKT_CFG2_CCA_MODE	2
#define  CC1120_PKT_CFG2_CCA_MODE_ALWAYS_CLEAR		0
#define  CC1120_PKT_CFG2_CCA_MODE_RSSI_THRESHOLD	1
#define  CC1120_PKT_CFG2_CCA_MODE_NOT_RECEIVING		2
#define  CC1120_PKT_CFG2_CCA_MODE_RSSI_OR_NOT		3
#define  CC1120_PKT_CFG2_CCA_MODE_RSSI_AND_ETSI_LBT	4
#define  CC1120_PKT_CFG2_CCA_MODE_MASK			7
#define  CC1120_PKT_CFG2_PKT_FORMAT	0
#define  CC1120_PKT_CFG2_PKT_FORMAT_NORMAL		0
#define  CC1120_PKT_CFG2_PKT_FORMAT_SYNCHRONOUS_SERIAL	1
#define  CC1120_PKT_CFG2_PKT_FORMAT_RANDOM		2
#define  CC1120_PKT_CFG2_PKT_FORMAT_TRANSPARENT_SERIAL	3
#define  CC1120_PKT_CFG2_PKT_FORMAT_MASK		3

#define CC1120_PKT_CFG1		0x27
#define  CC1120_PKT_CFG1_WHITE_DATA	6
#define  CC1120_PKT_CFG1_ADDR_CHECK_CFG	4
#define  CC1120_PKT_CFG1_ADDR_CHECK_CFG_NONE		0
#define  CC1120_PKT_CFG1_ADDR_CHECK_CFG_CHECK		1
#define  CC1120_PKT_CFG1_ADDR_CHECK_CFG_00_BROADCAST	2
#define  CC1120_PKT_CFG1_ADDR_CHECK_CFG_00_FF_BROADCAST	3
#define  CC1120_PKT_CFG1_ADDR_CHECK_CFG_MASK		3
#define  CC1120_PKT_CFG1_CRC_CFG	2
#define  CC1120_PKT_CFG1_CRC_CFG_DISABLED		0
#define  CC1120_PKT_CFG1_CRC_CFG_CRC16_INIT_ONES	1
#define  CC1120_PKT_CFG1_CRC_CFG_CRC16_INIT_ZEROS	2
#define  CC1120_PKT_CFG1_CRC_CFG_MASK			3
#define  CC1120_PKT_CFG1_BYTE_SWAP_EN	1
#define  CC1120_PKT_CFG1_APPEND_STATUS	0

#define CC1120_PKT_CFG0		0x28
#define  CC1120_PKT_CFG0_RESERVED7	7
#define  CC1120_PKT_CFG0_LENGTH_CONFIG	5
#define  CC1120_PKT_CFG0_LENGTH_CONFIG_FIXED		0
#define  CC1120_PKT_CFG0_LENGTH_CONFIG_VARIABLE		1
#define  CC1120_PKT_CFG0_LENGTH_CONFIG_INFINITE		2
#define  CC1120_PKT_CFG0_LENGTH_CONFIG_VARIABLE_5LSB	3
#define  CC1120_PKT_CFG0_LENGTH_CONFIG_MASK		3
#define  CC1120_PKT_CFG0_PKG_BIT_LEN	2
#define  CC1120_PKT_CFG0_PKG_BIT_LEN_MASK	7
#define  CC1120_PKT_CFG0_UART_MODE_EN	1
#define  CC1120_PKT_CFG0_UART_SWAP_EN	0

#define CC1120_RFEND_CFG1	0x29
#define CC1120_RFEND_CFG0	0x2a
#define CC1120_PA_CFG2		0x2b
#define CC1120_PA_CFG1		0x2c
#define CC1120_PA_CFG0		0x2d
#define CC1120_PKT_LEN		0x2e

#define CC1120_EXTENDED	0x2f

/* Command strobes */
#define CC1120_SRES		0x30
#define CC1120_SFSTXON		0x31
#define CC1120_SXOFF		0x32
#define CC1120_SCAL		0x33
#define CC1120_SRX		0x34
#define CC1120_STX		0x35
#define CC1120_SIDLE		0x36
#define CC1120_SAFC		0x37
#define CC1120_SWOR		0x38
#define CC1120_SPWD		0x39
#define CC1120_SFRX		0x3a
#define CC1120_SFTX		0x3b
#define CC1120_SWORRST		0x3c
#define CC1120_SNOP		0x3d

#define CC1120_DIRECT_FIFO	0x3e
#define CC1120_FIFO		0x3f

/* Extended register space */

#define CC1120_EXTENDED_BIT	0x8000

#define CC1120_IS_EXTENDED(r)	((r) & CC1120_EXTENDED_BIT)

#define CC1120_IF_MIX_CFG	(CC1120_EXTENDED_BIT | 0x00)
#define CC1120_FREQOFF_CFG	(CC1120_EXTENDED_BIT | 0x01)
#define CC1120_TOC_CFG		(CC1120_EXTENDED_BIT | 0x02)
#define CC1120_MARC_SPARE	(CC1120_EXTENDED_BIT | 0x03)
#define CC1120_ECG_CFG		(CC1120_EXTENDED_BIT | 0x04)
#define CC1120_SOFT_TX_DATA_CFG	(CC1120_EXTENDED_BIT | 0x05)
#define CC1120_EXT_CTRL		(CC1120_EXTENDED_BIT | 0x06)
#define CC1120_RCCAL_FINE	(CC1120_EXTENDED_BIT | 0x07)
#define CC1120_RCCAL_COARSE	(CC1120_EXTENDED_BIT | 0x08)
#define CC1120_RCCAL_OFFSET	(CC1120_EXTENDED_BIT | 0x09)
#define CC1120_FREQOFF1		(CC1120_EXTENDED_BIT | 0x0A)
#define CC1120_FREQOFF0		(CC1120_EXTENDED_BIT | 0x0B)
#define CC1120_FREQ2		(CC1120_EXTENDED_BIT | 0x0C)
#define CC1120_FREQ1		(CC1120_EXTENDED_BIT | 0x0D)
#define CC1120_FREQ0		(CC1120_EXTENDED_BIT | 0x0E)
#define CC1120_IF_ADC2		(CC1120_EXTENDED_BIT | 0x0F)
#define CC1120_IF_ADC1		(CC1120_EXTENDED_BIT | 0x10)
#define CC1120_IF_ADC0		(CC1120_EXTENDED_BIT | 0x11)
#define CC1120_FS_DIG1		(CC1120_EXTENDED_BIT | 0x12)
#define CC1120_FS_DIG0		(CC1120_EXTENDED_BIT | 0x13)
#define CC1120_FS_CAL3		(CC1120_EXTENDED_BIT | 0x14)
#define CC1120_FS_CAL2		(CC1120_EXTENDED_BIT | 0x15)
#define CC1120_FS_CAL1		(CC1120_EXTENDED_BIT | 0x16)
#define CC1120_FS_CAL0		(CC1120_EXTENDED_BIT | 0x17)
#define CC1120_FS_CHP		(CC1120_EXTENDED_BIT | 0x18)
#define CC1120_FS_DIVTWO	(CC1120_EXTENDED_BIT | 0x19)
#define CC1120_FS_DSM1		(CC1120_EXTENDED_BIT | 0x1A)
#define CC1120_FS_DSM0		(CC1120_EXTENDED_BIT | 0x1B)
#define CC1120_FS_DVC1		(CC1120_EXTENDED_BIT | 0x1C)
#define CC1120_FS_DVC0		(CC1120_EXTENDED_BIT | 0x1D)
#define CC1120_FS_LBI		(CC1120_EXTENDED_BIT | 0x1E)
#define CC1120_FS_PFD		(CC1120_EXTENDED_BIT | 0x1F)
#define CC1120_FS_PRE		(CC1120_EXTENDED_BIT | 0x20)
#define CC1120_FS_REG_DIV_CML	(CC1120_EXTENDED_BIT | 0x21)
#define CC1120_FS_SPARE		(CC1120_EXTENDED_BIT | 0x22)
#define CC1120_FS_VCO4		(CC1120_EXTENDED_BIT | 0x23)
#define CC1120_FS_VCO3		(CC1120_EXTENDED_BIT | 0x24)
#define CC1120_FS_VCO2		(CC1120_EXTENDED_BIT | 0x25)
#define CC1120_FS_VCO1		(CC1120_EXTENDED_BIT | 0x26)
#define CC1120_FS_VCO0		(CC1120_EXTENDED_BIT | 0x27)
#define CC1120_GBIAS6		(CC1120_EXTENDED_BIT | 0x28)
#define CC1120_GBIAS5		(CC1120_EXTENDED_BIT | 0x29)
#define CC1120_GBIAS4		(CC1120_EXTENDED_BIT | 0x2A)
#define CC1120_GBIAS3		(CC1120_EXTENDED_BIT | 0x2B)
#define CC1120_GBIAS2		(CC1120_EXTENDED_BIT | 0x2C)
#define CC1120_GBIAS1		(CC1120_EXTENDED_BIT | 0x2D)
#define CC1120_GBIAS0		(CC1120_EXTENDED_BIT | 0x2E)
#define CC1120_IFAMP		(CC1120_EXTENDED_BIT | 0x2F)
#define CC1120_LNA		(CC1120_EXTENDED_BIT | 0x30)
#define CC1120_RXMIX		(CC1120_EXTENDED_BIT | 0x31)
#define CC1120_XOSC5		(CC1120_EXTENDED_BIT | 0x32)
#define CC1120_XOSC4		(CC1120_EXTENDED_BIT | 0x33)
#define CC1120_XOSC3		(CC1120_EXTENDED_BIT | 0x34)
#define CC1120_XOSC2		(CC1120_EXTENDED_BIT | 0x35)
#define CC1120_XOSC1		(CC1120_EXTENDED_BIT | 0x36)
#define CC1120_XOSC0		(CC1120_EXTENDED_BIT | 0x37)
#define CC1120_ANALOG_SPARE	(CC1120_EXTENDED_BIT | 0x38)
#define CC1120_PA_CFG3		(CC1120_EXTENDED_BIT | 0x39)
#define CC1120_WOR_TIME1	(CC1120_EXTENDED_BIT | 0x64)
#define CC1120_WOR_TIME0	(CC1120_EXTENDED_BIT | 0x65)
#define CC1120_WOR_CAPTURE1	(CC1120_EXTENDED_BIT | 0x66)
#define CC1120_WOR_CAPTURE0	(CC1120_EXTENDED_BIT | 0x67)
#define CC1120_BIST		(CC1120_EXTENDED_BIT | 0x68)
#define CC1120_DCFILTOFFSET_I1	(CC1120_EXTENDED_BIT | 0x69)
#define CC1120_DCFILTOFFSET_I0	(CC1120_EXTENDED_BIT | 0x6A)
#define CC1120_DCFILTOFFSET_Q1	(CC1120_EXTENDED_BIT | 0x6B)
#define CC1120_DCFILTOFFSET_Q0	(CC1120_EXTENDED_BIT | 0x6C)
#define CC1120_IQIE_I1		(CC1120_EXTENDED_BIT | 0x6D)
#define CC1120_IQIE_I0		(CC1120_EXTENDED_BIT | 0x6E)
#define CC1120_IQIE_Q1		(CC1120_EXTENDED_BIT | 0x6f)
#define CC1120_IQIE_Q0		(CC1120_EXTENDED_BIT | 0x70)
#define CC1120_RSSI1		(CC1120_EXTENDED_BIT | 0x71)
#define CC1120_RSSI0		(CC1120_EXTENDED_BIT | 0x72)
#define CC1120_MARCSTATE	(CC1120_EXTENDED_BIT | 0x73)
#define CC1120_LQI_VAL		(CC1120_EXTENDED_BIT | 0x74)
#define CC1120_PQT_SYNC_ERR	(CC1120_EXTENDED_BIT | 0x75)
#define CC1120_DEM_STATUS	(CC1120_EXTENDED_BIT | 0x76)
#define CC1120_FREQOFF_EST1	(CC1120_EXTENDED_BIT | 0x77)
#define CC1120_FREQOFF_EST0	(CC1120_EXTENDED_BIT | 0x78)
#define CC1120_AGC_GAIN3	(CC1120_EXTENDED_BIT | 0x79)
#define CC1120_AGC_GAIN2	(CC1120_EXTENDED_BIT | 0x7a)
#define CC1120_AGC_GAIN1	(CC1120_EXTENDED_BIT | 0x7b)
#define CC1120_AGC_GAIN0	(CC1120_EXTENDED_BIT | 0x7c)
#define CC1120_SOFT_RX_DATA_OUT	(CC1120_EXTENDED_BIT | 0x7d)
#define CC1120_SOFT_TX_DATA_IN	(CC1120_EXTENDED_BIT | 0x7e)
#define CC1120_ASK_SOFT_RX_DATA	(CC1120_EXTENDED_BIT | 0x7f)
#define CC1120_RNDGEN		(CC1120_EXTENDED_BIT | 0x80)
#define CC1120_MAGN2		(CC1120_EXTENDED_BIT | 0x81)
#define CC1120_MAGN1		(CC1120_EXTENDED_BIT | 0x82)
#define CC1120_MAGN0		(CC1120_EXTENDED_BIT | 0x83)
#define CC1120_ANG1		(CC1120_EXTENDED_BIT | 0x84)
#define CC1120_ANG0		(CC1120_EXTENDED_BIT | 0x85)
#define CC1120_CHFILT_I2	(CC1120_EXTENDED_BIT | 0x86)
#define CC1120_CHFILT_I1	(CC1120_EXTENDED_BIT | 0x87)
#define CC1120_CHFILT_I0	(CC1120_EXTENDED_BIT | 0x88)
#define CC1120_CHFILT_Q2	(CC1120_EXTENDED_BIT | 0x89)
#define CC1120_CHFILT_Q1	(CC1120_EXTENDED_BIT | 0x8a)
#define CC1120_CHFILT_Q0	(CC1120_EXTENDED_BIT | 0x8b)
#define CC1120_GPIO_STATUS	(CC1120_EXTENDED_BIT | 0x8c)
#define CC1120_FSCAL_CTRL	(CC1120_EXTENDED_BIT | 0x8d)
#define CC1120_PHASE_ADJUST	(CC1120_EXTENDED_BIT | 0x8e)
#define CC1120_PARTNUMBER	(CC1120_EXTENDED_BIT | 0x8f)
#define CC1120_PARTVERSION	(CC1120_EXTENDED_BIT | 0x90)
#define CC1120_SERIAL_STATUS	(CC1120_EXTENDED_BIT | 0x91)
#define CC1120_RX_STATUS	(CC1120_EXTENDED_BIT | 0x92)
#define CC1120_TX_STATUS	(CC1120_EXTENDED_BIT | 0x93)
#define CC1120_MARC_STATUS1	(CC1120_EXTENDED_BIT | 0x94)
# define CC1120_MARC_STATUS1_NO_FAILURE		0
# define CC1120_MARC_STATUS1_RX_TIMEOUT		1
# define CC1120_MARC_STATUS1_RX_TERMINATION	2
# define CC1120_MARC_STATUS1_EWOR_SYNC_LOST	3
# define CC1120_MARC_STATUS1_MAXIMUM_LENGTH	4
# define CC1120_MARC_STATUS1_ADDRESS		5
# define CC1120_MARC_STATUS1_CRC		6
# define CC1120_MARC_STATUS1_TX_FIFO_OVERFLOW	7
# define CC1120_MARC_STATUS1_TX_FIFO_UNDERFLOW	8
# define CC1120_MARC_STATUS1_RX_FIFO_OVERFLOW	9
# define CC1120_MARC_STATUS1_RX_FIFO_UNDERFLOW	10
# define CC1120_MARC_STATUS1_TX_ON_CCA_FAILED	11
# define CC1120_MARC_STATUS1_TX_FINISHED	0x40
# define CC1120_MARC_STATUS1_RX_FINISHED	0x80
#define CC1120_MARC_STATUS0	(CC1120_EXTENDED_BIT | 0x95)
#define CC1120_PA_IFAMP_TEST	(CC1120_EXTENDED_BIT | 0x96)
#define CC1120_FSRF_TEST	(CC1120_EXTENDED_BIT | 0x97)
#define CC1120_PRE_TEST		(CC1120_EXTENDED_BIT | 0x98)
#define CC1120_PRE_OVR		(CC1120_EXTENDED_BIT | 0x99)
#define CC1120_ADC_TEST		(CC1120_EXTENDED_BIT | 0x9a)
#define CC1120_DVC_TEST		(CC1120_EXTENDED_BIT | 0x9b)
#define CC1120_ATEST		(CC1120_EXTENDED_BIT | 0x9c)
#define CC1120_ATEST_LVDS	(CC1120_EXTENDED_BIT | 0x9d)
#define CC1120_ATEST_MODE	(CC1120_EXTENDED_BIT | 0x9e)
#define CC1120_XOSC_TEST1	(CC1120_EXTENDED_BIT | 0x9f)
#define CC1120_XOSC_TEST0	(CC1120_EXTENDED_BIT | 0xa0)
#define CC1120_RXFIRST		(CC1120_EXTENDED_BIT | 0xd2)
#define CC1120_TXFIRST		(CC1120_EXTENDED_BIT | 0xd3)
#define CC1120_RXLAST		(CC1120_EXTENDED_BIT | 0xd4)
#define CC1120_TXLAST		(CC1120_EXTENDED_BIT | 0xd5)
#define CC1120_NUM_TXBYTES	(CC1120_EXTENDED_BIT | 0xd6)
#define CC1120_NUM_RXBYTES	(CC1120_EXTENDED_BIT | 0xd7)
#define CC1120_FIFO_NUM_TXBYTES	(CC1120_EXTENDED_BIT | 0xd8)
#define CC1120_FIFO_NUM_RXBYTES	(CC1120_EXTENDED_BIT | 0xd9)

/* Status byte */
#define CC1120_STATUS_CHIP_RDY	7
#define CC1120_STATUS_STATE	4
#define  CC1120_STATUS_STATE_IDLE		0
#define  CC1120_STATUS_STATE_RX			1
#define  CC1120_STATUS_STATE_TX			2
#define  CC1120_STATUS_STATE_FSTXON		3
#define  CC1120_STATUS_STATE_CALIBRATE		4
#define  CC1120_STATUS_STATE_SETTLING		5
#define  CC1120_STATUS_STATE_RX_FIFO_ERROR	6
#define  CC1120_STATUS_STATE_TX_FIFO_ERROR	7
#define  CC1120_STATUS_STATE_MASK		7

#endif /* _AO_CC1120_H_ */
