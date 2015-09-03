/*******************************************************************************
(c) 2015 by SE-Elektronic GmbH

*******************************************************************************/

/*!
********************************************************************************
@file spiflash.c

@moduleinformation
  QSPI Flash Driver for ATMEL SAMV71

@preprocessor
  .

********************************************************************************
*/

#define	SPIFLASH_C

/* Include-Dateien ************************************************************/
#include <stdlib.h>
#include <string.h>

#include "spiflash.h"
#include "dbg.h"
#include "prjset.h"

#include "chip.h"
#include "../src/niffs.h"
// #include "qspi_dma.h"
// #include "qspi.h"
// #include "samv7/component/component_qspi.h"

/* Konstanten *****************************************************************/
/*! @brief Debugstufen fuer die jeweiligen Modulbereiche */
#define	_DBG_LEVEL_VOID		DBG_LEVEL_INFO		//!< Standardbereich


/* Makros *********************************************************************/
// Debugausgaben
#if !defined(PRJSET_DBG_SPIFLASH)
#error "PRJSET_DBG_SPIFLASH isn't defined !"
#else
#if (PRJSET_DBG_SPIFLASH != DBG_LEVEL_NO_MSG)
#define	_DBGF(part,level,...) do {					\
		if (DBG_LEVEL_ ## level <= PRJSET_DBG_SPIFLASH)	\
		{							\
			DBG_PRINTF(part,level,__VA_ARGS__);		\
		}							\
	} while (0)

#define	_DBG(part,level,msg) do {					\
		if (DBG_LEVEL_ ## level <= PRJSET_DBG_SPIFLASH)	\
		{							\
			DBG_PRINT(part,level,msg);			\
		}							\
	} while (0)
#else
#define	_DBGF(part,level,...)
#define	_DBG(part,level,msg)
#endif
#endif

#define	_RETURN(x) DBG_RETURN(SPIFLASH,x)	//!< RETURN-Makro

/* Status Register Bit Definitions */
#define	STATUS_WRITE_READY		(0x0 << 0)
#define	STATUS_WRITE_BUSY		(0x1 << 0)
#define	STATUS_WRITE_ENABLE_CLEAR	(0x0 << 1)
#define	STATUS_WRITE_ENABLE_SET		(0x1 << 1)

#define	_DMA_CH		0
#define	_DMA_CH_BIT	(1 << _DMA_CH)


/* Typen **********************************************************************/
typedef	enum _state
{
	_STATE_UNINIT,	//!< Treiber nicht initialisiert
	_STATE_CMD,	//!< Treiber im Kommando Modus 
// 	_STATE_WRITE,	//!< Treiber im Schreib Modus
	_STATE_READ	//!< Treiber im Wahlfrei-Lesen Modus
} _STATE;

typedef enum _flash_cmd
{
	_FLASH_CMD_READ				= 0x03,
	_FLASH_CMD_FAST_READ			= 0x0B,
	_FLASH_CMD_DUAL_OUT_FAST_READ		= 0x3B,
	_FLASH_CMD_QUAD_OUT_FAST_READ		= 0x6B,
	
	_FLASH_CMD_WRITE_ENABLE			= 0x06,
	_FLASH_CMD_WRITE_DISABLE		= 0x04,
	
	_FLASH_CMD_PAGE_PROGRAM			= 0x02,
	
	_FLASH_CMD_SMALL_SECTOR_ERASE		= 0x20,
	_FLASH_CMD_BLOCK_ERASE			= 0xD8,
	_FLASH_CMD_BULK_ERASE			= 0xC7,
	
	_FLASH_CMD_MULTI_IO_READ_ID		= 0xAF,
	_FLASH_CMD_READ_ID_0			= 0x9E,
	_FLASH_CMD_READ_ID			= 0x9F,
	
	_FLASH_CMD_READ_STATUS_REGISTER	= 0x05,
	_FLASH_CMD_WRITE_STATUS_REGISTER	= 0x01,
	
	_FLASH_CMD_READ_LOCK_REGISTER		= 0xE8,
	_FLASH_CMD_WRITE_LOCK_REGISTER	= 0xE5,
	
	_FLASH_CMD_READ_FLAG_STATUS_REGISTER	= 0x70,
	_FLASH_CMD_CLEAR_FLAG_STATUS_REGISTER	= 0x50,
	
	_FLASH_CMD_READ_NV_CONFIG_REGISTER	= 0xB5,
	_FLASH_CMD_READ_V_CONFIG_REGISTER	= 0x81,
	_FLASH_CMD_READ_ENH_V_CONFIG_REGISTER	= 0x65,
	
} _FLASH_CMD;

typedef enum _cmd
{
	_CMD_NONE,		//!< KEIN cmd SENDEN
	_CMD_GET_ID,		//!< JEDEC ID lesen
	_CMD_WRITE_ENABLE,	//!< Write Enable setzen
	_CMD_WRITE_DISABLE,	//!< Write Disable setzen
	
	_CMD_READ_STATUS_REG,	//!< Status Register lesen
	_CMD_SMALL_SECTOR_ERASE,//!< kleinen (4k)Sektor loeschen
	_CMD_CHIP_ERASE,	//!< Chip loeschen
	_CMD_READ_MEM_FAST,	//!< Read Memory Mode (Memory Mode)
	_CMD_READ_MEM,		//!< Read Memory Mode 
	
	_CMD_PAGE_WRITE_MEM,	//!< Page Write ( Memory Mode)
	
	_CMD_LAST_ELEMENT,	// Letztes Enum in _CMD
} _CMD;

/*! @brief Flash Commando mit Instruction Frame */
typedef	struct _instruction {
	_FLASH_CMD		flcmd;		// Command
	union _QspiInstFrame	inst_frame;	// Instruction Frame
} _INSTRUCTION;

/* Prototypen *****************************************************************/

static const	_INSTRUCTION	_insts[_CMD_LAST_ELEMENT] = {
	[_CMD_GET_ID]		= {	.flcmd = _FLASH_CMD_READ_ID,
			.inst_frame =
				{.bm =	{
					.bwidth = 0,	// Single Bit SPI
					.bInstEn = 1,	/** Enable Inst */
					.bAddrEn = 0,	/** Enable Address */
					.bOptEn = 0,	/** Enable Option */
					.bDataEn = 1,	/** Enable Data */
					.bOptLen = 0,	/** Option Length*/
					.bAddrLen = 0,	/** Addrs Length = 24 bit*/
					.bXfrType = QSPI_IFR_TFRTYP_TRSFR_READ >> QSPI_IFR_TFRTYP_Pos ,	/** Transfer type: READ*/
					.bContinuesRead = 0, 	/** Continoues read mode*/
					.bDummyCycles = 0,	/** Dummy Cycles*/
					},
				}
			},
	[_CMD_WRITE_ENABLE]	= {	.flcmd = _FLASH_CMD_WRITE_ENABLE,
			.inst_frame =
				{.bm =	{
					.bwidth = 0,	// Single Bit SPI
					.bInstEn = 1,	/** Enable Inst */
					.bAddrEn = 0,	/** Enable Address */
					.bOptEn = 0,	/** Enable Option */
					.bDataEn = 0,	/** Enable Data */
					.bOptLen = 0,	/** Option Length*/
					.bAddrLen = 0,	/** Addrs Length = 24 bit*/
					.bXfrType = QSPI_IFR_TFRTYP_TRSFR_READ >> QSPI_IFR_TFRTYP_Pos ,	/** Transfer type: READ*/
					.bContinuesRead = 0, 	/** Continoues read mode*/
					.bDummyCycles = 0,	/** Dummy Cycles*/
					},
				}
			},
	[_CMD_WRITE_DISABLE]	= {	.flcmd = _FLASH_CMD_WRITE_DISABLE,
			.inst_frame =
				{.bm =	{
					.bwidth = 0,	// Single Bit SPI
					.bInstEn = 1,	/** Enable Inst */
					.bAddrEn = 0,	/** Enable Address */
					.bOptEn = 0,	/** Enable Option */
					.bDataEn = 0,	/** Enable Data */
					.bOptLen = 0,	/** Option Length*/
					.bAddrLen = 0,	/** Addrs Length = 24 bit*/
					.bXfrType = QSPI_IFR_TFRTYP_TRSFR_READ >> QSPI_IFR_TFRTYP_Pos ,	/** Transfer type: READ*/
					.bContinuesRead = 0, 	/** Continoues read mode*/
					.bDummyCycles = 0,	/** Dummy Cycles*/
					},
				}
			},
	[_CMD_READ_STATUS_REG]	= {	.flcmd = _FLASH_CMD_READ_STATUS_REGISTER,
			.inst_frame =
				{.bm =	{
					.bwidth = 0,	// Single Bit SPI
					.bInstEn = 1,	/** Enable Inst */
					.bAddrEn = 0,	/** Enable Address */
					.bOptEn = 0,	/** Enable Option */
					.bDataEn = 1,	/** Enable Data */
					.bOptLen = 0,	/** Option Length*/
					.bAddrLen = 0,	/** Addrs Length = 24 bit*/
					.bXfrType = QSPI_IFR_TFRTYP_TRSFR_READ >> QSPI_IFR_TFRTYP_Pos ,	/** Transfer type: READ*/
					.bContinuesRead = 0, 	/** Continoues read mode*/
					.bDummyCycles = 0,	/** Dummy Cycles*/
					},
				}
			},
	[_CMD_SMALL_SECTOR_ERASE]	= {	.flcmd = _FLASH_CMD_SMALL_SECTOR_ERASE,
			.inst_frame =
				{.bm =	{
					.bwidth = 0,	// Single Bit SPI
					.bInstEn = 1,	/** Enable Inst */
					.bAddrEn = 1,	/** Enable Address */
					.bOptEn = 0,	/** Enable Option */
					.bDataEn = 0,	/** Enable Data */
					.bOptLen = 0,	/** Option Length*/
					.bAddrLen = 0,	/** Addrs Length = 24 bit*/
					.bXfrType = QSPI_IFR_TFRTYP_TRSFR_READ >> QSPI_IFR_TFRTYP_Pos ,	/** Transfer type: READ*/
					.bContinuesRead = 0, 	/** Continoues read mode*/
					.bDummyCycles = 0,	/** Dummy Cycles*/
					},
				}
			},
	[_CMD_CHIP_ERASE]	= {	.flcmd = _FLASH_CMD_BULK_ERASE,
			.inst_frame =
				{.bm =	{
					.bwidth = 0,	// Single Bit SPI
					.bInstEn = 1,	/** Enable Inst */
					.bAddrEn = 0,	/** Enable Address */
					.bOptEn = 0,	/** Enable Option */
					.bDataEn = 0,	/** Enable Data */
					.bOptLen = 0,	/** Option Length*/
					.bAddrLen = 0,	/** Addrs Length = 24 bit*/
					.bXfrType = QSPI_IFR_TFRTYP_TRSFR_READ >> QSPI_IFR_TFRTYP_Pos ,	/** Transfer type: READ*/
					.bContinuesRead = 0, 	/** Continoues read mode*/
					.bDummyCycles = 0,	/** Dummy Cycles*/
					},
				}
			},
	[_CMD_READ_MEM_FAST]	= {	.flcmd = _FLASH_CMD_FAST_READ,
				.inst_frame =
				{.bm =	{
					.bwidth = 0,	// Single Bit SPI
					.bInstEn = 1,	/** Enable Inst */
					.bAddrEn = 1,	/** Enable Address */
					.bOptEn = 0,	/** Enable Option */
					.bDataEn = 1,	/** Enable Data */
					.bOptLen = 0,	/** Option Length*/
					.bAddrLen = 0,	/** Addrs Length = 24 bit*/
					.bXfrType = QSPI_IFR_TFRTYP_TRSFR_READ_MEMORY >> QSPI_IFR_TFRTYP_Pos ,	/** Transfer type: READ*/
					.bContinuesRead = 0, 	/** Continoues read mode*/
					.bDummyCycles = 8,	/** Dummy Cycles*/
					},
				}
			},
	[_CMD_READ_MEM]		= {	.flcmd = _FLASH_CMD_READ,
				.inst_frame =
				{.bm =	{
					.bwidth = 0,	// Single Bit SPI
					.bInstEn = 1,	/** Enable Inst */
					.bAddrEn = 1,	/** Enable Address */
					.bOptEn = 0,	/** Enable Option */
					.bDataEn = 1,	/** Enable Data */
					.bOptLen = 0,	/** Option Length*/
					.bAddrLen = 0,	/** Addrs Length = 24 bit*/
					.bXfrType = QSPI_IFR_TFRTYP_TRSFR_READ_MEMORY >> QSPI_IFR_TFRTYP_Pos ,	/** Transfer type: READ*/
					.bContinuesRead = 0, 	/** Continoues read mode*/
					.bDummyCycles = 0,	/** Dummy Cycles*/
					},
				}
			},
	[_CMD_PAGE_WRITE_MEM]	= {	.flcmd = _FLASH_CMD_PAGE_PROGRAM,
			.inst_frame =
				{.bm =	{
					.bwidth = 0,	// Single Bit SPI
					.bInstEn = 1,	/** Enable Inst */
					.bAddrEn = 1,	/** Enable Address */
					.bOptEn = 0,	/** Enable Option */
					.bDataEn = 1,	/** Enable Data */
					.bOptLen = 0,	/** Option Length*/
					.bAddrLen = 0,	/** Addrs Length = 24 bit*/
					.bXfrType = QSPI_IFR_TFRTYP_TRSFR_WRITE_MEMORY >> QSPI_IFR_TFRTYP_Pos ,	/** Transfer type: READ*/
					.bContinuesRead = 0, 	/** Continoues read mode*/
					.bDummyCycles = 0,	/** Dummy Cycles*/
					},
				}
			},

// 	[_CMD_GET_ID] = {},
};


/* Variablen ******************************************************************/
static	_STATE	_mstate;	//!< interner Modulzustand

static	SPIFLASH_TYPE	_type;

// QSPI im single mode pinnen.
#define	_QSPI_QUAD	1

static Pin _pins[] = {
/* PIOA : QCS, QSCK, QIO0-2 */
	{
		(PIO_PA11A_QCS 
		| PIO_PA14A_QSCK 
		| PIO_PA13A_QIO0	/* MOSI */
		| PIO_PA12A_QIO1	/* MISO */
#if (_QSPI_QUAD != 0)			
		| PIO_PA17A_QIO2
#endif			
		),
		PIOA, 
		ID_PIOA, 
		PIO_PERIPH_A, 
		PIO_DEFAULT
	}, 
/* PIOD: QIO3	*/
#if (_QSPI_QUAD != 0)		
	{
		PIO_PD31A_QIO3, 
		PIOD, 
		ID_PIOD, 
		PIO_PERIPH_A, 
		PIO_DEFAULT
	}
#endif	
};

/* Funktionen *****************************************************************/

/*!
********************************************************************************
@brief	CMD im QSPI setzen
********************************************************************************
*/
static	void	_cmd_send (
	_CMD	cmd,
	u32_t	addr)
{

	
	u32_t	dummy_read;
	if ( (_insts[cmd].inst_frame.bm.bAddrEn) && (!_insts[cmd].inst_frame.bm.bDataEn))
	{
		_DBGF(VOID, INFO, "CMD %u, Addr %08x",
			cmd, addr);
		QSPI->QSPI_IAR = addr;
	}
	else
	{
// 		_DBGF(VOID, INFO, "CMD %u", cmd);
	}
	
	QSPI->QSPI_ICR = _insts[cmd].flcmd;
	QSPI->QSPI_IFR = _insts[cmd].inst_frame.val;
	
	dummy_read = QSPI->QSPI_IFR;	// Sync
	UNUSED(dummy_read);
}

/*!
********************************************************************************
@brief	CMD beenden
********************************************************************************
*/
static	void	_cmd_close (void)
{
	QSPI->QSPI_CR = QSPI_CR_LASTXFER;
	
	if (_mstate == _STATE_READ)
	{
		while ((!(QSPI->QSPI_SR & QSPI_SR_CSS)))
		{
		}
	}
	else
	{
		// STATE CMD ; _STATE _ Uninit
		while ((!(QSPI->QSPI_SR & QSPI_SR_INSTRE)))
		{};
	}
	
	// Done
}

/*!
********************************************************************************
@brief	Register mit 4 Byte lesen
********************************************************************************
*/
static	u32_t	_reg_read32 (
	_CMD	cmd)
{
	u32_t	buffer;
	
	_cmd_send(cmd, 0);
	buffer = *((uint32_t*)QSPIMEM_ADDR); // Von Flash 0x00 == 0x8000.0000 lesen
	_cmd_close();

	_DBGF(VOID, NOTICE, "buffer32 0x%08x", buffer);
	return buffer;
}

/*!
********************************************************************************
@brief	Register mit 1 Byte lesen
********************************************************************************
*/
static	u8_t	_reg_read8 (
	_CMD	cmd)
{
	u8_t	buffer;
	
	_cmd_send(cmd, 0);
	buffer = *((uint8_t*)QSPIMEM_ADDR); // Von Flash 0x00 == 0x8000.0000 lesen
	_cmd_close();

// 	_DBGF(VOID, NOTICE, "buffer8 0x%x", buffer);
	
	return buffer;
}

/*!
********************************************************************************
@brief	JEDEC Daten lesen & ausgeben

(ggf. Treiber anpassen)



********************************************************************************
*/
static	SPIFLASH_RET	_jedec_detect (void)
{
	u32_t	buffer;
	
	buffer = _reg_read32(_CMD_GET_ID);
	
	_DBGF(VOID, ERROR, "QSPI Flash: Manufacturer and Device ID: %d %d %d (0x%08x)\n",
				buffer & 0xff,
				(buffer >> 8) & 0xff,
				(buffer >> 16) & 0xff,
				buffer
	);	
	
	_type.manufactor = buffer & 0xFF;
	_type.device_type = (buffer>>8) & 0xFF;
	_type.device_capacity = (buffer>>16) & 0xFF;
	
	if (_type.manufactor == SPIFLASH_MANUFACTOR_SPANSION)
	{
		_type.page_size = 0x100;	// 256 Byte
		_type.page_mask = 0xFF;		
		
		switch (_type.device_capacity)
		{
			case 0x15:	//!< S25FL116K
				_type.size_byte = 2 * 1024 * 1024;
				_type.sector_size = 4 * 1024;
				_type.sector_count = 512;
				_RETURN(SPIFLASH_RET_OK);
			break;
			case 0x16:	//!< S25FL132K
				_type.size_byte = 4 * 1024 * 1024;
				_type.sector_size = 4 * 1024;
				_type.sector_count = 1024;
				_RETURN(SPIFLASH_RET_OK);
			break;
			case 0x17:	//!< S25FL164K
				_type.size_byte = 8 * 1024 * 1024;
				_type.sector_size = 4 * 1024;
				_type.sector_count = 2048;
				_RETURN(SPIFLASH_RET_OK);
			break;

			default:
				_RETURN(SPIFLASH_RET_E_FLASH_UNKNOWN);
		}
	}
	
	_RETURN(SPIFLASH_RET_E_FLASH_UNKNOWN);
}

static	void	_read_cmd_set (void)
{
	volatile u32_t buffer;
	if (_mstate != _STATE_READ)
	{
// 		_DBG(VOID, INFO, "->> set Read");
// 		_cmd_close();
		_cmd_send(_CMD_READ_MEM_FAST, 0);
		_mstate = _STATE_READ;
		buffer = *(u32_t*)QSPIMEM_ADDR;
		UNUSED(buffer);
	}
	else
	{
		/*Already in Read*/
		_DBG(VOID, INFO, "->> was Read");
	}
}

static	void	_read_cmd_close (void)
{
	if (_mstate != _STATE_CMD)
	{
		_DBG(VOID, INFO, " <<- close Read");
		_cmd_close();
		_mstate = _STATE_CMD;
	}
	else
	{
		/*Done*/
		_DBG(VOID, INFO, "->> was CMD");
	}
}

static	void	_controller_cfg_qspi (void)
{
	
	QSPI->QSPI_CR = QSPI_CR_QSPIDIS;	// Disable
	QSPI->QSPI_CR = QSPI_CR_SWRST; 		//Reset
	
	// Takt + Modus
	QSPI->QSPI_SCR = QSPI_SCR_SCBR(1) | ClockMode_00;
	
	QSPI->QSPI_MR = QSPI_MR_SMM_MEMORY | QSPI_MR_CSMODE_LASTXFER;	// Memory Mode
	
	QSPI->QSPI_CR = QSPI_CR_QSPIEN;		// Enable
}


static	void	_controller_cfg_spi (void)
{
		
	QSPI->QSPI_CR = QSPI_CR_QSPIDIS;	// Disable
	QSPI->QSPI_CR = QSPI_CR_SWRST; 		//Reset
	
	// Takt + Modus
	QSPI->QSPI_SCR = QSPI_SCR_SCBR(1) | ClockMode_00;
	
	QSPI->QSPI_MR = /*QSPI_MR_SMM_MEMORY |*/ QSPI_MR_CSMODE_LASTXFER  ;	// Memory Mode
	
	QSPI->QSPI_CR = QSPI_CR_QSPIEN;		// Enable
}

/*!
********************************************************************************
@brief	QSPI Controller konfigurieren

Peripherie + Takt
Reset
Memory Mode
Enable
********************************************************************************
*/
extern	SPIFLASH_RET	spiflash_init (void)
{
	SPIFLASH_RET	ret;
	
	if (_mstate != _STATE_UNINIT)
	{
		_RETURN(SPIFLASH_RET_E_ALREADY_INIT);
	}

	_DBG(VOID, INFO, "QSPI Pins");
	PIO_Configure(_pins, PIO_LISTSIZE(_pins));
	PMC_EnablePeripheral(ID_QSPI);
	PMC_EnablePeripheral(ID_XDMAC);

	_controller_cfg_qspi();
	
	_DBG(VOID, INFO, "QSPI Init");

	
	ret = _jedec_detect();
	
	if (ret < SPIFLASH_RET_E)
	{
		_mstate = _STATE_CMD;
		
		_read_cmd_set();
	}
	
	return (ret);
}

/*!
********************************************************************************
@brief	Flash Type ausgeben

(intern, beim Init ausgelesen)

********************************************************************************
*/
extern	SPIFLASH_RET	spiflash_type_get (
	SPIFLASH_TYPE	*type)
{
	*type = _type;
	
	if (!_mstate)
	{
		_RETURN(SPIFLASH_RET_E_NOT_INIT);
	}
	
	if (type->size_byte)
	{
		_DBGF(VOID, INFO, "Manufactor %u", type->manufactor);
		_DBGF(VOID, INFO, "Device Type 0x%02x", type->device_type);
		_DBGF(VOID, INFO, "Device Capacity 0x%02x", type->device_capacity);
		_DBGF(VOID, INFO, "size %u [Byte], sector_count %u, sector_size %u",
			type->size_byte, type->sector_count, type->sector_size);
		_RETURN(SPIFLASH_RET_OK);
	}
	
	_RETURN(SPIFLASH_RET_E_FLASH_UNKNOWN);
}

#if 0 /* Read wird direkt im MemoryMode durchgefuehrt. */
/*!
********************************************************************************
@brief	Daten per DMA senden

34.5.4 XDMAC Transfer Software Operation

34.5.4.1. Single Block With Single Microblock Transfer

CH 0 fuer RX QSPI

read XDMAC_CISx
Write the XDMAC Channel x Source Address Register (XDMAC_CSAx) for channel x.
Write the XDMAC Channel x Destination Address Register (XDMAC_CDAx) for channel x.
Program field UBLEN in the XDMAC Channel x Microblock Control Register (XDMAC_CUBCx) with the
number of data.

Program the XDMAC Channel x Configuration Register (XDMAC_CCx):
a. Clear XDMAC_CCx.TYPE for a memory to memory transfer, otherwise set this bit.
b. Program XDMAC_CCx.MBSIZE to the memory burst size used.
c. Program XDMAC_CCx.SAM/DAM to the memory addressing scheme.
d. Program XDMAC_CCx.SYNC to select the peripheral transfer direction.
e. Program XDMAC_CCx.CSIZE to configure the channel chunk size (only relevant for peripheral synchronized transfer).
f. Program XDMAC_CCx.DWIDTH to configure the transfer data width.
g. Program XDMAC_CCx.SIF, XDMAC_CCx.DIF to configure the master interface used to read data and write data respectively.
h. Program XDMAC_CCx.PERID to select the active hardware request line (only relevant for a peripheral synchronized transfer).
i. Set XDMAC_CCx.SWREQ to use software request (only relevant for a peripheral synchronized transfer).

Clear the following five registers:
XDMAC Channel x Next Descriptor Control Register (XDMAC_CNDCx)
XDMAC Channel x Block Control Register (XDMAC_CBCx)
XDMAC Channel x Data Stride Memory Set Pattern Register (XDMAC_CDS_MSPx)
XDMAC Channel x Source Microblock Stride Register (XDMAC_CSUSx)
XDMAC Channel x Destination Microblock Stride Register (XDMAC_CDUSx)

This respectively indicates that the linked list is disabled, there is only one block and striding is disabled.

Enable the Microblock interrupt by writing a 1 to bit BIE in the XDMAC Channel x Interrupt Enable Register
(XDMAC_CIEx), enable the Channel x Interrupt Enable bit by writing a 1 to bit IEx in the XDMAC Global
Interrupt Enable Register (XDMAC_GIE).

Enable channel x by writing a 1 to bit ENx in the XDMAC Global Channel Enable Register (XDMAC_GE).
XDMAC_GS.STx (XDMAC Channel x Status bit) is set by hardware.



********************************************************************************
*/
static	void		_dma_read (
	u32_t		addr,
	u32_t		size,
	const u8_t	*buf)
{
	
	XdmacChid	*xdma = &(XDMAC->XDMAC_CHID[_DMA_CH]);
	
	// Read CIS, cleared by read
	xdma->XDMAC_CIS;
	
	xdma->XDMAC_CSA = QSPIMEM_ADDR | addr; // Src Addr
	xdma->XDMAC_CDA = (u32_t) buf; // Dst

	xdma->XDMAC_CUBC = size;
	
// 	xdma->XDMAC_CC = XDMAC_CC_TYPE_MEM_TRAN | 
// 			 XDMAC_CC_MBSIZE_SIXTEEN	|	//!< 16 Byte
// 				;
				

	xdma->XDMAC_CC = XDMAC_CC_TYPE_MEM_TRAN |
				XDMAC_CC_MEMSET_NORMAL_MODE |
				XDMAC_CC_DWIDTH_WORD |
				XDMAC_CC_MBSIZE_SIXTEEN |
				XDMAC_CC_SIF_AHB_IF1 |
				XDMAC_CC_DIF_AHB_IF1 |
				XDMAC_CC_SAM_INCREMENTED_AM |
				XDMAC_CC_DAM_INCREMENTED_AM 
				
// 			|	XDMAC_CC_PERID(5 /* Scheint so nach DMA: Table 34-1 */)
				;
	
	XDMAC->XDMAC_GE = _DMA_CH_BIT;	// Enable DMA Transfer, Write only
	
	memory_sync();
	
	while (xdma->XDMAC_CC & (XDMAC_CC_RDIP_IN_PROGRESS | XDMAC_CC_RDIP_IN_PROGRESS))	// Poll Status
	{
	}
}
#endif

#if 0 /* Read ueber Direktzugriff in Speicher */ 
/*!
********************************************************************************
@brief	#

#

@usage
\code
-
\endcode

@warning 	-
@bug 		-
@see		-

@return	-
********************************************************************************
*/
extern	s32_t	spiflash_read (
	u32_t	addr,
	u32_t	size,
	u8_t	*buf)
{
	if (!_mstate)
	{
		_DBG(VOID, ERROR, "!INIT");
		return ERR_NIFFS_NOT_MOUNTED;
	}
	
	if (size != 2)
	{
		_DBGF(VOID, INFO, "Addr: 0x%x, Size: %u, buf %p", addr, size, buf);
	}
	
	_cmd_send(_CMD_READ_MEM_FAST, 0);
	
	if (size != 2)
	{
		_DBG(VOID, INFO, "memcpy ...");
	}
	
	memcpy(buf, (void *)(QSPIMEM_ADDR | addr), size);
// 	_dma_read(addr, size, buf);
	
	if (size != 2)
	{
// 		_DBG(VOID, INFO, "cmd_close ...");
	}
	
	_cmd_close();
	
	if (size != 2)
	{	
		_DBG(VOID, INFO, "done ");
	}
	
//	return size;
	return NIFFS_OK;
}
#endif

#if 0
/*!
********************************************************************************
@brief	Daten per DMA senden

34.5.4 XDMAC Transfer Software Operation

34.5.4.1. Single Block With Single Microblock Transfer

CH 0 fuer TX QSPI

read XDMAC_CISx
Write the XDMAC Channel x Source Address Register (XDMAC_CSAx) for channel x.
Write the XDMAC Channel x Destination Address Register (XDMAC_CDAx) for channel x.
Program field UBLEN in the XDMAC Channel x Microblock Control Register (XDMAC_CUBCx) with the
number of data.

Program the XDMAC Channel x Configuration Register (XDMAC_CCx):
a. Clear XDMAC_CCx.TYPE for a memory to memory transfer, otherwise set this bit.
b. Program XDMAC_CCx.MBSIZE to the memory burst size used.
c. Program XDMAC_CCx.SAM/DAM to the memory addressing scheme.
d. Program XDMAC_CCx.SYNC to select the peripheral transfer direction.
e. Program XDMAC_CCx.CSIZE to configure the channel chunk size (only relevant for peripheral synchronized transfer).
f. Program XDMAC_CCx.DWIDTH to configure the transfer data width.
g. Program XDMAC_CCx.SIF, XDMAC_CCx.DIF to configure the master interface used to read data and write data respectively.
h. Program XDMAC_CCx.PERID to select the active hardware request line (only relevant for a peripheral synchronized transfer).
i. Set XDMAC_CCx.SWREQ to use software request (only relevant for a peripheral synchronized transfer).

Clear the following five registers:
XDMAC Channel x Next Descriptor Control Register (XDMAC_CNDCx)
XDMAC Channel x Block Control Register (XDMAC_CBCx)
XDMAC Channel x Data Stride Memory Set Pattern Register (XDMAC_CDS_MSPx)
XDMAC Channel x Source Microblock Stride Register (XDMAC_CSUSx)
XDMAC Channel x Destination Microblock Stride Register (XDMAC_CDUSx)

This respectively indicates that the linked list is disabled, there is only one block and striding is disabled.

Enable the Microblock interrupt by writing a 1 to bit BIE in the XDMAC Channel x Interrupt Enable Register
(XDMAC_CIEx), enable the Channel x Interrupt Enable bit by writing a 1 to bit IEx in the XDMAC Global
Interrupt Enable Register (XDMAC_GIE).

Enable channel x by writing a 1 to bit ENx in the XDMAC Global Channel Enable Register (XDMAC_GE).
XDMAC_GS.STx (XDMAC Channel x Status bit) is set by hardware.



********************************************************************************
*/
static	void		_dma_send (
	u32_t		addr,
	u32_t		size,
	const u8_t	*buf)
{
	u32_t		xdma_cc;
	XdmacChid	*xdma = &(XDMAC->XDMAC_CHID[_DMA_CH]);
	
	// Read CIS, cleared by read
	xdma->XDMAC_CIS;
	
	xdma->XDMAC_CSA = (u32_t) buf; // Src Addr
	xdma->XDMAC_CDA = QSPIMEM_ADDR | addr; // Dst

	xdma->XDMAC_CUBC = size;
	
// 	xdma->XDMAC_CC = XDMAC_CC_TYPE_MEM_TRAN | 
// 			 XDMAC_CC_MBSIZE_SIXTEEN	|	//!< 16 Byte
// 				;
				

/*	xdma->XDMAC_CC = XDMAC_CC_TYPE_MEM_TRAN |
				XDMAC_CC_MEMSET_NORMAL_MODE |
				XDMAC_CC_MBSIZE_SIXTEEN |
				XDMAC_CC_DWIDTH_BYTE |
				XDMAC_CC_SIF_AHB_IF1 |
				XDMAC_CC_DIF_AHB_IF1 |
				XDMAC_CC_SAM_INCREMENTED_AM |
				XDMAC_CC_DAM_INCREMENTED_AM 
				*/
// 			|	XDMAC_CC_PERID(5 /* Scheint so nach DMA: Table 34-1 */)
				;
	xdma_cc	 = XDMAC_CC_TYPE_MEM_TRAN |
				XDMAC_CC_MEMSET_NORMAL_MODE |

				XDMAC_CC_DWIDTH_BYTE |
				XDMAC_CC_SIF_AHB_IF1 |
				XDMAC_CC_DIF_AHB_IF1 |
				XDMAC_CC_SAM_INCREMENTED_AM |
				XDMAC_CC_DAM_INCREMENTED_AM 
// 				|	XDMAC_CC_PERID(5 /* Scheint so nach DMA: Table 34-1 */)
				;

// 	if (size < 4)
	{
		xdma_cc	|= XDMAC_CC_MBSIZE_SINGLE;
	}
// 	else if (size < 8)
// 	{
// 		xdma_cc	|= XDMAC_CC_MBSIZE_SIXTEEN;
// 	}
// 	else if (size < 8)
// 	{
// 		xdma_cc	|= XDMAC_CC_MBSIZE_SIXTEEN;
// 	}
// 	else
// 	{
// 		xdma_cc	|= XDMAC_CC_MBSIZE_SIXTEEN;
// 	}
				
	xdma->XDMAC_CC = xdma_cc;
	
	
	XDMAC->XDMAC_GE = _DMA_CH_BIT;	// Enable DMA Transfer, Write only
	
	memory_sync();
	
	while (xdma->XDMAC_CC & (XDMAC_CC_RDIP_IN_PROGRESS | XDMAC_CC_RDIP_IN_PROGRESS))	// Poll Status
	{
	}
}
#endif

/*!
*****************************************************QSPI->***************************
@brief	Schreiben mit SPI

* den QSPI Controller in den SPI Mode schalten
* Write Page CMD und Adresse schicken (einzelne Bytes, gepollt)
* den Puffer Byteweise schicken
* den QSPI Controller in den SPI Mode schalten

********************************************************************************
*/
static	void	_spi_write (
	u8_t	*addr, 
const	u8_t	*buf,
	u32_t	size)
{
	u8_t		data;
	INTARCH16U	i;
	u32_t		a = (u32_t) addr;
	
	_controller_cfg_spi();
	
	// Flash_CMD schicken
	
	QSPI->QSPI_TDR = _FLASH_CMD_PAGE_PROGRAM;
	while( !(QSPI->QSPI_SR & QSPI_SR_TDRE))
	{
	}
	
	// Schreibadresse senden
	for ( i = 3; i > 0; i--)
	{
		while( !(QSPI->QSPI_SR & QSPI_SR_TDRE))
		{
		}
		
		a <<= 8; // Erstmal 8 Byte hoch schieben. ( HIGH Byte ist beim ersten mal egal.)
		data = (a & 0xFF000000) >> 24;
		QSPI->QSPI_TDR = data;
	}

	// alle Daten senden
	for ( i = 0; i < size; i++)
	{
		while( !(QSPI->QSPI_SR & QSPI_SR_TDRE))
		{
		}
		QSPI->QSPI_TDR = buf[i];
	}

	QSPI->QSPI_CR = QSPI_CR_LASTXFER;	// Last Transfer setzen um den #CS zu loeschen
	
	// Warten, bis alle Daten .. wirklich .. gesendet wurden.
	while( !(QSPI->QSPI_SR & QSPI_SR_TXEMPTY))
	{
	}

	_controller_cfg_qspi();
}

/*!
********************************************************************************
@brief	#

#

@usage
\code
-
\endcode

@warning 	-
@bug 		-
@see		-

@return	-
********************************************************************************
*/
extern	int		spiflash_write (u8_t *addr, const  u8_t *buf, u32_t size)
{
	u32_t	buffer;
	u32_t	remaining_page_size;
	
	int	ret = NIFFS_OK;
	
	if (!_mstate)
	{
		_DBG(VOID, ERROR, "!INIT");
		return ERR_NIFFS_NOT_MOUNTED;
	}
	
	addr = (u8_t*)((u32_t)addr | QSPIMEM_ADDR); // hoechstes Bit in der Adresse setzen
	
	_DBGF(VOID, INFO, "Addr: 0x%x, Size: %u, first Word 0x%08x", addr, size, ((u32_t*) buf)[0]);
	
	_read_cmd_close();	// in dne Kommandomodus wechseln
	
	do
	{
		_cmd_send(_CMD_WRITE_ENABLE, 0);
		_cmd_close();
		
		buffer = _reg_read8(_CMD_READ_STATUS_REG);
		if (!(buffer & STATUS_WRITE_ENABLE_SET))
		{
			ret = ERR_NIFFS_NOT_WRITABLE;
			break;
		}
		
		remaining_page_size = _type.page_size - ((u32_t /*addr to uint*/ )addr & _type.page_mask);
		/*	addr	remaining_page_size
		 * 	0x0100	0x00 --> 0x100
		 * 	0x01AA
		 * 	0x01FF
		*/
		 
		remaining_page_size = (remaining_page_size > size) ? size : remaining_page_size;
// 		_DBGF(VOID, INFO, "remaining_page_size: %u", remaining_page_size);

#if 0
		
		_cmd_send(_CMD_PAGE_WRITE_MEM, 0);
// 		memory_sync();
// 		memcpy((void *)(QSPIMEM_ADDR | addr), buf, remaining_page_size);
		memory_sync();
		if (remaining_page_size <= 4)
		{
			uint32_t	padwrite = 0xFFFFFFFF;
			uint8_t *	_ptr = (uint8_t *)&padwrite;
			
			for (int i = 0; i < remaining_page_size; i++)
			{
				*_ptr++ = *buf++;
			}
			*((uint32_t *)(addr)) = padwrite;
			// todo shift
			*((uint32_t *)(addr)) = 0xFFFFFFFF;
			
			memory_sync();
		}
		else
		{
			_dma_send((u32_t /*addr to uint*/ )addr, remaining_page_size, buf);
		}
		_cmd_close();
#else
		_spi_write(addr, buf, remaining_page_size);
#endif
		
		// Dann ist die Page mal uebertragen und wird mit #CS = HIGH programmiert.

		do
		{
			buffer = _reg_read8(_CMD_READ_STATUS_REG);
			
		} while((buffer & STATUS_WRITE_BUSY));
			
		if (buffer & STATUS_WRITE_ENABLE_SET)
		{
			// DEAD Code ... Wird nicht benutzt.
			_cmd_send(_CMD_WRITE_DISABLE, 0);
			_cmd_close();
		}
		
		size = (size > remaining_page_size) ? (size - remaining_page_size) : 0;
		buf  += remaining_page_size;
		addr += remaining_page_size;
		
	} while (size > 0);
		
	_read_cmd_set();
	
#if 0
	
	for (u32_t i = 0; i < 8; i+=8)
	{
		_DBGF(VOID, WARN,"[%02x] 0x%08x, 0x%08x,", i,
			*(addr_orig),
			*(addr_orig+1));
	}
#endif
	
	return ret;
}

/*!
********************************************************************************
@brief	Subsektor loeschen

Flash in Write Modus ( Write Enable / Read Status Reg)
Subsector Erase
Busy pollen
Write Disable

********************************************************************************
*/
extern	int		spiflash_erase (u8_t *addr, u32_t size)
{
	u32_t	buffer;
	int ret = NIFFS_OK;
	
	if (!_mstate)
	{
		_DBG(VOID, ERROR, "!init");
		return ERR_NIFFS_NOT_MOUNTED;
	}
	
	addr = (u8_t*)((u32_t)addr & (~QSPIMEM_ADDR)); // hoechstes Bit aus Adresse entferen
	
	_read_cmd_close();
	
	_DBGF(VOID, NOTICE, "Addr: 0x%x, Size: %u", addr, size);
	do
	{
		_cmd_send(_CMD_WRITE_ENABLE, 0);
		_cmd_close();
		
		buffer = _reg_read8(_CMD_READ_STATUS_REG);
		if (!(buffer & STATUS_WRITE_ENABLE_SET))
		{
			_DBG(VOID, ERROR, "! write Enable");
			ret = ERR_NIFFS_NOT_WRITABLE;
			break;
		}
		
		_cmd_send(_CMD_SMALL_SECTOR_ERASE, (u32_t)addr);
		_cmd_close();

		do
		{
			buffer = _reg_read8(_CMD_READ_STATUS_REG);
			
		} while((buffer & STATUS_WRITE_BUSY));
			
		if (buffer & STATUS_WRITE_ENABLE_SET)
		{
			_cmd_send(_CMD_WRITE_DISABLE, 0);
			_cmd_close();
		}
		else
		{
// 			_DBG(VOID, INFO, "No Need to Disable WriteMode"); Ja, immer.
		}
		
		size = (size > _type.sector_size) ? (size - _type.sector_size) : 0;
		
	} while (size > 0);
	
	_read_cmd_set();
	
	return ret;
}


/*!
********************************************************************************
@brief	#

#

@usage
\code
-
\endcode

@warning 	-
@bug 		-
@see		-

@return	-
********************************************************************************
*/
extern	SPIFLASH_RET	spiflash_erase_chip (void)
{
	u32_t	buffer;
	
	_cmd_send(_CMD_WRITE_ENABLE, 0);
	_cmd_close();
	
// 	buffer = *((uint32_t*)QSPIMEM_ADDR); // Von Flash 0x00 == 0x8000.0000 lesen
	buffer = _reg_read8(_CMD_READ_STATUS_REG);
	if (!(buffer & STATUS_WRITE_ENABLE_SET))
	{
		_RETURN(SPIFLASH_RET_E_WRITE_ENABLE);
	}
	
	_cmd_send(_CMD_CHIP_ERASE, 0);
	_cmd_close();
	
	do
	{
		buffer = _reg_read8(_CMD_READ_STATUS_REG);
		
	} while((buffer & STATUS_WRITE_BUSY));
		
	if (buffer & STATUS_WRITE_ENABLE_SET)
	{
		_cmd_send(_CMD_WRITE_DISABLE, 0);
		_cmd_close();
	}
	else
	{
		_DBG(VOID, INFO, "No Need to Disable WriteMode");
	}
	
	_RETURN(SPIFLASH_RET_OK);
}