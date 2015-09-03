/*******************************************************************************
(c) 2015 by SE-Elektronic GmbH

*******************************************************************************/

/*!
********************************************************************************
@file spiflash.h

@moduleinformation
  QSPI Flash

@moduledescription
  QSPI Flash driver for ATMEL SAMV71XULT board, with Spansion QSPI Flash
  * Read in SPI Memory Mode
  * Send in native SPI Mode

@moduleglossary
  .

@preprocessor
  .

********************************************************************************
*/

#ifndef SPIFLASH_H
#define	SPIFLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include-Dateien ************************************************************/
#include "pdt.h"

/* Konstanten *****************************************************************/

/* Makros *********************************************************************/

/* Typen **********************************************************************/
/*! @brief Beschreibung der Rueckgabewerte */
typedef enum
{
	SPIFLASH_RET_OK,		//!< erfolgreich beendet
	SPIFLASH_RET_E,		//!< .
	SPIFLASH_RET_E_NOT_INIT,	//!< Modul nicht initialisiert
	SPIFLASH_RET_E_ALREADY_INIT,	//!< Modul bereits initialisiert
	SPIFLASH_RET_E_FLASH_UNKNOWN,	//!< unbekannter Flash
	SPIFLASH_RET_E_WRITE_ENABLE,	//!< Write Enable nicht gesetzt.
// 	SPIFLASH_RET_E_ADDR_MASK,	//!< Adresse kann so nicht geschrieben werden
} SPIFLASH_RET;

typedef enum spiflash_manufactor
{
	SPIFLASH_MANUFACTOR_SPANSION = 0x01,
	SPIFLASH_MANUFACTOR_MICRON   = 0x20,
} SPIFLASH_MANUFACTOR;

typedef	struct spiflash_type
{
	u8_t	manufactor;
	u8_t	device_type;
	u32_t	device_capacity;
	u32_t	size_byte;
	u32_t	sector_size;
	u32_t	sector_count;
	u32_t	page_size;	// 256
	u32_t	page_mask;	// als 0x00FF
	
} SPIFLASH_TYPE;

/* Exportierte Variablen ******************************************************/
#if defined(SPIFLASH_C)
	// <TYP>	spiflash_<varname>;
#else
	// extern <TYP>	spiflash_<varname>;
#endif

/* Prototypen *****************************************************************/
extern	SPIFLASH_RET	spiflash_init (void);
extern	SPIFLASH_RET	spiflash_type_get (
	SPIFLASH_TYPE	*type);

// extern	s32_t		spiflash_read (u32_t addr, u32_t size, u8_t *buf);
extern	int		spiflash_write (u8_t *addr, const  u8_t *buf, u32_t size);
extern	int		spiflash_erase (u8_t *addr, u32_t size);
extern	SPIFLASH_RET	spiflash_erase_chip (void);


#ifdef __cplusplus
}
#endif

#endif /* SPIFLASH_H */