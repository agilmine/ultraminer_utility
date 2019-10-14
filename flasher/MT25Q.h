/*
 *
 *  Header File for STFL-I based Serial Flash Memory Driver
 *
 *
 *  Filename:		MT25Q.h
 *  Description:	Header file for MT25Q low level driver.
 *		        	Also consult the C file for more details.
 *
 *  Version:		1.8
 *  Date:			June 2017
 *  Authors:		Micron Technology, Inc
 *
 *  THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS WITH
 *  CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME. AS A
 *  RESULT, MICRON SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL
 *  DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE
 *  AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN
 *  CONNECTION WITH THEIR PRODUCTS.
 *
 *  Version History
 *
 *  Ver.		Date				Comments
 *
 *  0.1			January 2013		Alpha version with a subset of commands implemented
 *  1.0			February 2013		Full alpha version
 *  1.1			June 2013			Added the following functions:
 *  									- FlashReadExtAddrReg()
 *  									- FlashWriteExtAddrReg()
 *  									- FlashEnterDeepPowerDown()
 *  									- FlashReleaseDeepPowerDown()
 *  									- FlashReadAdvancedSecProt()
 *  									- FlashProgramAdvancedSecProt()
 *  									- FlashPasswordRead()
 *  									- FlashPasswordProgram()
 *  									- FlashPasswordUnlock()
 *  									- FlashPermanentProtectionBitRead()
 *  									- FlashPermanentProtectionBitProgram()
 *  									- FlashPermanentProtectionBitErase()
 *  									- FlashWriteGlobalFreezeBit()
 *  									- FlashReadGlobalFreezeBit()
 *  									- FlashOTPProgram()
 *  									- FlashOTPRead()
 *
 *  1.2        August 2013       Tested Suspend and Resume.  Removed Die Erase (not
 *                               supported on device).  Enhanced error checking.
 *
 *  1.3			September 2013    Changed type definitions to remove "NMX_" prefix.
 *
 *  1.4			November 2013	 Changed Driver_Init() to use Flash Discovery Table
 *								 to determine device geometry and specific erase
 *								 commands for multiple sector sizes.
 *	1.6
 *				September
 *								Modify some unreasonable comments of functions.
 *	1.7       June 2016
 *					     Fixed FlashWriteEnable() variable num_of_try not initialed BUG.
 *	1.8       June 2017
 *					     Added Die erase function.
 */

/*************** User Change Area *******************************************

   The purpose of this section is to show how the SW Drivers can be customized
   according to the requirements of the hardware and Flash memory configurations.
   It is possible to choose the Flash memory start address, the CPU Bit depth, the number of Flash
   chips, the hardware configuration and performance data (TimeOut Info).

   The options are listed and explained below:

   ********* Data Types *********
   The source code defines hardware independent data types assuming that the
   compiler implements the numerical types as

   unsigned char    8 bits (defined as uint8)
   char             8 bits (defined as sint8)
   unsigned int     16 bits (defined as uint16)
   int              16 bits (defined as sint16)
   unsigned long    32 bits (defined as uint32)
   long             32 bits (defined as sint32)

   In case the compiler does not support the currently used numerical types,
   they can be easily changed just once here in the user area of the header file.
   The data types are consequently referenced in the source code as (u)sint8,
   (u)sint16 and (u)sint32. No other data types like 'CHAR','SHORT','INT','LONG'
   are directly used in the code.


   ********* Flash Type *********
   This driver supports the following Serial Flash memory Types:

	- MT25Q 512Mb (Mediterraneo)

   ********* Flash and Board Configuration *********
   The driver also supports different configurations of the Flash chips
   on the board. In each configuration a new data Type called
   'uCPUBusType' is defined to match the current CPU data bus width.
   This data type is then used for all accesses to the memory.

   Because SPI interface communications are controlled by the
   SPI master, which, in turn, is accessed by the CPU as an 8-bit data
   buffer, the configuration is fixed for all cases.

   ********* TimeOut *********
   There are timeouts implemented in the loops of the code, in order
   to enable a timeout detection for operations that would otherwise never terminate.
   There are two possibilities:

   1) The ANSI Library functions declared in 'time.h' exist

      If the current compiler supports 'time.h' the define statement
      TIME_H_EXISTS should be activated. This makes sure that
      the performance of the current evaluation HW does not change
      the timeout settings.

   2) or they are not available (COUNT_FOR_A_SECOND)

      If the current compiler does not support 'time.h', the define
      statement cannot be used. In this case the COUNT_FOR_A_SECOND
      value has to be defined so as to create a one-second delay.
      For example, if 100000 repetitions of a loop are
      needed to give a time delay of one second, then
      COUNT_FOR_A_SECOND should have the value 100000.

      Note: This delay is HW (Performance) dependent and therefore needs
      to be updated with every new HW.

      This driver has been tested with a certain configuration and other
      target platforms may have other performance data, therefore, the
      value may have to be changed.

      It is up to the user to implement this value to prevent the code
      from timing out too early and allow correct completion of the device
      operations.


   ********* Additional Routines *********
   The drivers also provide a subroutine which displays the full
   error message instead of just an error number.

   The define statement VERBOSE activates additional Routines.
   Currently it activates the FlashErrorStr() function

   No further changes should be necessary.

*****************************************************************************/

#define DEBUG

#ifndef __SERIAL__H__
#define __SERIAL__H__

#define DRIVER_VERSION_MAJOR 0
#define DRIVER_VERSION_MINOR 1

/* All HW dependent Basic Data Types */
typedef unsigned char   uint8;
typedef signed char     sint8;
typedef unsigned int uint16;
typedef int          sint16;
typedef unsigned long   uint32;
typedef long         sint32;

/* Enable device auto detect on init */
#define ADDR_MODE_AUTO_DETECT

#define SIZE_16MB		0x1000000
#define SIZE_64MB       0x4000000
#define SIZE_128MB      0x8000000
#define SIZE_256MB      0x10000000
/*******************************************************************************
Flash address byte mode (see Datasheet)
*******************************************************************************/
typedef enum {
    FLASH_3_BYTE_ADDR_MODE	= 0x03,			/* 3 byte address */
    FLASH_4_BYTE_ADDR_MODE	= 0x04			/* 4 byte address */

} AddressMode;

/*#define TIME_H_EXISTS*/  /* set this macro if C-library "time.h" is supported */
/* Possible Values: TIME_H_EXISTS
                    - no define - TIME_H_EXISTS */

#ifndef TIME_H_EXISTS
#define COUNT_FOR_A_SECOND 0xFFFFFF   				/* Timer Usage */
#endif

#define SE_TIMEOUT (3)                					/* Timeout in seconds suggested for Sector Erase Operation*/
#define BE_TIMEOUT (80)           						/* Timeout in seconds suggested for Bulk Erase Operation*/

/* Activates additional Routines */
#define VERBOSE
#define DEBUG

/********************** End of User Change Area *****************************/

/*******************************************************************************
	Device Constants
*******************************************************************************/

/* manufacturer id + mem type + mem capacity  */
#define MEM_TYPE_MT25Q1024	0x20BA21	/* ID for MT25Q   1Gb device */
#define MEM_TYPE_MT25Q512	0x20BA20	/* ID for MT25Q 512Mb device */
#define MEM_TYPE_N25Q8		0x20BB14	/* ID for N25Q    8Mb device */
#define MEM_TYPE_N25Q16		0x20BB15	/* ID for N25Q   16Mb device */
#define MEM_TYPE_N25Q32		0x20BA16	/* ID for N25Q   32Mb device */
#define MEM_TYPE_N25Q64		0x20BA17	/* ID for N25Q   64Mb device */
#define MEM_TYPE_N25Q128	0x20BA18	/* ID for N25Q  128Mb device */
#define MEM_TYPE_N25Q256	0x20BA19	/* ID for N25Q  256Mb device */
#define MEM_TYPE_N25Q512	0x20BA20	/* ID for N25Q  512Mb device */
#define MEM_TYPE_N25Q1G		0x20BA21	/* ID for N25Q    1Gb device */
#define MEM_TYPE_MICRON		0x20B000 	/* first ID byte */
#define MEM_TYPE_MASK		0xFFF000

#define DISCOVERY_TABLE1				0x0C
#define DTABLE1_SECTOR_DESCRIPTOR		0x1C
#define DTABLE1_FLASH_SIZE				0x04

/*******************************************************************************
	DERIVED DATATYPES
*******************************************************************************/
/******** InstructionsCode ********/
enum {
    /* Command definitions (please see datasheet for more details) */

    /* WRITE ENABLE commands */
    SPI_FLASH_INS_WREN        			= 0x06,	/* Write enable */
    SPI_FLASH_INS_WRDI        			= 0x04,	/* Write disable */

    /* RESET commands */
    SPI_FLASH_INS_REN		  			= 0x66,	/* Reset enable */
    SPI_FLASH_INS_RMEM		  			= 0x99,	/* Reset memory */

    /* IDENTIFICATION commands */
    SPI_FLASH_INS_RDID        			= 0x9F,	/* Read Identification */
    SPI_FLASH_INS_RDID_ALT    			= 0x9E,	/* Read Identification (alternative command) */
    SPI_FLASH_INS_MULT_IO_RDID   		= 0xAF, /* Read multiple I/O read id */
    SPI_FLASH_INS_DISCOVER_PARAMETER	= 0x5A, /* Read serial flash discovery parameter */

    /* DATA READ commands */
    SPI_FLASH_INS_READ 					= 0x03, /* Read Data Bytes */
    SPI_FLASH_INS_FAST_READ 			= 0x0B, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_DOFR 					= 0x3B,	/* Dual Output Fast Read */
    SPI_FLASH_INS_DIOFR 				= 0xBB, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QOFR 					= 0x6B, /* Quad Output Fast Read */
    SPI_FLASH_INS_QIOFR 				= 0xEB, /* Quad Input/Output Fast Read */
    SPI_FLASH_INS_4READ4D 				= 0xE7, /* Word Read Quad I/O */

    /* DATA READ commands (DTR dedicated instructions) */
    SPI_FLASH_INS_FAST_READDTR 			= 0x0D, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_DOFRDTR 				= 0x3D, /* Dual Output Fast Read */
    SPI_FLASH_INS_DIOFRDTR 				= 0xBD, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QOFRDTR 				= 0x6D, /* Quad Output Fast Read */
    SPI_FLASH_INS_QIOFRDTR 				= 0xED, /* Quad Input/Output Fast Read */

    /* DATA READ commands (32-bit address) */
    SPI_FLASH_INS_READ4BYTE 			= 0x13, /* Read Data Bytes */
    SPI_FLASH_INS_FAST_READ4BYTE 		= 0x0C, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_DOFR4BYTE 			= 0x3C, /* Dual Output Fast Read */
    SPI_FLASH_INS_DIOFR4BYTE 			= 0xBC, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QOFR4BYTE 			= 0x6C, /* Quad Output Fast Read */
    SPI_FLASH_INS_QIOFR4BYTE 			= 0xEC, /* Quad Input/Output Fast Read */

    /* DATA READ commands (32-bit address in DTR mode) */
    SPI_FLASH_INS_FAST_READDTR4BYTE 	= 0x0E, /* Read Data Bytes at Higher Speed */
    SPI_FLASH_INS_DIOFRDTR4BYTE 		= 0xBE, /* Dual Input/Output Fast Read */
    SPI_FLASH_INS_QIOFRDTR4BYTE 		= 0xEE, /* Quad Input/Output Fast Read */

    /* PROGRAM DATA commands */
    SPI_FLASH_INS_PP 					= 0x02, /* Page Program  */
    SPI_FLASH_INS_DIFP					= 0xA2, /* Dual Input Fast Program  */
    SPI_FLASH_INS_DIEFP 				= 0xD2, /* Dual Input Extended Fast Program */
    SPI_FLASH_INS_QIFP 					= 0x32, /* Quad Input Fast Program */
    SPI_FLASH_INS_QIEFP					= 0x12,	/* Quad Input Extended Fast Program */
    SPI_FLASH_INS_QIEFP_ALT				= 0x38, /* Quad Input Extended Fast Program (alternative command) */

    /* PROGRAM DATA commands (32-bit address) */
    SPI_FLASH_INS_PP4BYTE 				= 0x12, /* Page Program with */
    SPI_FLASH_INS_QIFP4BYTE 			= 0x34, /* Quad Input Fast Program with */
    SPI_FLASH_INS_QIEFP4BYTE 			= 0x3E, /* Quad Input Extended Fast Program */
    SPI_FLASH_INS_SE4BYTE 				= 0xDC, /* Sector Erase with */
    SPI_FLASH_INS_SSE4BYTE 				= 0x21, /* Sub-Sector Erase with */

    /* ERASE DATA commands */
    SPI_FLASH_INS_SE					= 0xD8, /* Sector Erase */
    SPI_FLASH_INS_SSE					= 0x20, /* Sub-Sector Erase */
    SPI_FLASH_INS_SSE32K				= 0x52, /* Sub-Sector Erase for 32KB */
    SPI_FLASH_INS_BE					= 0xC7, /* Bulk Erase */
	SPI_FLASH_INS_DE                    = 0xC4, /* Die Erase */
    SPI_FLASH_INS_BE_ALT				= 0x60, /* Bulk Erase (alternative command) */

    /* RESUME/SUSPEND commands */
    SPI_FLASH_INS_PER 					= 0x7A, /* Program/Erase Resume */
    SPI_FLASH_INS_PES 					= 0x75, /* Program/Erase Suspend */

    /* REGISTER commands */
    SPI_FLASH_INS_RDSR 					= 0x05, /* Read Status */
    SPI_FLASH_INS_WRSR 					= 0x01, /* Write Status */
    SPI_FLASH_INS_RDFSR 				= 0x70, /* Read Flag Status */
    SPI_FLASH_INS_CLRFSR 				= 0x50, /* Clear Flag Status */
    SPI_FLASH_INS_RDNVCR 				= 0xB5, /* Read NV Configuration */
    SPI_FLASH_INS_WRNVCR 				= 0xB1, /* Write NV Configuration */
    SPI_FLASH_INS_RDVCR 				= 0x85, /* Read Volatile Configuration */
    SPI_FLASH_INS_WRVCR 				= 0x81, /* Write Volatile Configuration */
    SPI_FLASH_INS_RDVECR 				= 0x65, /* Read Volatile Enhanced Configuration */
    SPI_FLASH_INS_WRVECR 				= 0x61, /* Write Volatile Enhanced Configuration */
    SPI_FLASH_INS_WREAR 				= 0xC5, /* Write Extended Address */
    SPI_FLASH_INS_RDEAR 				= 0xC8, /* Read Extended Address */
    SPI_FLASH_INS_PPMR 					= 0x68, /* Program Protection Mgmt */
    SPI_FLASH_INS_RDPMR 				= 0x2B, /* Read Protection Mgmt */
    SPI_FLASH_INS_RDGPRR 				= 0x96, /* Read General Purpose Read */

    /* Advanced Sectors Protection Commands */
    SPI_FLASH_INS_ASPRD 				= 0x2D, /* ASP Read */
    SPI_FLASH_INS_ASPP 					= 0x2C, /* ASP Program */
    SPI_FLASH_INS_DYBRD 				= 0xE8, /* DYB Read */
    SPI_FLASH_INS_DYBWR 				= 0xE5, /* DYB Write */
    SPI_FLASH_INS_PPBRD 				= 0xE2, /* PPB Read */
    SPI_FLASH_INS_PPBP 					= 0xE3, /* PPB Program */
    SPI_FLASH_INS_PPBE 					= 0xE4, /* PPB Erase */
    SPI_FLASH_INS_PLBRD 				= 0xA7, /* PPB Lock Bit Read */
    SPI_FLASH_INS_PLBWR 				= 0xA6, /* PPB Lock Bit Write */
    SPI_FLASH_INS_PASSRD 				= 0x27, /* Password Read */
    SPI_FLASH_INS_PASSP 				= 0x28, /* Password Write */
    SPI_FLASH_INS_PASSU 				= 0x29, /* Password Unlock */
    SPI_FLASH_INS_DYBRD4BYTE 			= 0xE0, /* DYB Read with 32-bit Address */
    SPI_FLASH_INS_DYBWR4BYTE 			= 0xE1, /* DYB Write with 32-bit Address */

    /* 4-byte address Commands */
    SPI_FLASH_INS_EN4BYTEADDR 			= 0xB7, /* Enter 4-byte address mode */
    SPI_FLASH_INS_EX4BYTEADDR 			= 0xE9, /* Exit 4-byte address mode */

    /* OTP commands */
    SPI_FLASH_INS_RDOTP					= 0x4B, /* Read OTP array */
    SPI_FLASH_INS_PROTP					= 0x42, /* Program OTP array */

    /* DEEP POWER-DOWN commands */
    SPI_FLASH_INS_ENTERDPD				= 0xB9, /* Enter deep power-down */
    SPI_FLASH_INS_RELEASEDPD			= 0xAB,  /* Release deep power-down */

    /* ADVANCED SECTOR PROTECTION commands */
    SPI_FLASH_ASPRD						= 0x2D, /* Advanced sector protection read */
    SPI_FLASH_ASPP						= 0x2C, /* Advanced sector protection program */
    SPI_FLASH_DYBRD						= 0xE8, /* Dynamic protection bits read */
    SPI_FLASH_DYBWR						= 0xE5, /* Dynamic protection bits write */
    SPI_FLASH_PPBRD						= 0xE2, /* Permanent protection bits read */
    SPI_FLASH_PPBP						= 0xE3, /* Permanent protection bits write */
    SPI_FLASH_PPBE						= 0xE4, /* Permanent protection bits erase */
    SPI_FLASH_PLBRD						= 0xA7, /* Permanent protection bits lock bit read */
    SPI_FLASH_PLBWR						= 0xA6, /* Permanent protection bits lock bit write	*/
    SPI_FLASH_PASSRD					= 0x27, /* Password read */
    SPI_FLASH_PASSP						= 0x28, /* Password write */
    SPI_FLASH_PASSU						= 0x29  /* Password unlock */

};

/******** InstructionsType ********/
typedef enum {
    InstructionWriteEnable,
    InstructionWriteDisable,
    ReadDeviceIdentification,
    ReadManufacturerIdentification,
    ReadStatusRegister,
    WriteStatusRegister,
    Read,
    ReadFlashDiscovery,
    ReadDataBytesHigherSpeed,
    DualOutputFastRead,
    DualInputOutputFastRead,
    QuadOutputFastRead,
    QuadInputOutputFastRead,
    WordReadQuadIO,
    PageProgram,
    DualInputFastProgram,
    DualInputExtendedFastProgram,
    QuadInputFastProgram,
    QuadInputExtendedFastProgram,
    SubSectorErase,
    SectorErase,
    DieErase,
    BulkErase,
} InstructionType;

/******** ReturnType ********/
typedef enum {
    Flash_Success = 0,
    Flash_AddressInvalid,
    Flash_MemoryOverflow,
    Flash_PageEraseFailed,
    Flash_PageNrInvalid,
    Flash_SubSectorNrInvalid,
    Flash_SectorNrInvalid,
    Flash_FunctionNotSupported,
    Flash_NoInformationAvailable,
    Flash_OperationOngoing,
    Flash_OperationTimeOut,
    Flash_ProgramFailed,
    Flash_SectorProtected,
    Flash_SectorUnprotected,
    Flash_SectorProtectFailed,
    Flash_SectorUnprotectFailed,
    Flash_SectorLocked,
    Flash_SectorUnlocked,
    Flash_SectorLockDownFailed,
    Flash_WrongType
} ReturnType;

/******** SectorType ********/
typedef uint16 uSectorType;

/******** SubSectorType ********/
typedef uint16 uSubSectorType;

/******** PageType ********/
typedef uint16 uPageType;

/******** AddrType ********/
typedef uint32 uAddrType;

/******** ParameterType ********/
typedef union {
	/**** WriteEnable has no parameters ****/

	/**** WriteDisable has no parameters ****/

	/**** ReadDeviceIdentification Parameters ****/
	struct {
		uint32 ucDeviceIdentification;
	} ReadDeviceIdentification;

	/**** ReadManufacturerIdentification Parameters ****/
	struct {
		uint8 ucManufacturerIdentification;
	} ReadManufacturerIdentification;

	/**** ReadStatusRegister Parameters ****/
	struct {
		uint8 ucStatusRegister;
	} ReadStatusRegister;

	/**** WriteStatusRegister Parameters ****/
	struct {
		uint8 ucStatusRegister;
	} WriteStatusRegister;

	/**** Read Parameters ****/
	struct {
		uAddrType udAddr;
		uint32 udNrOfElementsToRead;
		void *pArray;
	} Read;

	/**** PageProgram Parameters ****/
	struct {
		uAddrType udAddr;
		uint32 udNrOfElementsInArray;
		void *pArray;
	} PageProgram;

	/**** SectorErase Parameters ****/
	struct {
		uSectorType ustSectorNr;
	} SectorErase;

	/***** BulkErase has no parameters ****/

	/**** Clear  has no parameters ****/

} ParameterType;

typedef struct _FLASH_DESCRIPTION {
	uint32      FlashId;          /* Flash identification */
	uint32      FlashType;           /* Flash type */
	uint32      StartingAddress;     /* Start Address of the Flash Device */
	uint32      FlashAddressMask;    /* Bit mask used with address */
	uint32      FlashSize;           /* Total flash size */
	uint32      FlashOTPSize;        /* OTP area size */
	uint32       FlashDieCount;       /* Number of dies */
	uint32      FlashDieSize;        /* Single die size */
	uint32      FlashDieSize_bit;    /* Number of bits used to address a single die */

	uint32      FlashSectorSize;     /* Flash sector size */
	uint32      FlashSectorSize_bit; /* Number of bits used to addess sectors */
	uint32      FlashSectorCount;    /* Total number of sectors */
	uint32       FlashSectorEraseCmd;

	uint32      FlashSubSectorSize;     /* Sub-sector size */
	uint32      FlashSubSectorSize_bit; /* Number of bits used to address sub-sectors */
	uint32      FlashSubSectorCount; /* Total number of sub-sectors */
	uint32       FlashSubSectorEraseCmd;

	uint32      FlashPageSize;       /* Flash page size */
	uint32      FlashPageCount;         /* Total number of pages */

	uint32      BufferSize;          /* Internal flash buffer size (in bytes) */
	uint32      DataWidth;           /* Internal flash data width (in bytes) */
	AddressMode  NumAddrByte;         /* Number of bytes used to address memory */

}  FLASH_DESCRIPTION, *PFLASH_DESCRIPTION;

/* FLASH_OPERATION
 *
 * This object set low-level flash device operations
 */

typedef struct _FLASH_OPERATION {
	uAddrType  (*BlockOffset)(uSectorType);
	ReturnType (*DeviceId)(uint32 * );
	ReturnType (*ReadStatusRegister)(uint8 *);
	ReturnType (*DataProgram)(InstructionType, ParameterType *);
	ReturnType (*DataRead)(InstructionType, ParameterType *);
	ReturnType (*SectorErase)(uSectorType);
	ReturnType (*SubSectorErase)(uSectorType);
	ReturnType (*BulkErase)();
   ReturnType (*DieErase)(uSectorType uscDieNr);
	ReturnType (*WriteEnable)(void);
	ReturnType (*WriteDisable)(void);
	ReturnType (*ProgramEraseSuspend)(void);
	ReturnType (*ProgramEraseResume)(void);
	ReturnType (*ClearFlagStatusRegister)(void);
	ReturnType (*ReadNVConfigurationRegister)(uint16 *);
	ReturnType (*ReadVolatileConfigurationRegister)(uint8 *);
	ReturnType (*ReadVolatileEnhancedConfigurationRegister)(uint8 *);
	ReturnType (*ReadFlagStatusRegister)(uint8 *);
	ReturnType (*WriteNVConfigurationRegister)(uint16);
	ReturnType (*WriteVolatileConfigurationRegister)(uint8);
	ReturnType (*WriteVolatileEnhancedConfigurationRegister)(uint8);
	ReturnType (*Enter4ByteAddressMode) (void);
	ReturnType (*Exit4ByteAddressMode) (void);
	ReturnType (*LockSector)(uAddrType, uint32);
	ReturnType (*UnlockAllSector)(void);
} FLASH_OPERATION;

typedef struct {
	FLASH_DESCRIPTION 	Desc;
	FLASH_OPERATION   	GenOp;
	uint16         blocking;
} FLASH_DEVICE_OBJECT;

/******************************************************************************
    Standard functions
*******************************************************************************/


/******************************************************************************
    Standard functions
*******************************************************************************/
uAddrType BlockOffset(uSectorType uscSectorNr);
ReturnType FlashReadDeviceIdentification(uint32 *uwpDeviceIdentification);
ReturnType Driver_Init(FLASH_DEVICE_OBJECT *flash_device_object);
ReturnType FlashWriteEnable(void);
ReturnType FlashWriteDisable(void);
ReturnType FlashReadStatusRegister(uint8 *ucpStatusRegister);
ReturnType FlashWriteStatusRegister(uint8 ucStatusRegister);
ReturnType FlashGenProgram(uint32 udAddr, uint8 *pArray , uint32 udNrOfElementsInArray, uint8 ubSpiInstruction);
ReturnType DataProgram(InstructionType insInstruction, ParameterType *fp);
ReturnType DataRead(InstructionType insInstruction, ParameterType *fp);
ReturnType FlashDataProgram(uAddrType udAddr, uint8 *pArray , uint16 udNrOfElementsInArray, uint8 ubSpiInstruction);
ReturnType FlashDataRead(uAddrType udAddr, uint8 *ucpElements, uint32 udNrOfElementsToRead, InstructionType insInstruction, uint16 dataOffset);
ReturnType FlashSectorErase(uSectorType uscSectorNr);
ReturnType FlashSubSectorErase(uSectorType uscSectorNr);
ReturnType FlashDieErase(uSectorType uscDieNr );
ReturnType FlashBulkErase(void);
ReturnType FlashProgramEraseResume(void);
ReturnType FlashProgramEraseSuspend(void);
ReturnType FlashClearFlagStatusRegister(void);
ReturnType FlashReadNVConfigurationRegister(uint16 *ucpNVConfigurationRegister);
ReturnType FlashReadVolatileConfigurationRegister(uint8 *ucpVolatileConfigurationRegister);
ReturnType FlashReadVEConfigReg(uint8 *ucpVolatileEnhancedConfigurationRegister);
ReturnType FlashReadFlagStatusRegister(uint8 *ucpFlagStatusRegister);
ReturnType FlashWriteNVConfigurationRegister(uint16 ucNVConfigurationRegister);
ReturnType FlashWriteVolatileConfigurationRegister(uint8 ucVolatileConfigurationRegister);
ReturnType FlashWriteVEConfigReg(uint8 ucVolatileEnhancedConfigurationRegister);
ReturnType FlashEnter4ByteAddressMode(void);
ReturnType FlashExit4ByteAddressMode(void);
ReturnType FlashLockSector(uAddrType address, uint32 len);
ReturnType FlashUnlockAllSector();

/* added in 1.1 version */
ReturnType FlashReadExtAddrReg(uint8 *extAddrRegValue);
ReturnType FlashWriteExtAddrReg(uint8 extAddrRegValue);
ReturnType FlashEnterDeepPowerDown(void);
ReturnType FlashReleaseDeepPowerDown(void);
ReturnType FlashReadAdvancedSecProt(uint16 *aspRegValue);
ReturnType FlashProgramAdvancedSecProt(uint16 aspRegValue);
ReturnType FlashPasswordRead(uint8 *passwordValue);
ReturnType FlashPasswordProgram(uint8 *passwordValue);
ReturnType FlashPasswordUnlock(uint8 *passwordValue);
ReturnType FlashPermanentProtectionBitRead(uint32 uscSectorNr, uint8 *ppbRegisterValue);
ReturnType FlashPermanentProtectionBitProgram(uint32 uscSectorNr);
ReturnType FlashPermanentProtectionBitErase();
ReturnType FlashWriteGlobalFreezeBit(void);
ReturnType FlashReadGlobalFreezeBit(uint8 *globalfreezeBitValue);
ReturnType FlashOTPProgram(uint8 *pArray , uint32 udNrOfElementsInArray);
ReturnType FlashOTPRead(uint8 *ucpElements, uint32 udNrOfElementsToRead);



/******************************************************************************
    Utility functions
*******************************************************************************/
#ifdef VERBOSE
sint8 *FlashErrorStr( ReturnType rErrNum );
#endif

ReturnType FlashTimeOut( uint32 udSeconds );

/*******************************************************************************
List of Errors and Return values, Explanations and Help.
********************************************************************************

Error Name:   Flash_AddressInvalid
Description:  The address given is out of the range of the Flash device.
Solution:     Check whether the address is in the valid range of the
              Flash device.
********************************************************************************

Error Name:   Flash_PageEraseFailed
Description:  The Page erase Instruction did not complete successfully.
Solution:     Try to erase the Page again. If this fails once more, the device
              may be faulty and need to be replaced.
********************************************************************************

Error Name:   Flash_PageNrInvalid
Note:         The Flash memory is not at fault.
Description:  A Page has been selected (Parameter), which is not
              within the valid range. Valid Page numbers are from 0 to
              FLASH_PAGE_COUNT - 1.
Solution:     Check that the Page number given is in the valid range.
********************************************************************************

Error Name:   Flash_SectorNrInvalid
Note:         The Flash memory is not at fault.
Description:  A Sector has been selected (Parameter), which is not
              within the valid range. Valid Page numbers are from 0 to
              FLASH_SECTOR_COUNT - 1.
Solution:     Check that the Sector number given is in the valid range.
********************************************************************************

Return Name:  Flash_FunctionNotSupported
Description:  The user has attempted to make use of a functionality not
              available on this Fash device (and thus not provided by the
              software drivers).
Solution:     This may happen after changing Flash SW Drivers in existing
              environments. For example an application tries to use a
              functionality which is no longer provided with the new device.
********************************************************************************

Return Name:  Flash_NoInformationAvailable
Description:  The system cannot give any additional information about the error.
Solution:     None
********************************************************************************

Error Name:   Flash_OperationOngoing
Description:  This message is one of two messages that are given by the TimeOut
              subroutine. It means that the ongoing Flash operation is still within
              the defined time frame.
********************************************************************************

Error Name:   Flash_OperationTimeOut
Description:  The Program/Erase Controller algorithm could not finish an
              operation successfully. It should have set bit 7 of the Status
              Register from 0 to 1, but that did not happen within a predetermined
              time. The program execution was therefore cancelled by a
              timeout. This may be because the device is damaged.
Solution:     Try the previous Instruction again. If it fails a second time then it
              is likely that the device will need to be replaced.
********************************************************************************

Error Name:   Flash_ProgramFailed
Description:  The value that should be programmed has not been written correctly
              to the Flash memory.
Solutions:    Make sure that the Page which is supposed to receive the value
              was erased successfully before programming. Try to erase the Page and
              to program the value again. If it fails again then the device may
              be faulty.
********************************************************************************

Error Name:   Flash_WrongType
Description:  This message appears if the Manufacture and Device Identifications read from
              the current Flash device do not match the expected identifier
              codes. This means that the source code is not explicitely written for
              the currently used Flash chip. It may work, but the operation cannot be
              guaranteed.
Solutions:    Use a different Flash chip with the target hardware or contact
              STMicroelectronics for a different source code library.
********************************************************************************

Return Name:  Flash_Success
Description:  This value indicates that the Flash memory Instruction was executed
              correctly.
********************************************************************************/

/******************************************************************************
    External variable declaration
*******************************************************************************/

/* none in this version of the release */

/*******************************************************************************
Status Register Definitions (see Datasheet)
*******************************************************************************/
enum {
    SPI_FLASH_SRWD		= 0x80,		/* Status Register Write Protect */
    SPI_FLASH_BP3		= 0x40,		/* Block Protect Bit3 */
    SPI_FLASH_TB		= 0x20,		/* Top/Bottom bit */
    SPI_FLASH_BP2		= 0x10,		/* Block Protect Bit2 */
    SPI_FLASH_BP1		= 0x08,		/* Block Protect Bit1 */
    SPI_FLASH_BP0		= 0x04,		/* Block Protect Bit0 */
    SPI_FLASH_WEL		= 0x02,		/* Write Enable Latch */
    SPI_FLASH_WIP		= 0x01		/* Write/Program/Erase in progress bit */
};

/*******************************************************************************
Flag Status Register Definitions (see Datasheet)
*******************************************************************************/
enum {
    SPI_FSR_PROG_ERASE_CTL		= 0x80,
    SPI_FSR_ERASE_SUSP			= 0x40,
    SPI_FSR_ERASE				= 0x20,
    SPI_FSR_PROGRAM				= 0x10,
    SPI_FSR_VPP					= 0x08, //reserved
    SPI_FSR_PROG_SUSP			= 0x04,
    SPI_FSR_PROT				= 0x02,
    SPI_FSR_ADDR_MODE			= 0x01 //reserved
};

/* Status register masks */
#define SPI_SR1_WIP				(1 << 0)
#define SPI_SR1_WEL				(1 << 1)
#define SPI_SR1_BP0				(1 << 2)
#define SPI_SR1_BP1				(1 << 3)
#define SPI_SR1_BP2				(1 << 4)
#define SPI_SR1_E_FAIL			(1 << 5)
#define SPI_SR1_P_FAIL			(1 << 6)
#define SPI_SR1_SRWD			(1 << 7)

#define SPI_SR1_FAIL_FLAGS		(SPI_SR1_E_FAIL | SPI_SR1_P_FAIL)

/*******************************************************************************
Specific Function Prototypes
*******************************************************************************/
typedef unsigned char BOOL;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL_PTR
#define NULL_PTR 0x0
#endif

BOOL IsFlashBusy(void);
BOOL IsFlashWELBusy(void);


/*******************************************************************************
List of Specific Errors and Return values, Explanations and Help.
*******************************************************************************

// none in this version of the release
********************************************************************************/

#endif /* __Med__H__  */
/* In order to avoid a repeated usage of the header file */

/*******************************************************************************
     End of file
********************************************************************************/
