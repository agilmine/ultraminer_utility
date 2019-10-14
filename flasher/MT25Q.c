/*
 *
 *  STFL-I based Serial Flash Memory Driver
 *
 *
 *  Filename:		MT25Q.c
 *  Description:	Library routines for the MT25Q Serial Flash Memories series
 *
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
 *  1.3			September 2013   Changed type definitions to remove "NMX_" prefix.
 *
 *  1.4			November 2013	 Changed Driver_Init() to use Flash Discovery Table
 *								 to determine device geometry and specific erase
 *								 commands for multiple sector sizes.
 *
 *  1.5        March 2014
 *                                   Changed FlashPemanentProtectionBitRead() to use 4bytes address
 *                                   Changed FlashPemanentProtectionBitErase() to use 4bytes address
 *  1.6       September
 *					     Modify some unreasonable comments of functions.
 *	1.7       June 2016
 *					     Fixed FlashWriteEnable() variable num_of_try not initialed BUG.
 *	1.8       June 2017
 *					     Added Die erase function.
 */

#ifdef NUCLEUS
#include <nu_ncl.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "MT25Q.h" 			/* Header file with global prototypes */

/* Serialize.h
 *
 * Serialize.h contains the signature for Serialize_SPI function.
 * This function is platform depended and allows the driver to communicate with flash
 * device. Serialize_SPI has the following signature:
 *
 * SPI_STATUS Serialize_SPI(const CharStream* char_stream_send,
 *              CharStream* char_stream_recv,
 *             SpiConfigOptions optBefore,
 *             SpiConfigOptions optAfter *)
 *
 * where:
 *
 * -	char_stream_send: the char stream to be sent from the SPI master
 * 		to the Flash memory. Usually contains instruction, address, and data
 * 		to be programmed;
 * -	char_stream_recv: the char stream to be received by the SPI master,
 *      sent from the Flash memory. Usually contains data to be read from the
 *      memory;
 * -	optBefore: configurations of the SPI master before any transfer/receive;
 * -	optAfter: configurations of the SPI after any transfer/receive;
 * -	SPI_STATUS can be assume success or failed error value.
 *
 * and in particular optBefore and optAfter can assume the following values:
 *
 * -	OpsWakeUp: set the CS;
 * -	OpsInitTransfer: keep the CS unchanged;
 * -	OpsEndTransfer: clear the CS.
 *
 * This driver assume to use a SPI Flash specific controller who take care about the right
 * signals management for Dual, Quad, Extended mode and dummy bytes insertion. If you
 * use a generic SPI controller, some changes may be necessary.
 */

/* Enable this define to force Flag status register synch */
#define FSR_SYNC

#include "spi_drv.h"

#ifdef TIME_H_EXISTS
#include <time.h>
#endif

/* Global flash device object */
static FLASH_DEVICE_OBJECT *fdo;

/* Local function, not api */
void fill_addr_vect(uAddrType udAddr, uint8* pIns_Addr, uint8 num_address_byte);
ReturnType WAIT_TILL_Instruction_EXECUTION_COMPLETE(sint16 second);

/*******************************************************************************
Function:     uAddrType BlockOffset ( uSectorType uscSectorNr);
Arguments:    Sector Number

Description:  This function is used to obtain the sector's start address
Pseudo Code:
   Step 1: Return the sector start address
*******************************************************************************/
uAddrType BlockOffset(uSectorType uscSectorNr)
{
	return (uscSectorNr << fdo->Desc.FlashSectorSize_bit);
}

/*******************************************************************************
Function:     ReturnType Driver_Init(FLASH_DEVICE_OBJECT *flash_device_object)
Arguments:

Description:  This function is used to initialize the driver. The function perform
			  device detection and sets driver to use the right functions.

Pseudo Code:
   Step 1: Detect the device type
   Step 2: Set device parameters (shape and operation)
   Step 3: If device support 4Byte address, call FlashEnter4ByteAddressMode and verify that
		   device accept 4-byte address mode.
*******************************************************************************/
ReturnType Driver_Init(FLASH_DEVICE_OBJECT *flash_device_object)
{
	uint8 flag = 0;
	uint32 device = 0;
	ReturnType ret;
	uint32 tableAddress, toffset;
	ParameterType pm;
	uint8 buffer[128];

	fdo = flash_device_object;
	/* all operations block waiting for completion. */
	fdo->blocking = 1;

	FlashReadDeviceIdentification(&device);

	if ((device & MEM_TYPE_MASK) == MEM_TYPE_MICRON) {
		fdo->Desc.FlashId = device;
		/* set the device size and addressing
		 * bytes to do the read. The read
		 * routine checks the address for in-device
		 * and the number of bytes for the address. */
		device &= 0xFF;
		if (device >= 0x20)
			device -= 6;
		fdo->Desc.FlashSize = 1 << device;
		fdo->Desc.NumAddrByte = 3;

		/* read the discovery parameter signature. */
		pm.Read.pArray = buffer;
		pm.Read.udAddr = 0;
		pm.Read.udNrOfElementsToRead = sizeof(buffer);
		if ((ret = DataRead(ReadFlashDiscovery, &pm)) == Flash_Success) {
			if (memcmp(buffer, "SFDP", 4))
				return Flash_NoInformationAvailable;
		} else
			return ret;

		/* the parameter table pointer is at DISCOVERY_TABLE1. */
		tableAddress = buffer[DISCOVERY_TABLE1] |
		               (buffer[DISCOVERY_TABLE1 + 1] << 8) |
		               (buffer[DISCOVERY_TABLE1 + 2] << 16);

		/* get the official device size in bytes. */
		fdo->Desc.FlashSize = (*((uint32*)&buffer[tableAddress + DTABLE1_FLASH_SIZE]) + 1) / 8;

		/* Get the largest sector size and the sector count,
		 * and take one sub-sector size and sub-sector count.
		 * The first two sector definitions have the definitions
		 * that we use - usually 4K and 64K. */
		toffset = tableAddress + DTABLE1_SECTOR_DESCRIPTOR + 2;
		if (buffer[toffset] != 0) {
			fdo->Desc.FlashSectorSize_bit = buffer[toffset];
			fdo->Desc.FlashSectorSize = 1 << buffer[toffset];
			fdo->Desc.FlashSectorCount = fdo->Desc.FlashSize /
			                             fdo->Desc.FlashSectorSize;
			fdo->Desc.FlashSectorEraseCmd = buffer[toffset + 1];

			fdo->Desc.FlashSubSectorSize_bit = buffer[toffset - 2];
			fdo->Desc.FlashSubSectorSize = 1 << buffer[toffset - 2];
			fdo->Desc.FlashSubSectorCount = fdo->Desc.FlashSize /
			                                fdo->Desc.FlashSubSectorSize;
			fdo->Desc.FlashSubSectorEraseCmd = buffer[toffset - 1];
		}

		/* Hard-coded flash parameters. */
		fdo->Desc.FlashPageSize = 0x100;
		fdo->Desc.FlashPageCount = fdo->Desc.FlashSize / fdo->Desc.FlashPageSize;
		fdo->Desc.FlashAddressMask = 0xFF;

		fdo->Desc.FlashOTPSize = 0x40;

		/* Initial Die information */
		if (fdo->Desc.FlashSize > SIZE_64MB)
        {
		fdo->Desc.FlashDieCount = fdo->Desc.FlashSize / SIZE_64MB;
		fdo->Desc.FlashDieSize = SIZE_64MB;
		fdo->Desc.FlashDieSize_bit = 26;
		}


		/* Device operation */
		fdo->GenOp.DeviceId = FlashReadDeviceIdentification;
		fdo->GenOp.ReadStatusRegister = FlashReadStatusRegister;
		fdo->GenOp.DataProgram = DataProgram;
		fdo->GenOp.DataRead = DataRead;
		fdo->GenOp.SectorErase = FlashSectorErase;
		fdo->GenOp.SubSectorErase = FlashSubSectorErase;

		if (fdo->Desc.FlashSize <= SIZE_64MB)
		fdo->GenOp.BulkErase = FlashBulkErase;
		else
		fdo->GenOp.DieErase = FlashDieErase;
		fdo->GenOp.BlockOffset = BlockOffset;
		fdo->GenOp.WriteEnable = FlashWriteEnable;
		fdo->GenOp.WriteDisable = FlashWriteDisable;
		fdo->GenOp.ClearFlagStatusRegister = FlashClearFlagStatusRegister;
		fdo->GenOp.ReadNVConfigurationRegister = FlashReadNVConfigurationRegister;
		fdo->GenOp.ReadVolatileConfigurationRegister = FlashReadVolatileConfigurationRegister;
		fdo->GenOp.ReadVolatileEnhancedConfigurationRegister = FlashReadVEConfigReg;
		fdo->GenOp.ReadFlagStatusRegister  = FlashReadFlagStatusRegister;
		fdo->GenOp.WriteNVConfigurationRegister = FlashWriteNVConfigurationRegister;
		fdo->GenOp.WriteVolatileConfigurationRegister = FlashWriteVolatileConfigurationRegister;
		fdo->GenOp.WriteVolatileEnhancedConfigurationRegister = FlashWriteVEConfigReg;
		fdo->GenOp.Enter4ByteAddressMode = FlashEnter4ByteAddressMode;
		fdo->GenOp.Exit4ByteAddressMode = FlashExit4ByteAddressMode;
		fdo->GenOp.LockSector = FlashLockSector;
		fdo->GenOp.UnlockAllSector = FlashUnlockAllSector;
		fdo->GenOp.ProgramEraseSuspend = FlashProgramEraseSuspend;
		fdo->GenOp.ProgramEraseResume = FlashProgramEraseResume;

#ifdef ADDR_MODE_AUTO_DETECT
		if (fdo->Desc.FlashSize > SIZE_16MB) {
			/* assume you want to use the whole device size  */
			fdo->GenOp.Enter4ByteAddressMode();
			/* verify current addr mode */
			fdo->GenOp.ReadFlagStatusRegister(&flag);
			if (flag & 1)   /* test addressing bit of flag status reg (bit 0) */
				fdo->Desc.NumAddrByte = FLASH_4_BYTE_ADDR_MODE;
		} else
			fdo->Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;
#endif

		return Flash_Success;
	}

	return Flash_WrongType;
}



/*******************************************************************************
Function:     ReturnType FlashBulkErase( void )
Arguments:    none

Return Values:
   Flash_OperationOngoing
   Flash_OperationTimeOut
   Flash_Success

Description:  This function erases the whole Flash memory by sending an
              SPI_FLASH_INS_BE Instruction.
Note:
			  (Only for N25QxxxA8 devices)

              This function does not check whether the target memory area (or part of it)
			  is in a Software Protection Mode(SPM) or Hardware Protection Mode(HPM),
			  in which case the PP Instruction will be ignored.
              The function assumes that the target memory area has previously been unprotected at both
              the hardware and software levels.
              To unprotect the memory, please call FlashWriteStatusRegister(uint8 ucStatusRegister),
              and refer to the datasheet to set a proper ucStatusRegister value.

Pseudo Code:
   Step 1: Check whether any previous Write, Program or Erase cycle is on going
   Step 2: Disable the Write protection (the Flash memory will automatically enable it
           again after the execution of the Instruction)
   Step 3: Initialize the data (Instruction & address) packet to be sent serially
   Step 4: Send the packet (Instruction & address) serially
   Step 5: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  FlashBulkErase(void)
{
	CharStream char_stream_send;
	uint8  cBE = SPI_FLASH_INS_BE;
	uint8 fsr_value;
	ReturnType ret;

	// Step 1: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 2: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

      FlashReadFlagStatusRegister(&fsr_value); //test
   
	// Step 3: Initialize the data(Instruction & address) packet to be sent serially
	char_stream_send.length   = 1;
	char_stream_send.pChar    = &cBE;

	// Step 4: Send the packet(Instruction & address) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 5: Wait until the operation completes or a timeout occurs.
	ret = WAIT_TILL_Instruction_EXECUTION_COMPLETE(BE_TIMEOUT);

#ifdef FSR_SYNC
	FlashReadFlagStatusRegister(&fsr_value);
	FlashClearFlagStatusRegister();

	if((fsr_value & SPI_FSR_PROT) && (fsr_value & SPI_FSR_ERASE))
		return Flash_SectorProtected;
#endif

	return ret;
}


/*
 * DataProgram
 */
ReturnType DataProgram(InstructionType insInstruction, ParameterType *fp)
{
	ReturnType rRetVal;
	uint8 ucStatusRegister;
	uint8 insCode;
	uint8 ucManufacturerIdentification;

	switch (insInstruction) {

		/* Page Program */
	case PageProgram:
		insCode = SPI_FLASH_INS_PP;
		break;

		/* Dual Input Fast Program  */
	case DualInputFastProgram:
		insCode = SPI_FLASH_INS_DIFP;
		break;

		/* Dual Input Extended Fast Program */
	case DualInputExtendedFastProgram:
		insCode = SPI_FLASH_INS_DIEFP;
		break;

		/* Quad Input Fast Program */
	case QuadInputFastProgram:
		insCode = SPI_FLASH_INS_QIEFP;
		break;

		/* Quad Input Extended Fast Program */
	case QuadInputExtendedFastProgram:
		insCode = SPI_FLASH_INS_QIEFP;
		break;

	default:
		return Flash_FunctionNotSupported;
		break;

	} /* EndSwitch */

	rRetVal = FlashDataProgram( (*fp).PageProgram.udAddr,
	                            (*fp).PageProgram.pArray,
	                            (*fp).PageProgram.udNrOfElementsInArray,
	                            insCode
	                          );

	return rRetVal;
} /* EndFunction Flash */


/*******************************************************************************
Function:     ReturnType DataRead(InstructionType insInstruction, ParameterType *fp)
Arguments:    insInstruction is an enum which contains all the available Instructions
    of the SW driver.
              fp is a (union) parameter struct for all Flash Instruction parameters
Return Value: The function returns the following conditions:

   Flash_AddressInvalid,
   Flash_MemoryOverflow,
   Flash_PageEraseFailed,
   Flash_PageNrInvalid,
   Flash_SectorNrInvalid,
   Flash_FunctionNotSupported,
   Flash_NoInformationAvailable,
   Flash_OperationOngoing,
   Flash_OperationTimeOut,
   Flash_ProgramFailed,
   Flash_SpecificError,
   Flash_SectorProtected,
   Flash_SectorUnprotected,
   Flash_SectorProtectFailed,
   Flash_SectorUnprotectFailed,
   Flash_SectorLocked,
   Flash_SectorUnlocked,
   Flash_SectorLockDownFailed,
   Flash_Success,
   Flash_WrongType

Description:  This function is used to access all functions provided with the
   current Flash device.

Pseudo Code:
   Step 1: Select the right action using the insInstruction parameter
   Step 2: Execute the Flash memory Function
   Step 3: Return the Error Code
*******************************************************************************/
ReturnType DataRead(InstructionType insInstruction, ParameterType *fp)
{
	uint8 ucStatusRegister;
	uint8 ucManufacturerIdentification;
	uint8 insCode;
	uint16 dataOffset = 0;
	ReturnType rRetVal;

	switch (insInstruction) {
		/* Read Data Bytes */
	case Read:
		insCode = SPI_FLASH_INS_READ;
		break;

		/* Read Data Bytes at Higher Speed */
	case ReadDataBytesHigherSpeed:
      dataOffset = 1; //1 byte/8 cycles dummy
		insCode = SPI_FLASH_INS_FAST_READ;
		break;

		/* Dual Output Fast Read */
	case DualOutputFastRead:
		insCode = SPI_FLASH_INS_DOFR;
		break;

		/* Dual Input/Output Fast Read */
	case DualInputOutputFastRead:
		insCode = SPI_FLASH_INS_DIOFR;
		break;

		/* Quad Output Fast Read */
	case QuadOutputFastRead:
		insCode = SPI_FLASH_INS_QOFR;
		break;

		/* Quad Input/Output Fast Read */
	case QuadInputOutputFastRead:
		insCode = SPI_FLASH_INS_QIOFR;
		break;

		/* Word Read Quad I/O */
	case WordReadQuadIO:
		insCode = SPI_FLASH_INS_4READ4D;
		break;

	case ReadFlashDiscovery:
      dataOffset = 1; //1 byte/8 cycles dummy
		insCode = SPI_FLASH_INS_DISCOVER_PARAMETER;
		break;

	default:
		return Flash_FunctionNotSupported;
		break;

	} /* EndSwitch */
	rRetVal = FlashDataRead((*fp).Read.udAddr, (*fp).Read.pArray,
	                        (*fp).Read.udNrOfElementsToRead, insCode, dataOffset);
	return rRetVal;
}


/*******************************************************************************
Function:     FlashWriteEnable( void )
Arguments:    void

Return Value:
   Flash_Success

Description:  This function sets the Write Enable Latch(WEL)
              by sending a WREN Instruction.

Pseudo Code:
   Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
   Step 2: Send the packet serially
*******************************************************************************/
ReturnType FlashWriteEnable(void)
{
	CharStream char_stream_send;
	uint8 cWREN = SPI_FLASH_INS_WREN;
	uint8 ucSR;
	uint8 num_of_try = 0;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length = 1;
	char_stream_send.pChar  = &cWREN;

	// Step 2: Send the packet serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 3: Read the Status Register.
	do {
		num_of_try++;
		FlashReadStatusRegister(&ucSR);
	} while((~ucSR & SPI_FLASH_WEL) && (num_of_try < 255));

	if(num_of_try == 255)
		return Flash_OperationTimeOut;

	return Flash_Success;
}

/*******************************************************************************
Function:     FlashWriteDisable( void )
Arguments:    void

Return Value: Flash_Success

Description:  This function resets the Write Enable Latch(WEL)
              by sending a WRDI Instruction.

Pseudo Code:
   Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
   Step 2: Send the packet serially
*******************************************************************************/
ReturnType FlashWriteDisable(void)
{
	CharStream char_stream_send;
	uint8 cWRDI = SPI_FLASH_INS_WRDI;
	uint8 ucSR;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length = 1;
	char_stream_send.pChar  = &cWRDI;

	// Step 2: Send the packet serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 3: Read the Status Register.
	do {
		FlashReadStatusRegister(&ucSR);
	} while(ucSR & SPI_SR1_WEL);

	return Flash_Success;
}

/*******************************************************************************
Function:     FlashReadDeviceIdentification( uint32 *uwpDeviceIdentification)
Arguments:    uwpDeviceIdentificaiton, 32-bit buffer to hold the DeviceIdentification
			  read from the memory, with this parts:

unit32

 | 0x00 | MANUFACTURER_ID | MEM_TYPE | MEM_CAPACITY |
MSB                                                LSB

Return Value:
   Flash_Success

Description:  This function returns the Device Identification
			  (manufacurer id + memory type + memory capacity)
              by sending an SPI_FLASH_INS_RDID Instruction.

Pseudo Code:
   Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
   Step 2: Send the packet serially
   Step 3: Device Identification is returned
*******************************************************************************/
ReturnType FlashReadDeviceIdentification(uint32 *uwpDeviceIdentification)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8  cRDID = SPI_FLASH_INS_RDID;
	uint8  pIdentification[3];

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cRDID;
	char_stream_recv.length  = 3;
	char_stream_recv.pChar   = pIdentification;

	// Step 2: Send the packet serially
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

#ifdef DEBUG
	printf("DeviceId[0] = 0x%x\n", char_stream_recv.pChar[0]);
	printf("DeviceId[1] = 0x%x\n", char_stream_recv.pChar[1]);
	printf("DeviceId[2] = 0x%x\n", char_stream_recv.pChar[2]);
#endif

	// Step 3: Device Identification is returned (manufacturer id + memory type + memory capacity)
	*uwpDeviceIdentification = char_stream_recv.pChar[0];
	*uwpDeviceIdentification <<= 8;
	*uwpDeviceIdentification |= char_stream_recv.pChar[1];
	*uwpDeviceIdentification <<= 8;
	*uwpDeviceIdentification |= char_stream_recv.pChar[2];

	return Flash_Success;
}

/*******************************************************************************
Function:     FlashReadStatusRegister( uint8 *ucpStatusRegister)
Arguments:    ucpStatusRegister, 8-bit buffer to hold the Status Register value read
              from the memory

Return Value:
   Flash_Success

Description:  This function reads the Status Register by sending an
               SPI_FLASH_INS_RDSR Instruction.

Pseudo Code:
   Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
   Step 2: Send the packet serially, get the Status Register content

*******************************************************************************/
ReturnType FlashReadStatusRegister(uint8 *ucpStatusRegister)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8  cRDSR = SPI_FLASH_INS_RDSR;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cRDSR;
	char_stream_recv.length  = 1;
	char_stream_recv.pChar   = ucpStatusRegister;

	// Step 2: Send the packet serially, get the Status Register content
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;
}


/*******************************************************************************
Function:     FlashWriteStatusRegister( uint8 ucStatusRegister)
Arguments:    ucStatusRegister, an 8-bit new value to be written to the Status Register

Return Value:
   Flash_Success

Description:  This function modifies the Status Register by sending an
              SPI_FLASH_INS_WRSR Instruction.
              The Write Status Register (WRSR) Instruction has no effect
              on b6, b5, b1(WEL) and b0(WIP) of the Status Register.b6 and b5 are
              always read as 0.

Pseudo Code:
   Step 1: Disable Write protection
   Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
   Step 3: Send the packet serially
   Step 4: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  FlashWriteStatusRegister(uint8 ucStatusRegister)
{
	CharStream char_stream_send;
	uint8  pIns_Val[2];

	// Step 1: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
	char_stream_send.length = 2;
	char_stream_send.pChar  = pIns_Val;
	pIns_Val[0] = SPI_FLASH_INS_WRSR;
	pIns_Val[1] = ucStatusRegister;

	// Step 3: Send the packet serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 4: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}


/*******************************************************************************
Function:     ReturnType FlashDataRead( uAddrType udAddr, uint8 *ucpElements,
	uint32 udNrOfElementsToRead, InstructionType insInstruction)
Arguments:    udAddr, start address to read from
              ucpElements, buffer to hold the elements to be returned
              udNrOfElementsToRead, number of elements to be returned, counted in bytes.

Return Value:
   Flash_AddressInvalid
   Flash_Success

Description:  This function reads the Flash memory by sending an
              SPI_FLASH_INS_READ Instruction.
              by design, the whole Flash memory space can be read with one READ Instruction
              by incrementing the start address and rolling to 0x0 automatically,
              that is, this function is across pages and sectors.

Pseudo Code:
   Step 1: Validate address input
   Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
   Step 3: Send the packet serially, and fill the buffer with the data being returned
*******************************************************************************/
ReturnType FlashDataRead( uAddrType udAddr, uint8 *ucpElements,
                          uint32 udNrOfElementsToRead, InstructionType insInstruction,
                          uint16 dataOffset)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	//uint8  pIns_Addr[5];
	uint8  pIns_Addr[8];
   
   memset(pIns_Addr, 0, 8);

	// Step 1: Validate address input
	if(!(udAddr < fdo->Desc.FlashSize))
		return Flash_AddressInvalid;

	// Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length   = fdo->Desc.NumAddrByte + 1 + dataOffset;
	char_stream_send.pChar    = pIns_Addr;
	pIns_Addr[0]              = insInstruction;

	fill_addr_vect(udAddr, pIns_Addr, fdo->Desc.NumAddrByte);

	char_stream_recv.length   = udNrOfElementsToRead;
	char_stream_recv.pChar    = ucpElements;

	// Step 3: Send the packet serially, and fill the buffer with the data being returned
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;
}

/*******************************************************************************
Function:     FlashPageProgram( uint32 udAddr, uint8 *pArray, uint32 udNrOfElementsInArray)
Arguments:    udAddr, start address to write to
              pArray, buffer to hold the elements to be programmed
              udNrOfElementsInArray, number of elements to be programmed, counted in bytes

Return Value:
   Flash_AddressInvalid
   Flash_OperationOngoing
   Flash_OperationTimeOut
   Flash_Success

Description:  This function writes a maximum of <PAGE SIZE> bytes of data into the memory by
              sending an SPI_FLASH_INS_PP Instruction.
              by design, the PP Instruction is effective WITHIN ONE page,i.e. 0xXX00 - 0xXXff.
              when 0xXXff is reached, the address rolls over to 0xXX00 automatically.
Note:
              This function does not check whether the target memory area is in a Software
              Protection Mode(SPM) or Hardware Protection Mode(HPM), in which case the PP
              Instruction will be ignored.
              The function assumes that the target memory area has previously been unprotected at both
              the hardware and software levels.
              To unprotect the memory, please call FlashWriteStatusRegister(uint8 ucStatusRegister),
              and refer to the datasheet for the setup of a proper ucStatusRegister value.
Pseudo Code:
   Step 1: Validate address input
   Step 2: Check whether any previous Write, Program or Erase cycle is on going
   Step 3: Disable Write protection (the Flash memory will automatically enable it again after
           the execution of the Instruction)
   Step 4: Initialize the data (Instruction & address only) packet to be sent serially
   Step 5: Send the packet (Instruction & address only) serially
   Step 6: Initialize the data (data to be programmed) packet to be sent serially
   Step 7: Send the packet (data to be programmed) serially
   Step 8: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType FlashGenProgram(uAddrType udAddr, uint8 *pArray ,
                           uint32 udNrOfElementsInArray, uint8 ubSpiInstruction)
{
	CharStream char_stream_send;
	uint8 pIns_Addr[5];
	uint8 fsr_value;
	ReturnType ret;

	// Step 1: Validate address input
	if(!(udAddr < fdo->Desc.FlashSize))
		return Flash_AddressInvalid;

	// Step 2: Check whether any previous Write, Program or Erase cycle is on-going
	if(IsFlashBusy())
		return Flash_OperationOngoing;

	// Step 3: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 4: Initialize the data (Instruction & address only) packet to be sent serially
	char_stream_send.length   = fdo->Desc.NumAddrByte + 1;
	char_stream_send.pChar    = pIns_Addr;

	pIns_Addr[0]              = ubSpiInstruction;

	fill_addr_vect(udAddr, pIns_Addr, fdo->Desc.NumAddrByte);

	// Step 5: Send the packet (Instruction & address only) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsInitTransfer);

	// Step 6: Initialize the data (data to be programmed) packet to be sent serially
	char_stream_send.length   = udNrOfElementsInArray;
	char_stream_send.pChar    = pArray;

	// Step 7: Send the packet (data to be programmed) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 8: Wait until the operation completes or a timeout occurs.
	ret = WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);

#ifdef FSR_SYNC
	FlashReadFlagStatusRegister(&fsr_value);
	FlashClearFlagStatusRegister();

	if((fsr_value & SPI_FSR_PROT) && (fsr_value & SPI_FSR_PROGRAM))
		return Flash_SectorProtected;
#endif

	return ret;

}

/*******************************************************************************
Function:     ReturnType FlashDieErase( uSectorType uscDieNr )
Arguments:    uscDieNr is the number of the Die to be erased.

Return Values:
   Flash_SectorNrInvalid
   Flash_OperationOngoing
   Flash_OperationTimeOut
   Flash_Success

Description:  This function erases the Die specified in uscDieNr by sending an
              SPI_FLASH_INS_DE Instruction.
              The function checks that the Die number is within the valid range
              before issuing the erase Instruction. Once erase has completed the status
              Flash_Success is returned.
Note:
              This function does not check whether the target memory area is in a Software
              Protection Mode(SPM) or Hardware Protection Mode(HPM), in which case the PP
              Instruction will be ignored.
              The function assumes that the target memory area has previously been unprotected at both
              the hardware and software levels.
              To unprotect the memory, please call FlashWriteStatusRegister(uint8 ucStatusRegister),
              and refer to the datasheet to set a proper ucStatusRegister value.

Pseudo Code:
   Step 1: Validate the Die number input
   Step 2: Check whether any previous Write, Program or Erase cycle is on going
   Step 3: Disable Write protection (the Flash memory will automatically enable it
           again after the execution of the Instruction)
   Step 4: Initialize the data (Instruction & address) packet to be sent serially
   Step 5: Send the packet (Instruction & address) serially
   Step 6: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  FlashDieErase(uSectorType uscDieNr) {
    CharStream char_stream_send;
    uint8  pIns_Addr[5];
    uAddrType DieAddr;
   uint8 fsr_value;
	ReturnType ret;

    // Step 1: Validate the sector number input
    if(!(uscDieNr < fdo->Desc.FlashDieCount))
    	return Flash_SectorNrInvalid;

    DieAddr = uscDieNr << fdo->Desc.FlashDieSize_bit;

    // Step 2: Check whether any previous Write, Program or Erase cycle is on going
    if(IsFlashBusy())
    	return Flash_OperationOngoing;

    // Step 3: Disable Write protection
    if(FlashWriteEnable() == Flash_OperationTimeOut)
    	return Flash_OperationTimeOut;

    // Step 4: Initialize the data (Instruction & address) packet to be sent serially
    char_stream_send.length   = fdo->Desc.NumAddrByte + 1;;
    char_stream_send.pChar    = &pIns_Addr[0];

	pIns_Addr[0]              = SPI_FLASH_INS_DE;

	fill_addr_vect(DieAddr, pIns_Addr, fdo->Desc.NumAddrByte);

    // Step 5: Send the packet (Instruction & address) serially
    Serialize_SPI(&char_stream_send,
              NULL_PTR,
              OpsWakeUp,
              OpsEndTransfer);

    // Step 6: Wait until the operation completes or a timeout occurs.
   if (fdo->blocking)
   {
		ret = WAIT_TILL_Instruction_EXECUTION_COMPLETE(SE_TIMEOUT);

#ifdef FSR_SYNC
		FlashReadFlagStatusRegister(&fsr_value);
		FlashClearFlagStatusRegister();

		if((fsr_value & SPI_FSR_PROT) && (fsr_value & SPI_FSR_ERASE))
			return Flash_SectorProtected;
   }
#endif

	return Flash_Success;
}

/*******************************************************************************
Function:     ReturnType FlashSectorErase( uSectorType uscSectorNr )
Arguments:    uSectorType is the number of the Sector to be erased.

Return Values:
   Flash_SectorNrInvalid
   Flash_OperationOngoing
   Flash_OperationTimeOut
   Flash_Success

Description:  This function erases the Sector specified in uscSectorNr by sending an
              SPI_FLASH_INS_SE Instruction.
              The function checks that the sector number is within the valid range
              before issuing the erase Instruction. Once erase has completed the status
              Flash_Success is returned.
Note:
              This function does not check whether the target memory area is in a Software
              Protection Mode(SPM) or Hardware Protection Mode(HPM), in which case the PP
              Instruction will be ignored.
              The function assumes that the target memory area has previously been unprotected at both
              the hardware and software levels.
              To unprotect the memory, please call FlashWriteStatusRegister(uint8 ucStatusRegister),
              and refer to the datasheet to set a proper ucStatusRegister value.

Pseudo Code:
   Step 1: Validate the sector number input
   Step 2: Check whether any previous Write, Program or Erase cycle is on going
   Step 3: Disable Write protection (the Flash memory will automatically enable it
           again after the execution of the Instruction)
   Step 4: Initialize the data (Instruction & address) packet to be sent serially
   Step 5: Send the packet (Instruction & address) serially
   Step 6: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  FlashSectorErase(uSectorType uscSectorNr)
{
	CharStream char_stream_send;
	uint8  pIns_Addr[5];
	uAddrType SectorAddr;
	uint8 fsr_value = 0;
	ReturnType ret;

	// Step 1: Validate the sector number input
	if(!(uscSectorNr < fdo->Desc.FlashSectorCount))
		return Flash_SectorNrInvalid;

	SectorAddr = fdo->GenOp.BlockOffset(uscSectorNr);

	// Step 2: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy())
		return Flash_OperationOngoing;

	// Step 3: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 4: Initialize the data (Instruction & address) packet to be sent serially
	char_stream_send.length   = fdo->Desc.NumAddrByte + 1;;
	char_stream_send.pChar    = &pIns_Addr[0];
	pIns_Addr[0]              = fdo->Desc.FlashSectorEraseCmd;

	fill_addr_vect(SectorAddr, pIns_Addr, fdo->Desc.NumAddrByte);

	// Step 5: Send the packet (Instruction & address) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 6: Wait until the operation completes or a timeout occurs.
	if (fdo->blocking) {
		ret = WAIT_TILL_Instruction_EXECUTION_COMPLETE(SE_TIMEOUT);

      //while ((fsr_value&SPI_FSR_PROG_ERASE_CTL) == 0)
      if (1)
      {
         if (IsFlashBusy() || IsFlashWELBusy())
         {
            printf("Wrong status..\n");
            
            if(FlashWriteDisable() == Flash_OperationTimeOut)
               return Flash_OperationTimeOut;
         }
         // IsFlashBusy();
         // IsFlashBusy();
         // IsFlashBusy();
         // FlashReadFlagStatusRegister(&fsr_value);
         // FlashReadFlagStatusRegister(&fsr_value);
         // FlashReadFlagStatusRegister(&fsr_value);
         // FlashReadFlagStatusRegister(&fsr_value);
         // FlashReadFlagStatusRegister(&fsr_value);
      }

#ifdef FSR_SYNC
		FlashReadFlagStatusRegister(&fsr_value);
		FlashClearFlagStatusRegister();

		if((fsr_value & SPI_FSR_PROT) && (fsr_value & SPI_FSR_ERASE))
			return Flash_SectorProtected;
	}
#endif

	return Flash_Success;
}


/*******************************************************************************
Function:     ReturnType FlashSunSectorErase( uSectorType uscSectorNr )
Arguments:    uSectorType is the number of the subSector to be erased.

Return Values:
   Flash_SectorNrInvalid
   Flash_OperationOngoing
   Flash_OperationTimeOut
   Flash_Success

Description:  This function erases the SubSector (4k) specified in uscSectorNr by sending an
              SPI_FLASH_INS_SSE Instruction.
              The function checks that the sub sector number is within the valid range
              before issuing the erase Instruction. Once erase has completed the status
              Flash_Success is returned.
Note:
              This function does not check whether the target memory area is in a Software
              Protection Mode(SPM) or Hardware Protection Mode(HPM), in which case the PP
              Instruction will be ignored.
              The function assumes that the target memory area has previously been unprotected at both
              the hardware and software levels.
              To unprotect the memory, please call FlashWriteStatusRegister(uint8 ucStatusRegister),
              and refer to the datasheet to set a proper ucStatusRegister value.

Pseudo Code:
   Step 1: Validate the sub sector number input
   Step 2: Check whether any previous Write, Program or Erase cycle is on going
   Step 3: Disable Write protection (the Flash memory will automatically enable it
           again after the execution of the Instruction)
   Step 4: Initialize the data (Instruction & address) packet to be sent serially
   Step 5: Send the packet (Instruction & address) serially
   Step 6: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  FlashSubSectorErase(uSectorType uscSectorNr)
{
	CharStream char_stream_send;
	uint8  pIns_Addr[5];
	uAddrType SubSectorAddr;
	uint8 fsr_value;
	ReturnType ret;

	// Step 1: Validate the sector number input
	if(!(uscSectorNr < fdo->Desc.FlashSubSectorCount))
		return Flash_SectorNrInvalid;

	SubSectorAddr = uscSectorNr << fdo->Desc.FlashSubSectorSize_bit;

	// Step 2: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 3: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 4: Initialize the data (Instruction & address) packet to be sent serially
	char_stream_send.length   = fdo->Desc.NumAddrByte + 1;;
	char_stream_send.pChar    = &pIns_Addr[0];

	pIns_Addr[0]              = fdo->Desc.FlashSubSectorEraseCmd;

	fill_addr_vect(SubSectorAddr, pIns_Addr, fdo->Desc.NumAddrByte);

	// Step 5: Send the packet (Instruction & address) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 6: Wait until the operation completes or a timeout occurs.
	ret = WAIT_TILL_Instruction_EXECUTION_COMPLETE(SE_TIMEOUT);

#ifdef FSR_SYNC
	FlashReadFlagStatusRegister(&fsr_value);
	FlashClearFlagStatusRegister();

	if((fsr_value & SPI_FSR_PROT) && (fsr_value & SPI_FSR_ERASE))
		return Flash_SectorProtected;
#endif

	return Flash_Success;
}

/*******************************************************************************
Function:     IsFlashBusy( )
Arguments:    none

Return Value:
   TRUE
   FALSE

Description:  This function checks the Write In Progress (WIP) bit to determine whether
              the Flash memory is busy with a Write, Program or Erase cycle.

Pseudo Code:
   Step 1: Read the Status Register.
   Step 2: Check the WIP bit.
*******************************************************************************/
BOOL IsFlashBusy()
{
	uint8 ucSR;

	// Step 1: Read the Status Register.
	FlashReadStatusRegister(&ucSR);

	// Step 2: Check the WIP bit.
	if(ucSR & SPI_FLASH_WIP)
		return TRUE;
	else
		return FALSE;
}

/*******************************************************************************
Function:     IsFlashWELBusy( )
Arguments:    none

Return Value:
   TRUE
   FALSE

Description:  This function checks the Write Enable bit to determine whether
              the Flash memory is busy with a Write Enable or Write Disable Op.

Pseudo Code:
   Step 1: Read the Status Register.
   Step 2: Check the WEL bit.
*******************************************************************************/
BOOL IsFlashWELBusy()
{
	uint8 ucSR;

	// Step 1: Read the Status Register.
	FlashReadStatusRegister(&ucSR);

	// Step 2: Check the WEL bit.
	if(ucSR & SPI_FLASH_WEL)
		return TRUE;
	else
		return FALSE;
}

/*******************************************************************************
Function:     	FlashDataProgram( )
*******************************************************************************/
ReturnType FlashDataProgram(uAddrType udAddr, uint8 *pArray ,
                            uint16 udNrOfElementsInArray, uint8 ubSpiInstruction)
{
	ReturnType retValue = Flash_Success;
	uint16 dataOffset;

	// Step 1: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	if (retValue != Flash_Success)
		return retValue;

	// Computing the starting alignment, i.e. the distance from the page boundary
	dataOffset = (fdo->Desc.FlashPageSize - (udAddr & fdo->Desc.FlashAddressMask) ) & fdo->Desc.FlashAddressMask;
	if (dataOffset > udNrOfElementsInArray)
		dataOffset = udNrOfElementsInArray;
	if (dataOffset > 0) {
		retValue = FlashGenProgram(udAddr, pArray, dataOffset, ubSpiInstruction);
		if (Flash_Success != retValue)
			return retValue;
	}

	for ( ; (dataOffset + fdo->Desc.FlashPageSize) < udNrOfElementsInArray; dataOffset += fdo->Desc.FlashPageSize) {
		retValue = FlashGenProgram(udAddr + dataOffset, pArray + dataOffset, fdo->Desc.FlashPageSize, ubSpiInstruction);
		if (Flash_Success != retValue)
			return retValue;
	}

	if (udNrOfElementsInArray > dataOffset)
		retValue = FlashGenProgram(udAddr + dataOffset, pArray + dataOffset, (udNrOfElementsInArray - dataOffset), ubSpiInstruction);

	return retValue;
}

/*******************************************************************************
Function:     ReturnType FlashProgramEraseResume( void )
Arguments:    none

Return Values:
   Flash_Success

Description:  This function resumes the program/erase operation suspended by sending an
              SPI_FLASH_INS_PER Instruction.
Note:

Pseudo Code:
   Step 1: Check whether any previous Write, Program or Erase cycle is suspended
   Step 2: Initialize the data (Instruction & address) packet to be sent serially
   Step 3: Send the packet (Instruction & address) serially
 ******************************************************************************/
ReturnType  FlashProgramEraseResume(void)
{
	CharStream char_stream_send;
	uint8  cPER = SPI_FLASH_INS_PER;
	ReturnType ret;

	// Step 1: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 2: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 3: Initialize the data(Instruction & address) packet to be sent serially
	char_stream_send.length   = 1;
	char_stream_send.pChar    = &cPER;

	// Step 4: Send the packet(Instruction & address) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 5: Wait until the operation completes or a timeout occurs.
	ret = WAIT_TILL_Instruction_EXECUTION_COMPLETE(BE_TIMEOUT);

	return ret;
}

/*******************************************************************************
Function:     ReturnType FlashProgramEraseSuspend( void )
Arguments:    none

Return Values:
   Flash_Success

Description:  This function resumes the program/erase operation suspended by sending an
              SPI_FLASH_INS_PES Instruction.
Note:

Pseudo Code:
   Step 3: Initialize the data (Instruction) packet to be sent serially
   Step 4: Send the packet (Instruction) serially
 ******************************************************************************/
ReturnType  FlashProgramEraseSuspend (void)
{
	CharStream char_stream_send;
	uint8  cPES = SPI_FLASH_INS_PES;
	ReturnType ret;

	// Step 1: Check whether any previous Write, Program or Erase
	//         cycle is still going.  If not return success.
	if(!IsFlashBusy()) return Flash_Success;

	// Step 2: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 3: Initialize the data(Instruction & address) packet to
	//         be sent serially
	char_stream_send.length   = 1;
	char_stream_send.pChar    = &cPES;

	// Step 4: Send the packet(Instruction & address) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 5: Wait until the operation completes or a timeout occurs.
	ret = WAIT_TILL_Instruction_EXECUTION_COMPLETE(BE_TIMEOUT);

	return ret;
}


/*******************************************************************************
Function:     FlashReadFlagStatusRegister( uint8 *ucpFlagStatusRegister)    ----ok
Arguments:    ucpFlagStatusRegister, 8-bit buffer to hold the Flag Status Register value read
              from the memory

Return Value:
   Flash_Success

Description:  This function reads the Status Register by sending an
               SPI_FLASH_INS_CLRFSR Instruction.

Pseudo Code:
   Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
   Step 2: Send the packet serially

*******************************************************************************/
ReturnType  FlashClearFlagStatusRegister(void)
{
	CharStream char_stream_send;
	uint8  cCLFSR = SPI_FLASH_INS_CLRFSR;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cCLFSR;

	// Step 2: Send the packet serially, get the Status Register content
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;
}

/*******************************************************************************
Function:     FlashReadNVConfigurationRegister( uint16 *ucpNVConfigurationRegister)
Arguments:    ucpStatusRegister, 16-bit buffer to hold the Non Volatile Configuration Register
		value read from the memory

Return Value:
   Flash_Success

Description:  This function reads the Non Volatile Configuration Register by sending an
               SPI_FLASH_INS_RDNVCR Instruction.

Pseudo Code:
   Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
   Step 2: Send the packet serially, get the Configuration Register content

*******************************************************************************/
ReturnType  FlashReadNVConfigurationRegister(uint16 *ucpNVConfigurationRegister)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8  cRDNVCR = SPI_FLASH_INS_RDNVCR;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cRDNVCR;
	char_stream_recv.length  = 2;
	char_stream_recv.pChar   = (uint8 *)ucpNVConfigurationRegister;

	// Step 2: Send the packet serially, get the Status Register content
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;
}

/*******************************************************************************
Function:     FlashReadVolatileConfigurationRegister( uint8 *ucpVolatileConfigurationRegister)
Arguments:    ucpVolatileConfigurationRegister, 8-bit buffer to hold the Volatile Configuration Register
		value read from the memory

Return Value:
   Flash_Success

Description:  This function reads the Volatile Register by sending an
               SPI_FLASH_INS_RDVCR Instruction.

Pseudo Code:
   Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
   Step 2: Send the packet serially, get the Configuration Register content

*******************************************************************************/
ReturnType  FlashReadVolatileConfigurationRegister(uint8 *ucpVolatileConfigurationRegister)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8  cRDVCR = SPI_FLASH_INS_RDVCR;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cRDVCR;
	char_stream_recv.length  = 1;
	char_stream_recv.pChar   = ucpVolatileConfigurationRegister;

	// Step 2: Send the packet serially, get the Volatile Configuration Register content
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;
}

/*******************************************************************************
Function:     FlashReadVEConfigReg( uint8 *ucpVolatileEnhancedConfigurationRegister)
Arguments:    ucpVolatileEnhancedRegister, 8-bit buffer to hold the Volatile Enhanced Configuration Register
		value read from the memory

Return Value:
   Flash_Success

Description:  This function reads the Volatile Enhanced Register by sending an
               SPI_FLASH_INS_RDVECR Instruction.

Pseudo Code:
   Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
   Step 2: Send the packet serially, get the Configuration Register content

*******************************************************************************/
ReturnType  FlashReadVEConfigReg( uint8 *ucpVolatileEnhancedConfigurationRegister)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8  cRDVECR = SPI_FLASH_INS_RDVECR;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cRDVECR;
	char_stream_recv.length  = 1;
	char_stream_recv.pChar   = ucpVolatileEnhancedConfigurationRegister;

	// Step 2: Send the packet serially, get the Volatile Enhanced Configuration Register content
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;
}

/*******************************************************************************
Function:     FlashReadFlagStatusRegister( uint8 *ucp FlagStatusRegister)
Arguments:    ucpStatusRegister, 8-bit buffer to hold the Flag Status Register value read
              from the memory

Return Value:
   Flash_Success

Description:  This function reads the Status Register by sending an
               SPI_FLASH_INS_RFSR Instruction.

Pseudo Code:
   Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
   Step 2: Send the packet serially, get the Status Register content

*******************************************************************************/
ReturnType  FlashReadFlagStatusRegister(uint8 *ucpFlagStatusRegister)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8  cRFSR = SPI_FLASH_INS_RDFSR;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cRFSR;
	char_stream_recv.length  = 1;
	char_stream_recv.pChar   = ucpFlagStatusRegister;

	// Step 2: Send the packet serially, get the register content
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;
}

/*******************************************************************************
Function:     FlashWriteVolatileConfigurationRegister( uint8 ucVolatileConfigurationRegister)
Arguments:    ucVolatileConfigurationRegister, an 8-bit new value to be written to the Volatile Configuration Register

Return Value:
   Flash_Success

Description:  This function modifies the Volatile Configuration Register by sending an
              SPI_FLASH_INS_WRVCR Instruction.
              The Write Volatile Configuration Register (WRVCR) Instruction has effect immediatly

Pseudo Code:
   Step 1: Disable Write protection
   Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
   Step 3: Send the packet serially
   Step 4: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  FlashWriteVolatileConfigurationRegister(uint8 ucVolatileConfigurationRegister)
{
	CharStream char_stream_send;
	uint8  pIns_Val[2];

	// Step 1: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
	char_stream_send.length = 2;
	char_stream_send.pChar  = pIns_Val;
	pIns_Val[0] = SPI_FLASH_INS_WRVCR;
	pIns_Val[1] = ucVolatileConfigurationRegister;

	// Step 3: Send the packet serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 4: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}


/*******************************************************************************
Function:     FlashWriteVEConfigReg( uint8 ucVolatileEnhancedConfigurationRegister)
Arguments:    ucVolatileConfigurationRegister, an 8-bit new value to be written to the Volatile Enhanced Configuration Register

Return Value:
   Flash_Success

Description:  This function modifies the Volatile Enhanced Configuration Register by sending an
              SPI_FLASH_INS_WRVECR Instruction.
              The Write Volatile Enhanced Configuration Register (WRVECR) Instruction has effect immediatly

Pseudo Code:
   Step 1: Disable Write protection
   Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
   Step 3: Send the packet serially
   Step 5: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  FlashWriteVEConfigReg(uint8 ucVolatileEnhancedConfigurationRegister)
{
	CharStream char_stream_send;
	uint8  pIns_Val[2];

	// Step 1: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
	char_stream_send.length = 2;
	char_stream_send.pChar  = pIns_Val;
	pIns_Val[0] = SPI_FLASH_INS_WRVECR;
	pIns_Val[1] = ucVolatileEnhancedConfigurationRegister;

	// Step 3: Send the packet serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 4: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}

/*******************************************************************************
Function:     FlashWriteNVConfigurationRegister( uint16 ucNVConfigurationRegister)
Arguments:    ucVolatileConfigurationRegister, an 8-bit new value to be written to the Non Volatile Configuration Register

Return Value:
   Flash_Success

Description:  This function modifies the Non Volatile Configuration Register by sending an
              SPI_FLASH_INS_WRNVCR Instruction.
              The Write Non Volatile Configuration Register (WRVECR) Instruction has effect at the next power-on

Pseudo Code:
   Step 1: Disable Write protection
   Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
   Step 3: Send the packet serially
   Step 4: Wait until the operation completes or a timeout occurs.
*******************************************************************************/
ReturnType  FlashWriteNVConfigurationRegister(uint16 ucNVConfigurationRegister)
{
	CharStream char_stream_send;
	uint8  pIns_Val[2];

	// Step 1: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
	char_stream_send.length = 3;
	char_stream_send.pChar  = pIns_Val;
	pIns_Val[0] = SPI_FLASH_INS_WRNVCR;
	pIns_Val[1] = ucNVConfigurationRegister;

	// Step 3: Send the packet serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 4: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}


#ifdef VERBOSE
/*******************************************************************************
Function:     FlashErrorStr( ReturnType rErrNum );
Arguments:    rErrNum is the error number returned from other Flash memory Routines

Return Value: A pointer to a string with the error message

Description:  This function is used to generate a text string describing the
   error from the Flash memory. Call with the return value from other Flash memory routines.

Pseudo Code:
   Step 1: Return the correct string.
*******************************************************************************/
sint8 *FlashErrorStr(ReturnType rErrNum )
{
	switch(rErrNum) {
	case Flash_AddressInvalid:
		return "Flash - Address is out of Range";
	case Flash_MemoryOverflow:
		return "Flash - Memory Overflows";
	case Flash_PageEraseFailed:
		return "Flash - Page Erase failed";
	case Flash_PageNrInvalid:
		return "Flash - Page Number is out of Range";
	case Flash_SectorNrInvalid:
		return "Flash - Sector Number is out of Range";
	case Flash_FunctionNotSupported:
		return "Flash - Function not supported";
	case Flash_NoInformationAvailable:
		return "Flash - No Additional Information Available";
	case Flash_OperationOngoing:
		return "Flash - Operation ongoing";
	case Flash_OperationTimeOut:
		return "Flash - Operation TimeOut";
	case Flash_ProgramFailed:
		return "Flash - Program failed";
	case Flash_Success:
		return "Flash - Success";
	case Flash_WrongType:
		return "Flash - Wrong Type";
	default:
		return "Flash - Undefined Error Value";
	} /* EndSwitch */
} /* EndFunction FlashErrorString */
#endif /* VERBOSE Definition */


/*******************************************************************************
Function:     FlashTimeOut(uint32 udSeconds)
Arguments:    udSeconds holds the number of seconds before TimeOut occurs

Return Value:
   Flash_OperationTimeOut
   Flash_OperationOngoing

Example:   FlashTimeOut(0)  // Initializes the Timer

           While(1) {
              ...
              If (FlashTimeOut(5) == Flash_OperationTimeOut) break;
              // The loop is executed for 5 Seconds before the operation is aborted
           } EndWhile

*******************************************************************************/
#ifdef TIME_H_EXISTS
/*-----------------------------------------------------------------------------
Description:   This function provides a timeout for Flash polling actions or
   other operations which would otherwise never return.
   The Routine uses the function clock() inside ANSI C library "time.h".
-----------------------------------------------------------------------------*/
ReturnType FlashTimeOut(uint32 udSeconds)
{
	static clock_t clkReset, clkCount;

	if (udSeconds == 0) { /* Set Timeout to 0 */
		clkReset = clock();
	} /* EndIf */

	clkCount = clock() - clkReset;

	if (clkCount < (CLOCKS_PER_SEC * (clock_t)udSeconds))
		return Flash_OperationOngoing;
	else
		return Flash_OperationTimeOut;
}/* EndFunction FlashTimeOut */

#else

/*-----------------------------------------------------------------------------
Description:   This function provides a timeout for Flash polling actions or
   other operations which would otherwise never return.
   The Routine uses COUNT_FOR_A_SECOND which is considered to be a loop that
   counts for one second. It needs to be adapted to the target Hardware.
-----------------------------------------------------------------------------*/
ReturnType FlashTimeOut(uint32 udSeconds)
{

	static uint32 udCounter = 0;
	if (udSeconds == 0) { /* Set Timeout to 0 */
		udCounter = 0;
	} /* EndIf */

	if (udCounter == (udSeconds * COUNT_FOR_A_SECOND)) {
		udCounter = 0;
		return Flash_OperationTimeOut;
	} else {
		udCounter++;
		return Flash_OperationOngoing;
	} /* Endif */

} /* EndFunction FlashTimeOut */


#endif

/*-----------------------------------------------------------------------------
Description:   This function fill the vector in according with address mode
-----------------------------------------------------------------------------*/
void fill_addr_vect(uAddrType udAddr, uint8* pIns_Addr, uint8 num_address_byte)
{
	/* 3-addr byte mode */
	if(FLASH_3_BYTE_ADDR_MODE == num_address_byte) {
		pIns_Addr[1]              = udAddr >> 16;
		pIns_Addr[2]              = udAddr >> 8;
		pIns_Addr[3]              = udAddr;
	}

	/* 4-addr byte mode */
	if(FLASH_4_BYTE_ADDR_MODE == num_address_byte) {
		pIns_Addr[1]              = udAddr >> 24;
		pIns_Addr[2]              = udAddr >> 16;
		pIns_Addr[3]              = udAddr >> 8;
		pIns_Addr[4]              = udAddr;
	}
	return;
}

/*-----------------------------------------------------------------------------
Description:   This function wait till instruction execution is complete
-----------------------------------------------------------------------------*/
ReturnType WAIT_TILL_Instruction_EXECUTION_COMPLETE(sint16 second)
{
   BOOL busy = TRUE;
   uint8 fsr_value;
   
	FlashTimeOut(0);
	while(busy)
   {
      //delay 1ms
      usleep(1000);
      
      //confirm both write operation done and write controller is back up
      FlashReadFlagStatusRegister(&fsr_value);
      busy = IsFlashBusy() || ((fsr_value & SPI_FSR_PROG_ERASE_CTL) == 0);
      
		if(Flash_OperationTimeOut == FlashTimeOut(second))
			return  Flash_OperationTimeOut;
	}
	return Flash_Success;
}


/*******************************************************************************
Function:     ReturnType FlashEnter4ByteAddressMode(void)
Arguments:

Return Value:
   Flash_Success

Description:  This function set the 4-byte-address mode

Pseudo Code:
   Step 1: Write enable
   Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
   Step 3: Send the packet serially
   Step 4: Wait until the operation completes or a timeout occurs.
*******************************************************************************/

ReturnType FlashEnter4ByteAddressMode(void)
{
	CharStream char_stream_send;
	uint8 cPER = SPI_FLASH_INS_EN4BYTEADDR;
	ReturnType ret;
	uint8 flag;

	// Step 1: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	char_stream_send.length   = 1;
	char_stream_send.pChar    = &cPER;

	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

#ifdef DEBUG
	printf("ENTER 4-byte-addr mode\n");
#endif

	ret = WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);

	/* verify current addr mode */
	fdo->GenOp.ReadFlagStatusRegister(&flag);
	if (flag & 1)
		fdo->Desc.NumAddrByte = FLASH_4_BYTE_ADDR_MODE;
	else
		fdo->Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;

	return ret;
}

/*******************************************************************************
Function:     ReturnType FlashExit4ByteAddressMode(void)
Arguments:

Return Value:
   Flash_Success

Description:  This function unset 4-byte-address mode

Pseudo Code:
   Step 1: Write enable
   Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
   Step 3: Send the packet serially
   Step 4: Wait until the operation completes or a timeout occurs.
*******************************************************************************/

ReturnType FlashExit4ByteAddressMode(void)
{
	CharStream char_stream_send;
	uint8 cPER = SPI_FLASH_INS_EX4BYTEADDR;
	ReturnType ret;
	uint8 flag;

	// Step 1: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	char_stream_send.length   = 1;
	char_stream_send.pChar    = &cPER;

	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

#ifdef DEBUG
	printf("EXIT 4-byte-addr mode\n");
#endif

	ret = WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);

	/* verify current addr mode */
	fdo->GenOp.ReadFlagStatusRegister(&flag);
	if (flag & 1)
		fdo->Desc.NumAddrByte = FLASH_4_BYTE_ADDR_MODE;
	else
		fdo->Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;

	return ret;
}

/*
 * Lock a sector
 */
ReturnType FlashLockSector(uAddrType address,  uint32 len)
{
	uint8 reg_value;
	uint8 TB, BP, SR;
	int i, protected_area, start_sector;
	int sector_size, num_of_sectors;

	sector_size = fdo->Desc.FlashSectorSize;
	num_of_sectors = fdo->Desc.FlashSectorCount;

	// Step 1: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	start_sector = address / sector_size;
	protected_area = len / sector_size;

	if (protected_area == 0 || protected_area > num_of_sectors)
		return Flash_AddressInvalid;

	if ((start_sector != 0 && (start_sector + protected_area) != num_of_sectors) || (protected_area & (protected_area - 1)) != 0)
		return Flash_AddressInvalid;

	if (address / sector_size < num_of_sectors / 2) {
		TB = 1;
	} else {
		TB = 0;
	}

	BP = 1;
	for (i = 1; i <= num_of_sectors; i = i * 2) {
		if (protected_area == i) {
			break;
		}
		BP++;
	}

	SR = (((BP & 8) >> 3) << 6) | (TB << 5) | ((BP & 7) << 2);

	FlashWriteStatusRegister(SR);
	return Flash_Success;

}

/*
 * FlashUnlockAll
 */
ReturnType FlashUnlockAllSector()
{
	uint8 SR = 0;

	/* Set BP2, BP1, BP0 to 0 (all flash sectors unlocked) */
	FlashWriteStatusRegister(SR);

	return Flash_Success;
}
/* End of file */

/*
 * FlashReadExtAddrReg()
 *
 * extAddrRegValue: OUT parameter , 8 bits
 *
 */
ReturnType FlashReadExtAddrReg(uint8 *extAddrRegValue)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8  cRDEAR = SPI_FLASH_INS_RDEAR;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cRDEAR;
	char_stream_recv.length  = 1;
	char_stream_recv.pChar   = extAddrRegValue;

	// Step 2: Send the packet serially, get the register content
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;
}

/*
 * FlashWriteExtAddrReg()
 *
 * extAddrRegValue: IN parameter, 8 bits
 *
 */
ReturnType  FlashWriteExtAddrReg(uint8 extAddrRegValue)
{
	CharStream char_stream_send;
	uint8  pIns_Val[2];

	// Step 1: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
	char_stream_send.length = 2;
	char_stream_send.pChar  = pIns_Val;

	pIns_Val[0] = SPI_FLASH_INS_WREAR;
	pIns_Val[1] = extAddrRegValue;

	// Step 3: Send the packet serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Disabling the Write
	fdo->GenOp.WriteDisable();

	// Step 4: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}

/*
 * FlashEnterDeepPowerDown
 */

ReturnType  FlashEnterDeepPowerDown(void)
{
	CharStream char_stream_send;
	uint8  cENTERDPD = SPI_FLASH_INS_ENTERDPD;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cENTERDPD;

	// Step 2: Send the packet serially, get the register content
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;

}

/*
 * FlashReleaseDeepPowerDown
 */

ReturnType  FlashReleaseDeepPowerDown(void)
{
	CharStream char_stream_send;
	uint8 cRELEASEDPD = SPI_FLASH_INS_RELEASEDPD;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cRELEASEDPD;

	// Step 2: Send the packet serially, get the register content
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;

}

/*
 * FlashReadAdvancedSecProt()
 *
 * aspRegValue: OUT parameter, pointer to UINT16
 *
 */
ReturnType FlashReadAdvancedSecProt(uint16 *aspRegValue)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8 cASPRD = SPI_FLASH_ASPRD;
//    uint8 cASPRD = 0x9E;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cASPRD;
	char_stream_recv.length  = 2;
	char_stream_recv.pChar   = (uint8 *)aspRegValue;

	// Step 2: Send the packet serially, get the register content
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;
}

/*
 * FlashWriteAdvancedSecProt()
 *
 * aspRegValue: IN parameter, 16 bits
 */
ReturnType FlashProgramAdvancedSecProt(uint16 aspRegValue)
{
	CharStream char_stream_send;
	uint8  pIns_Val[3];

	// Step 1: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 2: Initialize the data (i.e. Instruction & value) packet to be sent serially
	char_stream_send.length = 3;
	char_stream_send.pChar  = pIns_Val;

	pIns_Val[0] = SPI_FLASH_ASPP;
	pIns_Val[1] = (uint8) (aspRegValue & 0x00FF);
	pIns_Val[2] = (uint8) (aspRegValue >> 8);

	// Step 3: Send the packet serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Disabling the Write
	fdo->GenOp.WriteDisable();

	// Step 3: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}

/*
 * FlashPasswordRead
 *
 * passwordValue: OUT parameter, pointer to 8 bytes
 *
 * Command details:
 *
 * The password are read with the PASSRD instruction after password is
 * programmed and before Password Protection Mode has been selected by
 * programming ASP.2 (Table 18 on page 50). After the Password Protection
 * Mode is selected, the PASSRD command is ignored.
 *
 * After the 8-bit instruction shifted in, the 64-bit data are shifted
 * out, the least significant byte first, most significant bit of each
 * byte first. The PASSRD instruction is terminated by driving Chip
 * Select (S#) high at any time during data output.
 */
ReturnType FlashPasswordRead(uint8 *passwordValue)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8  cPASSRD = SPI_FLASH_PASSRD;

	// Step 1: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length  = 1;
	char_stream_send.pChar   = &cPASSRD;
	char_stream_recv.length  = 8;
	char_stream_recv.pChar   = passwordValue;

	// Step 2: Send the packet serially, get the register content
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;
}

/*
 * FlashPasswordProgram()
 *
 * passwordValue: IN parameter, pointer to 8 bytes
 *
 * Command details:
 *
 * Before the Password Program (PASSP) instruction can be accepted, the
 * WEL bit must be set with WREN command, otherwise instruction is ignored
 * and no FSR error bits are set. The password can only be programmed before
 * the Password Protection Mode has been selected by programming ASP.2.
 * After the Password Protection Mode is selected, the PASSP command is ignored.
 * The PASSP command is entered by driving S# to the logic low state, followed
 * by the 8-bit instruction, then followed by the 64-bit password data, least
 * significant byte first, most significant bit of each byte first.
 * Chip Select (S#) must be driven high after the eighth bit of the last
 * password byte has been latched in, otherwise the PASSP instruction is
 * ignored, no FSR error bits are set and WEL state is unchanged.
 * Suspend of this command once initiated is not allowed. This command is
 * invalid while the Microcode is running or the device is suspended.
 * The PASSP command affects the FSR.4 and WIP bits in the same manner
 * as any other programming operation. When the PASSP instruction execution
 * is complete, the Write Enable Latch (WEL) bit is reset within tSHSL time.
 * This instruction is supported in all six SPI protocols.
 */
ReturnType FlashPasswordProgram(uint8 *passwordValue)
{
	CharStream char_stream_send;
	uint8 pIns_Addr[5];

	// Step 1: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 2: Initialize the data (Instruction & address only) packet to be sent serially
	char_stream_send.length   = 1;
	char_stream_send.pChar    = pIns_Addr;
	pIns_Addr[0]              = SPI_FLASH_PASSP;

	// Step 3: Send the packet (Instruction only) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsInitTransfer);

	// Step 4: Initialize the data (password) packet to be sent serially
	char_stream_send.length   = 8;
	char_stream_send.pChar    = passwordValue;

	// Step 5: Send the packet (data to be programmed) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 6: Disabling the Write
	fdo->GenOp.WriteDisable();

	// Step 7: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}

/*
 * FlashPasswordUnlock()
 *
 * passwordValue: IN parameter, pointer to 8 bytes
 *
 * Command details:
 *
 * The PASSU command is entered by driving S# to the logic low state, followed
 * by the 8-bit instruction, then followed by the 64-bit password data, least
 * significant byte first, most significant bit of each byte first.
 * PASSU command does not require WEL bit to be set with WREN command.
 * Chip Select (S#) must be driven high after the eighth bit of the last
 * password byte has been latched in, otherwise the PASSU instruction is ignored,
 * no FSR error bits are set and WEL state is unchanged.
 * Suspend of this command once initiated is not allowed.
 * If the password provided in the unlock sequence does not match the hidden
 * password programmed by the PASSP command, an error is reported by setting
 * the FSR.4 bit to ‘1’.
 * This instruction is supported in all six SPI protocols.
 *
 */

ReturnType FlashPasswordUnlock(uint8 *passwordValue)
{
	CharStream char_stream_send;
	uint8 pIns_Addr[5];

	// Step 4: Initialize the data (Instruction & address only) packet to be sent serially
	char_stream_send.length   = 1;
	char_stream_send.pChar    = pIns_Addr;
	pIns_Addr[0]              = SPI_FLASH_PASSU;

	// Step 5: Send the packet (Instruction only) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsInitTransfer);

	// Step 6: Initialize the data (password) packet to be sent serially
	char_stream_send.length   = 8;
	char_stream_send.pChar    = passwordValue;

	// Step 7: Send the packet (data to be programmed) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 3: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}

/*
 * FlashPermanentProtectionBitRead()
 *
 * uscSectorNr: IN parameter, 4 bytes
 * ppbRegisterValue: OUT parameter, 1 bytes
 *
 * Command details:
 *
 * The contents of non-volatile PPB Register are read with the PPBRD instruction.
 * After the 8-bit instruction shifted in, the 32-bit address selecting location
 * zero within the desired sector. Then the 8-bit PPB register contents are
 * shifted out.
 * PPB read always requires a 32-bit address despite of the address
 * configuration. The high order address bits not used by a particular
 * density device must be zero.
 * The PPBRD instruction is terminated by driving Chip Select (S#) high at any
 * time during data output. When read continuously, the device outputs the same
 * byte repeatedly.
 * This instruction is supported in all six SPI protocols.
 *
 */
ReturnType FlashPermanentProtectionBitRead(uint32 uscSectorNr, uint8 *ppbRegisterValue)
{
	CharStream char_stream_send, char_stream_recv;
	uAddrType SectorAddr;
	uint8 pIns_Addr[5];

	// Step 1: Validate the sector number input
	if(!(uscSectorNr < fdo->Desc.FlashSectorCount))
		return Flash_SectorNrInvalid;

	SectorAddr = fdo->GenOp.BlockOffset(uscSectorNr);

	// Step 2: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 3: Initialize the data (Instruction & address) packet to be sent serially

	/*
	 * PPB read always requires a 32-bit address despite of the address
	 * configuration. The high order address bits not used by a particular
	 * density device must be zero.
	 */
	char_stream_send.length   = 5;
	char_stream_send.pChar    = pIns_Addr;

	pIns_Addr[0]              = SPI_FLASH_PPBRD;
	fill_addr_vect(SectorAddr, pIns_Addr, FLASH_4_BYTE_ADDR_MODE);

	char_stream_recv.length   = 1;
	char_stream_recv.pChar    = ppbRegisterValue;

	// Step 4: Send the packet (Instruction & address) serially
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 5: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}

/*
 *
 * FlashPermanentProtectionBitProgram()
 *
 * uscSectorNr: IN parameter, 32 bits
 *
 * The PPB Register Program (PPBP) instruction is used to set non-volatile
 * be set with WREN command, otherwise instruction is ignored and no FSR
 * error bits are set.
 * The PPBP command is entered by driving S# to the logic low state,
 * individual sector lock-bit. Before it can be accepted, the WEL bit must
 * followed by the 8-bit instruction, followed by 32-bit address, most
 * significant byte first, selecting location zero within the desired
 * sector.
 * PPB write always requires a 32-bit address despite of the address
 * configuration. The high order address bits not used by a particular
 * density device must be zero.
 * Chip Select (S#) must be driven high after the eighth bit of the last
 * address byte has been latched in, otherwise the PPBP instruction is
 * ignored, no FSR error bits are set and WEL state is unchanged.
 * Suspend of this command once initiated is not allowed.
 * While the operation is in progress, status registers may be polled
 * for device status. The Write In Progress (WIP) bit is ‘1’ while the
 * operation is in progress, and is ‘0’ when the operation completes.
 * If the operation fails due to program time-out, the program error
 * bit is set. If PPBP is issued to a sector that has it’s PPB already
 * programmed (locked), the operation will end quickly and WEL bit is
 * cleared without setting any error bits.
 * The PPB protection settings take effect immediately after the PPBP
 * operation is complete. When the PPBP instruction execution is complete,
 * the Write Enable Latch (WEL) bit is reset within tSHSL time.
 * This instruction is supported in all six SPI protocols.
 */

ReturnType FlashPermanentProtectionBitProgram(uint32 uscSectorNr)
{

	CharStream char_stream_send;
	uAddrType SectorAddr;
	uint8 pIns_Addr[5];

	// Step 1: Validate the sector number input
	if(!(uscSectorNr < fdo->Desc.FlashSectorCount))
		return Flash_SectorNrInvalid;

	SectorAddr = fdo->GenOp.BlockOffset(uscSectorNr);

	// Step 2: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 3: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 4: Initialize the data (Instruction & address) packet to be sent serially

	/*
	 * PPB write always requires a 32-bit address despite of the address configuration.
	 * The high order address bits not used by a particular density device must be zero.
	 */
	char_stream_send.length   = 5;
	char_stream_send.pChar    = pIns_Addr;

	pIns_Addr[0]              = SPI_FLASH_PPBP;
	fill_addr_vect(SectorAddr, pIns_Addr, FLASH_4_BYTE_ADDR_MODE);

	// Step 5: Send the packet (Instruction & address) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	fdo->GenOp.WriteDisable();

	// Step 6: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}


/*
 * FlashPermanentProtectionBitErase()
 *
 * The PPB Erase (PPBE) instruction is used to unlock all PPB locked sectors
 * simultaneously, this erases all non-volatile sector-lock bits. Once
 * unlocked, all sectors can be erased or programmed again. Before PPBE can
 * be accepted, the WEL bit must be set with WREN command, otherwise
 * instruction is ignored and no FSR error bits are set.
 * Chip Select (S#) must be driven high after the eighth bit of the
 * instruction byte has been latched in, otherwise the PPBP instruction is
 * ignored, no FSR error bits are set and WEL state is unchanged.
 * Suspending the PPBE Command once initiated is not allowed. This command
 * is invalid while the WSM is busy or the device is suspended.
 * While the operation is in progress, status registers may be polled to
 * check the status of the operation. The Write In Progress (WIP) bit is ‘1’
 * while the operation is in progress, and is cleared (0) when the operation
 * completes. If the erase operation times out without success the erase
 * error bit FSR.5 is set and WEL is cleared.
 * When the PPBE instruction execution is complete, the Write Enable Latch
 * (WEL) bit is reset within tSHSL time.
 * This instruction is supported in all six SPI protocols.
 */
ReturnType FlashPermanentProtectionBitErase()
{
	CharStream char_stream_send;
	uAddrType SectorAddr;
	uint8 pIns_Addr[5];

	// Step 1: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 2: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 3: Initialize the data (Instruction & address) packet to be sent serially

	char_stream_send.length   = 1;
	char_stream_send.pChar    = pIns_Addr;

	pIns_Addr[0]              = SPI_FLASH_PPBE;

	// Step 4: Send the packet (Instruction & address) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	fdo->GenOp.WriteDisable();

	// Step 5: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}

/*
 * The PPB Lock Bit Program (PLBWR) instruction is used to clear the PPB Lock Bit
 * to ‘0’. Before it can be accepted, the WEL bit must be set with WREN command,
 * otherwise instruction is ignored and no FSR error bits are set.
 * The PLBWR command is entered by driving S# to the logic low state, followed
 * only by the 8-bit instruction.
 * Chip Select (S#) must be driven high after the eighth bit of the instruction
 * byte has been latched in, otherwise the PLBWR instruction is ignored, no FSR
 * error bits are set and WEL state is unchanged.
 * PPB Lock Bit is volatile, and therefore writes to it take effect immediately.
 * When the PPBP instruction execution is complete, the Write Enable Latch (WEL)
 * bit is reset within tSHSL time.
 * This instruction is supported in all six SPI protocols.
 */
ReturnType FlashWriteGlobalFreezeBit(void)
{
	CharStream char_stream_send;
	uAddrType SectorAddr;
	uint8 pIns_Addr[5];

	// Step 1: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 2: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 3: Initialize the data (Instruction & address) packet to be sent serially
	char_stream_send.length   = 1;
	char_stream_send.pChar    = pIns_Addr;

	pIns_Addr[0]              = SPI_FLASH_PLBWR;

	// Step 4: Send the packet (Instruction & address) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	fdo->GenOp.WriteDisable();

	// Step 5: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}

/*
 * The contents of PPB Lock Register are read with the PLBRD instruction. After
 * the 8-bit instruction shifted in, the 8-bit data are shifted out, the valid
 * PPB Lock Bit is presented only on bit0, the Unused PLB bits [7:1] read out
 * as ‘0’.
 * The PLBRD instruction is terminated by driving Chip Select (S#) high at any
 * time during data output. When read continuously, the device outputs the same
 * byte repeatedly.
 * This instruction is supported only in Extended-SPI and Extended-DTR-SPI.
 */
ReturnType FlashReadGlobalFreezeBit(uint8 *globalfreezeBitValue)
{
	CharStream char_stream_send, char_stream_recv;
	uAddrType SectorAddr;
	uint8 pIns_Addr[5];

	// Step 1: Check whether any previous Write, Program or Erase cycle is on going
	if(IsFlashBusy()) return Flash_OperationOngoing;

	// Step 2: Initialize the data (Instruction & address) packet to be sent serially
	char_stream_send.length   = 1;
	char_stream_send.pChar    = pIns_Addr;

	pIns_Addr[0]              = SPI_FLASH_PLBRD;

	char_stream_recv.length   = 1;
	char_stream_recv.pChar    = globalfreezeBitValue;

	// Step 3: Send the packet (Instruction & address) serially
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 4: Wait until the operation completes or a timeout occurs.
	return WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);
}


/*******************************************************************************
Function: FlashOTPProgram

Arguments: *pArray, udNrOfElementsInArray

Return Values: ReturnType

Description: Write the entire OTP area of the device, and lock it.
	If pArray does not fill the 64 bytes available the used portion
	is set to zero (0).

Note:

Pseudo Code:

*******************************************************************************/
ReturnType FlashOTPProgram(uint8 *pArray , uint32 udNrOfElementsInArray)
{
	CharStream char_stream_send;
	uint8 i;
	uint8 pIns_Addr[5];
	uint8 sendBuffer[fdo->Desc.FlashOTPSize + 1];
	uint32 udAddr;
	ReturnType ret;

	// Step 1: Validate address input
	if(udNrOfElementsInArray > fdo->Desc.FlashOTPSize)
		return Flash_AddressInvalid;

	/* Address is always 0x000000 */
	udAddr = 0x000000;

	/* Output buffer (with user data inside) is fixed to 65 elements */
	for(i = 0; i < udNrOfElementsInArray; i++)
		sendBuffer[i] = pArray[i];

	/* Fill the others bytes with 00 */
	for(i = udNrOfElementsInArray; i < fdo->Desc.FlashOTPSize; i++)
		sendBuffer[i] = 0x00;

	/* This is the byte 64, OTP Control byte (if bit 0 = 0 -> OTP Locked) */
	sendBuffer[fdo->Desc.FlashOTPSize] = 0;

	// Step 2: Check whether any previous Write, Program or Erase cycle is on-going
	if(IsFlashBusy())
		return Flash_OperationOngoing;

	// Step 3: Disable Write protection
	if(FlashWriteEnable() == Flash_OperationTimeOut)
		return Flash_OperationTimeOut;

	// Step 4: Initialize the data (Instruction & address only) packet to be sent serially
	char_stream_send.length   = fdo->Desc.NumAddrByte + 1;
	char_stream_send.pChar    = pIns_Addr;

	pIns_Addr[0]              = SPI_FLASH_INS_PROTP;

	/* Always use 3 bytes address and address is 0x000000 */
	fill_addr_vect(udAddr, pIns_Addr, 3);

	// Step 5: Send the packet (Instruction & address only) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsInitTransfer);

	// Step 6: Initialize the data (data to be programmed) packet to be sent serially
	char_stream_send.length   = udNrOfElementsInArray;
	char_stream_send.pChar    = pArray;

	// Step 7: Send the packet (data to be programmed) serially
	Serialize_SPI(&char_stream_send,
	              NULL_PTR,
	              OpsWakeUp,
	              OpsEndTransfer);

	// Step 8: Wait until the operation completes or a timeout occurs.
	ret = WAIT_TILL_Instruction_EXECUTION_COMPLETE(1);

	return ret;
}

/*******************************************************************************
Function: FlashOTPRead

Arguments: *ucpElements, udNrOfElementsToRead

Return Values: ReturnType

Description:

Note:

Pseudo Code:

*******************************************************************************/
ReturnType FlashOTPRead(uint8 *ucpElements, uint32 udNrOfElementsToRead)
{
	CharStream char_stream_send;
	CharStream char_stream_recv;
	uint8  pIns_Addr[5];
	uint32 udAddr;

	/* Address is always 0x000000 */
	udAddr = 0x000000;

	// Step 2: Initialize the data (i.e. Instruction) packet to be sent serially
	char_stream_send.length   = fdo->Desc.NumAddrByte + 1;
	char_stream_send.pChar    = pIns_Addr;
	pIns_Addr[0]              = SPI_FLASH_INS_RDOTP;

	fill_addr_vect(udAddr, pIns_Addr, 3);

	char_stream_recv.length   = udNrOfElementsToRead;
	char_stream_recv.pChar    = ucpElements;

	// Step 3: Send the packet serially, and fill the buffer with the data being returned
	Serialize_SPI(&char_stream_send,
	              &char_stream_recv,
	              OpsWakeUp,
	              OpsEndTransfer);

	return Flash_Success;
}

