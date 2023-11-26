/*
 * MFRC522.cpp - Library to use ARDUINO RFID MODULE KIT 13.56 MHZ WITH TAGS SPI W AND R BY COOQROBOT.
 * NOTE: Please also check the comments in MFRC522.h - they provide useful hints and background information.
 * Released into the public domain.
 */

#include "MFRC522.h"
#include <stdint.h>
#include <stdio.h>

// Size of the MFRC522 FIFO
const byte FIFO_SIZE = 64;		// The FIFO is 64 bytes.

const byte MFRC522_firmware_referenceV1_0[]  = {
	0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
	0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
	0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
	0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
	0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
	0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
	0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
	0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
};

// Version 2.0
const byte MFRC522_firmware_referenceV2_0[] = {
	0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
	0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
	0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
	0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
	0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
	0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
	0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
	0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
};

/**
 * Constructor.
 * Prepares the output pins.
 */
void MFRC522_create(MFRC522* M, byte sck, byte mosi, byte miso, byte rst, byte cs, dword baudrate, byte spi_id) {
    M->DEBUG = false;
    M->sck_pin = sck;
    M->mosi_pin = mosi;
    M->miso_pin = miso;
    M->rst_pin = rst;
    M->cs_pin = cs;
    M->baudrate = baudrate;
    M->spi_id = spi_id;

    gpio_init(M->rst_pin);
    gpio_set_dir(M->rst_pin, GPIO_OUT);
    gpio_put(M->rst_pin, 0);
    
    spi_init(spi_default, M->baudrate);
    
    gpio_set_function(M->sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(M->mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(M->miso_pin, GPIO_FUNC_SPI);

    gpio_init(M->cs_pin);
    gpio_set_dir(M->cs_pin, GPIO_OUT);    
    gpio_put(M->cs_pin, 1);
} // End constructor

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

//chip select by val. 0->active : 1->off
void MFRC522_cs(MFRC522* M, uint8_t val) {
    gpio_put(M->cs_pin, val);
}

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegister(MFRC522 *M,	
          byte reg,		  ///< The register to write to. One of the PCD_Register enums.
					byte value		///< The value to write.
					) {

  char data[2];
  data[0] = reg & 0x7E;
  data[1] = value;
  
  MFRC522_cs(M, 0);
  spi_write_blocking(spi_default, data, 2);
  MFRC522_cs(M, 1);
} // End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegisters(MFRC522 *M,	
          byte reg,		  ///< The register to write to. One of the PCD_Register enums.
					byte count,		///< The number of bytes to write to the register
					byte *values	///< The values to write. Byte array.
					) {
  for (byte index = 0; index < count; index++) {
  	PCD_WriteRegister(M, reg, values[index]);
	}

} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
byte PCD_ReadRegister(MFRC522 *M,
        	byte reg	///< The register to read from. One of the PCD_Register enums.
				) {
  
  char data[2];
  data[0] = 0x80 | ((reg) & 0x7E);
  MFRC522_cs(M, 0);
  spi_write_blocking(spi_default, &data[0], 1);
  spi_read_blocking(spi_default, 0, &data[1], 1);
  MFRC522_cs(M, 1);
  return (byte)data[1];
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_ReadRegisters(MFRC522 *M,
        byte reg,		  ///< The register to read from. One of the PCD_Register enums.
				byte count,		///< The number of bytes to read
				byte *values,	///< Byte array to store the values in.
				byte rxAlign	///< Only bit positions rxAlign..7 in values[0] are updated.
				) {
  if (count == 0) {
    return;
  }
  //Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));
  byte address = 0x80 | (reg & 0x7E);		// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  byte index = 0;							// Index in values array.
  count--;								// One read is performed outside of the loop
  MFRC522_cs(M, 0);
  spi_write_blocking(spi_default, &address, 1);
  while (index < count) {
    if (index == 0 && rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
      // Create bit mask for bit positions rxAlign..7
      byte mask = 0;
      for (byte i = rxAlign; i <= 7; i++) {
	      mask |= (1 << i);
      }
      // Read value and tell that we want to read the same address again.
      byte value;
      spi_write_read_blocking(spi_default, &address, &value, 1);
      // Apply mask to both current value of values[0] and the new data in value.
      values[0] = (values[index] & ~mask) | (value & mask);
    }
    else { // Normal case
      spi_write_read_blocking(spi_default, &address, values+index, 1);
    }
    index++;
  }
  spi_write_read_blocking(spi_default, 0, values+index, 1);
  MFRC522_cs(M, 1);
  /*printf("regs: \n  ");
  for(int i=0;i<=count;i++) {
    printf("%x, ", values[i]);
  }
  printf("\n");*/
} // End PCD_ReadRegister()
/**
 * Sets the bits given in mask in register reg.
 */
void PCD_SetRegisterBitMask(MFRC522 *M,	
          byte reg,	///< The register to update. One of the PCD_Register enums.
					byte mask	///< The bits to set.
					) { 
  byte tmp;
  tmp = PCD_ReadRegister(M, reg);
  PCD_WriteRegister(M, reg, tmp | mask);			// set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void PCD_ClearRegisterBitMask(MFRC522 *M,
        	byte reg,	///< The register to update. One of the PCD_Register enums.
					byte mask	///< The bits to clear.
					) {
  byte tmp;
  tmp = PCD_ReadRegister(M, reg);
  PCD_WriteRegister(M, reg, tmp & (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte PCD_CalculateCRC(MFRC522 *M,
      	byte *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
				byte length,	///< In: The number of bytes to transfer.
				byte *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
				) {
  PCD_WriteRegister(M, CommandReg, PCD_Idle);		// Stop any active command.
  PCD_WriteRegister(M, DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
  PCD_SetRegisterBitMask(M, FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
  PCD_WriteRegisters(M, FIFODataReg, length, data);	// Write data to the FIFO
  PCD_WriteRegister(M, CommandReg, PCD_CalcCRC);		// Start the calculation
	
  // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73�s.
  word i = 5000;
  byte n;
  while (1) {
    n = PCD_ReadRegister(M, DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
    if (n & 0x04) {						// CRCIRq bit set - calculation done
      break;
    }
    if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
      return STATUS_TIMEOUT;
    }
  }
  PCD_WriteRegister(M, CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.
	
  // Transfer the result from the registers to the result buffer
  result[0] = PCD_ReadRegister(M, CRCResultRegL);
  result[1] = PCD_ReadRegister(M, CRCResultRegH);
  return STATUS_OK;
} // End PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void PCD_Init(MFRC522 *M) {
  gpio_set_dir(M->rst_pin, GPIO_IN);
  if (gpio_get(M->rst_pin) == 0) {	//The MFRC522 chip is in power down mode.
    gpio_set_dir(M->rst_pin, GPIO_OUT);
    gpio_put(M->rst_pin, 1);		// Exit power down mode. This triggers a hard reset.
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
    sleep_ms(50);
  }
  else { // Perform a soft reset
    PCD_Reset(M);
  }

  PCD_WriteRegister(M, TxModeReg, 0x00);
  PCD_WriteRegister(M, RxModeReg, 0x00);
  PCD_WriteRegister(M, ModWidthReg, 0x26);
	
  // When communicating with a PICC we need a timeout if something goes wrong.
  // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
  // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
  PCD_WriteRegister(M, TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
  PCD_WriteRegister(M, TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25�s.
  PCD_WriteRegister(M, TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
  PCD_WriteRegister(M, TReloadRegL, 0xE8);
	
  PCD_WriteRegister(M, TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
  PCD_WriteRegister(M, ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
  PCD_AntennaOn(M);						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void PCD_Reset(MFRC522 *M) {
  PCD_WriteRegister(M, CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
  // The datasheet does not mention how long the SoftRest command takes to complete.
  // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
  // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
  sleep_ms(50);
  // Wait for the PowerDown bit in CommandReg to be cleared
  while (PCD_ReadRegister(M, CommandReg) & (1<<4)) {
    printf("do reset...\n");
    // PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
  }
} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void PCD_AntennaOn(MFRC522 *M) {
  byte value = PCD_ReadRegister(M, TxControlReg);
  if ((value & 0x03) != 0x03) {
    PCD_WriteRegister(M, TxControlReg, value | 0x03);
  }
} // End PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void PCD_AntennaOff(MFRC522 *M) {
  PCD_ClearRegisterBitMask(M, TxControlReg, 0x03);
} // End PCD_AntennaOff()



/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////


/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
bool PICC_IsNewCardPresent(MFRC522 *M) {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  byte result = PICC_RequestA(M, bufferATQA, &bufferSize);
  return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte PICC_RequestA(MFRC522 *M,
      byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
      byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
      ) {
  return PICC_REQA_or_WUPA(M, PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
byte PICC_REQA_or_WUPA(MFRC522 *M,
      byte command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
      byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
      byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
      ) {
  byte validBits;
  byte status;
	
  if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
    return STATUS_NO_ROOM;
  }
  PCD_ClearRegisterBitMask(M, CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
  validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
  status = PCD_TransceiveData(M, &command, 1, bufferATQA, bufferSize, &validBits, 0, false);
  if (status != STATUS_OK) {
    return status;
  }
  if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
    return STATUS_ERROR;
  }
  return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte PCD_TransceiveData(MFRC522 *M,
    byte *sendData,		///< Pointer to the data to transfer to the FIFO.
    byte sendLen,		///< Number of bytes to transfer to the FIFO.
    byte *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
    byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
    byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
    byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
    bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
    ) {
  byte waitIRq = 0x30;		// RxIRq and IdleIRq
  return PCD_CommunicateWithPICC(M, PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()


/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte PCD_CommunicateWithPICC(MFRC522 *M,
      byte command,		///< The command to execute. One of the PCD_Command enums.
      byte waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
      byte *sendData,		///< Pointer to the data to transfer to the FIFO.
      byte sendLen,		///< Number of bytes to transfer to the FIFO.
      byte *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
      byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
      byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
      byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
      bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
      ) {
  byte n, _validBits;
  unsigned int i;
	
  // Prepare values for BitFramingReg
  byte txLastBits = validBits ? *validBits : 0;
  byte bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
  PCD_WriteRegister(M, CommandReg, PCD_Idle);			// Stop any active command.
  PCD_WriteRegister(M, ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
  PCD_SetRegisterBitMask(M, FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
  PCD_WriteRegisters(M, FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
  PCD_WriteRegister(M, BitFramingReg, bitFraming);		// Bit adjustments
  PCD_WriteRegister(M, CommandReg, command);				// Execute the command
  if (command == PCD_Transceive) {
    PCD_SetRegisterBitMask(M, BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
  }
	
  // Wait for the command to complete.
  // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
  // Each iteration of the do-while-loop takes 17.86�s.
  i = 2000;
  while (1) {
    n = PCD_ReadRegister(M, ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
    if (n & waitIRq) {					// One of the interrupts that signal success has been set.
      break;
    }
    if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
      return STATUS_TIMEOUT;
    }
    if (--i == 0) {						// The emergency break. If all other condions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
      return STATUS_TIMEOUT;
    }
  }
	
  // Stop now if any errors except collisions were detected.
  byte errorRegValue = PCD_ReadRegister(M, ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
  if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
    return STATUS_ERROR;
  }	

  // If the caller wants data back, get it from the MFRC522.
  if (backData && backLen) {
    n = PCD_ReadRegister(M, FIFOLevelReg);			// Number of bytes in the FIFO
    if (n > *backLen) {
      return STATUS_NO_ROOM;
    }
    *backLen = n;											// Number of bytes returned
    PCD_ReadRegisters(M, FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
    _validBits = PCD_ReadRegister(M, ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
    if (validBits) {
      *validBits = _validBits;
    }
  }
	
  // Tell about collisions
  if (errorRegValue & 0x08) {		// CollErr
    return STATUS_COLLISION;
  }
	
  // Perform CRC_A validation if requested.
  if (backData && backLen && checkCRC) {
    // In this case a MIFARE Classic NAK is not OK.
    if (*backLen == 1 && _validBits == 4) {
      return STATUS_MIFARE_NACK;
    }
    // We need at least the CRC_A value and all 8 bits of the last byte must be received.
    if (*backLen < 2 || _validBits != 0) {
      return STATUS_CRC_WRONG;
    }
    // Verify CRC_A - do our own calculation and store the control in controlBuffer.
    byte controlBuffer[2];
    n = PCD_CalculateCRC(M, &backData[0], *backLen - 2, &controlBuffer[0]);
    if (n != STATUS_OK) {
      return n;
    }
    if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
      return STATUS_CRC_WRONG;
    }
  }
  return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return bool
 */
bool PICC_ReadCardSerial(MFRC522 *M) {
  byte result = PICC_Select(M, 0);
  return (result == STATUS_OK);
} // End PICC_ReadCardSerial()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte PICC_Select(MFRC522 *M, ///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
				byte validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
				) {
  bool uidComplete;
  bool selectDone;
  bool useCascadeTag;
  byte cascadeLevel = 1;
  byte result;
  byte count;
  byte index;
  byte uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
  signed char currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
  byte buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
  byte bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
  byte rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
  byte txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
  byte *responseBuffer;
  byte responseLength;
	
  // Description of buffer structure:
  //		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
  //		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
  //		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
  //		Byte 3: UID-data
  //		Byte 4: UID-data
  //		Byte 5: UID-data
  //		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
  //		Byte 7: CRC_A
  //		Byte 8: CRC_A
  // The BCC and CRC_A is only transmitted if we know all the UID bits of the current Cascade Level.
  //
  // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
  //		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
  //		========	=============	=====	=====	=====	=====
  //		 4 bytes		1			uid0	uid1	uid2	uid3
  //		 7 bytes		1			CT		uid0	uid1	uid2
  //						    2			uid3	uid4	uid5	uid6
  //		10 bytes		1			CT		uid0	uid1	uid2
  //						    2			CT		uid3	uid4	uid5
  //						    3			uid6	uid7	uid8	uid9
	
  // Sanity checks
  if (validBits > 80) {
    return STATUS_INVALID;
  }
	
  // Prepare MFRC522
  PCD_ClearRegisterBitMask(M, CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
  // Repeat Cascade Level loop until we have a complete UID.
  uidComplete = false;
  while (!uidComplete) {
    // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
    switch (cascadeLevel) {
      case 1:
        buffer[0] = PICC_CMD_SEL_CL1;
        uidIndex = 0;
        useCascadeTag = validBits && M->uid.size > 4;	// When we know that the UID has more than 4 bytes
        break;
        
      case 2:
        buffer[0] = PICC_CMD_SEL_CL2;
        uidIndex = 3;
        useCascadeTag = validBits && M->uid.size > 7;	// When we know that the UID has more than 7 bytes
        break;
        
      case 3:
        buffer[0] = PICC_CMD_SEL_CL3;
        uidIndex = 6;
        useCascadeTag = false;						// Never used in CL3.
        break;
        
      default:
        return STATUS_INTERNAL_ERROR;
        break;
    }
		
    // How many UID bits are known in this Cascade Level?
    currentLevelKnownBits = validBits - (8 * uidIndex);
    if (currentLevelKnownBits < 0) {
      currentLevelKnownBits = 0;
    }
    // Copy the known bits from uid->uidByte[] to buffer[]
    index = 2; // destination index in buffer[]
    if (useCascadeTag) {
      buffer[index++] = PICC_CMD_CT;
    }
    byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
    if (bytesToCopy) {
      byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
      if (bytesToCopy > maxBytes) {
	      bytesToCopy = maxBytes;
      }
      for (count = 0; count < bytesToCopy; count++) {
	      buffer[index++] = M->uid.uidByte[uidIndex + count];
      }
    }
    // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
    if (useCascadeTag) {
      currentLevelKnownBits += 8;
    }
    
    // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
    selectDone = false;
    while (!selectDone) {
      // Find out how many bits and bytes to send and receive.
      if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
        //Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
        buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
        // Calculate BCC - Block Check Character
        buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
        // Calculate CRC_A
        result = PCD_CalculateCRC(M, buffer, 7, &buffer[7]);
        if (result != STATUS_OK) {
          return result;
        }
        txLastBits		= 0; // 0 => All 8 bits are valid.
        bufferUsed		= 9;
        // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
        responseBuffer	= &buffer[6];
        responseLength	= 3;
      } else { // This is an ANTICOLLISION.
        //Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
        txLastBits		= currentLevelKnownBits % 8;
        count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
        index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
        buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
        bufferUsed		= index + (txLastBits ? 1 : 0);
        // Store response in the unused part of buffer
        responseBuffer	= &buffer[index];
        responseLength	= sizeof(buffer) - index;
      }
			
      // Set bit adjustments
      rxAlign = txLastBits;											// Having a seperate variable is overkill. But it makes the next line easier to read.
      PCD_WriteRegister(M, BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			// Transmit the buffer and receive the response.
      result = PCD_TransceiveData(M, buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
      if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
	      result = PCD_ReadRegister(M, CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
        if (result & 0x20) { // CollPosNotValid    
          return STATUS_COLLISION; // Without a valid collision position we cannot continue
        }
        byte collisionPos = result & 0x1F; // Values 0-31, 0 means bit 32.
        if (collisionPos == 0) {
          collisionPos = 32;
        }
        if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen     
          return STATUS_INTERNAL_ERROR;
        }
        // Choose the PICC with the bit set.
        currentLevelKnownBits = collisionPos;
        count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
        index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
        buffer[index]	|= (1 << count);
      } else if (result != STATUS_OK) {
        return result;
      } else { // STATUS_OK
        if (currentLevelKnownBits >= 32) { // This was a SELECT.
          selectDone = true; // No more anticollision 
          // We continue below outside the while.
        } else { // This was an ANTICOLLISION.
        // We now have all 32 bits of the UID in this Cascade Level
          currentLevelKnownBits = 32;
        // Run loop again to do the SELECT.
	      }
      }
    } // End of while (!selectDone)
		
    // We do not check the CBB - it was constructed by us above.
		
    // Copy the found UID bytes from buffer[] to uid->uidByte[]
    index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
    bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
    for (count = 0; count < bytesToCopy; count++) {
      M->uid.uidByte[uidIndex + count] = buffer[index++];
    }
		
    // Check response SAK (Select Acknowledge)
    if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).    
      return STATUS_ERROR;
    }
    // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
    result = PCD_CalculateCRC(M, responseBuffer, 1, &buffer[2]);
    if (result != STATUS_OK) {    
      return result;
    }
    if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
      return STATUS_CRC_WRONG;
    }
    if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
      cascadeLevel++;
    }
    else {
      uidComplete = true;
      M->uid.sak = responseBuffer[0];
    }
  } // End of while (!uidComplete)
	
  // Set correct uid->size
  M->uid.size = 3 * cascadeLevel + 1;
	return STATUS_OK;
} // End PICC_Select()

 
