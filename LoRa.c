//
// Created by MMKH on 10/23/2021.
//

#include "LoRa.h"

#define RESET_PIN 117
#define NSS_PIN 112
#define DIO0_PIN 129

#define SPI_BUS 0


/* ----------------------------------------------------------------------------- *\
		name        : newLoRa
		description : it's a constructor for LoRa structure that assign default values
									and pass created object (LoRa struct instanse)
		arguments   : Nothing
		returns     : A LoRa object whith these default values:
											----------------------------------------
										  |   carrier frequency = 433 MHz        |
										  |    spreading factor = 7				       |
											|           bandwidth = 125 KHz        |
											| 		    coding rate = 4/5            |
											----------------------------------------
\* ----------------------------------------------------------------------------- */
LoRa newLoRa(irq_handler_t handler) {
    LoRa new_LoRa;

    new_LoRa.frequency = 433;
    new_LoRa.spredingFactor = SF_7;
    new_LoRa.bandWidth = BW_500KHz;
    new_LoRa.crcRate = CR_4_5;
    new_LoRa.power = POWER_20db;
    new_LoRa.overCurrentProtection = 120;
    new_LoRa.preamble = 6;

    HAL_Init(SPI_BUS);

    HAL_GPIO_SetupPin(RESET_PIN, 1);
    HAL_GPIO_SetupPin(NSS_PIN, 1);
    HAL_GPIO_SetupPin(DIO0_PIN, 0);
    HAL_GPIO_IRQPin(DIO0_PIN, handler);

    return new_LoRa;
}

void LoRa_end(LoRa *_LoRa) {
    HAL_GPIO_FreeIRQ(DIO0_PIN);
    HAL_GPIO_FreePin(RESET_PIN);
    HAL_GPIO_FreePin(NSS_PIN);
    HAL_GPIO_FreePin(DIO0_PIN);

    HAL_End();
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_reset
		description : reset module
		arguments   :
			LoRa* LoRa --> LoRa object handler
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_reset(LoRa *_LoRa) {
    HAL_GPIO_WritePin(RESET_PIN, 0);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RESET_PIN, 1);
    HAL_Delay(100);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_gotoMode
		description : set LoRa Op mode
		arguments   :
			LoRa* LoRa    --> LoRa object handler
			mode	        --> select from defined modes
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_gotoMode(LoRa *_LoRa, int mode) {
    uint8_t read;
    uint8_t data;

    read = LoRa_read(_LoRa, RegOpMode);
    data = read;

    if (mode == SLEEP_MODE) {
        data = (read & 0xF8) | 0x00;
        _LoRa->current_mode = SLEEP_MODE;
    } else if (mode == STNBY_MODE) {
        data = (read & 0xF8) | 0x01;
        _LoRa->current_mode = STNBY_MODE;
    } else if (mode == TRANSMIT_MODE) {
        data = (read & 0xF8) | 0x03;
        _LoRa->current_mode = TRANSMIT_MODE;
    } else if (mode == RXCONTIN_MODE) {
        data = (read & 0xF8) | 0x05;
        _LoRa->current_mode = RXCONTIN_MODE;
    } else if (mode == RXSINGLE_MODE) {
        data = (read & 0xF8) | 0x06;
        _LoRa->current_mode = RXSINGLE_MODE;
    }

    LoRa_write(_LoRa, RegOpMode, data);
//    HAL_Delay(10);
}


/* ----------------------------------------------------------------------------- *\
		name        : LoRa_readReg
		description : read a register(s) by an address and a length,
									then store value(s) at outpur array.
		arguments   :
			LoRa* LoRa        --> LoRa object handler
			uint8_t* address  -->	pointer to the beginning of address array
			uint16_t r_length -->	detemines number of addresse bytes that
														you want to send
			uint8_t* output		--> pointer to the beginning of output array
			uint16_t w_length	--> detemines number of bytes that you want to read
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_readReg(LoRa *_LoRa, uint8_t *address, uint16_t r_length, uint8_t *output, uint16_t w_length) {
    HAL_GPIO_WritePin(NSS_PIN, 0);
//    HAL_Delay(1);
    HAL_SPI_Transmit(address, r_length);
    //HAL_Delay(2);
//    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
//        ;
    HAL_SPI_Receive(output, w_length);
    //HAL_Delay(2);
//    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
//        ;
    HAL_GPIO_WritePin(NSS_PIN, 1);
//    HAL_Delay(1);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_writeReg
		description : write a value(s) in a register(s) by an address
		arguments   :
			LoRa* LoRa        --> LoRa object handler
			uint8_t* address  -->	pointer to the beginning of address array
			uint16_t r_length -->	detemines number of addresse bytes that
														you want to send
			uint8_t* output		--> pointer to the beginning of values array
			uint16_t w_length	--> detemines number of bytes that you want to send
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_writeReg(LoRa *_LoRa, uint8_t *address, uint16_t r_length, uint8_t *values, uint16_t w_length) {
    HAL_GPIO_WritePin(NSS_PIN, 0);
//    HAL_Delay(1);
    HAL_SPI_Transmit(address, r_length);
    // HAL_Delay(2);
//    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
//        ;
    HAL_SPI_Transmit(values, w_length);
    //  HAL_Delay(2);
//    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
//        ;
    HAL_GPIO_WritePin(NSS_PIN, 1);
//    HAL_Delay(1);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setFrequency
		description : set carrier frequency e.g 433 MHz
		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   freq        --> desired frequency in MHz unit, e.g 434
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setFrequency(LoRa *_LoRa, int freq) {
    uint8_t data;
    uint32_t F;
    F = (freq * 524288) >> 5;

    // write Msb:
    data = F >> 16;
    LoRa_write(_LoRa, RegFrMsb, data);
    HAL_Delay(5);

    // write Mid:
    data = F >> 8;
    LoRa_write(_LoRa, RegFrMid, data);
    HAL_Delay(5);

    // write Lsb:
    data = F >> 0;
    LoRa_write(_LoRa, RegFrLsb, data);
    HAL_Delay(5);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setSpreadingFactor
		description : set spreading factor, from 6 to 12.
		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   SP          --> desired spreading factor e.g 7
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setSpreadingFactor(LoRa *_LoRa, int SF) {
    uint8_t data;
    uint8_t read;

    if (SF > 12)
        SF = 12;
    if (SF < 6)
        SF = 6;

    _LoRa->spredingFactor = SF;

    read = LoRa_read(_LoRa, RegModemConfig2);
    HAL_Delay(10);

    data = (SF << 4) + (read & 0x0F);
    LoRa_write(_LoRa, RegModemConfig2, data);
    HAL_Delay(10);

    // Set additional regs for SF = 6

    // Set implicit/explicit header
    read = LoRa_read(_LoRa, RegModemConfig1);
    HAL_Delay(10);

    data = (SF == 6 ? 1 : 0) + (read & 0xFE);
    LoRa_write(_LoRa, RegModemConfig1, data);
    HAL_Delay(10);

    // Set RegDetectOptimize (0x05 for SF6)
    read = LoRa_read(_LoRa, RegDetectOptimize);
    HAL_Delay(10);

    data = (SF == 6 ? 0x05 : 0x03) + (read & 0xF8);
    LoRa_write(_LoRa, RegDetectOptimize, data);
    HAL_Delay(10);


    // Set RegDetectionThreshold (0x0C for SF6)
    data = (SF == 6 ? 0x0C : 0x0A);
    LoRa_write(_LoRa, RegDetectionThreshold, data);
    HAL_Delay(10);


}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setPower
		description : set power gain.
		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   power       --> desired power e.g POWER_17db
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setPower(LoRa *_LoRa, uint8_t power) {
    LoRa_write(_LoRa, RegPaConfig, power);
    HAL_Delay(10);
}


/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setOCP
		description : set maximum allowed current.
		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   current     --> desired max currnet in mA, e.g 120
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
//void LoRa_setOCP(LoRa* _LoRa, uint8_t current){
//    uint8_t	OcpTrim = 0;
//    uint8_t current2 = current;
//
//    if(current2<45) {
//        current2 = 45;
//    }
//    if(current2>240) {
//        current2 = 240;
//    }
//
//    if(current2 <= 120) {
//        OcpTrim = (current2 - 45) / 5;
//    }
//    else if(current2 <= 240) {
//        OcpTrim = (current2 + 30) / 10;
//    }
//
//    OcpTrim = OcpTrim + (1 << 5);
//    LoRa_write(_LoRa, RegOcp, OcpTrim);
//    HAL_Delay(10);
//}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setTOMsb_setCRCon
		description : set timeout msb to 0xFF + set CRC enable.
		arguments   :
			LoRa* LoRa        --> LoRa object handler
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setTOMsb_setCRCon(LoRa *_LoRa) {
    uint8_t read, data;

    read = LoRa_read(_LoRa, RegModemConfig2);

//    data = read | 0x07;
    // Disable CRC!
    data = (read & 0xF8) | 0x03;
    LoRa_write(_LoRa, RegModemConfig2, data);\
    HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_read
		description : read a register by an address
		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D
		returns     : register value
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_read(LoRa *_LoRa, uint8_t address) {
    uint8_t read_data;
    uint8_t data_addr;

    data_addr = address & 0x7F;
    LoRa_readReg(_LoRa, &data_addr, 1, &read_data, 1);
    // HAL_Delay(5);

    return read_data;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_write
		description : write a value in a register by an address
		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D
			uint8_t value       --> value that you want to write
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_write(LoRa *_LoRa, uint8_t address, uint8_t value) {
    uint8_t data;
    uint8_t addr;

    addr = address | 0x80;
    data = value;
    LoRa_writeReg(_LoRa, &addr, 1, &data, 1);
    //  HAL_Delay(5);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_BurstWrite
		description : write a set of values in a register by an address respectively
		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D
			uint8_t *value      --> address of values that you want to write
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_BurstWrite(LoRa *_LoRa, uint8_t address, uint8_t *value, uint8_t length) {
    uint8_t addr;
    addr = address | 0x80;

    //NSS = 1
    HAL_GPIO_WritePin(NSS_PIN, 0);
//    HAL_Delay(1);
    //say module thai I want to write in RegFiFo
    HAL_SPI_Transmit(&addr, 1);
    //  HAL_Delay(2);
//    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
//        ;

    //Write data in FiFo
    HAL_SPI_Transmit(value, length);

    //  HAL_Delay(2);
//    while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
//        ;
    //NSS = 0
    //HAL_Delay(5);
    HAL_GPIO_WritePin(NSS_PIN, 1);
//    HAL_Delay(1);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_isvalid
		description : check the LoRa instruct values
		arguments   :
			LoRa* LoRa --> LoRa object handler
		returns     : returns 1 if all of the values were given, otherwise returns 0
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_isvalid(LoRa *_LoRa) {

    return 1;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_transmit
		description : Transmit data
		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t  data			--> A pointer to the data you wanna send
			uint8_t	 length   --> Size of your data in Bytes
			uint16_t timeOut	--> Timeout in milliseconds
		returns     : 1 in case of success, 0 in case of timeout
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_transmit(LoRa *_LoRa, uint8_t *data, uint8_t length, uint16_t timeout) {
    uint8_t read;
    uint8_t tmp_data[255];

    int mode = _LoRa->current_mode;
    LoRa_gotoMode(_LoRa, STNBY_MODE);

    memcpy(_LoRa->spredingFactor == 6 ? &tmp_data[1] : tmp_data, data, length);
    if (_LoRa->spredingFactor == 6)
        tmp_data[0] = length;

//    read = LoRa_read(_LoRa, RegFiFoTxBaseAddr);
//    LoRa_write(_LoRa, RegFiFoAddPtr, read);
    LoRa_write(_LoRa, RegFiFoAddPtr, 0);
    LoRa_BurstWrite(_LoRa, RegFiFo, tmp_data, length + (_LoRa->spredingFactor == 6 ? 1 : 0));
    LoRa_write(_LoRa, RegPayloadLength, length + (_LoRa->spredingFactor == 6 ? 1 : 0));
    LoRa_gotoMode(_LoRa, TRANSMIT_MODE);
    while (1) {
        read = LoRa_read(_LoRa, RegIrqFlags);
        if ((read & 0x08) != 0) {
            LoRa_write(_LoRa, RegIrqFlags, 0xFF);
            HAL_Delay(1);
            LoRa_gotoMode(_LoRa, mode);
//            LoRa_write(_LoRa, RegPayloadLength, 0xFF);
            return 1;
        } else {
            if (--timeout == 0) {
                LoRa_gotoMode(_LoRa, mode);
                return 0;
            }
        }
        HAL_Delay(1);
    }

}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_startReceiving
		description : Start receiving continuously
		arguments   :
			LoRa*    LoRa     --> LoRa object handler
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_startReceiving(LoRa *_LoRa) {
    LoRa_gotoMode(_LoRa, RXCONTIN_MODE);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_Receive
		description : Read received data from module
		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t  data			--> A pointer to the array that you want to write bytes in it
			uint8_t	 length   --> Determines how many bytes you want to read
		returns     : The number of bytes received
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_receive(LoRa *_LoRa, uint8_t *data, uint8_t length) {
    uint8_t read;
    uint8_t number_of_bytes;
    uint8_t min = 0;
    int i = 0;
    for (i = 0; i < length; i++)
        data[i] = 0;

    read = LoRa_read(_LoRa, RegIrqFlags);

//    printk(KERN_NOTICE "LoRa REC Flag: %x\n", read);
    if ((read & 0x40) != 0) {
        LoRa_write(_LoRa, RegIrqFlags, 0xFF);

//        if (_LoRa->spredingFactor == 6) {
//            start_addr = LoRa_read(_LoRa, RegFifoRxByteAddr);
//            printk("Start Addr: %x\n", start_addr);
//            while (LoRa_read(_LoRa, RegFifoRxByteAddr) < start_addr + 1) {
//                HAL_Delay(1);
//            }
//            printk("Start Addr2: %x\n", LoRa_read(_LoRa, RegFifoRxByteAddr));
//            LoRa_write(_LoRa, RegFiFoAddPtr, start_addr);
//            number_of_bytes = LoRa_read(_LoRa, RegFiFo);
//            min = length >= number_of_bytes ? number_of_bytes : length;
//            printk("Byte[0] (Size): %d\n", number_of_bytes);
//            while (LoRa_read(_LoRa, RegFifoRxByteAddr) < start_addr + 1 + min) {
//                HAL_Delay(1);
//            }
//            for (i = 0; i < min; i++)
//                data[i] = LoRa_read(_LoRa, RegFiFo);
//        } else {
            LoRa_write(_LoRa, RegFiFoAddPtr, LoRa_read(_LoRa, RegFiFoRxCurrentAddr));
//            LoRa_gotoMode(_LoRa, STNBY_MODE);
            number_of_bytes = LoRa_read(_LoRa, RegRxNbBytes);
            min = length >= number_of_bytes ? number_of_bytes : length;
            for (i = 0; i < min; i++)
                data[i] = LoRa_read(_LoRa, RegFiFo);
//            LoRa_gotoMode(_LoRa, RXCONTIN_MODE);
//        }
    }
    return min;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_getRSSI
		description : initialize and set the right setting according to LoRa sruct vars
		arguments   :
			LoRa* LoRa        --> LoRa object handler
		returns     : Returns the RSSI value of last received packet.
\* ----------------------------------------------------------------------------- */
int LoRa_getRSSI(LoRa *_LoRa) {
    uint8_t read;
    read = LoRa_read(_LoRa, RegPktRssiValue);
    return -164 + read;
}

int LoRa_getInstantRSSI(LoRa *_LoRa) {
    uint8_t read;
    read = LoRa_read(_LoRa, RegRssiValue);
    return -164 + read;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_init
		description : initialize and set the right setting according to LoRa sruct vars
		arguments   :
			LoRa* LoRa        --> LoRa object handler
		returns     : Nothing
\* ----------------------------------------------------------------------------- */
uint16_t LoRa_init(LoRa *_LoRa) {
    uint8_t data;
    uint8_t read;

    if (LoRa_isvalid(_LoRa)) {
        // goto sleep mode:
        LoRa_gotoMode(_LoRa, SLEEP_MODE);
        HAL_Delay(10);

        // turn on lora mode:
        read = LoRa_read(_LoRa, RegOpMode);
        HAL_Delay(10);
        data = read | 0x80;
        LoRa_write(_LoRa, RegOpMode, data);
        HAL_Delay(100);

        // set frequency:
        LoRa_setFrequency(_LoRa, _LoRa->frequency);

        // set base addrs:
        LoRa_write(_LoRa, RegFiFoRxBaseAddr, 0);
        LoRa_write(_LoRa, RegFiFoTxBaseAddr, 0);

        // set output power gain:
        LoRa_setPower(_LoRa, _LoRa->power);

        // set over current protection:
//        LoRa_setOCP(_LoRa, _LoRa->overCurrentProtection);

        // set LNA gain:
        LoRa_write(_LoRa, RegLna, 0x23);

        // set spreading factor, CRC on, and Timeout Msb:
        LoRa_setTOMsb_setCRCon(_LoRa);
        LoRa_setSpreadingFactor(_LoRa, _LoRa->spredingFactor);

        // set Timeout Lsb:
        LoRa_write(_LoRa, RegSymbTimeoutL, 0xFF);

        // set bandwidth, coding rate and expilicit mode:
        // 8 bit RegModemConfig --> | X | X | X | X | X | X | X | X |
        //       bits represent --> |   bandwidth   |     CR    |I/E|
        data = 0;
        data = (_LoRa->bandWidth << 4) + (_LoRa->crcRate << 1) + (_LoRa->spredingFactor == 6 ? 1 : 0);
        LoRa_write(_LoRa, RegModemConfig1, data);

        // set preamble:
        LoRa_write(_LoRa, RegPreambleMsb, _LoRa->preamble >> 8);
        LoRa_write(_LoRa, RegPreambleLsb, _LoRa->preamble >> 0);

        // DIO mapping:   --> DIO: RxDone
        read = LoRa_read(_LoRa, RegDioMapping1);
        data = read | 0x3F;
        LoRa_write(_LoRa, RegDioMapping1, data);

        // goto standby mode:
        LoRa_gotoMode(_LoRa, STNBY_MODE);
        _LoRa->current_mode = STNBY_MODE;
        HAL_Delay(10);

        read = LoRa_read(_LoRa, RegVersion);
        if (read == 0x12)
            return LORA_OK;
        else
            return LORA_NOT_FOUND;
    } else {
        return LORA_UNAVAILABLE;
    }
}

uint32_t our_pow(uint32_t base, unsigned int exp) {
    uint32_t result = 1;

    while (exp) {
        if (exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }

    return result;
}


uint32_t LoRa_calculateTOA(LoRa *_LoRa, uint8_t length) {
    uint32_t Tpre, Tpay, Tpacket, Ns, BW;

    switch (_LoRa->bandWidth) {
        case BW_7_8KHz:
            BW = 8;
            break;
        case BW_10_4KHz:
            BW = 11;
            break;
        case BW_15_6KHz:
            BW = 16;
            break;
        case BW_20_8KHz:
            BW = 21;
            break;
        case BW_31_25KHz:
            BW = 32;
            break;
        case BW_41_7KHz:
            BW = 42;
            break;
        case BW_62_5KHz:
            BW = 63;
            break;
        case BW_125KHz:
            BW = 125;
            break;
        case BW_250KHz:
            BW = 250;
            break;
        case BW_500KHz:
            BW = 500;
            break;
        default:
            BW = 125;
    }


    Tpre = ((4 * _LoRa->preamble + 17) * our_pow(2, _LoRa->spredingFactor)) / (4 * BW);

    Ns = 8 + 1 + ceil((8 * length - 4 * _LoRa->spredingFactor + 28 + 16 - 20 * (_LoRa->spredingFactor == 6 ? 1 : 0)),
                      (4 * (_LoRa->spredingFactor))) * (_LoRa->crcRate + 4);

    Tpay = (our_pow(2, _LoRa->spredingFactor) * Ns) / BW;

    Tpacket = Tpre + Tpay;

    return Tpacket;
}