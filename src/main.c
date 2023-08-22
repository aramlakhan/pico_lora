
/*
 * Author: Chris Schorn
 * Open Lora Mesh Network
 * Version 
 * Versioning Reason: 
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* FreeRTOS Includes */
#include "../FreeRTOS-Kernel/include/FreeRTOS.h" /* MUST COME FIRST */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */

/* Raspberry Pi Pico Inlcudes */
#include <stdio.h> /* pico/stdio.h" */
#include "pico/stdlib.h"
#include "hardware/adc.h"/*"../../pico-sdk/src/rp2_common/hardware_adc/include/hardware/adc.h"*/
#include "hardware/spi.h"/*"../../pico-sdk/src/rp2_common/hardware_spi/include/hardware/spi.h"*/
#include "hardware/gpio.h"

static TaskHandle_t xSimpleLEDTaskHandle = NULL;
static TaskHandle_t xUsbIOTaskHandle = NULL;
static TaskHandle_t xSx1280TaskHandle = NULL;

/* ------------ Defining SD Card Command Index with Hexadecimel Commands ------------ */

/*  Retrieve the transceiver status
    Cannot be the first command sent over the interface
    Is not strictly necessary for SPI b/c device returns status info
        also on cammand bytes
    params( void ) return( status ) */
#define GETSTATUS 0xC0

/*  Writes a block of bytes in a data memory space starting 
        at a specific address.
    params( address[15:8], address[7:0], data[0:n] ) return( void ) */
#define WRITEREGISTER 0x18 

/*  Reads a block of data starting at a given address
    The host must send a NOP after the address to receive data!!!!!!!!!!!!!!!
    params( address[15:8], address[7:0] ) return( data[0:n-1] )  */
#define READREGISTER 0x19

/*  Write the data payload to be transmitted
    Data sent in hex, most likely translated using ascii for text
    Audio data tbd
    params( offset, data[0:n] ) return( void ) */
#define WRITEBUFFER 0x1A

/*  Function allows reading (n-3) bytes of payload received 
        starting at offset.
    Data received in hex, most likely translated using ascii for text
    params( offset ) return( data[0:n-1] ) */
#define READBUFFER 0x1B

/*  Set transceiver to Sleep mode
    Lowest current consumption
    params( sleepConfig ) return( void )
    sleepConfig[7:4] unused 
    sleepConfig[1] 1: Data buffer in retention mode
    sleepConfig[0] 0: Data Ram is flushed 1: Data Ram in retention  */
#define SETSLEEP 0x84

/*  Set the device in either STDBY_RC or STDBY_XOSC mode
    Used to configure the transceiver 
    Intermediate levels of power consumption
    params( standbyConfig )
    standbyConfig 0: STDBY_RC mode 1: STDBY_XOSC mode
    return( void ) */
#define SETSTANDBY 0x80

/*  Set the device in Frequency Synthesizer mode
    PLL(Phase Locked Loop) is locked to the carrier frequency
    For test purposes of PLL
    params( void ) return( void ) */
#define SETFS 0xC1

/*  Set the device in Transmit mode
    Clear IRQ status before using this command
    Timeout = periodBase * periodBaseCount
    params( periodBase, periodBaseCount[15:8], periodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out Other: Time out active
    return( void ) */
#define SETTX 0x83

/*  Set the device in Receiver mode
    Timeout = periodBase * periodBaseCount 
    params( periodBase, periodBaseCount[15:8], periodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out 
                          0xFFFF: Rx Continuous mode, multi-packet Rx
                          Other: Time out active
    return( void ) */
#define SETRX 0x82

/*  Set transceiver in sniff mode
    setLongPreamble must be issued prior to setRxDutyCycle
    RxPeriod = periodBase * rxPeriodBaseCount
    SleepPeriod = periodBase * sleepPeriodBaseCount
    params( rxPeriodBase, rxPeriodBaseCount[15:8], 
        rxPeriodBaseCount[7:0], sleepPeriodBase,
        sleepPeriodBaseCount[15:8], sleepPeriodBaseCount[7:0] )
    periodBase 0x00: 15.625us 0x01: 62.5us 0x02: 1ms 0x03: 4ms
    periodBaseCount[15:0] 0x0000: No Time Out 
                          Other: Device will stay in Rx Mode for 
                                 RxPeriod and return 
                                 to Sleep Mode for SleepPeriod
    return( void ) */
#define SETRXDUTYCYCLE 0x94

/*  Set transceiver to Channel Activity Detection mode
    Device searches for a Lora signal
    Returns to STDBY_RC mode when finished
    Always sends CadDone IRQ, sends CadDetected IRQ if signal found
    Useful in Listen before Talk Applications
    params( void ) return( void ) */
#define SETCAD 0xC5

/*  Test command to generate a Continuous Wave (RF tone)
    Frequency and power settings from setRfFrequency, and setTxParams
    params( void ) return( void ) */
#define SETTXCONTINUOUSWAVE 0xD1

/*  Test command to generate infinite sequence pf symbol 0 in Lora
    params( void ) return( void ) */
#define SETTXCONTNIOUSPREAMBLE 0xD2

/*  Sets the transceiver radio frame
    MUST BE THE FIRST IN A RADIO CONFIGURATION SEQUENCE!!!!!!!
    params( packetType )
    packetType[8:0] 0x00: GFSK
                    0x01: Lora 
                    0x02: Ranging 
                    0x03: FLRC
                    0x04: BLE
    return( void ) */
#define SETPACKETTYPE 0x8A

/*  Returns the current operation packet type of the radio
    packetType probly comes in same format as setPacketType
    params( void ) return( packetType ) */
#define GETPACKETTYPE 0x03

/*  Set the frequency of the RF frequency mode
    rfFrequency sets the number of PLL steps
    Frf = ( Fxosc/2^18 ) * rfFrequency
        Gives frequency in kilohertz
    params( rfFrequency[23:16], rfFrequency[15:8], rfFrequency[7:0] )
    return( void ) */
#define SETRFFREQUENCY 0x86

/*  Sets the Tx output power and the Tx ramp time
    params( power, rampTime )
    power  Pout[dB] = -18 + power i.e. -18 + 0x1F(31) = 13dbm
    rampTime 0x00: 2um 0x20: 4us 0x40: 5us 0x60: 8us 
            0x80: 10us 0xA0: 12us 0xC0: 16us 0xE0: 20us
    return( void ) */
#define SETTXPARAMS 0x8E

/*  Sets number of symbols which Channel Activity Detected operates
    For symbols 1 & 2, there are higher risks of false detection.
    params( cadSymbolNum )
    cadSymbolNum 0x00: 1 symbol
                 0x20: 2 symbols
                 0x40: 4 symbols
                 0x60: 8 symbols
                 0x80: 16 symbols
    return( void ) */
#define SETCADPARAMS 0x88

/*  Fixes the base address for the packet handing operation
        in Tx and Rx mode for all packet types
    params( txBaseAddress, rxBaseAddress ) return( void ) */
#define SETBUFFERBASEADDRESS 0x8F

/*  Configure the modulation parameters of the radio
    Params passed will be interpreted depending on the frame type
    Frame Type 
    params( modParam1, modParam2, modParam3 )
    modParam1 BLE: BitrateBandwidth   Lora/Ranging: Spreading Factor
    modParam2 BLE: ModulationIndex    Lora/Ranging: Bandwith
    modParam3 BLE: ModulationShaping  Lora & Ranging: Coding Rate
    return( void ) */
#define SETMODULATIONPARAMS 0x8B

/*  Set the parameters of the packet handling block
    params( packetParam1, packetParam2, packetParam3, packetParam4,
        packetParam5, packetParam6, packetParam7 )
    packetParam1 BLE: ConnectionState Lora/Ranging: Preambl Length
    packetParam2 BLE: CrcLength       Lora/Ranging: Header Type
    packetParam3 BLE: BleTestPayload  Lora/Ranging: PayloadLength
    packetParam4 BLE: Whitening       Lora/Ranging: CRC
    packetParam5 BLE: Not Used     Lora/Ranging: InvertIQ/chirp invert
    packetParam6 BLE: Not Used        Lora/Ranging: Not Used
    packetParam7 BLE: Not Used        Lora/Ranging: not Used
    return( void ) */ 
#define SETPACKETPARAMS 0x8C

/*  Returns the length of the last received packet 
        and the address of the first byte received
    In Lora packet type, 0x00 always returned for rxPayloadLength.
        Instead read register 0x901, for Lora payload length
    params( void ) return( payloadLength, rxBufferOffset ) */
#define GETRXBUFFERSTATUS 0x17

/*  Retrieve information about the last received packet
    rssiSync: RSSI value latched upon  detection of sync address.
        Actual signal power is –(rssiSync)/2dBm
    snr: Estimation of Signal to Noise Ratio on last packet received. 
        In two’s compliment format multiplied by 4. 
        Actual Signal to Noise Ratio(SNR) is (snr)/4dB. If SNR ≤ 0, 
        RSSI_{packet, real} = RSSI_{packet,measured} – SNR_{measured}
    params( void ) 
    return( packetStatus[39:32], packetStatus[31:24],
        packetStatus[23:16], packetStatus[15:8], packetStatus[7:0] ) 
    packetStatus[7:0]   BLE: RFU        Lora/Ranging: rssiSync
    packetStatus[15:8]  BLE: rssiSync   Lora/Ranging: snr
    packetStatus[16:23] BLE: errors     Lora/Ranging: -
    packetStatus[24:31] BLE: status     Lora/Ranging: -
    packetStatus[32:39] BLE: sync       Lora/Ranging: - */
#define GETPACKETSTATUS 0x1D

/*  Returns instantaneous RSSI value during reception of a  packet
    rssilnst: Signal power is (–rssiInst)/2dBm
    params( void ) return( rssilnst ) */
#define GETRSSILNST 0x1F

/*  Enable IRQs and to route IRQs to DIO pins
    An interrupt is flagged in IRQ register if the corresponding 
        bit in flag register is set
    irqMask[15:0] set which IRQ's are active, 
        pg 95 in sx1280 manual has IRQ table
    dioMasks active bits correspond to the active bits irqMasks
        If coresponding bits are both on, IRQ is sent through that DIO
    params( irqMask[15:8], irqMask[7:0], dio1Mask[15:8],dio1Mask[7:0],
    dio2Mask[15:8], dio2Mask[7:0], dio3Mask[15:8], dio3Mask[7:0] )
    return( void ) */
#define SETDIOIRQPARAMS 0x8D

/*  Returns the value of the IRQ register
    IRQ register is only interacatable through these commands
    params( void ) return( irqStatus[15:8], irqStatus[7:0] ) */
#define GETIRQSTATUS 0x15

/*  Clears an IRQ flag in IRQ register
    Corresponding bits in irqMask will clear flag of that IRQ
    params( irqMask[15:8], irqMask[7:0] ) return( void ) */
#define CLRIRQSTATUS 0x97

/*  Why Kansas but no arkansas
    Havent found in book
    params( regulatorMode ) return( void ) */
#define SETREGULATORMODE 0x96

/*  Havent found in book
    params( void ) return( void ) */
#define SETSAVECONTEXT 0xD5

/*  Set the state following a Rx or Tx operation is FS, not STDBY
    Reduces switching time between consecutive Rx and/or Tx operations
    params( 0x00=disable or 0x01=enable ) return( void ) */
#define SETAUTOFS 0x9E

/*  Allows transceiver to send a packet at a user programmable time 
        after the end of a packet reception
    Must be issued in STDBY_RC mode
    TxDelay = time + 33us(time needed for transceiver to switch modes)
    params( time[15:8], time[7:0] ) return( void ) */
#define SETAUTOTX 0x98

/*  Set the transceiver into Long Preamble mode
    RxDutyCycle is modified so that if a preamble is detected,
         the Rx window is extended by SleepPeriod + 2 * RxPeriod
    params( enable )
    enable 0x00: disable 0x01: enable
    return( void ) */
#define SETLONGPREAMBLE 0x9B

/* #define SETUARTSPEED 0x9D, using spi not uart interface */

/*  params( 0x00=slave or 0x01=master ) return( void ) */
#define SETRANGINGROLE 0xA3

/* params( 0x00=slave or 0x01=master ) return( void ) */
#define SETADVANCEDRANGING 0x9A


/* --------------------------- sx1280 2.4GHz Lora Operation -------------------------------- */

/*  Driving the chip select pin low 
    Transactions with sx1280 start with chip select low */
static inline void sx1280Select(){

    asm volatile ("nop \n nop \n nop");/* Find out what it does */
    gpio_put( 13, 0 );
    asm volatile ("nop \n nop \n nop");
}

/*  Driving the chip select pin high 
    Transactions with sx1280 end with chip select high */
static inline void sx1280Deselect(){

     asm volatile ("nop \n nop \n nop");
     gpio_put( 13, 1 );
     asm volatile ("nop \n nop \n nop");
}

/* Function sending common transciever settings to sx1280 */ 
void sx1280Setup( uint8_t standbyMode, 
                  uint8_t packetType, 
                  uint8_t rfFrequency2316,
                  uint8_t rfFrequency158, 
                  uint8_t rfFrequency70, 
                  uint8_t spreadingFactor,
                  uint8_t bandwidth, 
                  uint8_t codingRate, 
                  uint8_t preambleLength, 
                  uint8_t headerType, 
                  uint8_t cyclicalRedundancyCheck, 
                  uint8_t chirpInvert, 
                  uint8_t *outboundMessage ){

    /* Format For SPI Communication with sx1280 */

    /* Allocate memory for a dynamic array to 
            send data to sx1280 (writeData) */
    /* Allocate memory for a dynamic array to 
            receive data from sx1280 (readData) */
    /* Add Opcode to first memory byte in writeData */
    /* Add data in hexadecimal to second, third, etc. 
            memory byte in writeData for sets, NOP for gets  */
    /* int spi_write_blocking( spi_inst_t *spi, 
                               const uint8_t *src,
                               size_t len ) */
    /* Deallocate memory for writeData and/or readData */

    uint8_t *setupWriteData;
    uint8_t *setupReadData;
    uint32_t payloadLength = 0;

    /* Driving pin 21 low to reset the sx1280 */
    gpio_put( 21, 0 );
    asm volatile ("nop \n nop \n nop");
    gpio_put( 21, 1 );

    /* Waiting till the busy pin is driven low 
       So the sx1280 is not sent a command while busy
            Because it wont receive the command */
    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after reset\n");
    }

    /* Setting sx1280 Standby mode */
    setupWriteData = ( uint8_t * ) pvPortMalloc( 2*sizeof( uint8_t ) );
    *( setupWriteData ) = SETSTANDBY;
    *( setupWriteData + 1 ) = standbyMode; /* Setting STDBY_RC Mode 0x01, STDBY_XOSC */
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 2*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after SETSTANDBY\n");
    }

    /* Setting sx1280 Packet Type */
    setupWriteData = ( uint8_t * ) pvPortMalloc( 2*sizeof( uint8_t ) );
    *( setupWriteData ) = SETPACKETTYPE;
    *( setupWriteData + 1 ) = packetType;
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 2*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after SETPACKETTYPE\n");
    }

    /* Setting RF Frequency */
    setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
    *( setupWriteData ) = SETRFFREQUENCY;
    *( setupWriteData + 1 ) = rfFrequency2316; /* rfFrequency[23:16] */
    *( setupWriteData + 2 ) = rfFrequency158; /* rfFrequency[15:8] */
    *( setupWriteData + 3 ) = rfFrequency70; /* rfFrequency[7:0] */
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after SETRFFREQUENCY\n");
    }

    /* Setting Tx and Rx Buffer Base Addresses
       Putting both at 0 since messages can be size of buffer */
    setupWriteData = ( uint8_t * ) pvPortMalloc( 3*sizeof( uint8_t ) );
    *( setupWriteData ) = SETBUFFERBASEADDRESS;
    *( setupWriteData + 1 ) = 0x00;
    *( setupWriteData + 2 ) = 0x00;
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 3*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after SETBUFFERBASEADDRESS\n");
    }

    /* Setting the Modulation Params */
    setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
    *( setupWriteData ) = SETMODULATIONPARAMS;
    *( setupWriteData + 1 ) = spreadingFactor; /* Spreading Factor */
    *( setupWriteData + 2 ) = bandwidth; /* Bandwidth */
    *( setupWriteData + 3 ) = codingRate; /* Coding Rate */
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );
    /* 0x1E Must be written to register 0x0925 for SF5 or SF6 */
    if( spreadingFactor == 0x50 || spreadingFactor == 0x60 ){

        setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        *( setupWriteData ) = WRITEREGISTER;
        *( setupWriteData + 1 ) = 0x09;
        *( setupWriteData + 2 ) = 0x25;
        *( setupWriteData + 3 ) = 0x1E;
        sx1280Select();
        spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        vPortFree( setupWriteData );
    }
    /* 0x37 Must be written to register 0x0925 for SF7 or SF8 */
    else if( spreadingFactor == 0x70 || spreadingFactor == 0x80 ){

        setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        *( setupWriteData ) = WRITEREGISTER;
        *( setupWriteData + 1 ) = 0x09;
        *( setupWriteData + 2 ) = 0x25;
        *( setupWriteData + 3 ) = 0x37;
        sx1280Select();
        spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        vPortFree( setupWriteData );
    }
    /* 0x32 Must be written to register 0x0925 for SF9, SF10, SF11, or SF12 */
    else if( spreadingFactor == 0x90 || spreadingFactor == 0xA0 || spreadingFactor == 0xB0 || spreadingFactor == 0xC0 ){
        
        setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        *( setupWriteData ) = WRITEREGISTER;
        *( setupWriteData + 1 ) = 0x09;
        *( setupWriteData + 2 ) = 0x25;
        *( setupWriteData + 3 ) = 0x32;
        sx1280Select();
        spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        vPortFree( setupWriteData );
    }
    /* 0x01 must be written to register 0x093C */
    setupWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
    *( setupWriteData ) = WRITEREGISTER;
    *( setupWriteData + 1 ) = 0x09;
    *( setupWriteData + 2 ) = 0x3C;
    *( setupWriteData + 3 ) = 0x01;
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after SETMODULATIONPARAMS\n");
    }

    /* Setting Packet Params */
    while( *( outboundMessage + payloadLength ) != 0x00 ){
        /* Maximum payloadLength on sx1280 is 255 */
        if( payloadLength > 255 ){
            payloadLength = 255;
            break;
        }
        payloadLength = payloadLength + 1;
    }
    setupWriteData = ( uint8_t * ) pvPortMalloc( 8*sizeof( uint8_t ) );
    *( setupWriteData ) = SETPACKETPARAMS;
    *( setupWriteData + 1 ) = preambleLength; /* Preamble Length */
    *( setupWriteData + 2 ) = headerType; /* Header Type */
    *( setupWriteData + 3 ) = payloadLength; /* Payload Length */
    *( setupWriteData + 4 ) = cyclicalRedundancyCheck; /* Cyclical Redundancy Check */
    *( setupWriteData + 5 ) = chirpInvert; /* Invert IQ/chirp invert */
    *( setupWriteData + 6 ) = 0x00; /* Not Used */
    *( setupWriteData + 7 ) = 0x00; /* Not Used */
    sx1280Select();
    spi_write_blocking( spi1, setupWriteData, 8*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( setupWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after SETPACKETPARAMS\n");
    }

    /* Testing connecting from pico to sx1280 by writing to and reading from buffer
       Working output should be "0xStatus 0xStatus 0xFF" */

    /* writeData = ( uint8_t * ) pvPortMalloc( 3*sizeof( uint8_t ) );
    *( writeData ) = WRITEBUFFER;
    *( writeData + 1 ) = 0x00;
    *( writeData + 2 ) = 0xFF;
    sx1280Select();
    spi_write_blocking( spi1, writeData, 3*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( writeData ); */

    /* Must use two NOP's for reads because data is
            returned beginning on the second NOP */
    /*readData = ( uint8_t * ) pvPortMalloc( 5*sizeof( uint8_t ) );
    writeData = ( uint8_t * ) pvPortMalloc( 5*sizeof( uint8_t ) );
    *( writeData ) = READBUFFER;
    *( writeData + 1 ) = 0x00;
    *( writeData + 2 ) = 0x00;
    *( writeData + 3 ) = 0x00;
    *( writeData + 4 ) = 0x00;
    sx1280Select();
    spi_write_read_blocking( spi1, writeData, readData, 5*sizeof( uint8_t ) );
    sx1280Deselect(); 
    printf( "%X %X %X %X %X\n", *( readData ), *( readData + 1 ), *( readData + 2 ), *( readData + 3 ), *( readData + 4 ) );
    vPortFree( writeData );
    vPortFree( readData ); */

}


/* Function setting up and running tx operation on an sx1280, taking 255 byte message packets */
void sx1280Tx( uint8_t power, 
               uint8_t rampTime,
               uint8_t *outboundMessage,
               uint8_t txIrq158, 
               uint8_t txIrq70, 
               uint8_t txPeriodBase,
               uint8_t txPeriodBaseCount158, 
               uint8_t txPeriodBaseCount70 ){

    uint8_t *txWriteData = 0;
    uint8_t *txReadData = 0;
    uint32_t txPayloadLength = 0;
    /* Iterators */
    uint32_t i = 0;

    /* Setting the tx parameters necessary for sending a message */
    txWriteData = ( uint8_t * ) pvPortMalloc( 3*sizeof( uint8_t ) );
    *( txWriteData ) = SETTXPARAMS;
    *( txWriteData + 1 ) = power;    /* power       */
    *( txWriteData + 2 ) = rampTime; /* rampTime    */
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, 3*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after SETTXPARAMS\n");
    }

    /* Writing a message to the sx1280 Tx message buffer */
    while( *( outboundMessage + txPayloadLength ) != 0x00 ){
        /* Getting size of a single outbound message, storing it in a holder variable */
        txPayloadLength = txPayloadLength + 1;
    }
    /* Allocating txPayloadLength+3 bytes to writeData, payloadLength is indexed from zero
            and space is needed for the WRITEBUFFER command and nop */
    txWriteData = ( uint8_t * )pvPortMalloc( ( txPayloadLength+3 )*sizeof( uint8_t ) );
    *( txWriteData ) = WRITEBUFFER;
    *( txWriteData + 1 ) = 0x00;
    /* Looping payloadLength times, writing outboundMessage data to WRITEBUFFER command */
    for( i = 0; i <= txPayloadLength; i++ ){
        *( txWriteData + i + 2 ) = *( outboundMessage + i );
        printf("Outbound Message: 0x%X\n", *( outboundMessage + i ) );
    }
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, ( txPayloadLength+3 )*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after tx WRITEBUFFER\n");
    }

    /* setting IRQ parameters for the outgoing message, looping SPI not DIO pins to check*/
    txWriteData = ( uint8_t * ) pvPortMalloc( 9*sizeof( uint8_t ) );
    *( txWriteData ) = SETDIOIRQPARAMS;
    *( txWriteData + 1 ) = txIrq158; /* IRQ Mask for bits 15:8 of IRQ register */
    *( txWriteData + 2 ) = txIrq70; /* IRQ Mask for bits 7:0 of IRQ register */
    *( txWriteData + 3 ) = 0x00; /* setting DIO 1 Mask bits 15:8 to 0 */
    *( txWriteData + 4 ) = 0x00; /* setting DIO 1 Mask bits 7:0 to 0 */
    *( txWriteData + 5 ) = 0x00; /* setting DIO 2 Mask bits 15:8 to 0 */
    *( txWriteData + 6 ) = 0x00; /* setting DIO 2 Mask bits 7:0 to 0 */
    *( txWriteData + 7 ) = 0x00; /* setting DIO 3 Mask bits 15:8 to 0 */
    *( txWriteData + 8 ) = 0x00; /* setting DIO 3 Mask bits 7:0 to 0 */
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, 9*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after tx SETDIOIRQPARAMS\n");
    }

    /* Putting sx1280 in transmit mode to send the message in sx1280's message buffer 
       Timeout is periodBase * periodBaseCount */
    txWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
    *( txWriteData ) = SETTX;
    *( txWriteData + 1 ) = txPeriodBase; /* setting periodBase, RTC step */
    *( txWriteData + 2 ) = txPeriodBaseCount158; /* setting periodBaseCount[15:8] */
    *( txWriteData + 3 ) = txPeriodBaseCount70; /* setting periodBaseCount[8:0] */
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, 4*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after tx SETTX\n");
    }

    /* Looping over GETIRQSTATUS using SPI, till TxDone bit is high */
    for( i = 0; i <= 100; i++){

        vPortFree( txReadData );
        vTaskDelay( 50 );

        txWriteData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        txReadData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        *( txWriteData ) = GETIRQSTATUS;
        *( txWriteData + 1 ) = 0x00;
        *( txWriteData + 2 ) = 0x00;
        *( txWriteData + 3 ) = 0x00;
        sx1280Select();
        spi_write_read_blocking( spi1, txWriteData, txReadData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        vPortFree( txWriteData );
 
        while( gpio_get( 22 ) == 1 ){
            vTaskDelay( 10 );
            printf("Busy after tx GETIRQSTATUS\n");
        }

        printf("IRQ Check: 0x%X %i\n", *( txReadData + 3 ), i );

        /* Checking bits [7:0] to see if the TxDone bit in the IRQ register is high
           Doing bitwise 'and' operation with 0x01 to mask the rest of the bits in 
                the IRQ register, giving a clear indication that a message has been sent
            Bits [15:8] would be in  *( readData + 4 ) */
        if( *( txReadData + 3 ) != 0x00 ){ /* GETIRQSTATUS TxDone == 1 */

            printf("IRQ: 0x%X %i \n", *( txReadData + 3 ), i );
            vPortFree( txReadData );
            break;
        }

    }

    /* Clearing the IRQ register, reseting IRQ Mask bits to 0 */
    txWriteData = ( uint8_t * ) pvPortMalloc( 3*sizeof( uint8_t ) );
    *( txWriteData ) = CLRIRQSTATUS;
    *( txWriteData + 1 ) = 0xFF; /* clearing bits 15:8 of IRQ mask */
    *( txWriteData + 2 ) = 0xFF; /* clearing bits 7:0 of IRQ mask */
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, 3*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after tx CLRIRQSTATUS\n");
    }

    /* Tx SETSANDBY */
    txWriteData = ( uint8_t * ) pvPortMalloc( 2*sizeof( uint8_t ) );
    *( txWriteData ) = SETSTANDBY;
    *( txWriteData + 1 ) = 0x00;
    sx1280Select();
    spi_write_blocking( spi1, txWriteData, 2*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( txWriteData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after tx SETSTANDBY\n");
    }
}


/* Function setting up and running rx operation on an sx1280, 2.4Ghz LORA Modem*/
void sx1280Rx( uint8_t rxIrq158, 
               uint8_t rxIrq70, 
               uint8_t rxPeriodBase,
               uint8_t rxPeriodBaseCount158, 
               uint8_t rxPeriodBaseCount70,
               uint8_t *inboundMessage ){

    uint8_t *writeData;
    uint8_t *readData;
    uint32_t totalSizeOfMessage = 0;
    uint32_t sizeOfMessageInBuffer = 0;
    /* Iterators */
    uint32_t i = 0;
    uint32_t j = 0;
    

    /* setting IRQ parameters for Rx mode */
    writeData = ( uint8_t * ) pvPortMalloc( 9*sizeof( uint8_t ) );
    *( writeData ) = SETDIOIRQPARAMS;
    *( writeData + 1 ) = rxIrq158; /* IRQ Mask for bits 15:8 of IRQ register */
    *( writeData + 2 ) = rxIrq70; /* IRQ Mask for bits 7:0 of IRQ register */ 
    *( writeData + 3 ) = 0x00; /* setting DIO 1 Mask bits 15:8 to 0 */
    *( writeData + 4 ) = 0x00; /* setting DIO 1 Mask bits 7:0 to 0 */
    *( writeData + 5 ) = 0x00; /* setting DIO 2 Mask bits 15:8 to 0 */
    *( writeData + 6 ) = 0x00; /* setting DIO 2 Mask bits 7:0 to 0 */
    *( writeData + 7 ) = 0x00; /* setting DIO 3 Mask bits 15:8 to 0 */
    *( writeData + 8 ) = 0x00; /* setting DIO 3 Mask bits 7:0 to 0 */
    sx1280Select();
    spi_write_blocking( spi1, writeData, 9*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( writeData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after rx SETDIOIRQPARAMS\n");
    }

    /* setting sx1280 to Rx mode
       Setting Rx mode to continuous, so multiple messages can be received */
    writeData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
    *( writeData ) = SETRX;
    *( writeData + 1 ) = rxPeriodBase; /* Setting the RTC step */
    *( writeData + 2 ) = rxPeriodBaseCount158; /* perdiodBase[15:8] for rx */
    *( writeData + 3 ) = rxPeriodBaseCount70; /* perdiodBase[7:0] for rx */
    sx1280Select();
    spi_write_blocking( spi1, writeData, 4*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( writeData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after SETRX\n");
    }

    /* Loop polling 100 times over rx mode
       Each loop has a 50 clock-tick delay allowing other tasks to run */
    for( i = 0; i <= 100; i++ ){ 

        printf("Listening: %i\n", i );
        sizeOfMessageInBuffer = 0;

        /* Using GETIRQSTATUS to check if there is a new message in the rx buffer */
        writeData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        readData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
        *( writeData ) = GETIRQSTATUS;
        *( writeData + 1 ) = 0x00;
        *( writeData + 2 ) = 0x00;
        *( writeData + 3 ) = 0x00;
        sx1280Select();
        spi_write_read_blocking( spi1, writeData, readData, 4*sizeof( uint8_t ) );
        sx1280Deselect();
        vPortFree( writeData );

        /* Checking to see if the RxDone bit in the IRQ register is high, with 0x02 bitmask */
        if( ( *( readData + 3 ) & 0x02 ) == 0x02 ){ /* GETIRQSTATUS RxDone == 1 */
 
            vPortFree( readData );

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after rx GETIRQSTATUS\n");
            }

            /* see what the message is and decide what to do with it */

            /* using GETPACKETSTATUS which returns rssiSync, and Signal to Noise Ratio ( SNR )
               Not currently using but it's in sx1280 Documentation for Rx operation
                    pretty sure it's used to see if the received message is useable or not */
            writeData = ( uint8_t * ) pvPortMalloc( 7*sizeof( uint8_t ) );
            readData = ( uint8_t * ) pvPortMalloc( 7*sizeof( uint8_t ) );
            *( writeData ) = GETPACKETSTATUS;
            *( writeData + 1 ) = 0x00;
            *( writeData + 2 ) = 0x00;
            *( writeData + 3 ) = 0x00;
            *( writeData + 4 ) = 0x00;
            *( writeData + 5 ) = 0x00;
            *( writeData + 6 ) = 0x00;
            sx1280Select();
            spi_write_read_blocking( spi1, writeData, readData, 4*sizeof( uint8_t ) );
            sx1280Deselect();
            vPortFree( writeData );
            vPortFree( readData );

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after rx GETPACKETSTATUS\n");
            }

            /* Clearing the IRQ register on the sx1280
               Not sure why it's done here in the rx operation in sx1280 documentation */
            writeData = ( uint8_t * ) pvPortMalloc( 3*sizeof( uint8_t ) );
            *( writeData ) = CLRIRQSTATUS;
            *( writeData + 1 ) = 0xFF;
            *( writeData + 2 ) = 0xFF;
            sx1280Select();
            spi_write_blocking( spi1, writeData, 3*sizeof( uint8_t ) );
            sx1280Deselect();
            vPortFree( writeData );

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after rx CLRIRQSTATUS\n");
            }

            /* Getting the length of the newly received message
               GETRXBUFFERSTATUS only works for LORA messages with headers, 
                    otherwise read register 0x0901 */
            writeData = ( uint8_t * ) pvPortMalloc( 4*sizeof( uint8_t ) );
            readData = ( uint8_t * ) pvPortMalloc( 4*sizeof(uint8_t ) );
            *( writeData ) = GETRXBUFFERSTATUS; 
            *( writeData + 1 ) = 0x00;
            *( writeData + 2 ) = 0x00;
            *( writeData + 3 ) = 0x00;
            sx1280Select();
            spi_write_read_blocking( spi1, writeData, readData, 4*sizeof( uint8_t ) );
            sx1280Deselect();
            /* Grabbing message size for correct memory allocation for incoming message */
            sizeOfMessageInBuffer = *( readData + 2 );
            vPortFree( writeData );
            vPortFree( readData );

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after rx READREGISTER\n");
            }

            /* Reading message buffer of sx1280
               Allocating the size of the message in the sx1280 buffer plus 3 because over 
                    spi you must send an opcode, the buffer offset, and a nop to receive the
                    payload on the buffer */
            writeData = ( uint8_t * ) pvPortMalloc((sizeOfMessageInBuffer + 3)*sizeof(uint8_t));
            readData = ( uint8_t * ) pvPortMalloc( (sizeOfMessageInBuffer + 3)*sizeof(uint8_t));
            *( writeData ) = READBUFFER;
            *( writeData + 1 ) = 0x00; /* sx1280 message buffer offset */
            *( writeData + 2 ) = 0x00; /* sending first nop */
            /* Looping through rest of writeData to add nops, i begins at *( writeData + 3 ) */
            printf("Final Address = 0x%X\n", ( writeData + sizeOfMessageInBuffer + 3 ));
            for( j = 0; j <= sizeOfMessageInBuffer; j++){
                *( writeData + j + 3 ) = 0x00;
                /* printf("writeData + j = 0x%X j = %i \n", ( writeData + j + 3 ), j ); */
            }
            sx1280Select();
            spi_write_read_blocking( spi1, writeData, readData, 
                                     ( sizeOfMessageInBuffer + 3 )*sizeof( uint8_t ) );
            sx1280Deselect();
            vPortFree( writeData );
            /* Passing newly received message pointer to vSx1280Task */
            inboundMessage = readData; 

            while( gpio_get( 22 ) == 1 ){
                vTaskDelay( 10 );
                printf("Busy after rx READBUFFER\n");
            }
        vTaskDelay( 50 ); /* 50 clock-tick delay */
    }

    /* Rx SETSANDBY */
    writeData = ( uint8_t * ) pvPortMalloc( 2*sizeof( uint8_t ) );
    *( writeData ) = SETSTANDBY;
    *( writeData + 1 ) = 0x00;
    sx1280Select();
    spi_write_blocking( spi1, writeData, 2*sizeof( uint8_t ) );
    sx1280Deselect();
    vPortFree( writeData );

    while( gpio_get( 22 ) == 1 ){
        vTaskDelay( 10 );
        printf("Busy after rx SETSTANDBY\n");
    }
}


/*  Function setting up sx1280 connection to rp2040 
    Initializing SPI interface on rp2040
    Using GP10-GP13
        GP10: SPI1 SCK
        GP11: SPI1 Tx
        GP12: SPI1 Rx
        GP13: SPI1 CSn */
void sx1280Rp2040Setup( ){

    /* Setting up Busy Pin */

    /* void gpio_init (uint gpio)
       Initializing GP22 to input pin for Busy */
    gpio_init( 22 );
    /* static void gpio_set_dir (uint gpio, bool out)
       Setting GP22 direction to input, True is out False is in
       Use gpio_get( uint gpio ) to get the state of a gpio */
    gpio_set_dir( 22, 0 );

    /* Setting up Reset Pin */

    /* Initializing GP21 to output pin for Reset */
    gpio_init( 21 );
    /* Setting GP21 direction to output, True is out False is in */
    gpio_set_dir( 21, 1 );
    /* static void gpio_put (uint gpio, bool value)
       Driving GP21 High
       A resit is initiated by driving Reset Low */
    gpio_put( 21, 1 );

    /* Setting up SPI1 Interface */

    /* Inializing spi1 at 1MHz */
    spi_init( spi1, 1000000 );

    /* void gpio_set_function( uint gpio, enum gpio_function fn )
       Setting  GP10-GP12 as SCK, TX, and RX respectively */
    gpio_set_function( 10, GPIO_FUNC_SPI );
    gpio_set_function( 11, GPIO_FUNC_SPI );
    gpio_set_function( 12, GPIO_FUNC_SPI );

    /* Initializing GP13 to output pin for Chip Select */
    gpio_init( 13 );
    /* Setting GP13 direction to output, True is out False is in */
    gpio_set_dir( 13, 1 );
    /* static void gpio_put (uint gpio, bool value)
       Driving GP13 High
       A data transfer is started by driving Chip Select low */
    gpio_put( 13, 1 );
}


/*  Task setting up and running sx1280
    Also now adding some elements of mesh in */
void vSx1280Task( void *pvParameters ){

    /* Instantiating pointer variables for dynamic arrays */
    uint8_t *writeData;
    uint8_t *readData;

    /* Instantiating pointer for address received from task notification */
    uint32_t *taskNotificationFromUSB = NULL;

    sx1280Rp2040Setup();

    /* Iterators */
    uint32_t i = 0;
    uint32_t j = 0;

    while( true ){

        /* Allocating 4 bytes (32 bits) for the address containing data from vUsbIOTask */
        taskNotificationFromUSB = ( uint32_t * ) pvPortMalloc( 1*sizeof( uint32_t ) );

        xTaskNotifyWait(
                0xffffffff,               /* uint32_t ulBitsToClearOnEntry */
                0,                        /* uint32_t ulBitsToClearOnExit */
                taskNotificationFromUSB,  /* uint32_t *pulNotificationValue */
                100 );                    /* TickType_t xTicksToWait */
 
        if( taskNotificationFromUSB != NULL ){
            for( uint32_t i = 0; *( ( uint8_t * ) *( taskNotificationFromUSB ) + i ) != 0; i++){
                printf("sx1280TaskNotificationFromUSB = %X %c\n", 
                       *( ( uint8_t * ) *( taskNotificationFromUSB ) + i ), 
                       *( ( uint8_t * ) *( taskNotificationFromUSB ) + i ) );
            }
        }

        /* Setting up writeData to arbitrarily make the payloadLength value in sx1280Setup 255
           Payload length does not matter for messages with headers */
        writeData = ( uint8_t * ) pvPortMalloc( 255*sizeof( uint8_t ) );
        for( uint32_t i = 0; i <= 254; i++){
            *( writeData + i ) = 0xFF;
            if( i == 254 ){
                *( writeData + i ) = 0x00;
            }
        }

        sx1280Setup( 0x00,          /* uint8_t standbyMode              */
                     0x01,          /* uint8_t packetType               */
                     0xB8,          /* uint8_t rfFrequency2316          */
                     0x9D,          /* uint8_t rfFrequency158           */
                     0x89,          /* uint8_t rfFrequency70            */
                     0x70,          /* uint8_t spreadingFactor          */
                     0x0A,          /* uint8_t bandwidth                */
                     0x01,          /* uint8_t codingRate               */
                     0x0C,          /* uint8_t preambleLength           */
                     0x00,          /* uint8_t headerType               */
                     0x20,          /* uint8_t cyclicalRedundancyCheck  */
                     0x40,          /* uint8_t chirpInvert              */
                     writeData );   /* uint8_t *outboundMessage         */

        vPortFree( writeData );

        /* Allocating one byte to readData, send to sx1280Rx to get the packets of a message */
        readData = ( uint8_t * ) pvPortMalloc( 1*sizeof( uint8_t ) );

        /* Assigning 0 to the 0th element of readData, no incoming message data if still 0 */
        *( readData ) = 0;

        sx1280Rx( 0x40,         /* uint8_t rxIrq158                 */
                  0x7E,         /* uint8_t rxIrq70                  */
                  0x02,         /* uint8_t rxPeriodBase             */
                  0xFF,         /* uint8_t rxPeriodBaseCount158     */
                  0xFF,         /* uint8_t rxPeriodBaseCount70      */
                  readData );   /* uint8_t *inboundMessage          */

        /* Checking message for "hi" in hexadecimal ascii in the first three bytes */
        if( *( readData ) == 0x68 && *( readData + 1 ) == 0x69 ){

            for( uint32_t i = 0; i <= 2; i++ ){
                printf( "Inbound Message: 0x%X\n", *( readData + i ) );
            }
        }
        else if( *( readData ) != 0 && ( *( readData + 3 ) != 0x68 && *( readData + 4 ) != 0x69 ) ){

            for( uint8_t i = 0; *( readData + i ) != 0x00; i++ ){
                printf("Inbound Message: 0x%X\n", *( readData + i ) );
            }
        }
        else if( *( readData ) == 0 ){

            printf("No Inbound Message");
        }
        vPortFree( readData );
        vPortFree( taskNotificationFromUSB );
        /* Explicitly setting taskNotificationFromUSB to NULL 
           vPortFree() does not set pointer to NULL only frees memory in heap to be reused */
        taskNotificationFromUSB = NULL;

        writeData = ( uint8_t * ) pvPortMalloc( 3*sizeof( uint8_t ) );
        *( writeData ) = 0x68;      /* "h"          */
        *( writeData + 3 ) = 0x69;  /* "i"          */
        *( writeData + 4 ) = 0x00;  /* "\0" or NULL */

        sx1280Setup( 0x00,          /* uint8_t standbyMode              */
                     0x01,          /* uint8_t packetType               */
                     0xB8,          /* uint8_t rfFrequency2316          */
                     0x9D,          /* uint8_t rfFrequency158           */
                     0x89,          /* uint8_t rfFrequency70            */
                     0x70,          /* uint8_t spreadingFactor          */
                     0x0A,          /* uint8_t bandwidth                */
                     0x01,          /* uint8_t codingRate               */
                     0x0C,          /* uint8_t preambleLength           */
                     0x00,          /* uint8_t headerType               */
                     0x20,          /* uint8_t cyclicalRedundancyCheck  */
                     0x40,          /* uint8_t chirpInvert              */
                     writeData );   /* uint8_t *outboundMessage         */

        sx1280Tx( 0x1F,         /* uint8_t power                    */
                  0xE0,         /* uint8_t rampTime                 */
                  writeData,    /* uint8_t *outboundMessage         */
                  0x40,         /* uint8_t txIrq158                 */
                  0x01,         /* uint8_t txIrq70                  */
                  0x02,         /* uint8_t txPeriodBase             */
                  0x01,         /* uint8_t txPeriodBaseCount158     */
                  0xF4 );       /* uint8_t txPeriodBaseCount70      */

        vPortFree( writeData );
    }
}

/* ----------------------------- Pi Pico Onboard LED Task ------------------------------- */

void vSimpleLEDTask( void *pvParameters ){

    gpio_init( PICO_DEFAULT_LED_PIN );
    gpio_set_dir( PICO_DEFAULT_LED_PIN, GPIO_OUT );

    while( true ){
        gpio_put( PICO_DEFAULT_LED_PIN, 1 );
        vTaskDelay( 100 );
        gpio_put( PICO_DEFAULT_LED_PIN, 0 );
        vTaskDelay( 100 );
    }
}

/* --------------------------- Serial Monitor USB IO Task -------------------------------- */

/*  Task running usb serial input
    Sends task notification to vSx1280Task with pointer to 
        buffer holding the message to be sent from the sx1280 */
void vUsbIOTask( void *pvParameters ){

    uint8_t currentChar = 0; /* 8 bit integer to hold hex value from getchar() */

    uint8_t *messageBuffer = NULL; /* 8 bit pointer to hold outgoing message */
    uint8_t *messageBufferRealloc = NULL; /* 8 bit pointer for dynamic message input */

    /* 32 bit integer for placing input characters in the correct places in messageBuffer */
    uint32_t messageCounter = 0;

    /* Allocating single byte to messageBuffer, done before realloc is called */
    messageBuffer = ( uint8_t * ) pvPortMalloc( 1*sizeof( uint8_t ) );

    /* Iterators */
    uint32_t i = 0;

    while( true ){

        /* Setting currentChar to the character being read in by getChar() */
        currentChar = getchar();
        /* Checking to see if the character being read in is not "\n" */
        if( currentChar != 0x0A ){
            /* Increasing messageBuffer size until "/n"
               messageCounter is indexed from 0, add one for correct sized message */
            messageBufferRealloc = messageBuffer;
            messageBuffer = ( uint8_t * ) pvPortMalloc( ( messageCounter+1 )*sizeof( uint8_t ));
            for( i = 0; i <= messageCounter; i++ ){
                *( messageBuffer + i ) = *( messageBufferRealloc + i );
            }
            /* Adding currentChar to the last cell in the pointer array */
            *( messageBuffer + messageCounter ) = currentChar;
            messageCounter = messageCounter + 1; /* Incrementing the value of messageCounter */
            vPortFree( messageBufferRealloc ); /* Freeing reallocation holder pointer */
        }
        else if( currentChar == 0x0A ){ /* Checking if the character being read in is "\n" */

            messageBufferRealloc = messageBuffer;
            /* An extra 8 bits is added to messageBuffer for "0x00", NULL terminated strings */
            messageBuffer = ( uint8_t * ) pvPortMalloc( ( messageCounter+2 )*sizeof( uint8_t ));
            for( i = 0; i <= messageCounter; i++ ){
                *( messageBuffer + i ) = *( messageBufferRealloc + i );
            }
            /* Adding currentChar to the last cell in the pointer array */
            *( messageBuffer + messageCounter ) = currentChar;
            *( messageBuffer + messageCounter + 1 ) = 0x00; 
            vPortFree( messageBufferRealloc ); /* Freeing reallocation holder pointer */

            for( i= 0; i<= messageCounter + 1; i++ ){
                printf( "Typed Message: 0x%X %c\n", *( messageBuffer + i ), 
                                                    *( messageBuffer + i ) );
            }

            xTaskNotify(
                xSx1280TaskHandle,                /* TaskHandle_t xTaskToNotify */ 
                ( uint32_t ) &*( messageBuffer ), /* uint32_t ulValue, (int)&buffer[0] */
                eSetValueWithoutOverwrite );      /* eNotifyAction eAction */

            messageCounter = 0;/* Setting messageCounter to 0 for new message to be taken in */
            messageBuffer = NULL; /* Set messageBuffer to NULL ensuring address isn't held  */ 
        }

        vTaskDelay( 10 );
    }
}


/* ------------------------------------- MAIN ------------------------------------------- */

int main( void ){

    stdio_init_all();

    uint32_t status = xTaskCreate(
                    vSimpleLEDTask,  
                    "Green Led",    
                    1024,           
                    NULL,           
                    1,              
                    &xSimpleLEDTaskHandle ); 

    uint32_t ioStatus = xTaskCreate(
                    vUsbIOTask,         /* TaskFunction_t pvTaskCode */
                    "Simple IO",        /* const char * const pcName */
                    4096,               /* uint16_t usStackDepth */
                    NULL,               /* void *pvParameters */
                    1,                  /* UBaseType_t uxPriority */
                    &xUsbIOTaskHandle );/*TaskHandle_t *pxCreatedTask*/

    uint32_t sx1280Status = xTaskCreate(
                    vSx1280Task,
                    "sx1280",
                    4096, /* usStackDepth * 4 = stack in bytes, because pico is 32 bits wide */
                    NULL,
                    1,
                    &xSx1280TaskHandle );

    vTaskStartScheduler();

    while( true ){

    }
}
