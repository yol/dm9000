/*  *************************************************************************

    DM9000A/B/BI (8-bit mode) driver for AVR CPUs
    
    Copyright 2009 Philipp Kerling

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

   ************************************************************************* */

#ifndef DM9000_H_INCLUDED
#define DM9000_H_INCLUDED

#include <stdint.h>
#include <avr/io.h>

#define DM9000_DATA_IN_PORT  PINA
#define DM9000_DATA_OUT_PORT PORTA
#define DM9000_DDR_REG       DDRA

#define NET_MAC_1 0x00
#define NET_MAC_2 0x00
#define NET_MAC_3 0x6C
#define NET_MAC_4 0x00
#define NET_MAC_5 0x00
#define NET_MAC_6 0x02

#define DM9000_CS_PORT  PORTC
#define DM9000_CS_PIN   PC3
#define DM9000_IOR_PORT PORTC
#define DM9000_IOR_PIN  PC5
#define DM9000_IOW_PORT PORTC
#define DM9000_IOW_PIN  PC4
#define DM9000_CMD_PORT PORTC
#define DM9000_CMD_PIN  PC7
#define DM9000_INT_PORT PINC
#define DM9000_INT_PIN  PC6

#define DM9000_RX_HEADER_SIZE 4
#define DM9000_CRC_PAYLOAD_SIZE 4

enum _DM9000_REG
{
	DM9_NCR = 0x00,
	DM9_NSR,
	DM9_TCR,
	DM9_TSR1,
	DM9_TSR2,
	DM9_RCR,
	DM9_RSR,
	DM9_ROCR,
	DM9_BPTR,
	DM9_FCTR,
	DM9_FCR,
	DM9_EPCR,
	DM9_EPAR,
	DM9_EPDRL,
	DM9_EPDRH,
	DM9_WCR,
	DM9_PAR1,
	DM9_PAR2,
	DM9_PAR3,
	DM9_PAR4,
	DM9_PAR5,
	DM9_PAR6,
	DM9_MAR1,
	DM9_MAR2,
	DM9_MAR3,
	DM9_MAR4,
	DM9_MAR5,
	DM9_MAR6,
	DM9_MAR7,
	DM9_MAR8,
	DM9_GPCR,
	DM9_GPR,
	DM9_TRPAL = 0x22,
	DM9_TRPAH,
	DM9_RWPAL,
	DM9_RWPAH,
	DM9_VIDL = 0x28,
	DM9_VIDH,
	DM9_PIDL,
	DM9_PIDH,
	DM9_CHIPR,
	DM9_TCR2,
	DM9_OCR,
	DM9_SMCR,
	DM9_ETXCSR,
	DM9_TCSCR,
	DM9_RCSCSR,
	DM9_MPAR,
	DM9_LEDCR,
	DM9_BUSCR = 0x38,
	DM9_INTCR,
	DM9_SCCR = 0x50,
	DM9_RSCCR,
	DM9_MRCMDX = 0xF0,
	DM9_MRCMDX1,
	DM9_MRCMD,
	DM9_MRRL = 0xF4,
	DM9_MRRH,
	DM9_MWCMDX,
	DM9_MWCMD = 0xF8,
	DM9_MWRL = 0xFA,
	DM9_MWRH,
	DM9_TXPLL,
	DM9_TXPLH,
	DM9_ISR,
	DM9_IMR,
};

enum _DM9000_PHYREG
{
	DM9PHY_BMCR = 0,
	DM9PHY_BMSR,
	DM9PHY_PHYID1,
	DM9PHY_PHYID2,
	DM9PHY_ANAR,
	DM9PHY_ANLPAR,
	DM9PHY_ANER,
	DM9PHY_DSCR = 16,
	DM9PHY_DSCSR,
	DM9PHY_10BTCSR,
	DM9PHY_PWDOR,
	DM9PHY_SPCONF,
};

enum _DM9000_NSR
{
	DM9NSR_RXOV = 1,
	DM9NSR_TX1END,
	DM9NSR_TX2END,
	DM9NSR_WAKEST = 5,
	DM9NSR_LINKST,
	DM9NSR_SPEED,
};

enum _DM9000_RSR
{
	DM9RSR_FOE = 0,
	DM9RSR_CE,
	DM9RSR_AE,
	DM9RSR_PLE,
	DM9RSR_RWTO,
	DM9RSR_LCS,
	DM9RSR_MF,
	DM9RSR_RF,
};

enum _DM9000_ISR
{
	DM9ISR_PR = 0,
	DM9ISR_PT,
	DM9ISR_ROS,
	DM9ISR_ROO,
	DM9ISR_UDRUN,
	DM9ISR_LNKCHG,
	DM9ISR_RESERVED,
	DM9ISR_IOMODE = 7,
};

enum _DM9000_IMR
{
	DM9IMR_PRI = 0,
	DM9IMR_PTI,
	DM9IMR_ROI,
	DM9IMR_ROOI,
	DM9IMR_UDRUNI,
	DM9IMR_LNKCHGI,
	DM9IMR_PAR = 7,
};

struct _IncomingPacketHeader
{
	/* Ethernet */
	uint8_t destination_address[6];
	uint8_t source_address[6];
	uint8_t packet_type_h;
	uint8_t packet_type_l;
	
	/* IP */
	uint8_t ip_version_and_header_length;
	uint8_t ip_type_of_service;
	uint16_t total_packet_length;
	uint16_t ip_identification;
	uint8_t ip_flags_and_fragment_offset_h;
	uint8_t ip_fragment_offset_l;
	uint8_t ip_time_to_live;
	uint8_t ip_protocol;
	uint16_t ip_header_checksum;
	uint8_t ip_source_address[4];
	uint8_t ip_destination_address[4];
	
	/* UDP */
	uint16_t source_port;
	uint8_t dest_port_h;
	uint8_t dest_port_l;
	uint8_t length_h;
	uint8_t length_l;
	uint16_t checksum;
};

typedef enum _DM9000_REG DM9000_REG;
typedef enum _DM9000_PHYREG DM9000_PHYREG;
typedef struct _IncomingPacketHeader IncomingPacketHeader;

uint8_t ethernet_read_register(DM9000_REG reg);
void ethernet_write_register(DM9000_REG reg, uint8_t val);
uint16_t ethernet_read_phy_register(DM9000_PHYREG reg);
void ethernet_write_phy_register(DM9000_PHYREG reg, uint16_t val);
uint8_t ethernet_is_packet_ready(void);
void ethernet_set_mac(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4, uint8_t m5, uint8_t m6);
void ethernet_init_chip(void);
void ethernet_activate_rx(void);
void ethernet_deactivate_rx(void);

void ethernet_rx_sync(uint8_t* data, const uint16_t len);
void ethernet_dump_rx_sync(const uint16_t len);
void ethernet_buffer_tx_sync(const uint8_t* const buf_start, const uint16_t len);
void ethernet_tx_async(void);

#endif
