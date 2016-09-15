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

// #define HAVE_LCD

#ifdef HAVE_LCD
# include "lcd.h"
#endif
#include "dm9000.h"
//#include "global.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define _IOR_ON()   { DM9000_IOR_PORT &= ~(1 << DM9000_IOR_PIN); }
#define _IOR_OFF()  { DM9000_IOR_PORT |=  (1 << DM9000_IOR_PIN); }
#define _IOW_ON()   { DM9000_IOW_PORT &= ~(1 << DM9000_IOW_PIN); }
#define _IOW_OFF()  { DM9000_IOW_PORT |=  (1 << DM9000_IOW_PIN); }
#define _REG_DATA() { DM9000_CMD_PORT |=  (1 << DM9000_CMD_PIN); }
#define _REG_INDEX(){ DM9000_CMD_PORT &= ~(1 << DM9000_CMD_PIN); }
#define _CS_ON()    { DM9000_CS_PORT  &= ~(1 << DM9000_CS_PIN); }
#define _CS_OFF()   { DM9000_CS_PORT  |=  (1 << DM9000_CS_PIN); }

#define _DO_DATA_IN() { DM9000_DDR_REG = 0; DM9000_DATA_OUT_PORT = 0xFF; }
#define _DATA_OUT(z)  { DM9000_DATA_OUT_PORT = z; DM9000_DDR_REG = 0xFF; }

#define _DO_CS() { _CS_ON(); _CS_OFF(); }

#define _SET_REGISTER(y) { _REG_INDEX(); _IOW_ON(); _DATA_OUT(y); _DO_CS(); _IOW_OFF(); }

uint8_t ethernet_read_register(uint8_t reg)
{
	uint8_t regval = 0;
	_SET_REGISTER(reg);
	// DM9000 IOW-to-IOR-delay should be satisfied by the two following commands already, so no extra NOP
	
	_DO_DATA_IN();
	_CS_ON();
	_REG_DATA();
	_IOR_ON();
	// IMPORTANT! Let values propagate to register (sync delay)
	asm volatile ("nop");
	// Get value
	regval = DM9000_DATA_IN_PORT;
	_IOR_OFF();
	_CS_OFF();

	return regval;
}

void ethernet_write_register(DM9000_REG reg, uint8_t val)
{
	_SET_REGISTER(reg);

	_REG_DATA();
	_IOW_ON();
	// Set value
	_DATA_OUT(val);
	_DO_CS();
	_IOW_OFF();
}

uint16_t ethernet_read_phy_register(DM9000_PHYREG reg)
{
	// FIXME: throw error
	if (reg > 0b00111111)
		return 0;
	// Write register address
	ethernet_write_register(DM9_EPAR, 0b01000000 + reg);
	// PHY Register Read Command
	ethernet_write_register(DM9_EPCR, 0b00001100);
	
	// Wait for completion
	uint8_t watchdog = 0;
	uint8_t epcr;
	do
	{
		_delay_us(1.0);
		epcr = ethernet_read_register(DM9_EPCR);
		watchdog++;
	} while (watchdog < 200 && (epcr & (0b00000001)));
	_delay_us(5.0);
	
	// Clear command
	ethernet_write_register(DM9_EPCR, 0b00001000);
	
	// Get data
	uint16_t retval = ethernet_read_register(DM9_EPDRL) + (ethernet_read_register(DM9_EPDRH) << 8);
	return retval;
}

void ethernet_write_phy_register(DM9000_PHYREG reg, uint16_t val)
{
	// FIXME: throw error
	if (reg > 0b00111111)
		return;
	// Write register address
	ethernet_write_register(DM9_EPAR, 0b01000000 + reg);
	// Write data
	ethernet_write_register(DM9_EPDRL, (uint8_t) (val));
	ethernet_write_register(DM9_EPDRH, (uint8_t) (val >> 8));
	// PHY Register Write Command
	ethernet_write_register(DM9_EPCR, 0b00001010);
	
	// Wait for completion
	uint8_t watchdog = 0;
	uint8_t epcr;
	do
	{
		_delay_us(1.0);
		epcr = ethernet_read_register(DM9_EPCR);
		watchdog++;
	} while (watchdog < 200 && (epcr & (0b00000001)));
	_delay_us(5.0);
	
	// Clear command
	ethernet_write_register(DM9_EPCR, 0b00001000);
}

static void _ethernet_reset(void)
{
	ethernet_write_register(DM9_NCR, 0b00000001);
	
	uint8_t watchdog = 0;
	uint8_t ncr = 0;	
	do
	{
		_delay_ms(1.0);
		ncr = ethernet_read_register(DM9_NCR);
		watchdog++;
	} while (watchdog < 200 && (ncr & 0b00000001));

#ifdef HAVE_LCD	
	if (watchdog >= 200)
	{
		cli();
		lcd_return_home();
		lcd_write_string("!! DM9000 is not  !!");
		lcd_set_dd_ram_address(LCD_DD_RAM_SECOND_ROW);
		lcd_write_string("!!   responding   !!");
		for (;;)
			asm volatile ("nop");
	}
#endif
}

static void _ethernet_reset_phy(void)
{
	ethernet_write_phy_register(DM9PHY_BMCR, 0b1000000000000000);
	
	uint8_t watchdog = 0;
	uint16_t bmcr = 0;
	do
	{
		_delay_us(10.0);
		bmcr = ethernet_read_phy_register(DM9PHY_BMCR);
		watchdog++;
	} while (watchdog < 200 && (bmcr & 0b1000000000000000));
	
#ifdef HAVE_LCD
	if (watchdog >= 200)
	{
		cli();
		lcd_return_home();
		lcd_write_string("DM9000 PHY is not");
		lcd_set_dd_ram_address(LCD_DD_RAM_SECOND_ROW);
		lcd_write_string("answering");
		for (;;)
			asm volatile ("nop");
	}
#endif
}

void ethernet_set_mac(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4, uint8_t m5, uint8_t m6)
{
	ethernet_write_register(DM9_PAR1, m1);
	ethernet_write_register(DM9_PAR2, m2);
	ethernet_write_register(DM9_PAR3, m3);
	ethernet_write_register(DM9_PAR4, m4);
	ethernet_write_register(DM9_PAR5, m5);
	ethernet_write_register(DM9_PAR6, m6);
}

void ethernet_activate_rx(void)
{
	uint8_t rcr = ethernet_read_register(DM9_RCR);
	ethernet_write_register(DM9_RCR, rcr | 0b00000001);
}

void ethernet_deactivate_rx(void)
{
	uint8_t rcr = ethernet_read_register(DM9_RCR);
	ethernet_write_register(DM9_RCR, rcr & ~0b00000001);
}

void ethernet_init_chip(void)
{
	// Power up PHY: Clear PHYPD
	ethernet_write_register(DM9_GPR, 0b00000000);
	// Wait for PHY
	_delay_ms(60.0);
	
	_ethernet_reset();
	_ethernet_reset_phy();
	
	// Set normal mode: Clear LBK
	ethernet_write_register(DM9_NCR, 0b00000000);
	
	// Special mode
	ethernet_write_register(DM9_SMCR, 0);
	
#ifdef HAVE_LCD
	// Query IO mode
	uint8_t isr = ethernet_read_register(DM9_ISR);
	if (!(isr & (1 << DM9ISR_IOMODE)) || (isr & (1 << DM9ISR_RESERVED)))
	{
		lcd_display_clear();
		lcd_return_home();
		cli();
		lcd_write_string("DM9000 IO width is");
		lcd_set_dd_ram_address(LCD_DD_RAM_SECOND_ROW);
		lcd_write_string("not 8 bit (");
		if (isr & (1 << DM9ISR_IOMODE))
			lcd_write_char('1');
		else
			lcd_write_char('0');
		if (isr & (1 << DM9ISR_RESERVED))
			lcd_write_char('1');
		else
			lcd_write_char('0');
		lcd_write_char(')');
		for (;;) asm volatile ("nop");
	}
#endif
	
	// Activate pointer wrap: Set PAR
	// Activate interrupts: packet received, packet transmitted, receive overflow, receive overflow counter overflow, transmit under-run
	ethernet_write_register(DM9_IMR, 0b10011111);
	
	ethernet_set_mac(NET_MAC_1, NET_MAC_2, NET_MAC_3, NET_MAC_4, NET_MAC_5, NET_MAC_6);
	
	// Enable flow control
	ethernet_write_register(DM9_FCR, 0b00101001);
	
	// Write hash table
	ethernet_write_register(DM9_MAR1, 0);
	ethernet_write_register(DM9_MAR2, 0);
	ethernet_write_register(DM9_MAR3, 0);
	ethernet_write_register(DM9_MAR4, 0);
	ethernet_write_register(DM9_MAR5, 0);
	ethernet_write_register(DM9_MAR6, 0);
	ethernet_write_register(DM9_MAR7, 0);
	// Receive broadcast
	ethernet_write_register(DM9_MAR8, 0x80);
	
	// Activate UDP and IP checksum generation
	ethernet_write_register(DM9_TCSCR, 0b00000101);
	
	// Activate receive checksum checking
	ethernet_write_register(DM9_RCSCSR, 0b00000011);
	// Discard packet on CRC error or length > 1522 bytes
	ethernet_write_register(DM9_RCR, 0b00110000);
	
	// Back pressure
	ethernet_write_register(DM9_BPTR, 0b00100101);
	ethernet_write_register(DM9_FCTR, 0b00100100);
	
	// Activate re-transmission at late collision
	ethernet_write_register(DM9_TCR2, 0b01000000);
	
	// Clear flags: TX1END, TX2END, WAKEST
	ethernet_write_register(DM9_NSR, 0b00101100);
	// PR, PT, ROS, ROO, UDRUN, LNKCHNG
	ethernet_write_register(DM9_ISR, 0b00111111);
	
	// Setup PHY: Advertise flow control, 100BASE-TX/10BASE-T Full/Half duplex
	                                              //10111100001
	ethernet_write_phy_register(DM9PHY_ANAR, 0b0000010111100001);
	// Restart auto-negotiation
//	                                             11001000000000
	ethernet_write_phy_register(DM9PHY_BMCR, 0b0011001100000000);
}

uint8_t ethernet_is_packet_ready(void)
{
	// Take a peek
	// Dummy read
	_SET_REGISTER(DM9_MRCMDX);
	_REG_DATA();
	_DO_DATA_IN();
	_CS_ON();
	_IOR_ON();
	asm volatile ("nop");
	_IOR_OFF();
	_CS_OFF();
	
	_CS_ON();
	_IOR_ON();
	asm volatile ("nop");
	uint8_t b = DM9000_DATA_IN_PORT;
	_IOR_OFF();
	_CS_OFF();
	
	if ((b & 0b00000011) == 0b01)
		return 1;
		
	return 0;
}

void ethernet_rx_sync(uint8_t* data, const uint16_t len)
{
	// Memory Data Read Command with Address Increment Register
	_SET_REGISTER(DM9_MRCMD);
	
	_REG_DATA();
	_DO_DATA_IN();
	
	uint16_t i;
	for (i = 0; i < len; i++)
	{
		// Read DATA port
		_CS_ON();
		_IOR_ON();
		asm volatile ("nop");
		*data++ = DM9000_DATA_IN_PORT;
		_IOR_OFF();
		_CS_OFF();
	}
}

void ethernet_dump_rx_sync(const uint16_t len)
{
	uint16_t mrr = (ethernet_read_register(DM9_MRRH) << 8) + (ethernet_read_register(DM9_MRRL));
	mrr += len;
	// !!!!! Wrap around?
	// Have to receive 1 byte to check MAC header
	ethernet_write_register(DM9_MRRL, (uint8_t) mrr);
	ethernet_write_register(DM9_MRRH, (uint8_t) (mrr >> 8));
}


uint16_t _g_tx_len_counter = 0;

void ethernet_buffer_tx_sync(const uint8_t* const data, const uint16_t len)
{
	// Memory Data Write Command with Address Increment Register
	_SET_REGISTER(DM9_MWCMD);
	_REG_DATA();
	
	uint16_t i = 0;
	const uint8_t* data_ptr = data;
	for (i = 0; i < len; i++)
	{
		// Set DATA port
		_IOW_ON();
		_DATA_OUT(*data_ptr++);
		_DO_CS();
		_IOW_OFF();
	}
	
	_g_tx_len_counter += len;
}

void ethernet_tx_async(void)
{
	uint8_t watchdog = 0;
	uint8_t tcr = ethernet_read_register(DM9_TCR);
	while (watchdog < 200 && (tcr & 0b00000001))
	{
		// FIXME: If this fails, try to use NSR and TX1END/TX2END instead
		tcr = ethernet_read_register(DM9_TCR);
		watchdog++;
		_delay_us(1.0);
	} 
	if (watchdog >= 200)
	{
		watchdog = 0;
		do
		{
			// Wait for 5s
			_delay_ms(25.0);
			// FIXME: If this fails, try to use NSR and TX1END/TX2END instead
			tcr = ethernet_read_register(DM9_TCR);
			watchdog++;
		} while (watchdog < 200 && (tcr & 0b00000001));
#ifdef HAVE_LCD
		if (watchdog >= 200)
		{
			cli();
			lcd_return_home();
			lcd_write_string("DM9000 TX timed");
			lcd_set_dd_ram_address(LCD_DD_RAM_SECOND_ROW);
			lcd_write_string("out");
			for (;;)
				asm volatile ("nop");
		}
#endif
	}
	
	// Clear flags
//	ethernet_write_register(DM9_NSR, (0 << 2) | (0 << 3));
	
	// Set packet length
	ethernet_write_register(DM9_TXPLH, (uint8_t) (_g_tx_len_counter >> 8));
	ethernet_write_register(DM9_TXPLL, (uint8_t) _g_tx_len_counter);
	
	_g_tx_len_counter = 0;
	
	// Activate transmission: Set TXREQ
	ethernet_write_register(DM9_TCR, 0b00000001);
}
