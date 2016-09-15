This is a simple driver for DAVICOM DM9000 A/B/BI Ethernet MACs running on
8-bit Atmel AVR microcontrollers I wrote some years ago. Tested and running
for hours with continuous data flow on multiple ATmega128.

Usage
-----
* Adjust dm9000.h to your needs:
  * Change `DM9000_DATA_IN_PORT`, `DM9000_DATA_OUT_PORT`, and `DM9000_DDR_REG` to 
    the `PIN`, `PORT`, and `DDR` registers for the AVR I/O port that is connected
    to the data bus of the MAC. The data pins `SD0` to `SD7` must be connected to
    exactly one AVR I/O port and have their pin numbering match, i.e. connect `SD3`
    to `PA3` if you use `PORTA`. Using the DM9000 pins `SD8` to `SD15` for 16-bit
    I/O is not supported. If you use an AVR port with special functions for this,
    you need to make sure they are disabled.
  * Adjust `NET_MAC_1` to `NET_MAC_6` to the desired MAC address of your device.
  * Change `DM9000_CS_PORT` and `DM9000_CS_PIN` to the port and pin definition
    you connected the DM9000 `CS#` pin to.
  * Change `DM9000_IOR_PORT` and `DM9000_IOR_PIN` to the port and pin definition
    you connected the DM9000 `IOR#` pin to.
  * Change `DM9000_IOW_PORT` and `DM9000_IOW_PIN` to the port and pin definition
    you connected the DM9000 `IOW#` pin to.
  * Change `DM9000_CMD_PORT` and `DM9000_CMD_PIN` to the port and pin definition
    you connected the DM9000 `CMD` pin to.
* Make sure the DM9000 is configured to use a data bus width of 8 bit and a `CS#`
  polarity of active low as nothing else is supported. Check the data sheet for
  details on how to do this using the strap pins.
* Add `dm9000.c` into your firmware.
* Include `dm9000.h` somewhere in your program and use it:
  * Call `ethernet_init_chip` at some point in your global initialization routine.
    Also call `ethernet_activate_rx` if you want to receive packets.
  * If you want to transmit a packet: Call `ethernet_buffer_tx_sync` with a pointer
    to the data to send and the length of the buffer. You can call it multiple times
    to gradually fill the buffer, but all data has to stay in the same packet. Then
    call `ethernet_tx_async` which will actually start the transmission. It will
    return immediately after instructing the DM9000 to transmit and not wait for
    completion. It will, however, wait for the previous packet to finish transmission
    before sending a new one.
  * If you want to receive a packet: Call `ethernet_is_packet_ready` periodically
    to check whether a packet has been received (interrupts are not supported, but
    you can easily wire it up on your own if necessary).
    Then call `ethernet_rx_sync` with a pointer to a buffer that will receive the
    data and the length of the buffer, which is also the number of data bytes
    to read from the DM9000-internal RX ring buffer. You have to interpret the
    data yourself, which means that you will first want to read the `DM9000_RX_HEADER_SIZE`
    bytes that make up the DM9000 packet header. This tells you how many data bytes
    were received, which you can then again fetch via `ethernet_rx_sync`. If you
    don't want the packet, you still have to read it out in order to free space
    in the DM9000 RX buffer. You may use `ethernet_dump_rx_sync` if you have to
    skip some bytes without actually saving them anywhere at all.
  * If you need to do fancy things, you can use `ethernet_read_register`,
    `ethernet_write_register`, `ethernet_read_phy_register` and `ethernet_write_phy_register`
    to directly affect the internal workings of the DM9000. Be aware that this
    might interfere with the operation of this code, so be careful.
  * The MAC address is initialized with the data from `NET_MAC_1` etc. in `dm9000.h`.
    If you want to change it later on, use `ethernet_set_mac`.

Notes
-----
* All TX/RX data is entirely raw. Nothings is done to the data at all, which means
  you have to send/receive full Ethernet MAC-layer frames. For RX packets, the
  DM9000 4-byte packet header and the CRC32 trailer is also included.
* When receiving data, only trust the packet size advertised in the DM9000 packet
  header and not in any additional headers (IP etc.) for determining how much
  bytes you have to be read.
* Be sure to correctly set the MAC address. The DM9000 will be configured to drop
  all received packets that are not addressed either to the set address or the
  broadcast address by default.
* The throughput will obviously not reach 100 Mbps because of the bitbanging, but
  it should be enough for simple control applications.
* The code is very light on checking error conditions, so it's mainly useful for
  hobbyists.
  
Links
-----
[Davicom Ethernet product page with datasheets](http://www.davicom.tw/page1.aspx?no=143762)
