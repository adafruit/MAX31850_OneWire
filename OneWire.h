/*!
 * @file OneWire.h
 */
#ifndef OneWire_h
#define OneWire_h

#include <inttypes.h>

#if ARDUINO >= 100
#include "Arduino.h" // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"     // for delayMicroseconds
#include "pins_arduino.h" // for digitalPinToBitMask, etc
#endif

// You can exclude certain features from OneWire.  In theory, this
// might save some space.  In practice, the compiler automatically
// removes unused code (technically, the linker, using -fdata-sections
// and -ffunction-sections when compiling, and Wl,--gc-sections
// when linking), so most of these will not result in any code size
// reduction.  Well, unless you try to use the missing features
// and redesign your program to not need them!  ONEWIRE_CRC8_TABLE
// is the exception, because it selects a fast but large algorithm
// or a small but slow algorithm.

// you can exclude onewire_search by defining that to 0
#ifndef ONEWIRE_SEARCH
#define ONEWIRE_SEARCH 1 //!< Starts a onewire search
#endif

// You can exclude CRC checks altogether by defining this to 0
#ifndef ONEWIRE_CRC
#define ONEWIRE_CRC 1 //!< Runs the cyclic redundancy checks
#endif

// Select the table-lookup method of computing the 8-bit CRC
// by setting this to 1.  The lookup table enlarges code size by
// about 250 bytes.  It does NOT consume RAM (but did in very
// old versions of OneWire).  If you disable this, a slower
// but very compact algorithm is used.
#ifndef ONEWIRE_CRC8_TABLE
#define ONEWIRE_CRC8_TABLE                                                     \
  1 //!< Select the table-lookup method of computing the 8-bit CRC by setting
    //!< this to 1
#endif

// You can allow 16-bit CRC checks by defining this to 1
// (Note that ONEWIRE_CRC must also be 1.)
#ifndef ONEWIRE_CRC16
#define ONEWIRE_CRC16                                                          \
  1 //!< You can allow 16-bit CRC checks by defining this to 1
#endif

#define FALSE 0 //!< False
#define TRUE 1  //!< True

// Platform specific I/O definitions

#if defined(__AVR__)
#define PIN_TO_BASEREG(pin) (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin) (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define IO_REG_ASM asm("r30")
#define DIRECT_READ(base, mask) (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask) ((*((base) + 1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask) ((*((base) + 1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask) ((*((base) + 2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask) ((*((base) + 2)) |= (mask))

#elif defined(__MK20DX128__)
#define PIN_TO_BASEREG(pin) (portOutputRegister(pin))
#define PIN_TO_BITMASK(pin) (1)
#define IO_REG_TYPE uint8_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask) (*((base) + 512))
#define DIRECT_MODE_INPUT(base, mask) (*((base) + 640) = 0)
#define DIRECT_MODE_OUTPUT(base, mask) (*((base) + 640) = 1)
#define DIRECT_WRITE_LOW(base, mask) (*((base) + 256) = 1)
#define DIRECT_WRITE_HIGH(base, mask) (*((base) + 128) = 1)

#elif defined(__SAM3X8E__)
// Arduino 1.5.1 may have a bug in delayMicroseconds() on Arduino Due.
// http://arduino.cc/forum/index.php/topic,141030.msg1076268.html#msg1076268
// If you have trouble with OneWire on Arduino Due, please check the
// status of delayMicroseconds() before reporting a bug in OneWire!
#define PIN_TO_BASEREG(pin) (&(digitalPinToPort(pin)->PIO_PER))
#define PIN_TO_BITMASK(pin) (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask) (((*((base) + 15)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask) ((*((base) + 5)) = (mask))
#define DIRECT_MODE_OUTPUT(base, mask) ((*((base) + 4)) = (mask))
#define DIRECT_WRITE_LOW(base, mask) ((*((base) + 13)) = (mask))
#define DIRECT_WRITE_HIGH(base, mask) ((*((base) + 12)) = (mask))
#ifndef PROGMEM
#define PROGMEM
#endif
#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const uint8_t *)(addr))
#endif

#elif defined(__PIC32MX__)
#define PIN_TO_BASEREG(pin) (portModeRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin) (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)                                                \
  (((*(base + 4)) & (mask)) ? 1 : 0) // PORTX + 0x10
#define DIRECT_MODE_INPUT(base, mask)                                          \
  ((*(base + 2)) = (mask)) // TRISXSET +
                           // 0x08
#define DIRECT_MODE_OUTPUT(base, mask)                                         \
  ((*(base + 1)) = (mask)) // TRISXCLR + 0x04
#define DIRECT_WRITE_LOW(base, mask)                                           \
  ((*(base + 8 + 1)) = (mask)) // LATXCLR  + 0x24
#define DIRECT_WRITE_HIGH(base, mask)                                          \
  ((*(base + 8 + 2)) = (mask)) // LATXSET + 0x28

#elif defined(ARDUINO_ARCH_ESP8266)
#define PIN_TO_BASEREG(pin) ((volatile uint32_t *)GPO)
#define PIN_TO_BITMASK(pin) (1 << pin)
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask) ((GPI & (mask)) ? 1 : 0) // GPIO_IN_ADDRESS
#define DIRECT_MODE_INPUT(base, mask)                                          \
  (GPE &= ~(mask)) // GPIO_ENABLE_W1TC_ADDRESS
#define DIRECT_MODE_OUTPUT(base, mask)                                         \
  (GPE |= (mask))                                    // GPIO_ENABLE_W1TS_ADDRESS
#define DIRECT_WRITE_LOW(base, mask) (GPOC = (mask)) // GPIO_OUT_W1TC_ADDRESS
#define DIRECT_WRITE_HIGH(base, mask) (GPOS = (mask)) // GPIO_OUT_W1TS_ADDRESS

#elif defined(__SAMD21G18A__) || defined(__SAMD21E18A__)
#define PIN_TO_BASEREG(pin) portModeRegister(digitalPinToPort(pin))
#define PIN_TO_BITMASK(pin) (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask) (((*((base) + 8)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask) ((*((base) + 1)) = (mask))
#define DIRECT_MODE_OUTPUT(base, mask) ((*((base) + 2)) = (mask))
#define DIRECT_WRITE_LOW(base, mask) ((*((base) + 5)) = (mask))
#define DIRECT_WRITE_HIGH(base, mask) ((*((base) + 6)) = (mask))

#elif defined(__SAMD51__)
#define PIN_TO_BASEREG(pin) portModeRegister(digitalPinToPort(pin))
#define PIN_TO_BITMASK(pin) (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask) (((*((base) + 8)) & (mask)) ? 1 : 0) // IN
#define DIRECT_MODE_INPUT(base, mask) ((*((base) + 1)) = (mask))     // DIRCLR
#define DIRECT_MODE_OUTPUT(base, mask) ((*((base) + 2)) = (mask))    // DIRSET
#define DIRECT_WRITE_LOW(base, mask) ((*((base) + 5)) = (mask))      // OUTCLR
#define DIRECT_WRITE_HIGH(base, mask) ((*((base) + 6)) = (mask))     /// OUTSET

#elif defined(ARDUINO_NRF52_ADAFRUIT)
#define PIN_TO_BASEREG(pin) (0)
#define PIN_TO_BITMASK(pin) digitalPinToPinName(pin)
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, pin) nrf_gpio_pin_read(pin)
#define DIRECT_WRITE_LOW(base, pin) nrf_gpio_pin_clear(pin)
#define DIRECT_WRITE_HIGH(base, pin) nrf_gpio_pin_set(pin)
#define DIRECT_MODE_INPUT(base, pin)                                           \
  nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_NOPULL)
#define DIRECT_MODE_OUTPUT(base, pin) nrf_gpio_cfg_output(pin)

#else
#error "Please define I/O register types here"
#endif

/*!
 * @brief Stores the state and functions for Max31850 OneWire
 */
class OneWire {
private:
  IO_REG_TYPE bitmask;
  volatile IO_REG_TYPE *baseReg;

#if ONEWIRE_SEARCH
  // global search state
  unsigned char ROM_NO[8];
  uint8_t LastDiscrepancy;
  uint8_t LastFamilyDiscrepancy;
  uint8_t LastDeviceFlag;
#endif

public:
  /*!
   * @brief OneWire constructor
   * @param pin Pin to use
   */
  OneWire(uint8_t pin);

  /*!
   * @brief Perform a 1-Wire reset cycle. Returns 1 if a device responds
   * with a presence pulse.  Returns 0 if there is no device or the
   * bus is shorted or otherwise held low for more than 250uS
   * @returns if the reset was successful
   */
  uint8_t reset(void);

  /*!
   * @brief Issue a 1-Wire rom select command, you do the reset first.
   * @param rom ROM to select
   */
  void select(const uint8_t rom[8]);

  /*!
   * @brief Issue a 1-Wire rom skip command, to address all on bus.
   */
  void skip(void);

  /*!
   * @brief Write a byte. If 'power' is one then the wire is held high at
   * the end for parasitically powered devices. You are responsible
   * for eventually depowering it by calling depower() or doing
   * another read or write.
   * @param v Byte to write
   * @param power If you need power after the write then set 'power' to 1.
   */
  void write(uint8_t v, uint8_t power = 0);

  /*!
   * @brief Writes bytes
   * @param buf Buffer to read from
   * @param count how many bytes to read
   * @param power IF you need power after the write then set power to 1.
   */
  void write_bytes(const uint8_t *buf, uint16_t count, bool power = 0);

  /*!
   * @brief Read a byte.
   * @return Returns the read byte
   */
  uint8_t read(void);

  /*!
   * @brief Reads bytes
   * @param buf Buffer to read from
   * @param count How many bytes to read
   */
  void read_bytes(uint8_t *buf, uint16_t count);

  /*!
   * @brief Write a bit. The bus is always left powered at the end, see
   *note in write() about that.
   * @param v Bit to write
   */
  void write_bit(uint8_t v);

  /*!
   * @brief Read a bit.
   * @return Returns the read bit
   */
  uint8_t read_bit(void);

  /*!
   * @brief Stop forcing power onto the bus. You only need to do this if
   * you used the 'power' flag to write() or used a write_bit() call
   * and aren't about to do another read or write. You would rather
   * not leave this powered if you don't have to, just in case
   * someone shorts your bus.
   */
  void depower(void);

#if ONEWIRE_SEARCH
  /*!
   * @brief Clear the search state so that if will start from the beginning
   * again.
   */
  void reset_search();

  /*!
   * @brief Setup the search to find the device type 'family_code' on the next
   * call to search(*newAddr) if it is present.
   * @param family_code What to search for
   */
  void target_search(uint8_t family_code);

  /*!
   * @brief Look for the next device. Returns 1 if a new address has been
   * returned. A zero might mean that the bus is shorted, there are
   * no devices, or you have already retrieved all of them.  It
   * might be a good idea to check the CRC to make sure you didn't
   * get garbage.  The order is deterministic. You will always get
   * the same devices in the same order.
   * @param newAddr Address to search for
   * @return Returns true if device is found, false if it is not
   */
  uint8_t search(uint8_t *newAddr);
#endif

#if ONEWIRE_CRC
  /*!
   * @brief Compute a Dallas Semiconductor 8 bit CRC, these are used in the
   * ROM and scratchpad registers.
   * @param addr Buffer address to read from
   * @param len How much to read
   * @return Returns the 8-bit CRC
   */
  static uint8_t crc8(const uint8_t *addr, uint8_t len);

#if ONEWIRE_CRC16
  /*!
   * @brief Compute the 1-Wire CRC16 and compare it against the received CRC.
   * Example usage (reading a DS2408):
   *    // Put everything in a buffer so we can compute the CRC easily.
   *    uint8_t buf[13];
   *    buf[0] = 0xF0;    // Read PIO Registers
   *    buf[1] = 0x88;    // LSB address
   *    buf[2] = 0x00;    // MSB address
   *    WriteBytes(net, buf, 3);    // Write 3 cmd bytes
   *    ReadBytes(net, buf+3, 10);  // Read 6 data bytes, 2 0xFF, 2 CRC16
   *    if (!CheckCRC16(buf, 11, &buf[11])) {
   *        // Handle error.
   *    }
   *
   * @param input Array of bytes to checksum.
   * @param len How many bytes to use.
   * @param inverted_crc The two CRC16 bytes in the received data.
   *                       This should just point into the received data,
   *                       *not* at a 16-bit integer.
   * @param crc The crc starting value (optional)
   * @return True, if the CRC matches.
   */
  static bool check_crc16(const uint8_t *input, uint16_t len,
                          const uint8_t *inverted_crc, uint16_t crc = 0);

  /*!
   * @brief Compute a Dallas Semiconductor 16 bit CRC.  This is required to
   * check the integrity of data received from many 1-Wire devices.  Note that
   * the CRC computed here is *not* what you'll get from the 1-Wire network, for
   * two reasons: 1) The CRC is transmitted bitwise inverted. 2) Depending on
   * the endian-ness of your processor, the binary representation of the
   * two-byte return value may have a different byte order than the two bytes
   * you get from 1-Wire.
   * @param input Array of bytes to checksum.
   * @param len How many bytes to use.
   * @param crc The crc starting value (optional)
   * @return Returns the CRC16, as defined by Dallas Semiconductor.
   */
  static uint16_t crc16(const uint8_t *input, uint16_t len, uint16_t crc = 0);
#endif
#endif
};

#endif
