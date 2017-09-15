/*
    Copyright 2016 fishpepper <AT> gmail.com

    This program is free software: you can redistribute it and/ or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.

    author: fishpepper <AT> gmail.com
*/

/*
    the hex file will be encrypted data
    this will be stored in flash:
    BOOTLOADER | ENCRYPTED_DATA | PARAMS:32Byte

    on boot we will check
        if PARAMS[ENCRYPTED_SIGNATURE] = 0xDEADF158 -> decrypt flash
        if PARAMS[CRC_SIGNATURE] = CRC(FLASH) -> boot
        else stay in bl

 */


#include "bootloader.h"
#include "config.h"
#include "aes.h"

#ifndef BUILD_TEST
  #include "main.h"
  #include "serial.h"
  #include "macros.h"
  #include "led.h"

  #include <libopencm3/stm32/flash.h>
  #include <libopencm3/stm32/crc.h>
  #include <libopencmsis/core_cm3.h>

#endif  // BUILD_TEST

#ifndef DEBUG_PRINTF
#define DEBUG_PRINTF(format, ...) {}
#endif

#include <stdint.h>
#include <stdbool.h>

static uint8_t bootloader_secret_key[] = {
    0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71, 0xbe,
    0x2b, 0x73, 0xae, 0xf0, 0x85, 0x7d, 0x77, 0x81,
    0x1f, 0x35, 0x2c, 0x07, 0x3b, 0x61, 0x08, 0xd7,
    0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf, 0xf4
};

static uint8_t bootloader_magic_signature[] = {
    0xAF, 0xFE, 0xDE, 0xAD, 0xBA, 0xBE, 0x33, 0x44,
    0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc
};

// initial IV
static uint8_t bootloader_iv[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};


void bootloader_init(void) {
    // disable global interrupts
    GLOBAL_INT_DISABLE();
}

static bool bootloader_valid_address(uint32_t address) {
    // verify if this is within allowed memory bounds:
    if (address < (BOOTLOADER_APP_START)) {
        // inside bootloader space -> invalid!
        DEBUG_PRINTF("invalid address 0x%X (inside bootloader)\n", address);
        return false;
    }
    if (address >= (BOOTLOADER_APP_END)) {
        // exceeds flash size -> invalid!
        DEBUG_PRINTF("invalid address 0x%X (>flash size)\n", address);
        return false;
    }

    // safe!
    return true;
}

static bool bootloader_write_buffer16(uint32_t address, uint16_t *data, uint32_t len) {
    DEBUG_PRINTF("write flash 0x%X (len = %d) = 0x%X\n", address, len, *data);
    uint32_t i;

    for (i = 0; i < len; i++) {
        // verify if this is within allowed memory bounds:
        if (!bootloader_valid_address(address)) {
            return false;
        }

        // execute
        flash_unlock();
        flash_program_half_word(address, data[i]);
        flash_lock();

        // prepare next address:
        address += 2;  // 2 * 8bit
    }

    return true;
}

static void bootloader_decrypt_flash(void) {
    DEBUG_PRINTF("decrypting flash...\n");
    // bootloader has written the flash with encrypted data,
    // decrypt it pagewise
    uint8_t decrypted_buffer[DEVICE_PAGE_SIZE];

    // init key/iv
    uint8_t *key_ptr = bootloader_secret_key;
    uint8_t *iv_ptr  = bootloader_iv;

    uint32_t flash_address = BOOTLOADER_APP_START;
    bool last_page = false;

    while (flash_address < BOOTLOADER_APP_END) {
        DEBUG_PRINTF("decrypting 0x%X\n", flash_address);

        // make sure to have a valid address
        if (!bootloader_valid_address(flash_address)) {
            return;
        }

        // fetch pointer to flash
        uint8_t *flash_ptr = FLASH_U8_PTR(flash_address);
        uint32_t buffer_len = DEVICE_PAGE_SIZE;

        // decrypt buffer
        aes_cbc_decrypt(decrypted_buffer, flash_ptr, buffer_len, key_ptr, iv_ptr);

        if (flash_address == (BOOTLOADER_APP_END - DEVICE_PAGE_SIZE)) {
            // the last 32 bytes store crc and a magic signature
            // store "sucessfully decrypted" signature:
            uint8_t *magic_ptr = bootloader_magic_signature;
            for (uint32_t a = DEVICE_PAGE_SIZE-32; a < DEVICE_PAGE_SIZE-16; a++) {
                decrypted_buffer[a] = *magic_ptr++;
            }
        }

        // clear flash page
        flash_unlock();
        flash_erase_page(flash_address);
        flash_lock();


        // write page
        bootloader_write_buffer16(flash_address, (uint16_t *)decrypted_buffer, DEVICE_PAGE_SIZE/2);

        // next iteration:
        flash_address += DEVICE_PAGE_SIZE;

        // key/iv will be derived internally
        key_ptr = 0;
        iv_ptr = 0;
    }
}

static bool bootloader_flash_verify_signature(void) {
    // check if the flash content is encoded:
    uint8_t *flash_signature = FLASH_U32_PTR(BOOTLOADER_APP_END - 32);
    uint8_t *exp_signature   = bootloader_magic_signature;

    // compare:
    bool valid_signature = true;
    for(uint32_t i = 0; i < 16; i++) {
        if ((*flash_signature) != (*exp_signature)) {
            // do not abort here to make side channel attacks
            // a bit more complex
            valid_signature = false;
        }
        flash_signature++;
        exp_signature++;
    }

    if (!valid_signature) {
        DEBUG_PRINTF("flash is encrypted\n");
    } else {
        DEBUG_PRINTF("flash is decrypted\n");
    }
    return valid_signature;
}

static bool bootloader_verify_flash(void) {
    // expect a valid crc32 sum on last flash word:
    uint32_t flash_crc = *FLASH_U32_PTR(BOOTLOADER_APP_END - 4);

    // calc crc over flash content:
    uint32_t *app_start = FLASH_U32_PTR(BOOTLOADER_APP_START);
    uint32_t app_len    = BOOTLOADER_APP_END - BOOTLOADER_APP_START - 32;

    /*uint8_t data[]  = "12345678AFFEAFFE";
    app_len = strlen(data);

    app_start = (uint32_t*)data;*/
    printf("CRC start 0x%X ", *app_start);
    printf(" 0x%X\n", *(app_start+1));
    printf("CRC end 0x%X\n", *(app_start+app_len/4-1));
    printf("CRC over %d bytes\n", app_len);

    crc_reset();
    uint32_t expected_crc = crc_calculate_block(app_start, app_len/4);


    DEBUG_PRINTF("> got crc 0x%X, expected 0x%X\n", flash_crc, expected_crc);
    return expected_crc == flash_crc;
}

void bootloader_jump_to_app(void) {
    if (!bootloader_flash_verify_signature()) {
        // still encrypted! start decryption!
        bootloader_decrypt_flash();
    }

    if (bootloader_verify_flash()) {
        // app is intact thus it is safe to run it!
        // remap user program vector table to APP start location:
        // this is necessary for interrupts to work
        RELOCATE_VTABLE(BOOTLOADER_APP_START);

        // re-enable global interrupts (global ints are on per default on startup)
        GLOBAL_INT_ENABLE();

        // prepare jump pointer to
#ifndef BUILD_TEST
        void (*jump_ptr)(void);
        jump_ptr = (void (*)(void)) (*FLASH_U32_PTR(BOOTLOADER_APP_START));
        jump_ptr();

        // this should never be reached
        while (1) {
        }
#else
        DEBUG_PRINTF("YAY! all set, jumnping to app\n");
#endif  // BUILD_TEST

    } else {
        // no valid flash signature found, we stay in the bootloader for now!
    }
}


static bool bootloader_decode_address(uint32_t *address) {
    uint8_t rx;
    uint8_t checksum = 0;

    // start with empty address
    *address = 0;

    for (uint8_t i = 0; i < 4; i++) {
        // fetch 4 bytes -> 32 bit
        rx        = serial_getc();
        *address  = ((*address) << 8) | rx;
        checksum ^= rx;
    }

    // read address checksum
    rx = serial_getc();

    // verify checksum
    if (rx != checksum) {
        // checksum invalid -> abort here
        return false;
    }

    // verify if this is within allowed memory bounds:
    if (!bootloader_valid_address(*address)) {
        return false;
    }

    // everything is fine
    return true;
}

static bool bootloader_erase_page(uint8_t page) {
    uint32_t page_address = BOOTLOADER_APP_START + page * DEVICE_PAGE_SIZE;

    // verify if this is within allowed memory bounds:
    if (!bootloader_valid_address(page_address)) {
        return false;
    }

    // execute erase
    flash_unlock();
    flash_erase_page(page_address);
    flash_lock();

    return true;
}

void bootloader_main(void) {
    uint8_t buffer[256+2];
    uint8_t state = 0;
    uint8_t command = 0;
    uint8_t rx = 0;
    uint32_t address;
    uint8_t *data_ptr = 0;
    uint8_t checksum;
    uint8_t len = 0;
    uint16_t len16 = 0;
    uint8_t i;
    uint8_t max_bad_commands = BOOTLOADER_MAX_BAD_COMMANDS;

    // visual indicator that we entered bl main
    led_on();

    // the bootloader enable pin was high or
    // there was no valid code uploaded yet -> enter bootloader mode
    while (1) {
        // do main statemachine
        switch (state) {
            default:
            case(0):
                // fetch command byte
                command = serial_getc();
                if (command == BOOTLOADER_COMMAND_INIT) {
                    // init sequence, send ack
                    serial_putc(BOOTLOADER_RESPONSE_ACK);
                } else {
                    // real command
                    state   = 1;
                }
                break;

            case(1):
                // check command checksum (inverted)
                rx = serial_getc();
                // workaround for strange warning gcc bug(?)
                uint8_t ncommand = ~command;
                if (rx == ncommand) {
                    // fine, valid command -> decode
                    switch (command) {
                        // unknown or unsupported command
                        default:
                            // invalid command, abort
                            state = 0xFF;
                            // give up after n tries
                            if (max_bad_commands-- == 0) {
                                bootloader_jump_to_app();
                            }
                            break;

                        // all known commands
                        case(BOOTLOADER_COMMAND_GET):
                        case(BOOTLOADER_COMMAND_GET_VERSION):
                        case(BOOTLOADER_COMMAND_GET_ID):
                        case(BOOTLOADER_COMMAND_READ_MEMORY):
                        case(BOOTLOADER_COMMAND_GO):
                        case(BOOTLOADER_COMMAND_WRITE_MEMORY):
                        case(BOOTLOADER_COMMAND_ERASE):
                            // send ACK and continue with command handler
                            serial_putc(BOOTLOADER_RESPONSE_ACK);
                            state = 10 + command;
                            break;
                    }
                } else {
                    // mismatch - this was either a comm error or we are
                    // in the middle of a command, retry with the current byte as cmd byte:
                    if (rx == BOOTLOADER_COMMAND_INIT) {
                        // init sequence, send ack
                        serial_putc(BOOTLOADER_RESPONSE_ACK);
                        state   = 0;
                    } else {
                        // real command
                        command = rx;
                    }
                }
                break;

            // send GET response
            case(10 + BOOTLOADER_COMMAND_GET):
                // number of command bytes that will follow
                serial_putc(7);
                // version
                serial_putc(BOOTLOADER_VERSION);
                // send supported commands
                serial_putc(BOOTLOADER_COMMAND_GET);
                serial_putc(BOOTLOADER_COMMAND_GET_VERSION);
                serial_putc(BOOTLOADER_COMMAND_GET_ID);
                serial_putc(BOOTLOADER_COMMAND_READ_MEMORY);
                serial_putc(BOOTLOADER_COMMAND_GO);
                serial_putc(BOOTLOADER_COMMAND_WRITE_MEMORY);
                serial_putc(BOOTLOADER_COMMAND_ERASE);
                // send ack
                serial_putc(BOOTLOADER_RESPONSE_ACK);
                // wait for next command
                state = 0;
                break;

            // send GET_ID response
            case(10 + BOOTLOADER_COMMAND_GET_ID):
                // number of response bytes to follow
                serial_putc(1);
                // send product id of an F1 chip with the same pagesize (1024)
                serial_putc(BOOTLOADER_DEVICE_ID >> 8);
                serial_putc(BOOTLOADER_DEVICE_ID & 0xFF);
                // send ack
                serial_putc(BOOTLOADER_RESPONSE_ACK);
                // wait for next command
                state = 0;
                break;

            // send GET_VERSION response
            case (10 + BOOTLOADER_COMMAND_GET_VERSION):
                // bootloader version
                serial_putc(BOOTLOADER_VERSION);
                // send option bytes
                serial_putc(0x00);
                serial_putc(0x00);
                // send ack
                serial_putc(BOOTLOADER_RESPONSE_ACK);
                // wait for next command
                state = 0;
                break;

            // send READ_MEMORY response
            case(10 + BOOTLOADER_COMMAND_READ_MEMORY):
                if (!bootloader_decode_address(&address)) {
                    // abort now
                    state = 0xFF;
                    break;
                }

                // addresss is valid, send ack
                serial_putc(BOOTLOADER_RESPONSE_ACK);

                // fetch data
                len      = serial_getc();
                checksum = serial_getc();

                // verify checksum
                // workaround for strange warning gcc bug(?)
                uint8_t nchecksum = ~checksum;
                if (len != nchecksum) {
                    // checksum invalid -> abort here
                    state = 0xFF;
                    break;
                }

                // checksum test passed, send ack
                serial_putc(BOOTLOADER_RESPONSE_ACK);

#if !BOOTLOADER_ALLOW_READ
                // we do not allow read access,
                // return empty buffer with len+1 bytes
                memset(buffer, ((uint16_t)len) + 1, 0xEE);
                data_ptr = buffer;
#else
                // set pointer to flash
                data_ptr = FLASH_U8_PTR(address);
#endif  // BOOTLOADER_ALLOW_READ
                // send len+1 bytes
                serial_putc(*data_ptr++);
                while (len--) {
                    serial_putc(*data_ptr++);
                }

                // wait for next command
                state = 0;
                break;

            // send GO response
            case(10 + BOOTLOADER_COMMAND_GO):
                if (!bootloader_decode_address(&address)) {
                    // abort now
                    state = 0xFF;
                    break;
                }

                // addresss is valid, send ack
                serial_putc(BOOTLOADER_RESPONSE_ACK);

                // we do not support jump to any address
                // exec main app in any case:
                bootloader_jump_to_app();

                // in case the jump fails, wait for next command
                state = 0;
                break;

            // send WRITE_MEMORY response
            case(10 + BOOTLOADER_COMMAND_WRITE_MEMORY):
                if (!bootloader_decode_address(&address)) {
                    // abort now
                    state = 0xFF;
                    break;
                }

                // addresss is valid, send ack
                serial_putc(BOOTLOADER_RESPONSE_ACK);

                // fetch len
                len      = serial_getc();
                checksum = len;

                // place to store data
                data_ptr = &buffer[0];

                // we will have to write len+1 bytes
                len16 = ((uint16_t) len) + 1;

                // retrieve N+1 data bytes
                rx          = serial_getc();
                *data_ptr++ = rx;
                checksum   ^= rx;

                for (i=0; i < len; i++) {
                    rx          = serial_getc();
                    *data_ptr++ = rx;
                    checksum   ^= rx;
                }

                // verify checksum
                rx = serial_getc();
                if (checksum != rx) {
                    // checksum invalid -> abort here
                    state = 0xFF;
                    break;
                }

                // we can only write an even number of bytes
                // correct for this:
                if (len16 & 1) {
                    len16++;
                }

                // we will write 16 bit words at once
                // buffer len is given in 8 bit
                len16 = len16 / 2;

                // checksum ok  - store data
                if (!bootloader_write_buffer16(address, (uint16_t *)buffer, len16)) {
                    // error!
                    serial_putc(BOOTLOADER_RESPONSE_NACK);
                } else {
                    // done
                    serial_putc(BOOTLOADER_RESPONSE_ACK);
                }

                // wait for next command
                state = 0;
                break;

            // send ERASE response
            case(10 + BOOTLOADER_COMMAND_ERASE):
                // get number of pages to be erased
                len      = serial_getc();
                checksum = len;

                if (len == 0xFF) {
                    // special case, full flash erase
                    if (serial_getc() == 0x00) {
                        // valid command, mark all pages to be erased
                        len = 0;
                        data_ptr = &buffer[0];
                        for (i = BOOTLOADER_SIZE_PAGES; i < BOOTLOADER_FLASH_SIZE_PAGES; i++) {
                            buffer[len] = i;
                            len++;
                        }
                    } else {
                        // checksum error, abort
                        state = 0xFF;
                        break;
                    }
                } else {
                    // fetch len+1 pages to be erased
                    data_ptr = &buffer[0];
                    rx          = serial_getc();
                    *data_ptr++ = rx;
                    checksum   ^= rx;

                    for (i = 0; i < len; i++) {
                        rx          = serial_getc();
                        *data_ptr++ = rx;
                        checksum   ^= rx;
                    }

                    // fetch checksum
                    rx = serial_getc();

                    if (rx != checksum) {
                        // checksum mismatch, abort
                        state = 0xFF;
                        break;
                    }
                }

                // fine, the len+1 pages to be erased are now in buffer[]
                // execute the erase of len+1 pages
                data_ptr = buffer;

                if (!bootloader_erase_page(*data_ptr++)) { state = 0xFF; break;}

                while (len--) {
                    if (!bootloader_erase_page(*data_ptr++)) { state = 0xFF; break;}
                }

                // execute suceeded!
                serial_putc(BOOTLOADER_RESPONSE_ACK);

                // wait for next command
                state = 0;
                break;

            // ABORT STATE - send nack and goto idle
            case(0xFF):
                serial_putc(BOOTLOADER_RESPONSE_NACK);
                state = 0;
                break;
        }
    }
}
