#include "rn4871.h"
#include <stdio.h>
#include <string.h>

#define RN4871_FMT(buf, buf_size, ...)                      \
    do {                                                     \
        int _n = snprintf((buf), (buf_size), __VA_ARGS__);   \
        if (_n < 0 || (size_t)_n >= (buf_size)) return -1;  \
        return _n;                                           \
    } while (0)

// Controlo de modo


int rn4871_cmd_enter_mode(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "$$$");
}

int rn4871_cmd_exit_mode(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "---" RN4871_EOL);
}

int rn4871_cmd_reboot(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "R,1" RN4871_EOL);
}

// Comandos set

int rn4871_set_name(char *buf, size_t buf_size, const char *name)
{
    RN4871_FMT(buf, buf_size, "SN,%s" RN4871_EOL, name);
}

int rn4871_set_pin(char *buf, size_t buf_size, const char *pin)
{
    RN4871_FMT(buf, buf_size, "SP,%s" RN4871_EOL, pin);
}

int rn4871_set_baudrate(char *buf, size_t buf_size, uint8_t baud_code)
{
    RN4871_FMT(buf, buf_size, "SB,%02X" RN4871_EOL, baud_code);
}

int rn4871_set_services(char *buf, size_t buf_size, uint8_t services_bitmap)
{
    RN4871_FMT(buf, buf_size, "SS,%02X" RN4871_EOL, services_bitmap);
}

int rn4871_set_features(char *buf, size_t buf_size, uint16_t features_bitmap)
{
    RN4871_FMT(buf, buf_size, "SR,%04X" RN4871_EOL, features_bitmap);
}

int rn4871_factory_reset(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "SF,1" RN4871_EOL);
}

int rn4871_factory_reset_full(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "SF,2" RN4871_EOL);
}

int rn4871_set_adv_timing(char *buf, size_t buf_size,
                          uint16_t adv_interval,
                          uint16_t adv_window,
                          uint16_t scan_window)
{
    RN4871_FMT(buf, buf_size, "STA,%04X,%04X,%04X" RN4871_EOL,
               adv_interval, adv_window, scan_window);
}

int rn4871_set_connection_params(char *buf, size_t buf_size,
                                 uint16_t interval_min,
                                 uint16_t interval_max,
                                 uint16_t latency,
                                 uint16_t timeout)
{
    RN4871_FMT(buf, buf_size, "ST,%04X,%04X,%04X,%04X" RN4871_EOL,
               interval_min, interval_max, latency, timeout);
}

int rn4871_set_tx_power(char *buf, size_t buf_size, uint8_t power_level)
{
    if (power_level > 5) return -1;
    RN4871_FMT(buf, buf_size, "SGA,%u" RN4871_EOL, power_level);
}

int rn4871_set_security(char *buf, size_t buf_size, uint8_t mode)
{
    if (mode > 5) return -1;
    RN4871_FMT(buf, buf_size, "SA,%u" RN4871_EOL, mode);
}

int rn4871_set_connectable(char *buf, size_t buf_size, uint8_t mode)
{
    if (mode > 2) return -1;
    RN4871_FMT(buf, buf_size, "SC,%u" RN4871_EOL, mode);
}

int rn4871_set_status_delimiters(char *buf, size_t buf_size,
                                 const char *pre, const char *post)
{
    RN4871_FMT(buf, buf_size, "S%%,%s,%s" RN4871_EOL, pre, post);
}

int rn4871_set_script_char(char *buf, size_t buf_size, char script_char)
{
    RN4871_FMT(buf, buf_size, "S$,%c" RN4871_EOL, script_char);
}

int rn4871_set_power_save(char *buf, size_t buf_size, uint8_t enable)
{
    RN4871_FMT(buf, buf_size, "SO,%u" RN4871_EOL, enable ? 1u : 0u);
}

// Comandos get

int rn4871_get_name(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "GN" RN4871_EOL);
}

int rn4871_get_mac(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "GBD" RN4871_EOL);
}

int rn4871_get_firmware_version(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "V" RN4871_EOL);
}

int rn4871_get_baudrate(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "GB" RN4871_EOL);
}

int rn4871_get_connection_status(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "GK" RN4871_EOL);
}

int rn4871_get_all_settings(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "D" RN4871_EOL);
}

int rn4871_list_services(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "LS" RN4871_EOL);
}

// Aviso/ligação

int rn4871_start_advertising(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "A" RN4871_EOL);
}

int rn4871_stop_advertising(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "Y" RN4871_EOL);
}

int rn4871_connect(char *buf, size_t buf_size,
                   const char *mac, uint8_t addr_type)
{
    RN4871_FMT(buf, buf_size, "C,%s,%u" RN4871_EOL, mac, addr_type);
}

int rn4871_disconnect(char *buf, size_t buf_size)
{
    RN4871_FMT(buf, buf_size, "K,1" RN4871_EOL);
}

// Gat

int rn4871_write_local_char(char *buf, size_t buf_size,
                             uint16_t handle, const char *hex_data)
{
    RN4871_FMT(buf, buf_size, "SHW,%04X,%s" RN4871_EOL, handle, hex_data);
}

int rn4871_read_local_char(char *buf, size_t buf_size, uint16_t handle)
{
    RN4871_FMT(buf, buf_size, "SHR,%04X" RN4871_EOL, handle);
}

int rn4871_write_remote_char(char *buf, size_t buf_size,
                              uint16_t handle, const char *hex_data)
{
    RN4871_FMT(buf, buf_size, "CHW,%04X,%s" RN4871_EOL, handle, hex_data);
}

int rn4871_read_remote_char(char *buf, size_t buf_size, uint16_t handle)
{
    RN4871_FMT(buf, buf_size, "CHR,%04X" RN4871_EOL, handle);
}

// Informação do dispositivo

int rn4871_set_dis_firmware(char *buf, size_t buf_size, const char *text)
{
    RN4871_FMT(buf, buf_size, "SDF,%s" RN4871_EOL, text);
}

int rn4871_set_dis_hardware(char *buf, size_t buf_size, const char *text)
{
    RN4871_FMT(buf, buf_size, "SDH,%s" RN4871_EOL, text);
}

int rn4871_set_dis_manufacturer(char *buf, size_t buf_size, const char *text)
{
    RN4871_FMT(buf, buf_size, "SDM,%s" RN4871_EOL, text);
}

int rn4871_set_dis_model(char *buf, size_t buf_size, const char *text)
{
    RN4871_FMT(buf, buf_size, "SDN,%s" RN4871_EOL, text);
}

int rn4871_set_dis_software(char *buf, size_t buf_size, const char *text)
 {
    RN4871_FMT(buf, buf_size, "SDR,%s" RN4871_EOL, text);
}

int rn4871_set_dis_serial(char *buf, size_t buf_size, const char *text)
{
    RN4871_FMT(buf, buf_size, "SDS,%s" RN4871_EOL, text);
}

// Giopo

int rn4871_set_gpio_config(char *buf, size_t buf_size,
                            uint8_t pin_bitmap, uint8_t func_bitmap)
{
    RN4871_FMT(buf, buf_size, "SW,%02X,%02X" RN4871_EOL,
               pin_bitmap, func_bitmap);
}

int rn4871_gpio_read(char *buf, size_t buf_size, uint8_t pin_bitmap)
{
    RN4871_FMT(buf, buf_size, "|I,%02X" RN4871_EOL, pin_bitmap);
}

int rn4871_gpio_write(char *buf, size_t buf_size,
                       uint8_t pin_bitmap, uint8_t value_bitmap)
{
    RN4871_FMT(buf, buf_size, "|O,%02X,%02X" RN4871_EOL,
               pin_bitmap, value_bitmap);
}
