#ifndef RN4871_H
#define RN4871_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Definição do terminador de linha padrão para comandos RN4871.
 */
#ifndef RN4871_EOL
#define RN4871_EOL "\r"
#endif

/* --- Controlo de Modo --- */

int rn4871_cmd_enter_mode(char *buf, size_t buf_size);
int rn4871_cmd_exit_mode(char *buf, size_t buf_size);
int rn4871_cmd_reboot(char *buf, size_t buf_size);

/* --- Comandos de Configuração (Set) --- */

int rn4871_set_name(char *buf, size_t buf_size, const char *name);
int rn4871_set_pin(char *buf, size_t buf_size, const char *pin);
int rn4871_set_baudrate(char *buf, size_t buf_size, uint8_t baud_code);
int rn4871_set_services(char *buf, size_t buf_size, uint8_t services_bitmap);
int rn4871_set_features(char *buf, size_t buf_size, uint16_t features_bitmap);
int rn4871_factory_reset(char *buf, size_t buf_size);
int rn4871_factory_reset_full(char *buf, size_t buf_size);

int rn4871_set_adv_timing(char *buf, size_t buf_size,
                          uint16_t adv_interval,
                          uint16_t adv_window,
                          uint16_t scan_window);

int rn4871_set_connection_params(char *buf, size_t buf_size,
                                 uint16_t interval_min,
                                 uint16_t interval_max,
                                 uint16_t latency,
                                 uint16_t timeout);

int rn4871_set_tx_power(char *buf, size_t buf_size, uint8_t power_level);
int rn4871_set_security(char *buf, size_t buf_size, uint8_t mode);
int rn4871_set_connectable(char *buf, size_t buf_size, uint8_t mode);

int rn4871_set_status_delimiters(char *buf, size_t buf_size,
                                 const char *pre, const char *post);

int rn4871_set_script_char(char *buf, size_t buf_size, char script_char);
int rn4871_set_power_save(char *buf, size_t buf_size, uint8_t enable);

/* --- Comandos de Consulta (Get) --- */

int rn4871_get_name(char *buf, size_t buf_size);
int rn4871_get_mac(char *buf, size_t buf_size);
int rn4871_get_firmware_version(char *buf, size_t buf_size);
int rn4871_get_baudrate(char *buf, size_t buf_size);
int rn4871_get_connection_status(char *buf, size_t buf_size);
int rn4871_get_all_settings(char *buf, size_t buf_size);
int rn4871_list_services(char *buf, size_t buf_size);

/* --- Aviso e Ligação --- */

int rn4871_start_advertising(char *buf, size_t buf_size);
int rn4871_stop_advertising(char *buf, size_t buf_size);
int rn4871_connect(char *buf, size_t buf_size,
                   const char *mac, uint8_t addr_type);
int rn4871_disconnect(char *buf, size_t buf_size);

/* --- GATT Services --- */

int rn4871_write_local_char(char *buf, size_t buf_size,
                             uint16_t handle, const char *hex_data);
int rn4871_read_local_char(char *buf, size_t buf_size, uint16_t handle);

int rn4871_write_remote_char(char *buf, size_t buf_size,
                              uint16_t handle, const char *hex_data);
int rn4871_read_remote_char(char *buf, size_t buf_size, uint16_t handle);

/* --- Device Information Service (DIS) --- */

int rn4871_set_dis_firmware(char *buf, size_t buf_size, const char *text);
int rn4871_set_dis_hardware(char *buf, size_t buf_size, const char *text);
int rn4871_set_dis_manufacturer(char *buf, size_t buf_size, const char *text);
int rn4871_set_dis_model(char *buf, size_t buf_size, const char *text);
int rn4871_set_dis_software(char *buf, size_t buf_size, const char *text);
int rn4871_set_dis_serial(char *buf, size_t buf_size, const char *text);

/* --- GPIO Control --- */

int rn4871_set_gpio_config(char *buf, size_t buf_size,
                            uint8_t pin_bitmap, uint8_t func_bitmap);
int rn4871_gpio_read(char *buf, size_t buf_size, uint8_t pin_bitmap);
int rn4871_gpio_write(char *buf, size_t buf_size,
                       uint8_t pin_bitmap, uint8_t value_bitmap);

#endif /* RN4871_H */
