/*
 * ssd1306_conf.h
 *
 *  Created on: Jan 5, 2026
 *      Author: carlo
 */
/**
 * Private configuration file for the SSD1306 library.
 * CONFIGURADO PARA STM32F411
 */

#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

// 1. ELEGIR FAMILIA DEL MICRO (¡IMPORTANTE!)
// #define STM32F0  <-- ESTA NO
#define STM32F4  // <-- ESTA SÍ (Es tu placa)

// 2. ELEGIR BUS
#define SSD1306_USE_I2C

// 3. CONFIGURACIÓN I2C
#define SSD1306_I2C_PORT        hi2c1
// 0x3C << 1 es matemáticamente igual a 0x78 (tu dirección)
#define SSD1306_I2C_ADDR        (0x3C << 1)

// 4. CONFIGURACIÓN DE PANTALLA
// Si tu pantalla se ve "rara" o partida, descomenta estas líneas:
// #define SSD1306_MIRROR_VERT
// #define SSD1306_MIRROR_HORIZ
// #define SSD1306_INVERSE_COLOR

// 5. FUENTES (Déjalas todas activas para probar)
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26
#define SSD1306_INCLUDE_FONT_16x24
#define SSD1306_INCLUDE_FONT_16x15

// 6. TAMAÑO (Por defecto suele ser 128x64, pero aseguramos)
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64

#endif /* __SSD1306_CONF_H__ */
