/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:17:19
 */

#pragma once

#include <cstdint>
#include <string>
#include <thread>
#include <vector>

namespace xyber_utils {

/**
 * @description: log helper
 * @return {*}
 */
#define LOG_INFO(s, ...) printf("[INFO] " s "\n", ##__VA_ARGS__)
#define LOG_WARN(s, ...) printf("\033[33m[WARN] " s "\033[0m\n", ##__VA_ARGS__)
#define LOG_ERROR(s, ...) printf("\033[31m[ERROR] " s "\033[0m\n", ##__VA_ARGS__)
#define LOG_DEBUG(s, ...) printf("\033[32m[DEBUG] " s "\033[0m\n", ##__VA_ARGS__)

/**
 * @description: float转Bytes
 * @param {float} _f
 * @param {uint8_t} *_bytes
 * @return void
 */
void FloatToBytes(const float _f, uint8_t *_bytes);

/**
 * @description: Bytes转float
 * @param {uint8_t} *_bytes
 * @return {float}
 */
float BytesToFloat(const uint8_t *_bytes);

/**
 * @description: uint16转bytes
 * @param {uint16_t} _f
 * @param {uint8_t} *_bytes
 * @return void
 */
void Uint16ToBytes(const uint16_t _f, uint8_t *_bytes);

/**
 * @description: bytes转uint16
 * @param {uint8_t} *_bytes
 * @return {uint16_t}
 */
uint16_t BytesToUint16(const uint8_t *_bytes);

/**
 * @brief Set the Real Time Thread object
 *
 * @param pid
 * @param name
 * @param rt_priority
 * @param bind_cpu
 * @return true
 * @return false
 */
bool SetRealTimeThread(pthread_t pid, std::string name, int rt_priority = -1, int bind_cpu = -1);

}  // namespace xyber_utils