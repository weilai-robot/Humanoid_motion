/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:18:24
 */

#include <string.h>

#include "internal/common_utils.h"

namespace xyber_utils {

void FloatToBytes(const float _f, uint8_t *_bytes) {
  float tempF = _f;
  auto *b = (uint8_t *)&tempF;
  _bytes[0] = b[0];
  _bytes[1] = b[1];
  _bytes[2] = b[2];
  _bytes[3] = b[3];
}

float BytesToFloat(const uint8_t *_bytes) {  // TODO: imu data is big endian
  uint8_t tempU[4] = {_bytes[3], _bytes[2], _bytes[1], _bytes[0]};
  return *((float *)tempU);
}

void Uint16ToBytes(const uint16_t _f, uint8_t *_bytes) {
  uint16_t tempF = _f;
  auto *b = (uint8_t *)&tempF;
  _bytes[0] = b[0];
  _bytes[1] = b[1];
}

uint16_t BytesToUint16(const uint8_t *_bytes) {
  uint8_t tempU[2] = {_bytes[1], _bytes[0]};
  return *((uint16_t *)tempU);
}

bool SetRealTimeThread(pthread_t pid, std::string name, int rt_priority, int bind_cpu) {
  bool ret = true;
  if (name.size()) {
    pthread_setname_np(pid, name.c_str());
  }

  // set rt-thread.
  if (rt_priority >= 0) {
    int max = sched_get_priority_max(SCHED_FIFO);
    if (rt_priority > max) rt_priority = max;

    struct sched_param s_parm;
    s_parm.sched_priority = rt_priority;
    if (pthread_setschedparam(pid, SCHED_FIFO, &s_parm) < 0) {
      LOG_ERROR("setschedu error %s", strerror(errno));
      ret = false;
    } else {
      LOG_DEBUG("Thread %s set priority: %d ", name.c_str(), rt_priority);
    }
  }

  // bind cpu core
  if (bind_cpu >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(bind_cpu, &cpuset);
    if (pthread_setaffinity_np(pid, sizeof(cpuset), &cpuset) != 0) {
      LOG_ERROR("setaffinity error %s", strerror(errno));
      ret = false;
    } else {
      LOG_DEBUG("Thread %s bind cpu core: %d", name.c_str(), bind_cpu);
    }
  }

  return ret;
}

}  // namespace xyber_utils