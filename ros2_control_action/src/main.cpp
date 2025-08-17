#include <Arduino.h>
#include "bot.h"

void microros_task(void *param)
{
  while (1)
  {
    loop_bot_transport();
 
  }
}

void setup()
{
  Serial0.begin(921600);
  // 初始化Serial2用於調試信息 (GPIO16=RX, GPIO17=TX)
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  set_microros_serial_transports(Serial0);
  // 设置通过串口进行MicroROS通信

  delay(2000);
  initializeHardware();
  xTaskCreatePinnedToCore(microros_task, "microros_task", 10280, NULL, 1, NULL, 0); // 核心0

}

void loop()
{
  delay(10); // 避免阻塞
  loop_bot_control();

}