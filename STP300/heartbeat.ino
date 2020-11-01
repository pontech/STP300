//////////////////////////////////////////////////////////
// Heartbeat Task to flash LED
//////////////////////////////////////////////////////////
uint8_t heartbeat_led_pin_number;

void createHeartbeat(uint8_t new_pin, uint32_t heartbeat_period)
{
  heartbeat_led_pin_number = new_pin;
  createTask(heartbeat_task, heartbeat_period / 2, TASK_ENABLE, (void*)NULL);
}

void heartbeat_task(int id, void* tptr) {
  digitalWrite(heartbeat_led_pin_number, !digitalRead(heartbeat_led_pin_number)); // Toggle pin state
}
