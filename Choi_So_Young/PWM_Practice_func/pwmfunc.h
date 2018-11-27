#define GPIOC Motorport
#define GPIO_PIN_0 MotorA1
#define GPIO_PIN_1 MotorA2
#define GPIO_PIN_2 MotorB1
#define GPIO_PIN_3 MotorB2

void Dir_control(uint8_t str[]);
uint16_t Conversion(uint8_t str[]);
void speed(uint16_t val);