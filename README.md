MIMIC_ROBOT_STM32_
==================

Mimic robot on STMF4 DISCOVERY

Files list:
main.c - program w wersji beta - komunikuje się, ale problem jest taki, że mogę mu wysłać liczbę tylko do 0xFF. Trzeba z informatykami się porozumieć, jak liczby będzemy przekazywać.
Timer4 działa, servo dobrze działa. Program jest brzydki, bo robiony bezmyslnie, ale wraz z jego rozwojem to się zmieni.
Następny problem to EEPROM. Nie ma go. PCBowcy proszeni są o uwzględnienie go na płytce.

TIM2 TIM3 TIM4 TIM5

USART1 - PB6, PB7

I2C dla EEPROM PB8 - I2C1_SCL, PB9 - I2C1_SDA
