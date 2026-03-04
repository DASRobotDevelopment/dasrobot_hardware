DASRobot drive controller

ROS2 Hardware Interface для контроллера колес Dasrobot. Поддерживает 4 колеса с обратной связью через последовательный порт.

## Особенности
- 4  колеса (FL, FR, BL, BR)
- Velocity control (rad/s → RPM)
- Encoder feedback (position + velocity)
- Конфигурируемый serial порт (115200, 57600, 9600 baud)
- Полная совместимость с `ros2_control`

## Структура пакета
```
dasrobot_hardware/
├── include/dasrobot_hardware/
│   └── dasrobot_drive_controller.hpp
├── src/
│   └── dasrobot_drive_controller.cpp
├── dasrobot_hardware.xml
├── CMakeLists.txt
└── package.xml
```

## Быстрый старт

### 1. Установка
```bash
cd ~/ros2_ws/src
git clone https://github.com/your-repo/dasrobot_hardware.git
cd ~/ros2_ws && colcon build --packages-select dasrobot_hardware
source install/setup.bash
```

### 2. Конфигурация (robot_hardware.yaml)
```yaml
controller_manager:
  ros__parameters:
    robot_hardware:
      - name: "Dasrobot"
        type: "dasrobot_hardware/DasrobotDriveController"
        serial_port: "/dev/ttyUSB0"
        baudrate: "115200"
        wheel_radius: 0.1
        wheel_separation_horizontal: 0.4
        wheel_separation_vertical: 0.5
        encoder_ppr: 988
```

### 3. URDF пример
```xml
<ros2_control name="DasrobotSystem" type="system">
  <hardware>
    <plugin>dasrobot_hardware/DasrobotDriveController</plugin>
    <param name="serial_port">/dev/ttyUSB0</param>
  </hardware>
  <joint name="base_front_left_wheel">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- 3 других колеса -->
</ros2_control>
```

### 4. Запуск
```bash
ros2 launch your_robot_bringup robot_bringup.launch.py
```

## Протокол связи

**Запрос энкодеров:** `e\n`  
**Ответ:** `e,1234,5678,-2345,890\n`  

**Управление скоростью:** `v,100,-50,75,-25\n`  
**Формат:** RPM (оборотов в минуту)

## Параметры конфигурации

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `serial_port` | string | `/dev/ttyUSB0` | Порт микроконтроллера |
| `baudrate` | int | `115200` | Скорость порта |
| `wheel_radius` | double | `0.1` | Радиус колеса (м) |
| `wheel_separation_horizontal` | double | `0.4` | Расстояние L-R (м) |
| `wheel_separation_vertical` | double | `0.5` | Расстояние F-B (м) |
| `encoder_ppr` | double | `988` | Импульсов на оборот |

## Отладка

```bash
# Логи инициализации
RCLCPP_LOGGING_SEVERITY=INFO ros2 launch ...

# Мониторинг состояния
ros2 topic hz /joint_states
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

## Зависимости
```
ROS2 Humble/Iron/Jazzy
├── rclcpp>=1.0
├── pluginlib>=5.0
└── hardware_interface>=2.0
```

## Лицензия
Apache-2.0 © DASRobot



## Контакты

- GitHub: [Issues](https://github.com/DASRobotDevelopment/fourbox/issues)
- Telegram: [@dasbot_support](https://t.me/dasrobot_support)

***