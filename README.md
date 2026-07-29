## Example Summary

Empty C++ project using DriverLib.
This example shows a basic empty project using DriverLib with just main C++ file
and SysConfig initialization.

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| SYSCTL |  |  |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.

| Pin | Peripheral | Function | LaunchPad Pin | LaunchPad Settings |
| --- | --- | --- | --- | --- |
| PA20 | DEBUGSS | SWCLK | N/A | <ul><li>PA20 is used by SWD during debugging<br><ul><li>`J101 15:16 ON` Connect to XDS-110 SWCLK while debugging<br><li>`J101 15:16 OFF` Disconnect from XDS-110 SWCLK if using pin in application</ul></ul> |
| PA19 | DEBUGSS | SWDIO | N/A | <ul><li>PA19 is used by SWD during debugging<br><ul><li>`J101 13:14 ON` Connect to XDS-110 SWDIO while debugging<br><li>`J101 13:14 OFF` Disconnect from XDS-110 SWDIO if using pin in application</ul></ul> |

### Device Migration Recommendations
This project was developed for a superset device included in the LP_MSPM0G3507 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#sysconfig-project-migration)
for information about migrating to other MSPM0 devices.

### Low-Power Recommendations
TI recommends to terminate unused pins by setting the corresponding functions to
GPIO and configure the pins to output low or input with internal
pullup/pulldown resistor.

SysConfig allows developers to easily configure unused pins by selecting **Board**→**Configure Unused Pins**.

For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0G3507 User's Guide](https://www.ti.com/lit/slau873).

## Example Usage

Compile, load and run the example.

## TB6612 Motor Driver

The project runs the CPU at 80 MHz from the board's 40 MHz HFXT and SYSPLL. It
configures one synchronized 20 kHz TIMA1 PWM peripheral for both drive wheels.
Channel A is the right wheel and channel B is the left wheel. STBY must remain
connected to 3.3 V.

| TB6612 signal | MSPM0G3507 pin | Assignment |
| --- | --- | --- |
| PWMA | PA16 | TIMA1 CCP1, right wheel |
| AIN1 | PA14 | Right direction |
| AIN2 | PA15 | Right direction |
| PWMB | PA17 | TIMA1 CCP0, left wheel |
| BIN1 | PA12 | Left direction |
| BIN2 | PA13 | Left direction |

Include `tb6612.hpp` and call the APIs after `SYSCFG_DL_init()`:

```cpp
TB6612::init();                 // Safe coast state
TB6612::setSpeeds(500, 500);   // left, right; range -1000..1000
TB6612::coast();
TB6612::brake();
```

Positive commands select IN1 low and IN2 high for both wheels. If either wheel
rotates opposite to the vehicle-forward direction, swap that motor's AO1/AO2
or BO1/BO2 wires.

## OLED Display

The project includes the SSD1306 128x64 framebuffer driver and fonts ported
from `MSPM0_K230_OLED_Echo`. The display uses 7-bit I2C address `0x3C` at
500 kHz.

| OLED signal | MSPM0G3507 pin | Assignment |
| --- | --- | --- |
| SCL | PA1 | I2C0 SCL |
| SDA | PA0 | I2C0 SDA |
| VCC | 3.3 V | OLED supply |
| GND | GND | Common ground |

Call `OLED_Init()` after `SYSCFG_DL_init()`, update the framebuffer with
`OLED_ShowString()`, `OLED_ShowNum()` or the drawing APIs, then call
`OLED_Refresh()`. Most OLED modules include SDA/SCL pull-ups; add about
4.7 kohm pull-ups to 3.3 V if using a bare panel.

## Wheel Encoders

Both Hall encoders are decoded using GPIO interrupts on every A/B edge.
Sampling runs at 100 Hz using SysTick, while the OLED refreshes at 5 Hz.
No PID output is applied yet.

| Encoder signal | MSPM0G3507 pin | Wheel |
| --- | --- | --- |
| E1A | PB2 | Right |
| E1B | PB3 | Right |
| E2A | PB6 | Left |
| E2B | PB7 | Left |

The current conversion assumes 13 motor-shaft lines, a 28:1 gearbox and
quadrature x4 decoding: `13 * 28 * 4 = 1456` counts per wheel revolution.
Encoder signals must not exceed 3.3 V. The left encoder polarity is inverted
in software so that forward motion produces positive feedback on both wheels.

## 100 Hz Wheel Speed PID

The encoder sampling ISR runs two independent wheel-speed PID controllers at
100 Hz. The initial test settings are:

- Target: 100 RPM for each wheel
- Kp: 4.0
- Ki: 6.0
- Kd: 0.0
- PWM command limit: +/-700 of 1000
- Speed feedback filter coefficient: 0.25

Use `SpeedControl::setTargetRpm(left, right)` to change the wheel targets and
`SpeedControl::setTunings(kp, ki, kd)` to tune both controllers. The OLED
shows target RPM (`T`), measured RPM (`M`) and the signed PWM command.

The current automatic speed test starts both wheel targets at 50 RPM and
raises them by 50 RPM every two seconds. The ramp stops increasing when it
reaches 500 RPM and then holds that target.
