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
After reversing the vehicle heading, channel A is the new left wheel and
channel B is the new right wheel. STBY must remain connected to 3.3 V.

| TB6612 signal | MSPM0G3507 pin | Assignment |
| --- | --- | --- |
| PWMA | PA16 | TIMA1 CCP1, new left wheel |
| AIN1 | PA14 | New left direction |
| AIN2 | PA15 | New left direction |
| PWMB | PA17 | TIMA1 CCP0, new right wheel |
| BIN1 | PA12 | New right direction |
| BIN2 | PA13 | New right direction |

Include `tb6612.hpp` and call the APIs after `SYSCFG_DL_init()`:

```cpp
TB6612::init();                 // Safe coast state
TB6612::setSpeeds(500, 500);   // left, right; range -1000..1000
TB6612::coast();
TB6612::brake();
```

The vehicle-forward direction has been reversed in software. Positive commands
now select IN1 high and IN2 low for both wheels, so the car travels opposite to
its original forward direction without changing motor wiring. Negative commands
select the original forward direction.

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
The wheel-speed PID is the active motor-control path.

| Encoder signal | MSPM0G3507 pin | Wheel |
| --- | --- | --- |
| E1A | PB2 | New left (original right) |
| E1B | PB3 | New left (original right) |
| E2A | PB6 | New right (original left) |
| E2B | PB7 | New right (original left) |

The current conversion assumes 13 motor-shaft lines, a 28:1 gearbox and
quadrature x4 decoding: `13 * 28 * 4 = 1456` counts per wheel revolution.
Encoder signals must not exceed 3.3 V. Both encoder polarities are matched to
the redefined vehicle-forward direction: positive motor commands and motion in
the new forward direction produce positive speed feedback on both wheels.

## 100 Hz Wheel-Speed PID

SysTick snapshots both encoders and runs the two wheel-speed PID controllers
at 100 Hz. Speed measurement uses a rolling four-sample count window, improving
the steady-state quantization from about 4.12 RPM/count to 1.03 RPM/count while
retaining the 100 Hz control update.

The line-tracking outer loop supplies separate left and right RPM targets. The
speed-loop default gains are Kp=11.1, Ki=70 and Kd=0, with the PWM command
limited to +/-800. Positive target RPM uses the redefined forward direction;
negative target RPM uses the original forward direction.

## Application Structure

`empty_cpp.cpp` only initializes SysConfig and runs the application loop.
`car_app.cpp` owns subsystem startup and cooperative scheduling,
and `pid_dashboard.cpp` owns OLED formatting and refresh timing. The encoder
SysTick ISR records each sample and runs the 100 Hz speed-control update. The
line-tracking outer loop runs once per new sensor sample and updates the two
speed targets. The stepped speed test remains available but is not scheduled.

## Yahboom K230 UART Link

The K230 link is a bidirectional 115200-8-N-1 UART. Incoming bytes are placed
in a 512-byte interrupt-driven ring buffer. The current application echoes
the bytes unchanged and does not parse any Yahboom packet format.

The MSPM0 stream parser accepts `BALL,x\r\n`, validates X against the
640-pixel AI-frame width, and shows `K230 BALL`, `X`, and the decoded frame
sequence on the OLED. `BALL,-1\r\n` displays `K230 NO BALL` with `X:---`.
Invalid, partial, and overlong lines do not update the displayed position.
After the first valid frame, the OLED keeps the latest decoded K230 position
instead of timing out to the PID page.

Before the first valid frame, the OLED alternates once per second between the
motor page and receive diagnostics. `K230 WAIT` with `RX:000000` means UART2
has not received any byte. `K230 RAW` means bytes are arriving, but no complete
valid `BALL,x` line has been decoded. The `DROP` line reports UART ring-buffer
overflow.

| Signal | MSPM0G3507 | Yahboom K230 communication connector |
| --- | --- | --- |
| MSPM0 RX | PA22 / UART2_RX | UART1_TXD / GPIO9 |
| MSPM0 TX | PA21 / UART2_TX | UART1_RXD / GPIO10 |
| Reference | GND | GND |

Use 3.3 V UART logic, cross TX and RX, and connect the grounds. Do not connect
a 5 V UART signal to either processor.

Run `k230_uart_echo_test.py` in CanMV IDE. It sends one test line every 500 ms
and prints `PASS` only when the MSPM0 returns exactly the same bytes. The
script uses `machine.UART` directly and does not depend on `YbUart`,
`YbProtocol`, or the reference project's K230 Python code.

### Ball coordinate sender

`k230_ball_detection_uart.py` is based on the steel-ball detector in
`D:\designDocs\test.py`. It selects the largest valid detection and sends only
its horizontal center in the 640-pixel AI-frame coordinate system:

```text
BALL,320\r\n
```

The valid coordinate field is zero-padded to three digits. When no ball is
detected it sends `BALL,-1\r\n`; set `SEND_NO_TARGET_FRAME = False` to
suppress no-target frames. The script drains the previous MSPM0 echo before
each transmission so the K230 receive FIFO does not accumulate echoed data.
It also sends one no-target frame immediately after UART initialization, before
model loading, to verify the complete K230-to-MSPM0-to-OLED path.

The MSPM0 arms UART2 before the OLED power-up delay and drains any bytes already
present in the hardware RX FIFO before enabling its transition-triggered
interrupt. This makes reception independent of the K230/MSPM0 power-up order.
The K230 UART uses an explicit zero-millisecond read timeout, so discarding the
previous echo never delays the next coordinate transmission.

## Ganwei Eight-channel Grayscale Sensor

The sensor uses its eight parallel digital outputs instead of I2C. Install
the sensor board's `PULL` jumper and then power-cycle the sensor. This selects
open-drain output mode; the MSPM0 GPIO inputs use their internal pull-ups to
3.3 V, so no external pull-up resistors are required. Keep the sensor powered
from 5 V and connect all grounds.

| Sensor output | MSPM0G3507 pin |
| --- | --- |
| OUT1 | PB0 |
| OUT2 | PB1 |
| OUT3 | PB4 |
| OUT4 | PB5 |
| OUT5 | PB8 |
| OUT6 | PB9 |
| OUT7 | PB10 |
| OUT8 | PB11 |

Sampling runs at the encoder's 100 Hz rate. The packed digital byte maps OUT1
to bit 0 through OUT8 to bit 7. The OLED `GRAY GPIO` page shows the hexadecimal
mask and each raw channel state. A disconnected open-drain signal normally
reads high because of the internal pull-up, so this interface cannot provide
automatic disconnect detection.

Do not connect the sensor SCL/SDA pins. PA0/PA1 remain dedicated to the OLED's
hardware I2C bus.

## Cascaded Line-Tracking PID

Line tracking is the outer loop of the active cascaded controller. It runs at
the 100 Hz sensor sampling rate and outputs a differential RPM correction. The
two inner wheel-speed PID controllers independently convert their RPM errors
to TB6612 PWM commands.

The active-low sensor byte is converted to the same OUT1-to-bit7 line mask used
by the `D:\design\5_29` reference controller. The unchanged sensor module is
mounted at the new vehicle front, so its channel order and per-sensor weights
use the symmetric per-sensor weights derived from the original STM32 state
table: `+6, +4, +2, +1, -1, -2, -4, -6`. For every nonzero mask, including
masks with three or more active sensors, the error is the average of all active
sensor weights. Every new sample updates the outer loop. The correction
magnitude reaches the 100 RPM limit at either outermost sensor. The controller
settings are:

- Base speed: 100 RPM
- Kp: 25
- Ki: 0
- Kd: 0
- Differential correction limit: +/-100 RPM

The target mix is `left_rpm = base_rpm - correction_rpm` and
`right_rpm = base_rpm + correction_rpm`, clamped to 0 through twice the base
speed. Use `LineTracking::setTunings()` and `LineTracking::setBaseRpm()` for
real-car tuning. Set `LineTracking::enabled` to `false`, or call
`LineTracking::stop()`, to command zero speed on both wheels.

When three or more sensors detect black, the controller still records the
wide-line flag and black-sensor count for diagnostics, but applies the same
weighted-average calculation to the complete mask. If no sensor detects black,
the outer PID state is reset and both wheels continue straight at the base
speed.

## Reusable PID Controller

`pid_controller.hpp` and `pid_controller.cpp` provide a hardware-independent,
allocation-free positional PID controller. Each instance owns its own gains,
limits, integral and derivative history. The API uses a control interval in
seconds and supports output limiting, integral limiting, conditional
anti-windup, derivative-on-error or derivative-on-measurement, and access to
the latest P/I/D terms.

The two wheel-speed controllers each use one independent `PidController`
instance. The line tracker uses a third instance as the outer loop and writes
only RPM targets; it never writes PWM directly. Future vision-position and
stepper-motor controllers can create additional instances without depending
on the car, encoder, sensor, or TB6612 modules.
