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
from `MSPM0_K230_OLED_Echo`. The display uses 7-bit I2C address `0x3C` on a
100 kHz bus shared with the grayscale sensor.

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

- Initial target: 50 RPM for each wheel
- Kp: 30.0
- Ki: 20.0
- Kd: 0.0
- PWM command limit: +/-800 of 1000
- Speed feedback filter coefficient: 0.25

Use `SpeedControl::setTargetRpm(left, right)` to change the wheel targets and
`SpeedControl::setTunings(kp, ki, kd)` to tune both controllers. The OLED
shows target RPM (`T`), measured RPM (`M`) and the signed PWM command.

Set `SpeedControl::pidEnabled` to `false` (0) to disable PID output at runtime.
Encoder measurement continues, but both motor outputs and the saved PID state
are cleared. Set it back to `true` (1) to resume closed-loop speed control.

The current automatic speed test starts both wheel targets at 50 RPM and
raises them by 50 RPM every two seconds. The ramp stops increasing when it
reaches 500 RPM and then holds that target.

## Application Structure

`empty_cpp.cpp` only initializes SysConfig and runs the application loop.
`car_app.cpp` owns subsystem startup and cooperative scheduling,
`speed_test.cpp` owns the stepped target test, and `pid_dashboard.cpp` owns
OLED formatting and refresh timing. Hardware drivers and the 100 Hz control
ISR remain isolated in their existing modules.

## Yahboom K230 UART Link

The K230 link is a bidirectional 115200-8-N-1 UART. Incoming bytes are placed
in a 512-byte interrupt-driven ring buffer. The current application echoes
the bytes unchanged and does not parse any Yahboom packet format.

The MSPM0 stream parser accepts `BALL,x,y\r\n`, validates coordinates against
the 640x480 AI frame, and shows `K230 BALL`, `X`, `Y`, and the decoded frame
sequence on the OLED. `BALL,-1,-1\r\n` displays `K230 NO BALL` with `X:---`
and `Y:---`. Invalid, partial, and overlong lines do not update the displayed
position. After the first valid frame, the OLED keeps the latest decoded K230
position instead of timing out to the PID page.

Before the first valid frame, the OLED alternates once per second between the
motor page and receive diagnostics. `K230 WAIT` with `RX:000000` means UART2
has not received any byte. `K230 RAW` means bytes are arriving, but no complete
valid `BALL,x,y` line has been decoded. The `DROP` line reports UART ring-buffer
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
`D:\designDocs\test.py`. It selects the largest valid detection and sends its
center in the 640x480 AI-frame coordinate system:

```text
BALL,320,240\r\n
```

The valid coordinate fields are zero-padded to three digits. When no ball is
detected it sends `BALL,-1,-1\r\n`; set `SEND_NO_TARGET_FRAME = False` to
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

The grayscale sensor shares I2C0 with the OLED. With both AD0 and AD1 address
jumpers open, its 7-bit address is `0x4C`. The driver retries the manual's
`0xAA` ping until it receives `0x66`; it does not block vehicle startup while
the sensor is absent or still initializing. Once connected, it samples at
20 Hz:

- `0xDD`: one digital byte, bit 0 through bit 7 map to channels 1 through 8.
- `0xB0`: eight analog bytes returned in channel 1 through channel 8 order.

The OLED alternates every second between a grayscale page and the existing
K230/PID pages. `GRAY WAIT A:4C` means the ping has not succeeded. `GRAY OK`
shows the digital bit mask, sample count, all eight analog values and the
accumulated I2C error count.

| Sensor signal | Connection |
| --- | --- |
| VCC | Regulated 5 V |
| GND | Common ground |
| SCL/SDA | 5 V side of a bidirectional I2C level shifter |
| Level-shifter 3.3 V side | PA1 SCL / PA0 SDA, shared with OLED |

Keep the OLED and MSPM0 side pulled up to 3.3 V. Put the sensor's optional
5 V pull-ups only on the level shifter's sensor side. Do not directly connect
the OLED to a bus pulled up to 5 V.
