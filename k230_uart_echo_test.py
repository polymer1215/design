"""Yahboom K230 <-> MSPM0 raw UART echo test.

Run this directly in CanMV IDE. It deliberately uses machine.UART instead of
Yahboom's YbUart/YbProtocol helpers, so no application packet is generated or
decoded.
"""

from machine import FPIOA, UART
import time


BAUD_RATE = 115200
TEST_INTERVAL_MS = 500
REPLY_TIMEOUT_MS = 300


fpioa = FPIOA()
# Yahboom K230 communication connector: GPIO9=UART1_TXD, GPIO10=UART1_RXD.
fpioa.set_function(9, FPIOA.UART1_TXD, ie=0, oe=1)
fpioa.set_function(10, FPIOA.UART1_RXD, ie=1, oe=0)
uart = UART(
    UART.UART1,
    baudrate=BAUD_RATE,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
)


def discard_stale_input():
    while uart.read(128) is not None:
        pass


def receive_exact(length, timeout_ms):
    reply = bytearray()
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while len(reply) < length and time.ticks_diff(deadline, time.ticks_ms()) > 0:
        chunk = uart.read(length - len(reply))
        if chunk is not None:
            reply.extend(chunk)
        else:
            time.sleep_ms(1)
    return bytes(reply)


discard_stale_input()
sequence = 0
while True:
    message = ("K230 PING %06d\r\n" % sequence).encode()
    uart.write(message)
    reply = receive_exact(len(message), REPLY_TIMEOUT_MS)

    if reply == message:
        print("PASS", sequence, reply)
    else:
        print("FAIL", sequence, "sent=", message, "received=", reply)

    sequence += 1
    time.sleep_ms(TEST_INTERVAL_MS)
