from gpiozero import LED
import serial
from time import sleep
from bitstring import BitArray

import spidev

led = LED(17)

spi = spidev.SpiDev()

spi.open(bus=0, device=0)
spi.mode = 0b00
spi.max_speed_hz = 5000

MCP3202_STARTBIT = 1
MCP3202_SGL = 1
MCP3202_ODD = 0b000  # 0: Ch0, 1: Ch1
MCP3202_MSBF = 1

MCP3202_VDD = 5
MCP3202_VSS = 0

# The Starting Payload configures the output of the A/D Converter via DOUT
STARTPAYLOAD = bytearray()
STARTPAYLOAD.append(MCP3202_STARTBIT)
STARTPAYLOAD.append(MCP3202_SGL << 7 | MCP3202_ODD << 6 )
STARTPAYLOAD.append(0b0)

MOSFET_1 = 17

# bus = 0 from ls /dev/*spi*
# device = 1, chip select pin

ttys0 = serial.Serial("/dev/ttyS0", 9600, parity=serial.PARITY_NONE, timeout=.1)
print(ttys0.name)


def read_command():
  try:
    buf = ttys0.read(2)
    if (buf == b''):
      return 0x0
    else:
      ttys0.reset_input_buffer()
      return buf
  except:
    return


def process_btn(data):
  if data == 0:
    return
  if data == b'\x02\x03':
    led.on()
  elif data == b'\x02\x04':
    led.off()


def main():
  while True:
    bytes = spi.xfer(STARTPAYLOAD)
    print("len(spi.xfer(STARTPAYLOAD)): ", len(bytes))
    print("bytes[0]: ", bytes[0])
    print("bytes[1]: ", bytes[1])
    print("bytes[2]: ", bytes[2])
    summ = (bytes[1] << 8) | bytes[2]
    print("Sum: " + str(summ))
    print("Bitarray: ", BitArray(bytes).bin)
    voltage = (summ / 0b111111111111) * MCP3202_VDD
    print("Voltage: " + str(voltage))
    sleep(0.05)

    print("------")

    data_from_hmi = read_command()
    print(str(data_from_hmi))
    process_btn(data_from_hmi)



main()