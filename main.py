from gpiozero import LED
import serial
from time import sleep
import spidev

led = LED(17)
MOSFET_1 = 17

spi = spidev.SpiDev()

spi.open(bus=0, device=0)
spi.mode = 0b00
spi.max_speed_hz = 5000

MCP3202_STARTBIT = 1
MCP3202_SGL = 1
MCP3202_ODD = 0b111 # Binary representation of the channel to read, ex. 0b101 = CH5, 0b100 = CH4
MCP3202_MSBF = 1

MCP3202_VDD = 5
MCP3202_VSS = 0





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

def generate_mcp3208_payload(channel=0):
  # The Starting Payload configures the output of the A/D Converter via DOUT
  # for further details look into the MCP3208 Documentation, basically it's a 3 Byte Payload
  # 1. Byte: 0b00000<STARTBIT><SGL/DIFF><D2>
  # 2. Byte: <D1><D0><D0>XXXXX
  # 3. Byte: XXXXXXXX
  payload = bytearray()
  channel_selector = bin(channel)
  payload.append((MCP3202_STARTBIT << 2) | (MCP3202_SGL << 1) | (((channel_selector & 0b100) >> 2) << 0))
  payload.append((channel_selector & 0b011) << 6)
  payload.append(0b11111111)
  return payload


def read_adc_value_from_channel(channel=0):
  bytes = spi.xfer(generate_mcp3208_payload(channel))
  summ = (bytes[1] << 8) | bytes[2]
  voltage = (summ / 0b111111111111) * MCP3202_VDD
  # print("len(spi.xfer(STARTPAYLOAD)): ", len(bytes))
  # print("bytes[0]: ", bytes[0])
  # print("bytes[1]: ", bytes[1])
  # print("bytes[2]: ", bytes[2])
  # summ = (bytes[1] << 8) | bytes[2]
  # print("Sum: " + str(summ))
  # print("Bitarray0: ", bin(bytes[0]))
  # print("Bitarray1: ", bin(bytes[1]))
  # print("Bitarray2: ", bin(bytes[2]))
  return voltage



def main():
  while True:


    print("Voltage CH0: " + str(read_adc_value_from_channel(0)))
    print("Voltage CH1: " + str(read_adc_value_from_channel(1)))
    print("Voltage CH2: " + str(read_adc_value_from_channel(2)))
    print("Voltage CH3: " + str(read_adc_value_from_channel(3)))
    print("Voltage CH4: " + str(read_adc_value_from_channel(4)))
    print("Voltage CH5: " + str(read_adc_value_from_channel(5)))
    print("Voltage CH6: " + str(read_adc_value_from_channel(6)))
    print("Voltage CH7: " + str(read_adc_value_from_channel(7)))

    print("------")

    data_from_hmi = read_command()
    print(str(data_from_hmi))
    process_btn(data_from_hmi)



main()