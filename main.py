from gpiozero import LED
import serial
from time import sleep
import spidev

MOSFET_LEFT = LED(17)  # GPIO PIN for left dispenser
MOSFET_RIGHT = LED(0)  # GPIO PIN for right dispenser
MOSFET_3 = LED(0)  # Stuck one

SENSOR_LEFT_DOWN = 0
SENSOR_RIGHT_DOWN = 1
SENSOR_LEFT_UP = 2
SENSOR_RIGHT_UP = 3
SENSOR_ACKKNOWLEDGE = 4

MCP3208_STARTBIT = 1
MCP3208_SGL = 1
MCP3208_ODD = 0b111  # Binary representation of the channel to read, ex. 0b101 = CH5, 0b100 = CH4
MCP3208_MSBF = 1
MCP3208_VDD = 5
MCP3208_VSS = 0

# spi used for communcation with HMI
spi = spidev.SpiDev()
spi.open(bus=0, device=0)
spi.mode = 0b00
spi.max_speed_hz = 5000

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


def write_command_to_HMI(command):
  pass

def toggle_dispenser(mosfet):
  mosfet.on()
  sleep(.5)
  mosfet.off()
  sleep(1)


def drop_medication():
  left = 1
  right = 4

  for i in range(left):
    if is_sensor_occupied(SENSOR_LEFT_DOWN):
      toggle_dispenser(MOSFET_LEFT)
    else:
      write_command_to_HMI("Füll mach nach, ALARM CRIT, #TODO")

  for i in range(right):
    if is_sensor_occupied(SENSOR_RIGHT_DOWN):
      toggle_dispenser(MOSFET_RIGHT)
    else:
      write_command_to_HMI("Füll mach nach, ALARM CRIT, #TODO")


def process_hmi_command(data):
  if data == 0:
    return
  if data == b'\x02\x03':  # debug
    MOSFET_LEFT.on()
  elif data == b'\x02\x04':  # debug
    MOSFET_LEFT.off()
  elif data == b'\x04\x01':  # Settings Screen, up left
    pass
  elif data == b'\x04\x02':  # Settings Screen, down left
    pass
  elif data == b'\x04\x03':  # Settings Screen, up right
    pass
  elif data == b'\x04\x04':  # Settings Screen, down right
    pass
  elif data == b'\x03\x01':  # Drop the desired medication
    drop_medication()


def generate_mcp3208_payload(channel=0):

  """
  The Starting Payload configures the output of the A/D Converter via DOUT
  for further details look into the MCP3208 Documentation, basically it's a 3 Byte Payload
  1. Byte: 0b00000<STARTBIT><SGL/DIFF><D2>
  2. Byte: <D1><D0><D0>XXXXX
  3. Byte: XXXXXXXX

  Return the bitwise payload for xferring to the spi device
  """
  payload = bytearray()
  channel_selector = channel
  payload.append((MCP3208_STARTBIT << 2) | (MCP3208_SGL << 1) | (((channel_selector & 0b100) >> 2) << 0))
  payload.append((channel_selector & 0b011) << 6)
  payload.append(0b11111111)
  return payload


def read_adc_value_from_channel(channel=0):
  """
  This method reads the value from the ADC, sets the correct channel value c ( 0<c<8) and
  returns the linarised voltage with the reference voltage (MCP3208_VDD)
  :param channel: channel you want to read the value from, 0..7
  :return: a float value betwen 0 and MCP3208:_VDD
  """
  bytes = spi.xfer(generate_mcp3208_payload(channel))
  summ = (bytes[1] << 8) | bytes[2]
  voltage = (summ / 0b111111111111) * MCP3208_VDD
  return voltage


def is_sensor_occupied(channel):
  """
  This method evaluates the sensor reading and puts it in 0 or 1
  0: sensor detects no object
  1: sensore detects object
  :return: bool
  """
  return 1 if read_adc_value_from_channel(channel) > 3 else 0


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

    process_hmi_command(data_from_hmi)

    sleep(1)


main()
