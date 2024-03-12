import asyncio
import json
import time
from asyncio import AbstractEventLoop
from gpiozero import LED
import serial
import spidev

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 9200

MOSFET_LEFT = LED(12)  # GPIO PIN for left dispenser
MOSFET_RIGHT = LED(6)  # GPIO PIN for right dispenser

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

FILE = "data.txt"


class DeviceSettings:
  def __init__(self, dispenser_left, dispenser_right):
    self.dispenser_left = dispenser_left
    self.dispenser_right = dispenser_right

  def serialize(self):
    return json.dumps({
      "dispenser_left": self.dispenser_left,
      "dispenser_right": self.dispenser_right
    })

  @staticmethod
  def deserialize(string):
    obj = json.loads(string)
    return DeviceSettings(dispenser_left=obj["dispenser_left"], dispenser_right=obj["dispenser_right"])


def initialize_device_settings():
  try:
    with open(FILE, "r") as f:
      data = f.read()
      print("Successfully read DeviceSettings from file: " + data)
      return DeviceSettings.deserialize(data)
  except FileNotFoundError:
    try:
      with open(FILE, "w") as f:
        f.write(DeviceSettings(0, 0).serialize())
        print("Successfully created DeviceSettings file")
      return DeviceSettings(0, 0)
    except Exception as e:
      print(e)
      return DeviceSettings(0, 0)


# spi used for communcation with ADC
spi = spidev.SpiDev()
spi.open(bus=0, device=0)
spi.mode = 0b00
spi.max_speed_hz = 5000


def read_command():
  pass


def write_command_to_HMI(command):
  pass


async def toggle_dispenser(mosfet):
  mosfet.on()
  await asyncio.sleep(.5)
  mosfet.off()
  await asyncio.sleep(1)


async def drop_medication():
  left = device_settings.dispenser_left
  right = device_settings.dispenser_right

  for i in range(left):
    await toggle_dispenser(MOSFET_LEFT)

  for i in range(right):
    await toggle_dispenser(MOSFET_RIGHT)


async def process_hmi_command(data):

  if data == 0:
    return
  if data == b'\x02\x03':  # debug
    MOSFET_LEFT.on()
  elif data == b'\x02\x04':  # debug
    MOSFET_LEFT.off()
  elif data == b'\x04\x01':  # Settings Screen, up left
    device_settings.dispenser_left += 1
  elif data == b'\x04\x02':  # Settings Screen, down left
    device_settings.dispenser_left -= 1
  elif data == b'\x04\x03':  # Settings Screen, up right
    device_settings.dispenser_right += 1
  elif data == b'\x04\x04':  # Settings Screen, down right
    device_settings.dispenser_right -= 1
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


def write_warning_to_HMI(warning):
  write_command_to_HMI("w-w " + warning)


def is_sensor_occupied(channel):
  """
  This method evaluates the sensor reading and puts it in 0 or 1
  0: sensor detects no object
  1: sensore detects object
  :return: bool
  """
  return 1 if read_adc_value_from_channel(channel) > 3 else 0


class EventHolder:
  def __init__(self, loop: AbstractEventLoop):
    self.loop = loop

  async def run(self):
    if (not is_sensor_occupied(SENSOR_LEFT_UP)):
      self.loop.call_soon(write_warning_to_HMI("Left Upper Sensor not working"), self.loop)


async def int_print():
  while True:
    print("int_print")
    await asyncio.sleep(1)


async def main_loop():
  print("Starting main loop")

device_settings = initialize_device_settings()


async def main(event_loop):
  event_holder = EventHolder(event_loop)
  event_loop.set_debug(True)
  event_loop.create_task(main_loop())

  await asyncio.sleep(1)
  hmi = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, bytesize=8, parity='N', stopbits=1, timeout=2,
                      rtscts=False, dsrdtr=False)
  while True:
    await asyncio.sleep(0.01)  # absolute minimum sleep time
    try:
      if hmi.in_waiting:
        data = hmi.read(hmi.in_waiting)
        current_time_with_format = time.strftime("%H:%M:%S")
        print("[" + current_time_with_format + "] Received Data: " + data.__str__())
        await process_hmi_command(data.__str__()) # process data from hmi
    except Exception as e:
      print(e)
      await asyncio.sleep(.01)
      hmi = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE)
      hmi.write("com_star".encode('utf-8'))

asyncio.get_event_loop().run_until_complete(main(asyncio.get_event_loop()))
