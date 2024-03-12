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
MOSFET_HMI = LED(?)   # GPIO PIN for controlling the on/off behaviour of the HMI

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

HMI_CORRECTLY_POWERED = False

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


class Payload:
  """
  page:
    0: Main Page
    1: Startup Menu Page
    2: Prototype Page
    3: Production Page
    4: Settings Page (Setting the correct dispenser settings

  action:
    ButtonPush, ButtonRelease, Shutdown, Error, PowerOn

  payload:
    ToggleMedicationDrop
    InitialSendingPayload
    A:0,B:0 // Payload used for modifying dispenser settings
  """
  def __init__(self, page, action, payload):
    self.page = page
    self.action = action
    self.payload = payload

  @staticmethod
  def deserialize(raw):
    pl = json.loads(raw.decode('utf'))
    return Payload(pl['page'], pl['action'], pl['payload'])

async def process_hmi_command(data):
  if data == 0:
    return

  try:
    packet = Payload.deserialize(data)
  except Exception as e:
    HMI_CORRECTLY_POWERED = False
    print(e)

  if packet.page == 0:
    pass

  if packet.page == 1: # Startup Menu Page
    if packet.action == "PowerOn":
      HMI_CORRECTLY_POWERED = True

  if packet.page == 2:
    if packet.action == "ButtonPush":
      if packet.payload == "?":
        drop_medication()

  if packet.page == 4: # Device Settings Page
    if packet.action == "UpdateDropsettings":
      # payload has the type of "A:x,B:y,..." so here comes some string splitting into play
      lists = packet.payload.split(";")
      device_settings.dispenser_left = int(lists[0].split(":")[1])
      device_settings.dispenser_right = int(lists[1].split(":")[1])


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


class NextionFacade:
  """
  This class holds a facade for displaying all kind of messages and seding data to the nextion display.

  """

  NEXTION_WARNING_PAGE_NO = -1
  NEXTION_WARNING_PAGE_idnt = "txt_nextion_warning_page"
  NEXTION_ERROR_PAGE_NO = -1
  NEXTION_ERROR_PAGE_idnt = "txt_nextion_error_page"
  NEXTION_INFO_PAGE_NO = -1
  NEXTION_INFO_PAGE_idnt = "txt_nextion_info_page"


  def __init__(self, hmi: Serial, active_page = 0):
    self.hmi = hmi
    self.active_page = active_page

  def set_page(self, page: int):
    self.send_command("page {}".format(page))

  def set_display_message(self, display_message: str):
    if active_page == self.NEXTION_INFO_PAGE_NO:
      self.send_command(self.NEXTION_INFO_PAGE_idnt + ".txt=" + display_message)
    if active_page == self.NEXTION_WARNING_PAGE_NO:
      self.send_command(self.NEXTION_WARNING_PAGE_idnt + ".txt=" + display_message)
    if active_page == self.NEXTION_ERROR_NO:
      self.send_command(self.NEXTION_ERROR_PAGE_NO + ".txt=" + display_message)


  def send_command(self, command):
    self.hmi.write(command)
    self.hmi.write(0xFF)
    self.hmi.write(0xFF)
    self.hmi.write(0xFF)


async def int_print():
  while True:
    print("int_print")
    await asyncio.sleep(1)




async def main_loop():
  print("toggle_dispenser start")
  while True:
    await asyncio.sleep(2)


device_settings = initialize_device_settings()

async def main(event_loop):
  event_holder = EventHolder(event_loop)
  event_loop.set_debug(True)
  event_loop.create_task(main_loop())

  await asyncio.sleep(1)
  hmi = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, bytesize=8, parity='N', stopbits=1, timeout=2,
                      rtscts=False, dsrdtr=False)
  print("Starting main loop")
  nextion_controller = NextionFacade(hmi)
  
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
