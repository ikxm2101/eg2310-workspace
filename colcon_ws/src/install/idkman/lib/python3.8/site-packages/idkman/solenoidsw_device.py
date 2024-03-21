from gpiozero import OutputDevice

class SolenoidSwitch_device(OutputDevice):
  def __init__(self, switch_pin: int) -> None:
    super().__init__(switch_pin)
