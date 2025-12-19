import time
import argparse

# Prefer Adafruit CircuitPython for non-Raspberry Pi boards (RDK X5)
from adafruit_pixelbuf import PixelBuf
import spidev
import toml
import atexit
import signal
import sys
try:
    # SPI-based NeoPixel driver
    from neopixel_spi import NeoPixel_SPI
except Exception:
    NeoPixel_SPI = None
try:
    # PWM/PIO NeoPixel driver (may not be supported on RDK X5)
    import neopixel
except Exception:
    neopixel = None

LED_COUNT      = 60
LED_BRIGHTNESS = 0.10  # 0.0-1.0 for CircuitPython neopixel

# SPI configuration for RDK X5 (WS2812 over SPI via NeoPixel_SPI)
# Default device: /dev/spidev1.0
SPI_BAUDRATE   = 3000000  # 3MHz is safe for WS2812B encoding

def colorWipe(strip, color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(len(strip)):
        strip[i] = color
        strip.show()
        time.sleep(wait_ms/1000.0)

def theaterChase(strip, color, wait_ms=50, iterations=10):
    """Movie theater light style chaser animation."""
    for j in range(iterations):
        for q in range(3):
            for i in range(0, len(strip), 3):
                strip[i+q] = color
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, len(strip), 3):
                strip[i+q] = (0, 0, 0)

def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return (pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return (0, pos * 3, 255 - pos * 3)

def rainbow(strip, wait_ms=20, iterations=1):
    """Draw rainbow that fades across all pixels at once."""
    for j in range(256*iterations):
        for i in range(len(strip)):
            strip[i] = wheel((i+j) & 255)
        strip.show()
        time.sleep(wait_ms/1000.0)

def rainbowCycle(strip, wait_ms=20, iterations=5):
    """Draw rainbow that uniformly distributes itself across all pixels."""
    for j in range(256*iterations):
        for i in range(len(strip)):
            strip[i] = wheel((int(i * 256 / len(strip)) + j) & 255)
        strip.show()
        time.sleep(wait_ms/1000.0)

def theaterChaseRainbow(strip, wait_ms=50):
    """Rainbow movie theater light style chaser animation."""
    for j in range(256):
        for q in range(3):
            for i in range(0, len(strip), 3):
                strip[i+q] = wheel((i+j) % 255)
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, len(strip), 3):
                strip[i+q] = (0, 0, 0)

class SpiDevWrapper:
    def __init__(self, bus: int = 0, device: int = 0, baudrate: int = SPI_BAUDRATE):
        self._spi = spidev.SpiDev()
        self._spi.open(bus, device)
        self._spi.max_speed_hz = baudrate
        self._spi.mode = 0
        self._locked = False
    def write(self, b: bytes):
        # neopixel_spi expects a write() API taking bytes
        self._spi.xfer2(list(b))
    # Methods to mimic CircuitPython busio.SPI API used by SPIDevice
    def try_lock(self) -> bool:
        if not self._locked:
            self._locked = True
            return True
        return False
    def unlock(self):
        self._locked = False
    def configure(self, *, baudrate: int = SPI_BAUDRATE, polarity: int = 0, phase: int = 0):
        self._spi.max_speed_hz = baudrate
        self._spi.mode = (polarity << 1) | phase
    # Context manager support via SPIDevice
    def __enter__(self):
        return self
    def __exit__(self, exc_type, exc, tb):
        return False

def _init_strip_spi(count: int, bus: int = 0, device: int = 0, baudrate: int = SPI_BAUDRATE) -> PixelBuf:
    def _list_spidev():
        devs = []
        for p in glob.glob('/dev/spidev*'):
            m = re.search(r'spidev(\d+)\.(\d+)$', p)
            if m:
                devs.append((int(m.group(1)), int(m.group(2))))
        return sorted(devs)

    try:
        spi = SpiDevWrapper(bus, device, baudrate)
    except FileNotFoundError:
        candidates = _list_spidev()
        if candidates:
            alt_bus, alt_dev = candidates[0]
            print(f"/dev/spidev{bus}.{device} が見つかりません。代わりに /dev/spidev{alt_bus}.{alt_dev} を使用します。")
            spi = SpiDevWrapper(alt_bus, alt_dev, baudrate)
        else:
            print("利用可能なSPIデバイスがありません。/dev/spidev* を確認してください。")
            raise
    strip = NeoPixel_SPI(spi, count, brightness=LED_BRIGHTNESS, auto_write=False)
    return strip

def _init_strip_pwm(count: int) -> PixelBuf:
    raise RuntimeError("PWM driver not supported on RDK X5 without Blinka board pins")

def load_config(path: str) -> dict:
    try:
        data = toml.load(path)
        return data
    except Exception:
        return {}

def apply_config_defaults(cfg: dict):
    global LED_COUNT, LED_BRIGHTNESS, SPI_BAUDRATE
    led = cfg.get('led', {})
    spi = cfg.get('spi', {})
    LED_COUNT = int(led.get('count', LED_COUNT))
    LED_BRIGHTNESS = float(led.get('brightness', LED_BRIGHTNESS))
    SPI_BAUDRATE = int(spi.get('baudrate', SPI_BAUDRATE))

def run_effect(strip, cfg_effect: dict):
    mode = cfg_effect.get('mode', 'demo')
    wait_ms = int(cfg_effect.get('wait_ms', 50))
    iterations = int(cfg_effect.get('iterations', 10))
    loop = bool(cfg_effect.get('loop', True))
    color = tuple(cfg_effect.get('color', [255, 0, 0]))

    def do_once():
        if mode == 'color_wipe':
            colorWipe(strip, color, wait_ms)
        elif mode == 'theater_chase':
            theaterChase(strip, color, wait_ms, iterations)
        elif mode == 'rainbow':
            rainbow(strip, wait_ms, 1)
        elif mode == 'rainbow_cycle':
            rainbowCycle(strip, wait_ms, 1)
        elif mode == 'theater_chase_rainbow':
            theaterChaseRainbow(strip, wait_ms)
        elif mode == 'demo':
            print('Color wipe animations.')
            colorWipe(strip, (255, 0, 0))
            colorWipe(strip, (0, 255, 0))
            colorWipe(strip, (0, 0, 255))

            print('Theater chase animations.')
            theaterChase(strip, (127, 127, 127))
            theaterChase(strip, (127, 0, 0))
            theaterChase(strip, (0, 0, 127))

            print('Rainbow animations.')
            rainbow(strip)
            rainbowCycle(strip)
            theaterChaseRainbow(strip)
        else:
            raise ValueError(f"Unknown effect mode: {mode}")

    if loop:
        while True:
            do_once()
    else:
        do_once()

def clear_strip(strip):
    try:
        # Prefer PixelBuf/NeoPixel fill API if available
        strip.fill((0, 0, 0))
    except Exception:
        for i in range(len(strip)):
            strip[i] = (0, 0, 0)
    try:
        strip.show()
    except Exception:
        pass

# Main program logic follows:
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--clear', action='store_true', help='clear the display on exit')
    parser.add_argument('--driver', choices=['spi', 'pwm'], default='spi', help='WS2812B driver: spi (recommended for RDK X5) or pwm')
    parser.add_argument('--count', type=int, default=LED_COUNT, help='Number of LEDs')
    parser.add_argument('--spi-bus', type=int, default=1, help='SPI bus number (default 1 on RDK X5)')
    parser.add_argument('--spi-dev', type=int, default=0, help='SPI device number (e.g., 0)')
    parser.add_argument('--config', type=str, default='config.toml', help='Path to config.toml')
    args = parser.parse_args()

    # Load configuration and apply defaults
    cfg = load_config(args.config)
    apply_config_defaults(cfg)
    # Override LED_COUNT via CLI if provided
    if args.count != LED_COUNT:
        LED_COUNT = args.count

    # Initialize strip using selected driver
    if args.driver == 'spi':
        if NeoPixel_SPI is None:
            raise RuntimeError('neopixel_spi not available; install adafruit-circuitpython-neopixel-spi')
        bus = int(cfg.get('spi', {}).get('bus', args.spi_bus))
        dev = int(cfg.get('spi', {}).get('device', args.spi_dev))
        strip = _init_strip_spi(LED_COUNT, bus, dev, SPI_BAUDRATE)
    else:
        strip = _init_strip_pwm(LED_COUNT)

    # Ensure LEDs are turned off on exit and Ctrl-C
    atexit.register(lambda: clear_strip(strip))
    def _handle_exit_signal(signum, frame):
        clear_strip(strip)
        sys.exit(0)
    signal.signal(signal.SIGINT, _handle_exit_signal)
    signal.signal(signal.SIGTERM, _handle_exit_signal)

    print('Press Ctrl-C to quit.')
    if not args.clear:
        print('Use "-c" argument to clear LEDs on exit')
    try:
        run_effect(strip, cfg.get('effect', {}))

    except KeyboardInterrupt:
        # Always clear on Ctrl-C
        clear_strip(strip)
