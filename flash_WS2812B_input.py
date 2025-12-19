import time
import argparse

# RDK X5 では Raspberry Pi 以外の環境向けに Adafruit CircuitPython の PixelBuf を利用
from adafruit_pixelbuf import PixelBuf
import spidev
import toml
import atexit
import signal
import sys
import threading
from typing import Optional
import glob
import re
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

# Prefer Hobot.GPIO on RDK-X5 for 40-pin header GPIO access
try:
    import Hobot.GPIO as GPIO
except Exception:
    GPIO = None

LED_COUNT      = 60
LED_BRIGHTNESS = 0.10  # 0.0-1.0 for CircuitPython neopixel

"""
SPI 設定（NeoPixel_SPI 経由で WS2812 を駆動）
- 既定デバイス: /dev/spidev1.0（RDK X5 では bus=1, dev=0 が一般的）
- 3MHz は WS2812B のエンコードに安全な値

GPIO 設定（40ピンヘッダのボタン入力を上書き制御に使用）
- `GPIO_PULL='up'` の場合、通常は HIGH、ボタン押下で LOW になります（プルアップ構成）
- `GPIO_ACTIVE_LOW=False` にすると「HIGH をアクティブ（割り込みで制御対象）」として扱います
    （LOW/HIGH の論理を反転して扱いたい場合に便利）
- 割り込みが利用できない場合はポーリング（`GPIO_POLL_MS` 間隔）にフォールバックします
"""
SPI_BAUDRATE   = 3000000
GPIO_PIN       = 16       # BOARD ピン番号 16（RDK-X5 で割り込み対応）
GPIO_ACTIVE_LOW = False   # False で HIGH をアクティブとして扱う（プルアップによりLOW/HIGH を反転した制御）
GPIO_ENABLE     = True    # GPIO 上書き（ボタンによる制御）を有効化
GPIO_POLL_MS    = 100     # 割り込みが使えない場合のポーリング間隔(ms)
GPIO_PULL       = 'up'    # 'up' | 'down' | 'none'

SHUTDOWN_EVENT = threading.Event()

def colorWipe(strip, color, wait_ms=50, should_abort=None):
    """Wipe color across display a pixel at a time."""
    for i in range(len(strip)):
        if should_abort and should_abort():
            return
        strip[i] = color
        strip.show()
        time.sleep(wait_ms/1000.0)

def theaterChase(strip, color, wait_ms=50, iterations=10, should_abort=None):
    """Movie theater light style chaser animation."""
    for j in range(iterations):
        if should_abort and should_abort():
            return
        for q in range(3):
            if should_abort and should_abort():
                return
            for i in range(0, len(strip), 3):
                strip[i+q] = color
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, len(strip), 3):
                strip[i+q] = (0, 0, 0)

def heartbeat(strip, color, wait_ms=50, iterations=1):
    """Heartbeat animation - LED pulses like a heartbeat (two pulses per cycle)."""
    for iteration in range(iterations):
        # First pulse: fade in and out
        for brightness in range(0, 256, 10):
            ratio = brightness / 255.0
            dimmed_color = tuple(int(c * ratio) for c in color)
            strip.fill(dimmed_color)
            strip.show()
            time.sleep(wait_ms/1000.0)
        
        for brightness in range(255, -1, -10):
            ratio = brightness / 255.0
            dimmed_color = tuple(int(c * ratio) for c in color)
            strip.fill(dimmed_color)
            strip.show()
            time.sleep(wait_ms/1000.0)
        
        # Pause between pulses
        strip.fill((0, 0, 0))
        strip.show()
        time.sleep(wait_ms * 2 / 1000.0)
        
        # Second pulse: fade in and out
        for brightness in range(0, 256, 10):
            ratio = brightness / 255.0
            dimmed_color = tuple(int(c * ratio) for c in color)
            strip.fill(dimmed_color)
            strip.show()
            time.sleep(wait_ms/1000.0)
        
        for brightness in range(255, -1, -10):
            ratio = brightness / 255.0
            dimmed_color = tuple(int(c * ratio) for c in color)
            strip.fill(dimmed_color)
            strip.show()
            time.sleep(wait_ms/1000.0)
        
        # Pause before next cycle
        strip.fill((0, 0, 0))
        strip.show()
        time.sleep(wait_ms * 3 / 1000.0)

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

def rainbow(strip, wait_ms=20, iterations=1, should_abort=None):
    """Draw rainbow that fades across all pixels at once."""
    for j in range(256*iterations):
        if should_abort and should_abort():
            return
        for i in range(len(strip)):
            strip[i] = wheel((i+j) & 255)
        strip.show()
        time.sleep(wait_ms/1000.0)

def rainbowCycle(strip, wait_ms=20, iterations=5, should_abort=None):
    """Draw rainbow that uniformly distributes itself across all pixels."""
    for j in range(256*iterations):
        if should_abort and should_abort():
            return
        for i in range(len(strip)):
            strip[i] = wheel((int(i * 256 / len(strip)) + j) & 255)
        strip.show()
        time.sleep(wait_ms/1000.0)

def theaterChaseRainbow(strip, wait_ms=50, should_abort=None):
    """Rainbow movie theater light style chaser animation."""
    for j in range(256):
        if should_abort and should_abort():
            return
        for q in range(3):
            if should_abort and should_abort():
                return
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
        with open(path, 'r', encoding='utf-8-sig') as f:
            data = toml.load(f)
        print(f"✓ config.toml を読み込みました: {path}")
        return data
    except FileNotFoundError:
        print(f"⚠ {path} が見つかりません。デフォルト設定を使用します。")
        return {}
    except Exception as e:
        print(f"⚠ config.toml の読み込みに失敗: {e}。デフォルト設定を使用します。")
        return {}

def apply_config_defaults(cfg: dict):
    global LED_COUNT, LED_BRIGHTNESS, SPI_BAUDRATE, GPIO_PIN, GPIO_ACTIVE_LOW, GPIO_ENABLE, GPIO_POLL_MS, GPIO_PULL
    led = cfg.get('led', {})
    spi = cfg.get('spi', {})
    gpio = cfg.get('gpio', {})
    LED_COUNT = int(led.get('count', LED_COUNT))
    LED_BRIGHTNESS = float(led.get('brightness', LED_BRIGHTNESS))
    SPI_BAUDRATE = int(spi.get('baudrate', SPI_BAUDRATE))
    GPIO_PIN = int(gpio.get('pin', GPIO_PIN))
    GPIO_ACTIVE_LOW = bool(gpio.get('active_low', GPIO_ACTIVE_LOW))
    GPIO_ENABLE = bool(gpio.get('enabled', GPIO_ENABLE))
    GPIO_POLL_MS = int(gpio.get('poll_ms', GPIO_POLL_MS))
    GPIO_PULL = str(gpio.get('pull', GPIO_PULL)).lower()

class GpioOverride:
    """
    40ピン GPIO 入力によるエフェクト上書き制御。

    - `active_low=True`: LOW をアクティブ（押下をアクティブ）として扱う
    - `active_low=False`: HIGH をアクティブ（非押下をアクティブ）として扱う（LOW/HIGH の反転）

    割り込み（エッジ検出）が使える場合は優先し、使えない場合はポーリングに切り替えます。
    `is_active()` は現在のアクティブ状態（論理アサート）を返します。
    """
    def __init__(self, board_pin: int, active_low: bool, poll_ms: int):
        self.board_pin = board_pin
        self.active_low = active_low
        self.poll_ms = max(10, poll_ms)
        self._active = False
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._ready = False

    def start(self):
        if GPIO is None:
            return
        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            # Configure input with requested pull mode if supported
            pud = None
            try:
                if GPIO_PULL == 'up' and hasattr(GPIO, 'PUD_UP'):
                    pud = GPIO.PUD_UP
                elif GPIO_PULL == 'down' and hasattr(GPIO, 'PUD_DOWN'):
                    pud = GPIO.PUD_DOWN
            except Exception:
                pud = None
            try:
                if pud is not None:
                    GPIO.setup(self.board_pin, GPIO.IN, pull_up_down=pud)
                else:
                    GPIO.setup(self.board_pin, GPIO.IN)
            except Exception:
                GPIO.setup(self.board_pin, GPIO.IN)
            # まず割り込み（エッジ検出）を利用。BOARD 16 は割り込み対応。
            try:
                # エッジ選択の方針：
                # - BOTH が使えるなら両エッジ（HIGH/LOW どちらも）を検出
                # - 使えない場合：active_low=True は FALLING（HIGH→LOW）
                #                  active_low=False は RISING（LOW→HIGH）
                edge = getattr(GPIO, 'BOTH', None) or (getattr(GPIO, 'FALLING', None) if self.active_low else getattr(GPIO, 'RISING', None))
                if edge is not None:
                    # 初期状態を読み取って、現在のアクティブ状態を決定
                    try:
                        val = GPIO.input(self.board_pin)
                        # active_low の意味：True→LOW がアクティブ、False→HIGH がアクティブ
                        self._active = (val == GPIO.LOW) if self.active_low else (val == GPIO.HIGH)
                    except Exception:
                        self._active = False
                    GPIO.add_event_detect(self.board_pin, edge, bouncetime=10)
                    # 両エッジ（または選択したエッジ）で状態更新するコールバックを登録
                    GPIO.add_event_callback(self.board_pin, self._on_edge)
                    self._ready = True
                    self._thread = None
                    return
            except Exception:
                # Fall through to polling thread
                pass
            # フォールバック：割り込みが使えない場合はポーリングスレッドを開始
            self._thread = threading.Thread(target=self._run, name="gpio-override", daemon=True)
            self._thread.start()
        except Exception:
            # If GPIO init fails, leave override disabled
            pass

    def _run(self):
        self._ready = True
        while not self._stop.is_set():
            try:
                val = GPIO.input(self.board_pin)
                # ポーリング時のアクティブ判定：active_low=True→LOW、False→HIGH
                asserted = (val == GPIO.LOW) if self.active_low else (val == GPIO.HIGH)
                self._active = bool(asserted)
            except Exception:
                self._active = False
            self._stop.wait(self.poll_ms / 1000.0)

    def _on_edge(self, channel):
        try:
            val = GPIO.input(self.board_pin)
            # 割り込みコールバック時のアクティブ判定：active_low=True→LOW、False→HIGH
            self._active = (val == GPIO.LOW) if self.active_low else (val == GPIO.HIGH)
        except Exception:
            self._active = False

    def is_active(self) -> bool:
        return self._active

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.2)
        try:
            if GPIO is not None:
                try:
                    GPIO.remove_event_detect(self.board_pin)
                except Exception:
                    pass
                GPIO.cleanup(self.board_pin)
        except Exception:
            pass

def run_effect(strip, cfg_effect: dict, should_abort=None):
    mode = cfg_effect.get('mode', 'demo')
    wait_ms = int(cfg_effect.get('wait_ms', 50))
    iterations = int(cfg_effect.get('iterations', 10))
    loop = bool(cfg_effect.get('loop', True))
    color = tuple(cfg_effect.get('color', [255, 0, 0]))

    def do_once():
        if should_abort and should_abort():
            return
        if mode == 'color_wipe':
            colorWipe(strip, color, wait_ms, should_abort)
        elif mode == 'theater_chase':
            theaterChase(strip, color, wait_ms, iterations, should_abort)
        elif mode == 'heartbeat':
            heartbeat(strip, color, wait_ms, 1)
        elif mode == 'rainbow':
            rainbow(strip, wait_ms, 1, should_abort)
        elif mode == 'rainbow_cycle':
            rainbowCycle(strip, wait_ms, 1, should_abort)
        elif mode == 'theater_chase_rainbow':
            theaterChaseRainbow(strip, wait_ms, should_abort)
        elif mode == 'demo':
            print('Color wipe animations.')
            colorWipe(strip, (255, 0, 0), should_abort=should_abort)
            colorWipe(strip, (0, 255, 0), should_abort=should_abort)
            colorWipe(strip, (0, 0, 255), should_abort=should_abort)

            print('Theater chase animations.')
            theaterChase(strip, (127, 127, 127), should_abort=should_abort)
            theaterChase(strip, (127, 0, 0), should_abort=should_abort)
            theaterChase(strip, (0, 0, 127), should_abort=should_abort)

            print('Heartbeat animation.')
            heartbeat(strip, (127, 127, 127))
            heartbeat(strip, (127, 0, 0))
            heartbeat(strip, (0, 0, 127))

            print('Rainbow animations.')
            rainbow(strip, should_abort=should_abort)
            rainbowCycle(strip, should_abort=should_abort)
            theaterChaseRainbow(strip, should_abort=should_abort)
        else:
            raise ValueError(f"Unknown effect mode: {mode}")

    if loop:
        while True:
            if should_abort and should_abort():
                return
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
    parser.add_argument('--config', type=str, default='', help='Path to config.toml')
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

    # 終了時や Ctrl-C で LED を消灯するよう登録
    atexit.register(lambda: clear_strip(strip))
    def _handle_exit_signal(signum, frame):
        SHUTDOWN_EVENT.set()
    signal.signal(signal.SIGINT, _handle_exit_signal)
    signal.signal(signal.SIGTERM, _handle_exit_signal)

    print('Press Ctrl-C to quit.')
    if not args.clear:
        print('Use "-c" argument to clear LEDs on exit')
    # GPIO 上書き（ボタン入力）監視を開始
    override = None
    if GPIO_ENABLE and GPIO is not None:
        override = GpioOverride(GPIO_PIN, GPIO_ACTIVE_LOW, GPIO_POLL_MS)
        override.start()
        logic_desc = "LOW をアクティブ" if GPIO_ACTIVE_LOW else "HIGH をアクティブ（論理反転）"
        print(f"GPIO 上書き有効: BOARD {GPIO_PIN}, {logic_desc}, pull={GPIO_PULL}")
        print("説明: プルアップの場合、通常は HIGH、押下で LOW になります。\n"
              "この設定では、'アクティブ' の定義を上記のとおり扱います。\n"
              "割り込みが利用可能ならエッジ検出（BOTH/RISING/FALLING）で即時に更新し、\n"
              "利用不可の場合はポーリングで状態を更新します。")
        atexit.register(lambda: override.stop())
    elif GPIO_ENABLE and GPIO is None:
        print('GPIO override requested but Hobot.GPIO not available; continuing without GPIO override')

    try:
        effect_cfg = dict(cfg.get('effect', {}))
        # Default wait for override steps
        override_wait = int(effect_cfg.get('wait_ms', 50))
        while not SHUTDOWN_EVENT.is_set():
            if override is not None and (override.is_active() and not SHUTDOWN_EVENT.is_set()):
                # 上書きがアクティブな間は赤のシアター・チェイスを継続（1イテレーションずつ）
                theaterChase(strip, (255, 0, 0), wait_ms=override_wait, iterations=1, should_abort=lambda: SHUTDOWN_EVENT.is_set())
            else:
                # 上書きが非アクティブ時に設定済みエフェクトを1回実行。
                # 実行中に上書きがアクティブへ遷移した場合は中断（プリエンプト）します。
                effect_cfg_once = dict(effect_cfg)
                effect_cfg_once['loop'] = False
                def abort_cb():
                    if SHUTDOWN_EVENT.is_set():
                        return True
                    return (override.is_active() if override is not None else False)
                run_effect(strip, effect_cfg_once, should_abort=abort_cb)
    except KeyboardInterrupt:
        SHUTDOWN_EVENT.set()
    finally:
        try:
            if override is not None:
                override.stop()
        except Exception:
            pass
        clear_strip(strip)
        try:
            sys.exit(0)
        except SystemExit:
            pass