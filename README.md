# RDK X5 WS2812B 制御ガイド (日本語)

このリポジトリは、`D-Robotics RDK X5` 上で `WS2812B (NeoPixel)` を `SPI` 経由で制御するためのサンプルです。  
`D-Robotics RDK X5` では `rpi_ws281x` が未対応ですが、`adafruit-circuitpython-neopixel-spi` と `spidev` を用いて安定動作します。  


## ファイル一覧
- LED点灯スクリプト: [RDK-X5_GPIO/flash_WS2812B.py](RDK-X5_GPIO/flash_WS2812B.py)
- GPIO割り込みサンプル: [RDK-X5_GPIO/GPIO_Input_interrupt.py](RDK-X5_GPIO/GPIO_Input_interrupt.py)
- LED点灯スクリプト（GPIO割り込みによる点灯パターン変更込み）: [RDK-X5_GPIO/flash_WS2812B_input.py](RDK-X5_GPIO/flash_WS2812B_input.py)
- 設定: [RDK-X5_GPIO/config.toml](RDK-X5_GPIO/config.toml)
- 依存: [RDK-X5_GPIO/requirements.txt](RDK-X5_GPIO/requirements.txt)

## 仕様
- `SPI` ベースで `WS2812B` を駆動 (RDK X5 対応)
- 外部設定ファイル `config.toml` により LED 本数・明るさ・SPI バス/デバイス・エフェクトを切り替え
- デモモード/単一エフェクトモードの両対応
- 割り込み対応ピン(16ピン)を使用して、LEDの点灯パターンを変更

## 依存関係のインストール
仮想環境を有効化し、依存をインストールします。

```bash
python -m venv .venv --system-site-packages  # 仮想環境でHobot.GPIOを使用するため、system site-packages込みで仮想環境を構築する。
source .venv/bin/activate
pip install -r requirements.txt
```

## 配線
| Connected | Value | Pin| Pin | Value | Connected|
| - | - | - | - | - | - |
| `10kΩ IN` | 3V3 | 1 | 2 | 5V | - |
| - | GPIO2 | 3 | 4 | 5V | `WS2812B VCC`<BR>(25%以下の出力を推奨) |
| - | GPIO3 | 5 | 6 | GND | - |
| - | GPIO4 | 7 | 8 | GPIO14 | - |
| - | GND | 9 | 10 | GPIO15 | - |
| - | GPIO17 | 11 | 12 | GPIO18 | - |
| - | GPIO27 | 13 | 14 | GND | `SW OUT` |
| - | GPIO22 | 15 | 16 | GPIO23 | `SW IN`<BR>`10kΩ IN` |
| - | 3V3 | 17 | 18 | GPIO24 | - |
| `WS2812B DIN` | GPIO10 | 19 | 20 | GND | `WS2812B GND` |
| - | GPIO9 | 21 | 22 | GPIO25 | - |
| - | GPIO11 | 23 | 24 | GPIO8 | - |
| - | GND | 25 | 26 | GPIO7 | - |
| - | GPIO0 | 27 | 28 | GPIO1 | - |
| - | GPIO5 | 29 | 30 | GND | - |
| - | GPIO6 | 31 | 32 | GPIO12 | - |
| - | GPIO13 | 33 | 34 | GND | - |
| - | GPIO19 | 35 | 36 | GPIO16 | - |
| - | GPIO26 | 37 | 38 | GPIO20 | - |
| - | GND | 39 | 40 | GPIO21 | - |

```
GPIO16 (物理ピン) ----+---- タクトスイッチ ---- GND
                      |
VCC (3.3V) ---- (外部プルアップ 10kΩ)

VCC (5.0V) ---- WS2812B RED
GPIO19 (SPI1.0 MOSI) ---- WS2812B DIN(GREEN)
GND ---- WS2812B GND(WHITE)
```

- `WS2812B DIN`はレベルシフト推奨: 3.3V から 5V へ論理レベル変換 (長距離・高輝度時は特に有効)

詳細なピンアサインは下記を参照ください。
![40-Pin Definition](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/01_40pin_user_sample/image/40pin_user_sample/image-20241217-202319.png)

RDK X5 のデバイスノード例: `/dev/spidev1.0`。存在デバイスは以下で確認できます。

```bash
ls /dev/spidev*
```

## 実行方法
基本の起動 (デフォルトでSPI1.0 を利用する):

```bash
python flash_WS2812B.py
```

SPIを指定し、LED 本数を変更する場合:

```bash
python flash_WS2812B.py --driver spi --spi-bus 1 --spi-dev 0 --count 60
```

## 設定ファイル `config.toml`
設定を外部化して運用できます。  
ファイルは同ディレクトリの `config.toml` を読み込みます (パス変更は `--config` で指定可能)。

```toml
[led]
count = 60
brightness = 0.10  # 0.0 - 1.0

[spi]
bus = 1
device = 0
baudrate = 3000000

[effect]
# mode: "demo" で既定のシーケンスをループ実行。
# 他に "color_wipe", "theater_chase", "rainbow", "rainbow_cycle", "theater_chase_rainbow" が選択可能。
mode = "demo"
color = [255, 0, 0]
wait_ms = 50
iterations = 10
loop = true

[gpio]
# GPIO上書き監視の設定。GPIO3とGNDショートで赤のtheater_chaseを継続。
enabled = true        # trueで有効化（Hobot.GPIOが見つからない場合は自動的に無効）
pin = 16              # BOARD番号。割り込み対応ピン（例: 13/16/18/22/27/28/32/33/37）を推奨
active_low = false    # false で HIGH をアクティブとして扱う（プルアップによりLOW/HIGH を反転した制御）
poll_ms = 100         # 割り込み非対応時のポーリング周期(ms)
pull = "up"           # 入力プル設定: "up" | "down" | "none"
```

- `led.count`: LED の本数
- `led.brightness`: 明るさ (0.0〜1.0)
- `spi.bus` / `spi.device`: 使用する SPI バス/デバイス番号
- `spi.baudrate`: SPI 速度 (WS2812B 用エンコードに安全な範囲で)
- `effect.mode`: 実行するエフェクトモード (上記のいずれか)
- `effect.color`: `color_wipe`/`theater_chase` などで用いる RGB 値
- `effect.wait_ms`: ステップ間の待ち時間 (ミリ秒)
- `effect.iterations`: 反復回数 (一部エフェクトで使用)
- `effect.loop`: true なら継続ループ、false なら一度だけ実行

### GPIO上書き（theater_chase赤の継続）
- `[gpio].enabled`: 上書き機能の有効/無効。
- `[gpio].pin`: BOARD番号で指定します。GPIO3とGNDショートを検出したい場合、RDK-X5の割り込み対応ピンとしてSPIで使用していない`16`を推奨。
- `[gpio].active_low`: Lowが有効（GNDショート）として扱います。
- `[gpio].poll_ms`: 割り込みに対応していないピン・環境でのポーリング周期。割り込み対応ピンでは`GPIO.add_event_detect`で即時反応します。
- `[gpio].pull`: 可能なら内部プルを設定します。ノイズで誤検出する場合は`"up"`（既定）を使い、必要に応じて`"down"`や`"none"`も選択可能。

上書き有効時の動作:
- ショート検出中は赤の`theater_chase`を継続します（解除で自動復帰）。
- 割り込み対応ピン（例: BOARD 16）ではイベントで即時検出、非対応ピンではポーリングします。

注意:
- RDK-X5のGPIOアクセスは`Hobot.GPIO`ライブラリを使用します（ボードにプリインストール）。  
  権限によっては`sudo`が必要です。  
  例: `sudo -E "$(which python3)" flash_WS2812B.py --driver spi --spi-bus 1 --spi-dev 0 --count 60`
- 手を近づけるだけで反応するなど感度が高い場合、物理的に10kΩ程度で3.3Vへプルアップ（active_low=true時）することを強く推奨します。

#### 物理プルアップ（推奨ハードウェア対策）
- 目的: 誤検出（手を近づけるだけでLOW判定になる等）を防ぎ、入力を安定化します。
- 配線例（active_low=trueの場合）:
  - BOARD 16（GPIO入力）→ 抵抗（10kΩ）→ 3.3V ピン（BOARD 1など）
  - BOARD 16 → スイッチ/短絡先 → GND ピン（BOARD 6/9/14/20/25/30/34/39 など）
  - これにより通常時は明確にHIGH、ショート時は明確にLOWになります。
- 注意事項:
  - 抵抗値は10kΩ前後を推奨（5k〜20kの範囲で調整可能）。
  - ケーブルが長い/周辺ノイズが多い場合はシールドやレイアウトの見直しも有効です。
  - 内部プル（`pull = "up"`）は環境により十分でない場合があり、外部プルアップが最も確実です。

#### GPIO機能の詳細（本リポジトリの拡張）
- 割り込み対応ピンの最適化: BOARD 16 を既定とし、`GPIO.add_event_detect`で即時反応します。  
  （非対応ピンはポーリング）
- 中断・上書きの挙動: 入力が有効（LOW）になると赤の`theater_chase`へ継続的に切り替え、解除で通常エフェクトへ復帰します。
- 中断の即時性: すべてのアニメーションにプリエンプト用`should_abort`を導入し、入力変化やCtrl-Cで即座に中断可能です。
- 終了処理: Ctrl-Cで安全にGPIO監視停止→LED消灯→終了します。

#### 入力確認の手順（診断用）
以下のスクリプトでBOARD 16の入力状態を確認できます。  
ショート時に`LOW`、通常時に`HIGH`が安定して出ることを確認してください。

```bash
sudo -E python3 - <<'PY'
import Hobot.GPIO as GPIO, time
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.IN, pull_up_down=getattr(GPIO, 'PUD_UP', None))
for _ in range(30):
    v = GPIO.input(16)
    print('BOARD 16 =', 'LOW' if v == GPIO.LOW else 'HIGH')
    time.sleep(0.2)
GPIO.cleanup(16)
PY
```

#### ピン情報の確認（hb_gpioinfo）
RDK-X5では`hb_gpioinfo`でSoCのピン状態/番号を確認できます。  
割り込み対応ピンの一例: 13、16、18、22、27、28、32、33、37  
参考: https://developer.d-robotics.cc/rdk_doc/Basic_Application/01_40pin_user_sample/gpio

```bash
sudo hb_gpioinfo
```

`--config` で別の設定ファイルを指定可能です。

```bash
python flash_WS2812B.py --driver spi --spi-bus 1 --spi-dev 0 --config /path/to/config.toml
```

## トラブルシューティング
### 内部プルアップが効かない/弱い場合
- 事象: 入力が未接続状態で`HIGH`にならず、手を近づけるだけで`LOW`へ誤判定する等。
- 原因候補:
  - RDK-X5の該当PADが内部プルアップ非対応、またはピンコントローラで未設定。
  - ユーザー空間ライブラリ（Hobot.GPIO）の`PUD_UP`がSoCのPADバイアスを変更できない構成。
  - 配線長/環境ノイズが大きく、弱いプルで電位が不安定。
- 推奨対処:
  - 外部プルアップを追加（10kΩ程度で3.3Vへ）。最も確実です。
  - OS側でPADバイアスを設定（Device Tree/overlayで`bias-pull-up`を指定）。詳細はD-Roboticsのpinctrlドキュメント参照。
    - 参考: https://developer.d-robotics.cc/rdk_doc/Advanced_development/linux_development/driver_development_x5/driver_pinctrl_dev

#### libgpiodでの状態確認（任意）
RDK-X5で`gpiodetect`/`gpioinfo`が利用可能なら、GPIOラインのバイアス（pull-up/down/none）を確認できます。

```bash
gpiodetect
gpioinfo  # 各ラインの方向/バイアス/消費者名を表示
# 例: 特定ラインの読み取り（gpiochipXのラインY）
gpioget gpiochip0 23
```

`gpioinfo`の出力に`bias pull-up`が表示されない場合、内部プルが未設定の可能性があります。

- `FileNotFoundError: /dev/spidevX.Y`: 該当デバイスが存在するか `ls /dev/spidev*` で確認し、`--spi-bus/--spi-dev` を調整してください。
- 点灯が不安定: 電源容量不足、配線長、レベル変換の有無を確認。`brightness` を下げると改善することがあります。  
  - `count = 60`, `brightness = 0.25`, `baudrate = 3000000` では `5V x 0.22A = 1.1W` が最大電力のようです。
- 色ずれ・チラつき: SPI 速度 (`baudrate`) を既定の 3MHz 近辺に調整し、ケーブル品質を確認してください。

## 参考
- Adafruit CircuitPython NeoPixel SPI: https://github.com/adafruit/Adafruit_CircuitPython_NeoPixel_SPI
- D-Robotics RDK Suite 3.1.1 Pin Configuration and Definition: https://developer.d-robotics.cc/rdk_doc/Basic_Application/01_40pin_user_sample/40pin_define
- D-Robotics RDK Suite 3.1.2 GPIO応用: https://developer.d-robotics.cc/rdk_doc/Basic_Application/01_40pin_user_sample/gpio
- D-Robotics RDK Suite 3.1.6 Using SPI: https://developer.d-robotics.cc/rdk_doc/Basic_Application/01_40pin_user_sample/spi
- D-Robotics RDK Suite 3.1.2 Using GPIO: https://developer.d-robotics.cc/rdk_doc/Basic_Application/01_40pin_user_sample/gpio

## ライセンス
本プロジェクトは MIT ライセンスです。詳細は [RDK-X5_GPIO/LICENSE](RDK-X5_GPIO/LICENSE) を参照してください。
