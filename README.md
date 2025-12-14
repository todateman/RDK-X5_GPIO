# RDK X5 WS2812B 制御ガイド (日本語)

このリポジトリは、`D-Robotics RDK X5` 上で `WS2812B (NeoPixel)` を `SPI` 経由で制御するためのサンプルです。  
`D-Robotics RDK X5` では `rpi_ws281x` が未対応ですが、`adafruit-circuitpython-neopixel-spi` と `spidev` を用いて安定動作します。

## 特長
- `SPI` ベースで `WS2812B` を駆動 (RDK X5 対応)
- 外部設定ファイル `config.toml` により LED 本数・明るさ・SPI バス/デバイス・エフェクトを切り替え
- デモモード/単一エフェクトモードの両対応

## 依存関係のインストール
仮想環境を有効化し、依存をインストールします。

```bash
source /.venv/bin/activate
pip install -r requirements.txt
```

## 配線
- MOSI (SPI1 MOSI) → WS2812B DIN
- GND → WS2812B GND
- 5V → WS2812B VCC (十分な電源容量を用意)
- レベルシフト推奨: 3.3V から 5V へ論理レベル変換 (長距離・高輝度時は特に有効)

ピンアサインは下記を参照ください。
![40-Pin Definition](https://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/03_Basic_Application/01_40pin_user_sample/image/40pin_user_sample/image-20241217-202319.png)

RDK X5 のデバイスノード例: `/dev/spidev1.0`。存在デバイスは以下で確認できます。

```bash
ls /dev/spidev*
```

## 実行方法
基本の起動 (SPI1.0 を利用する例):

```bash
python /home/sunrise/RDK-X5_GPIO/flash_WS2812B.py --driver spi --spi-bus 1 --spi-dev 0
```

LED 本数を変更する場合:

```bash
python /home/sunrise/RDK-X5_GPIO/flash_WS2812B.py --driver spi --spi-bus 1 --spi-dev 0 --count 60
```

## 設定ファイル `config.toml`
設定を外部化して運用できます。ファイルは同ディレクトリの `config.toml` を読み込みます (パス変更は `--config` で指定可能)。

```toml
[led]
count = 60
brightness = 0.25  # 0.0 - 1.0

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

`--config` で別の設定ファイルを指定可能です。

```bash
python flash_WS2812B.py --driver spi --spi-bus 1 --spi-dev 0 --config /path/to/config.toml
```

## トラブルシューティング
- `FileNotFoundError: /dev/spidevX.Y`: 該当デバイスが存在するか `ls /dev/spidev*` で確認し、`--spi-bus/--spi-dev` を調整してください。
- 点灯が不安定: 電源容量不足、配線長、レベル変換の有無を確認。`brightness` を下げると改善することがあります。  
  - `count = 60`, `brightness = 0.25`, `baudrate = 3000000` では `5V x 0.22A = 1.1W` が最大電力のようです。
- 色ずれ・チラつき: SPI 速度 (`baudrate`) を既定の 3MHz 近辺に調整し、ケーブル品質を確認してください。

## ファイル一覧
- スクリプト: [RDK-X5_GPIO/flash_WS2812B.py](RDK-X5_GPIO/flash_WS2812B.py)
- 設定: [RDK-X5_GPIO/config.toml](RDK-X5_GPIO/config.toml)
- 依存: [RDK-X5_GPIO/requirements.txt](RDK-X5_GPIO/requirements.txt)

## 参考
- Adafruit CircuitPython NeoPixel SPI: https://github.com/adafruit/Adafruit_CircuitPython_NeoPixel_SPI
- D-Robotics RDK Suite 3.1.1 Pin Configuration and Definition: https://developer.d-robotics.cc/rdk_doc/Basic_Application/01_40pin_user_sample/40pin_define
- D-Robotics RDK Suite 3.1.6 Using SPI: https://developer.d-robotics.cc/rdk_doc/Basic_Application/01_40pin_user_sample/spi

## ライセンス
本プロジェクトは MIT ライセンスです。詳細は [RDK-X5_GPIO/LICENSE](RDK-X5_GPIO/LICENSE) を参照してください。
