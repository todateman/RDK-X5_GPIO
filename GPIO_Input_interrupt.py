#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
try:
    import Hobot.GPIO as GPIO
except ImportError:
    print("Hobot.GPIO が見つかりません。RDK-X5のシステムPythonに含まれるため、仮想環境を --system-site-packages 付きで作成するか、system Python を使用してください。\n例: python -m venv .venv --system-site-packages && source .venv/bin/activate")
    raise

# 使用する物理ピン番号
SWITCH_PIN = 16

def switch_callback(channel):
    if GPIO.input(channel) == GPIO.HIGH:
        print("スイッチが押されました (GNDにショート)")
    else:
        print("スイッチが離されました")
    # コールバックでは重い処理を行わず、ログのみ出力します。

def verify_internal_pullup(pin, samples=20, interval_sec=0.01):
    """プルアップが効いていそうか簡易確認します。
    通常、プルアップが有効なら未接続時は安定してLOWになります。
    LOW率が低い場合は設定ミスか接続不良の可能性が高いです。

    戻り値: True=プルアップが効いていそう, False=疑わしい
    """
    highs = 0
    for _ in range(samples):
        if GPIO.input(pin) == GPIO.LOW:
            highs += 1
        time.sleep(interval_sec)
    ratio = highs / float(samples)
    if ratio < 0.9:
        print("[警告] プルアップが安定していません (LOW率: {:.0%})".format(ratio))
        print("- 対策1: 外部10kΩ程度でVCCへのプルアップを追加してください")
        print("- 対策2: RDK-X5のピンコントローラで対象PADにbias-pull-up設定を適用し、内部プルアップを有効化 (DT/overlay)")
        print("  参考: developer.d-robotics.ccのpinctrlドキュメントを参照")
        print("        https://developer.d-robotics.cc/rdk_doc/Advanced_development/linux_development/driver_development_x5/driver_pinctrl_dev")
        return False
    else:
        print("プルアップは概ね有効です (LOW率: {:.0%})".format(ratio))
        return True

def main():
    try:
        GPIO.setwarnings(False)
        # GPIOモードを物理ピン番号モードに設定
        GPIO.setmode(GPIO.BOARD)
        
        # 指定ピンを入力モードに設定。内部プルアップが未サポートの場合は通常入力で設定
        pud_up = getattr(GPIO, 'PUD_UP', None)
        if pud_up is not None:
            print("内部プルアップを有効化して設定")
            GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=pud_up)
        else:
            print("内部プルアップを無効化して設定")
            GPIO.setup(SWITCH_PIN, GPIO.IN)

        # プルアップの効き具合を簡易チェック
        verify_internal_pullup(SWITCH_PIN)
        
        # 割り込み設定: 押下/解放の両エッジを検出
        # 外部プルアップ使用時はFALLINGのみでも可
        GPIO.add_event_detect(
            SWITCH_PIN,
            GPIO.BOTH,
            callback=switch_callback,
            bouncetime=200  # チャタリング防止（200ms）
        )

        print(f"GPIO{SWITCH_PIN}ピンの監視を開始しました")
        print("Ctrl+Cで終了します")

        # メインループ（コールバックが別スレッドで動作するため、待機のみ）
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nプログラムを終了します")
        
    finally:
        # GPIOのクリーンアップ
        GPIO.cleanup()
        print("GPIOをクリーンアップしました")
if __name__ == "__main__":
    main()

## プログラムの説明

## 1. **プルアップ**: 外部に10kΩ程度の抵抗を介してGPIOピンとVCC(3.3V)を接続し、通常時はHigh状態
## 2. **割り込み検出**: 押下/解放の両エッジを検出（外部プルアップ時はFALLINGのみでも可）
## 3. **チャタリング防止**: `bouncetime=200`で200msのデバウンス処理
## 4. **コールバック関数**: ショート検出時に`switch_callback`が自動実行

## 配線
## 
## GPIO (物理ピン) ----+---- タクトスイッチ ---- GND
##                       |
## VCC (3.3V) ---- (外部プルアップ 10kΩ)