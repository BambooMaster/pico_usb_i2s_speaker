# pico_usb_i2s_speaker
Raspberry Pi Picoとtinyusbを使ったマスタークロック付きのi2sを出力するUSB DDCです。
[uac2_speaker_fb](https://github.com/hathach/tinyusb/tree/0.20.0/examples/device/uac2_speaker_fb)をベースに、ハイレゾ(96kHz 24bit)に対応しました。
USBスタックには、[tinyusb](https://github.com/hathach/tinyusb.git)を使用しています。

## Interpolation 機能
RP2350のDSPを使用したインターポレーション（オーバーサンプリング）機能を実装しています。
本機能は[interpolation](https://github.com/BambooMaster/pico_usb_i2s_speaker/tree/interpolation)ブランチで利用可能です。
インターポレーション処理は、[usb_sound_card_hires](https://github.com/BambooMaster/usb_sound_card_hires/tree/interpolation)のものを使用しています。

### インターポレーション倍率
- **44.1/48kHz**: **8倍**
- **88.2/96kHz**: **4倍**

### フィルタ特性 (44.1KHz)
- Passband: **20.5kHz**
- Passband Ripple: **0.001dB**
- Stopband: **22.05kHz**
- Stopband Attenuation: **-140dB**

## i2s
[pico-i2s-pio](https://github.com/BambooMaster/pico-i2s-pio.git)を使っています。RP2040/RP2350のシステムクロックをMCLKの整数倍に設定し、pioのフラクショナル分周を使わないlowジッタモードを搭載しています。  
i2s、PT8211の16bit右詰め、AK449XのEXDF、i2sのスレーブモードに対応しています。また、i2sとPT8211をデュアルモノで動作させることも可能です。

### デフォルト
lowジッタ、i2sモード  

|name|pin|
|----|---|
|DATA|GPIO18|
|LRCLK|GPIO20|
|BCLK|GPIO21|
|MCLK|GPIO22|

## build
### vscodeの拡張機能を使う場合
```
git clone https://github.com/BambooMaster/pico_usb_i2s_speaker.git
cd pico_usb_i2s_speaker
git submodule update --init
```
を実行した後、vscodeの拡張機能(Raspberry Pi Pico)でインポートし、ビルドしてください。

### vscodeの拡張機能を使わない場合
```
git clone https://github.com/BambooMaster/pico_usb_i2s_speaker.git
cd pico_usb_i2s_speaker
git submodule update --init
mkdir build && cd build
cmke .. && make -j4
```

## 対応機種
Windows11で動作確認をしています。Android (Pixel6a Android14)ではフィードバックが動作しませんでした。