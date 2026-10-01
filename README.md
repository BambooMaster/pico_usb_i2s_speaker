# pico_usb_i2s_speaker
Raspberry Pi Picoとtinyusbを使ったマスタークロック付きのi2sを出力するUSB DDCです。
[uac2_speaker_fb](https://github.com/hathach/tinyusb/tree/0.20.0/examples/device/uac2_speaker_fb)をベースに、ハイレゾ(96kHz 24bit)に対応しました。
USBスタックには、[tinyusb](https://github.com/hathach/tinyusb.git)を使用しています。

## Interpolation 機能
RP2350のDSPを使用したインターポレーション（オーバーサンプリング）機能を実装しています。
RP2350のデュアルコアを活かし、高性能なフィルタを実装したものが[interpolation](https://github.com/BambooMaster/pico_usb_i2s_speaker/tree/interpolation)ブランチ、シングルコアのみで実装をしたものが[interpolation-lite](https://github.com/BambooMaster/pico_usb_i2s_speaker/tree/interpolation-lite)ブランチです。

### インターポレーション倍率
- **44.1/48kHz**: **8倍**
- **88.2/96kHz**: **4倍**

### フィルタ特性 (44.1KHz)
#### interpolation ブランチ
- Passband: **20.5kHz**
- Passband Ripple: **0.001dB**
- Stopband: **22.05kHz**
- Stopband Attenuation: **-140dB**

#### interpolation-lite ブランチ
- Passband: **20kHz**
- Passband Ripple: **0.002dB**
- Stopband: **24.1kHz**
- Stopband Attenuation: **-100dB**

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
```bash
git clone https://github.com/BambooMaster/pico_usb_i2s_speaker.git
cd pico_usb_i2s_speaker
git switch main
git submodule update --init
```
を実行した後、vscodeの拡張機能(Raspberry Pi Pico)でインポートし、ビルドしてください。  
interpolation機能を使用する場合は、`git switch main`を`git switch interpolation`に変更してください。


### vscodeの拡張機能を使わない場合
```bash
git clone https://github.com/BambooMaster/pico_usb_i2s_speaker.git
cd pico_usb_i2s_speaker
git switch main
git submodule update --init
mkdir build && cd build
cmake .. && make -j4
```
interpolation機能を使用する場合は、`git switch main`を`git switch interpolation`または`git switch interpolation-lite`に変更してください。

## 動作確認環境
- Windows11
- Ubuntu 24.04
- Pixel6a (Android 16)
- iPad Air Gen4 (iPadOS 26.3.1)