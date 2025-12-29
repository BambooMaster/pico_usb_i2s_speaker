# pico_usb_i2s_speaker
Raspberry Pi Picoとtinyusbを使ったマスタークロック付きのi2sを出力するUSB DDCです。USBスタックには、[tinyusb](https://github.com/hathach/tinyusb.git)を使用しています。

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