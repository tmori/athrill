athrill
=======

Athrill は CPU エミュレータです. 
Athrill を使用すると，仮想マイコン上で組込みプログラムを簡単に実行/デバッグすることができます．
ベアメタルプログラムやリアルタイムOS上で動作する組込み制御プログラムを評価できます．
現時点の Athrill は V850 のCPU命令をサポートしています．

# Table of Contents
-----------------
  * [Requirements](#requirements)
  * [Install](#install)
  * [Deomo](#demo)
  * [License](#license)

# Requirements
------------
Athrill は， 以下の環境で動作します：

  * OS
    * [Linux]
      * Ubuntu(32bit)
    * [Windows]
      * MinGW32_NT-6.2(Windows10, Windows7)

Athrill は， ソースデバッグ用に以下のエディタを使用します:

  * エディタ
    * [Linux]
      * geany
    * [Windows]
      * Sakura Editor

# Install
-----
Athrill プロジェクトをダウンロード後,  環境変数(PATH)にathrillの実行バイナリフォルダのパスを追加しててください．

```
export PATH=[athrill root folder]/src/bin:$PATH 
```
任意のフォルダ上でathrillを空打ちし， usageが表示されればインストール完了です：

	Usage:athrill [OPTION]... <load_file>
	-i                             : execute on the interaction mode. if -i is not set, execute on the background mode.
	-r                             : execute on the remote mode. this option is valid on the interaction mode.
	-t<timeout>                    : set program end time using <timeout> clocks. this option is valid on the background mode.
	-p<fifo config file>           : set communication path with an another emulator.
	-d<device config file>         : set device parameter.

# Demo
---
Athrill を使用してリアルタイムOS (asp3)をデバッグするデモです.
 
![demo](https://github.com/tmori/athrill/blob/media/demo.gif)

# License
-------
Athrill is licensed under the TOPPERS License Agreement (http://www.toppers.jp/en/license.html).
