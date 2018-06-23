athrill
=======

Athrill is a CPU emulator. 
Athrill was developed to easily execute and debug embedded programs on virtual microcomputer. 
You can evaluate bare metal programs or embedded control programs running on real-time OS. 
Athrill at the moment supports V850 CPU instructions.

# Table of Contents
-----------------
  * [Requirements](#requirements)
  * [Install](#install)
  * [Deomo](#demo)
  * [License](#license)

# Requirements
------------
Athrill requires the following to run:

  * OS
    * [Linux]
      * Ubuntu(32bit)
    * [Windows]
      * MinGW32_NT-6.2(Windows10, Windows7)

Athrill uses the following editors for source debugging:

  * Editor
    * [Linux]
      * geany
    * [Windows]
      * Sakura Editor

# Install
-----
After downloading Athrill project, add the following athrill executable binary folder path on the environment-variable (PATH).

```
export PATH=[athrill root folder]/src/bin:$PATH 
```
Then make sure you can display usage of athrill on an arbitrary folder:

  Usage:athrill -m <memory config file> [OPTION]... <load_file>
    -i                             : execute on the interaction mode. if -i is not set, execute on the background mode.
    -r                             : execute on the remote mode. this option is valid on the interaction mode.
    -t<timeout>                    : set program end time using <timeout> clocks. this option is valid on the background mode.
    -m<memory config file>         : set athrill memory configuration. rom, ram region is configured on your system.
    -d<device config file>         : set device parameter.

# Demo
---
Athrill debugging Real-time OS (asp3) demonstration.
 
![demo](https://github.com/tmori/athrill/blob/media/demo.gif)

# License
-------
Athrill is licensed under the TOPPERS License Agreement (http://www.toppers.jp/en/license.html).
