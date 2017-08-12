ruby -Ku ../configure.rb -T v850_gcc
make
athrill -i -d device_config.txt asp
