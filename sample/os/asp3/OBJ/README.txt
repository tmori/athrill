ruby -Ku ../configure.rb -T v850_gcc
make clean
make
athrill2 -c1 -i -m memory.txt -d device_config.txt asp
