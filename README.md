To build:

```bash
mkdir build
cd build
cmake ..
make -j5

# as root
make install
ldconfig
```

After that re-open GNU radio and you should see a "SDR4" block.

Before running a GNU radio graph with sdr4 you need to configure the AD9361 (after each power-up) using:
https://github.com/gabriel-tenma-white/sdr4-sw/tree/master/ad9361/sw
