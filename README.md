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
