This repository contains example code for generating outputs to recreate a number of issues on the USRP N300, UHD version 3.13.1

Questions: sprager@usc.edu

# n300_txrx_pulse_test

## Installation
This program can be built directly on the N300. It is intended to run in embedded mode.

```
cd n300_txrx_pulse_test
mkdir build
cd build
cmake ../
make -j4
```

Tested with HG image:
```
uhd_image_loader --args "type=n3xx" --fpga-path=/usr/share/uhd/images/usrp_n300_fpga_HG.bit
```

## Running the program

### Example (Issue 3): Testing spectrum asymmetry
From the n300_txrx_pulse_test/build directory:
```
./n300_txrx_pulse_test --freq 1e9 --txgain 35 --rxgain 30 --ch_tx 0 --ch_rx 0 --wavefile ../../waveforms/chirpN100.bin --file ../../outputs/usrp_samples_default_fpga_HG_image_spectrumtest_35db.dat
```
### Example (Issue 4): Impulses at 2048, 4096,...
From the n300_txrx_pulse_test/build directory:
```
./n300_txrx_pulse_test --freq 1e9 --txgain 0 --rxgain 0 --ch_tx -1 --ch_rx 0 --nsamps 4096 --npulses 10 --wavefile ../../waveforms/chirpN100.bin --file ../../outputs/usrp_samples_default_fpga_HG_image_impulsetest.dat
```
### Waveform files
A few waveform files can be found in **n300_issue_tests/waveforms/**. They are binary complex int16 format and should be saved with the .bin extension. They can be generated using matlab with the function **n300_issue_tests/matlabtools/wave2file.m**.

### Output data
Output files should have the .dat extension. They can be read into matlab with the function **n300_issue_tests/matlabtools/file2wave.m**.

**NOTE:** I/Q sample ordering is swapped in the binary .dat and .bin files. So long as .dat is used for RX samples from the USRP and .bin is used for TX samples from file, the program/scripts will handle this.
