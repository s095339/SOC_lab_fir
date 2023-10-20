# FIR - Verilog implementation
## Specification
- Data_Width  32
- Tape_Num    11
- Data_Num    To be determined by data size
## Interface
-- data_in  stream （Xn）
-- data_out: stream ( Yn)
-- coef[Tape_Num-1:0]  axilite
-- len: axilite
-- ap_start:  axilite
-- ap_done: axilite
- Using one Multiplier and one Adder
- - Shift register implemented with SRAM (Shift_RAM, size = 10 DW) – size = 10 DW
- Tap coefficient implemented with SRAM (Tap_RAM = 11 DW) and initialized by axilite write
Operation
- ap_start to initiate FIR engine (ap_start valid for one clock cycle)
- Stream-in Xn. The rate is depending on the FIR processing speed. Use axi-stream valid/ready for flow control
- Stream out Yn, the output rate depends on FIR processing speed.

## waveform


### configuration write
write
![Alt text](image-1.png)
read back
![Alt text](image-2.png)
### ap_start ap_done
![Alt text](image.png)
66020ns (6602 clk cycle)
### Xn stream-in, and Yn stream_out
stream in
![Alt text](image-3.png)
stream out
![Alt text](image-4.png)