# Motion Driver

###### Code is for microcontroller: *dsPIC33FJ128M802*
http://www.microchip.com/wwwproducts/en/dsPIC33FJ128MC802


## Compile
Used Arch linux to compile.
Install xc16 MPLAB compiler, makefile is using binaries located by default at:
`/opt/microchip/xc16/v1.30/bin/`

## PicKit3 (upload .hex program to chip)
Use `mplab_ipe` located by default at: `/opt/microchip/mplabx/v3.50/mplab_ipe/mplab_ipe` when installed on Arch linux

## Getting started
### Arch (yaourt)
- Install required tools `sudo yaourt -S microchip-mplabx-bin microchip-mplabxc16-bin`
- Compile `make`
- Run `mplab_ipe` to upload .hex
