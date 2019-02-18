# Motion Driver

###### Code is for microcontroller: *dsPIC33FJ128M802*
http://www.microchip.com/wwwproducts/en/dsPIC33FJ128MC802


## Compile
Used Arch linux to compile.
Install xc16 MPLAB compiler, makefile is using binaries located by default at:
`/opt/microchip/xc16/v1.30/bin/`

## PicKit3 (upload .hex program to chip)
Use `mplab_ipe` located by default at: `/opt/microchip/mplabx/v3.50/mplab_ipe/mplab_ipe` when installed on Arch linux

To upload hex with mplab version 4.15
```
make upload mplab=v4.15
```

## Getting started
### Arch (yaourt)
- Install required tools `sudo yaourt -S microchip-mplabx-bin microchip-mplabxc16-bin`
- Compile `make`
- Run `mplab_ipe` to upload .hex


## Generating config
```
make config robot=big
make py robot=big
make js robot=big
```

### To use config from file: conf/robots/big.py
```
make robot=big
```

### Old board version (with different pinout interface with H-Bridge)
```
make board=OLD
```

## Simulation Mode
To compile in simulation mode:
```
make sim
```
To create virtual CAN device named can_big
```
make dev dev=can_big
```
To run simulation which uses device named can_big
```
./sim can_big
```
## Robot Axis
- X is forward
- Y is 90 deg CCW from X (as natural)


# Boards

- make board=V1 ver=OLD - dsPIC33FJ128M802 with older pcb design
- make board=V1 ver=NEW - dsPIC33FJ128M802 with newer pcb design
- make board=V2 - new PIC32MK1024MCF064 board
