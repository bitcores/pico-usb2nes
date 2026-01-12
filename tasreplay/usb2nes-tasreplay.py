import os
import sys
from smbus2 import SMBus, i2c_msg
from ctypes import c_int8
from pathlib import Path
from time import sleep

# configure i2c bus to use
i2cbus = 0
repeat = False

os.system('clear')

inputslist = list()
if len(sys.argv) == 1:
    print("no replay file selected")
    print("usb2nes-tasreplay.py tasfilename.fm2 (or r08)")
    os._exit(0)

filename = sys.argv[1]
if len(sys.argv) > 2:
    if sys.argv[2] == "repeat" or sys.argv[2] == "r":
        repeat = True
    
replaypath = Path(filename)
if replaypath.exists():
    if filename[-3:] == "fm2":
        with open(filename) as fp:
            for line in fp:
                inputbyte = 0
                line = line.rstrip()
                if not line[0] == "|":
                    continue
                
                for x in range(3,11):
                    inputbyte <<= 1
                    if not line[x] == ".":
                        inputbyte += 1
                inputslist.append(inputbyte)
    elif filename[-3:] == "r08":
        with open(filename, "rb") as fp:
            while (byte := fp.read(1)):
                # r08 has reverse bit order to fm2, whatever
                joybyte = int.from_bytes(byte, byteorder="little")
                outbyte = 0
                for _ in range(8):
                    outbyte <<= 1
                    outbyte |= joybyte & 1
                    joybyte >>= 1
                inputslist.append(outbyte)
                fp.read(1) # read and discard the player 2 byte
    else:
        print("unsupported replay type. use fm2 or r08 files")
        os._exit(0)

    print("\nInitiating i2c connect")
    # presuming using i2c bus 0 on Pi, addr is 0x17 -> 23
    # addr 0 is for sending inputs, addr 1 for buffer state, addr 16 to set/read mode
    with SMBus(i2cbus) as bus:
        try:
            chkmode = 0
            retry = 16
            while chkmode != 16:
                bus.write_byte_data(23, 16, 16)
                chkmode = int(bus.read_word_data(23, 0)) & 0xFF
                #print("chkmode is ", chkmode)
                retry -= 1
                if retry == 0:
                    break
            
            if not chkmode == 16:
                print("tasreplay mode could not be set, aborting")
                os._exit(0)

            print("tasreplay mode started, sending inputs")
            # input pointer
            ip = 0
            while ip < len(inputslist):
                buffree = int(bus.read_word_data(23, 0) >> 8)
                #print(buffree, "bytes free in buffer")

                if not buffree == 0:
                    # send a max of 32 inputs per update
                    if buffree > 32:
                        #print("reducing send size")
                        buffree = 32
                    if ip + buffree > len(inputslist):
                        #print("reaching end of input")
                        buffree = len(inputslist) - ip
                
                    bus.write_block_data(23, 0, inputslist[ip:ip+buffree])
                    print("sending bytes ", ip, inputslist[ip:ip+buffree])
                    ip += buffree
                
                sleep(25/1000)
            
            print("tasreplay complete, exiting")
            os._exit(0)

        except:
            print("pico-usb2nes not found, check config and wiring")
            os._exit(0)

else:
    print(filename, " not found")
    os._exit(0)
