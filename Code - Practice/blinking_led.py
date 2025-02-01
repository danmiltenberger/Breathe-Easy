# build the blinking led example via https://realpython.com/arduino-python/

import pyfirmata
import time

# create a class instance of the board vai pyfirmata
port = "COM3"
board = pyfirmata.Arduino(port)  # device manager / ports / details / port name

# create and start an iter for a value that is constantly updating
it = pyfirmata.util.Iterator(board)
it.start()

# the built in LED on the board is digital 13
board.digital[13].mode = pyfirmata.INPUT

# blink the built in LED on and off
while True:
    board.digital[13].write(1)
    time.sleep(0.1)
    board.digital[13].write(0)
    time.sleep(0.1)
