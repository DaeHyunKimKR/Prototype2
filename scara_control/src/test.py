
import serial
import threading
import time

line = ''
port = '/dev/SCARA2' 
baud = 115200 

ser = serial.Serial(port, baud, timeout=3)

alivethread = True


def readthread(ser):
    global line
    
 
    while alivethread:
      
        for c in ser.read():
        
            line += (chr(c))
            if line.startswith('[') and line.endswith(']'):  
                print('receive data=' + line)
                line = ''

    ser.close()


def main():

    thread = threading.Thread(target=readthread, args=(ser,))
    thread.start()

    count = 10
    while count > 0:
        strcmd = '[test' + str(count) + ']'
        print('send data=' + strcmd)
        ser.write(strcmd.encode())
        time.sleep(1)
        count -= 1
        
    alivethread = False


main()
