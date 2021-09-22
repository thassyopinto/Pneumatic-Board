"Serial Pneumatic Board v1.0"
"by Thassyo Pinto - thassyo@ieee.org"

import serial

PORT = '/dev/ttyACM0'
BAUDRATE = 1000000

ser = serial.Serial(port=PORT, baudrate=BAUDRATE)
fn = "filename.csv"
f = open(fn, "w+")
f.write("Time,Setpoint,Pressure\n")

while True:
  try:
    string_list = str(ser.readline().strip()).split()
    print(string_list[0])
    string_list = ",".join(string_list)[2:-1]
    f.write(string_list)
    f.write("\n")
  except KeyboardInterrupt:
    break

ser.close()
