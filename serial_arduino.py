"Serial Pneumatic Board v1.0"
"by Thassyo Pinto - thassyo@ieee.org"

import serial

PORT = '/dev/ttyACM0'
BAUDRATE = 1000000

ser = serial.Serial(port=PORT, baudrate=BAUDRATE)
fn = "stair_0-50psi_47ohm_350u_15t_70a_2s_110320_0335_largepuck_center.csv"
#fn = input("enter filename: ")
f = open(fn, "w+")
f.write("Time,Setpoint,Pressure,Resistance\n")

while True:
  try:
    string_list = str(ser.readline().strip()).split()
    print(string_list[0])
    string_list = ",".join(string_list)[2:-1]
    # string_list = [float(i) for i in string_list]
    # float_list =
    # print(string_list)
    f.write(string_list)
    f.write("\n")
  except KeyboardInterrupt:
    break

  # dont need this because error from beginning means it was junk data.
  # except:
    # print("error")
ser.close()
