import serial
ser = serial.Serial('/dev/pts/3')
print(ser.name)
start = [0xFE,0xFF]
command = [0xFF]
datalen=[0x0D]
data=[[0x07, 0xB5, 0x05, 0xF1, 0x07, 0xD5, 0x07, 0xFE, 0x09, 0x58, 0x06, 0xF4, 0x03],
      [0x07, 0xB6, 0x05, 0xEC, 0x07, 0xD5, 0x07, 0xFE, 0x09, 0x60, 0x06, 0xF3, 0x03],
      [0x07, 0xB8, 0x05, 0xE7, 0x07, 0xD5, 0x08, 0x07, 0x09, 0x68, 0x06, 0xF4, 0x02],
      [0x07, 0xBB, 0x05, 0xE6, 0x07, 0xD1, 0x08, 0x00, 0x09, 0x6D, 0x06, 0xF5, 0x02],
      [0x07, 0xBE, 0x05, 0xE6, 0x07, 0xCB, 0x08, 0x0C, 0x09, 0x70, 0x06, 0xF6, 0x01],
      [0x07, 0xC2, 0x05, 0xE7, 0x07, 0xC6, 0x07, 0xEB, 0x09, 0x72, 0x06, 0xF6, 0x01],
      [0x07, 0xC5, 0x05, 0xE7, 0x07, 0xC2, 0x08, 0x13, 0x09, 0x74, 0x06, 0xF8, 0x00]]
stop=[0xAF,0xFF]
for i in range(len(data)):
        
    senddata = start+command+datalen+data[i] +stop

    ser.write(bytes(senddata))
    print(senddata)
ser.close()