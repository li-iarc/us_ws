import serial
import struct

def modbus_crc16(data):
    """ 計算 Modbus CRC-16 """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)  # 轉成小端序的 2-byte 格式

def verify_crc(data):
    """ 驗證 Modbus CRC-16 """
    if len(data) < 2:
        return False
    received_crc = data[-2:]  # 最後兩個 byte 是 CRC
    calculated_crc = modbus_crc16(data[:-2])  # 計算 CRC
    return received_crc == calculated_crc

def parse_sensor_data(raw_data):
    """ 解析 8 個超聲波感測器數據 """
    if raw_data[:3] != b'\x01\x03\x10':  # 確保開頭正確
        return None
    try:
        sensor_values = []
        for i in range(3, 19, 2):  # 解析 8 個感測器數據
            value = int.from_bytes(raw_data[i:i+2], byteorder='big', signed=True)
            sensor_values.append(value)
        return sensor_values
    except ValueError:
        return None

def main():
    port = '/dev/ttyUSB0'  # 修改為你的串口
    baudrate = 115200
    
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print(f'Listening on {port} at {baudrate} baud.')
        
        while True:
            raw_data = ser.read(21)  # 讀取 21 個 byte (含 CRC)
            if len(raw_data) == 21:
                hex_data = raw_data.hex().upper()
                print(f'Received: {hex_data}')  # 印出收到的資料
                
                if verify_crc(raw_data):
                    sensor_values = parse_sensor_data(raw_data)
                    if sensor_values:
                        print(f'Parsed Data: {sensor_values}')
                else:
                    print(f'CRC Error: {hex_data}')  # 印出 CRC 錯誤的資料
    except serial.SerialException as e:
        print(f'Error opening serial port: {e}')
    except KeyboardInterrupt:
        print('\nTerminating program.')
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print('Serial port closed.')

if __name__ == '__main__':
    main()
