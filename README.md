# us_ws
Ultrasonic sensor program

## us_pkg
- 此為舊方法,請改使用us_pkg8
  
## us_pkg8
- 可用於接收超聲波轉接盒資訊,以下說明各程式功能
- 初次使用時若找不到板子,使用以下程式並重開機
```bash
sudo apt purge brltty
```
- 確認串口
```bash
ls /dev/ttyUSB* /dev/ttyACM*
```
- 依據使用的串口開啟權限 (以ttyUSB0為例)
```bash
sudo chmod 777 /dev/ttyUSB0
```
- 依據使用的串口需修改下方各receiver_node程式中的串口

### test.py
- 用於接收超聲波數值,無ROS2功能,僅為測試接收用途
- 此程式會顯示RX的21位元組資訊,經crc驗證轉換為感測器讀數(毫米)

### us_modbus_receiver_node.py
- 接收超聲波回傳的數值 (使用modbus協議的轉接盒時使用)
- 彙整八個感測器數值發布sensor_data_1 topic供其他節點訂閱
- 輸出格式為Int16MultiArray,無論數值是否更新都會定時發布
```bash
ros2 run us_pkg us_modbus_receiver_node
```

### us_modbus_receiver_node2.py
- 接收超聲波回傳的數值 (使用第二個modbus協議的轉接盒時使用)
- 彙整八個感測器數值發布sensor_data_2 topic供其他節點訂閱
- 輸出格式為Int16MultiArray,無論數值是否更新都會定時發布
```bash
ros2 run us_pkg us_modbus_receiver_node2
```

### us_uart_receiver_node.py
- 接收超聲波回傳的數值 (使用uart協議的轉接盒時使用)
- 彙整八個感測器數值發布ultrasonic_uart_data topic供其他節點訂閱
- 輸出格式為Int16MultiArray,無論數值是否更新都會定時發布
```bash
ros2 run us_pkg us_uart_receiver_node
```


### us_show_node.py
- 訂閱sensor_data_1並繪製成可視化結果
- 四個角落各兩個感測器,文字顯示感測器編號與當前數值
- 顏色可自行設定數值範圍為多少時文字會轉變為紅、橘、綠色
```bash
ros2 run us_pkg us_show_node
```


