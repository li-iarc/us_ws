# us_ws
Ultrasonic sensor program

## us_pkg
- 所有程式都在此package中,以下說明各程式功能
  
### test.py
- 用於接收超聲波數值,無ROS2功能,僅為測試接收用途

### us_receiver_node.py
- 接收超聲波回傳的數值
- 彙整四個感測器數值發布sensor_data topic供其他節點訂閱
- sensor_data輸出格式為Float32MultiArray,無論數值是否更新都會定時發布
```bash
ros2 run us_pkg us_receiver_node
```

### us_drawline_node.py
- 訂閱sensor_data並繪製成可視化結果
- 四個角落對應到四個感測器,文字顯示當前該感測器數值
- 顏色可自行設定數值範圍為多少時文字會轉變為紅、橘、綠色
```bash
ros2 run us_pkg us_drawline_node
```

### us_colorbox_node.py
- 訂閱sensor_data並繪製成可視化結果
- 四個角落對應到四個感測器,文字顯示當前該感測器數值
- 顏色可自行設定數值範圍為多少時方格會轉變為紅、橘、綠色
```bash
ros2 run us_pkg us_colorbox_node
```

