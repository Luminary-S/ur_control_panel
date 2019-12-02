# 爬绳擦窗项目-盐田测试代码

# 现场测试工作
1. 测试 rcr arduino 中，将 rcr 反馈信息，放在一起
2. 超声波和力传感器不放在一起
3. 测试是否需要给 rcr arduino 给init cmd

# 程序结构

主要分成两个部分：
1. qt gui
2. ur3 control

## qt gui node
1. 实现 control arduino board 相关信息的显示 还有 车、清洗电机的手动控制
2. 实现 base cam 视频流的打开的关闭
3. 实现 sensor arduino board 中 **一维力传感器** 和 **超声波** 传感器信息的 显示

#### 测试参数服务器 

node relation:
1. gui node
2. node param server ( define through yaml )
3. 

### cam frame
选择串口


### sensor frame



### rcr frame



### UR frame

### 
