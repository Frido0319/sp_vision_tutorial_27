# nav_lecture4小作业：Qos_debugger
这道题的目标就是让你快速上手、理解什么是ros。抛开复杂的概念，ros本质上完成的任务就是便利的进程间通信。比如，我有两个进程，一个进程发布雷达数据，另一个进程接收。使用ros就可以方便的完成通讯。你可以搜索以下，发送信息有哪些类型，分别有何特点。特别注意，不同的信息传输方式有不同的质量要求。你不会允许送的外卖没到你手上，但是一个电话过来，也许漏接了也无所谓，可能只是个诈骗。ros2也是这样。重点关注这一点会对这道题有所帮助

> 环境要求：ROS2 Humble

## 包结构

```
src/
  nav_hw_interfaces/     # 接口包：只放 .msg，无业务代码
    msg/SensorData.msg   # 可以打开.msg文件查看接口详细内容
  qos_debugger/          # 业务节点包
    src/qos_debugger_pub.cpp   # 发布 /SensorData
    src/qos_debugger_sub.cpp   # 订阅 /SensorData
```

## 编译

```bash
cd lecture4/homework
colcon build 
source install/setup.bash
```


## 任务一：实现pub和sub的通信

```bash
ros2 -h     //有忘记的命令就输入-h去查询用法
```

**现象**：启动pub和sub节点后sub节点订阅不到任何消息
提示：如果两个节点不能通过话题通信，我们应该如何区查看话题的详细信息（有没有相关的命令）
任务一仅修复qos_debugger_pub.cpp的一处或几处代码即可完成


---

## 任务二：为什么收到的消息会丢包？/(ㄒoㄒ)/~~

第一问找到问题并修改代码后，记得重新
```colcon build```
```source install/setup.bash```
**现象**：sub会打印黄色的warning输出告诉你丢包的序列，每秒还会打印出丢包率

提示：
有没有什么命令可以查看节点的配置(ros2 param -h)
可以通过修复qos_debugger_sub.cpp中的一处或几处代码解决该问题（可能会有多种解决方法）

## 任务三：把收到的消息的帧率计算并打印出来（放在定时器回调函数中每秒打印一次即可）
补全qos_debugger_sub.cpp即可



在下面按顺序完成三个任务，要求把用到的命令放入代码块中并讲解命令，每一问最好加入自己的理解

## 任务一：让 pub 和 sub 能够通信

先分别运行两个节点，再在第三个终端检查话题两端的 QoS：

```bash
ros2 run qos_debugger qos_debugger_sub
ros2 run qos_debugger qos_debugger_pub
ros2 topic info /sensor_data --verbose
ros2 param get /sensor_publisher reliability
ros2 param get /sensor_subscriber reliability
```

`ros2 topic info --verbose` 会显示话题的 publisher 和 subscriber，包括每一端的 reliability、
history 和 depth。`ros2 param get` 用来再确认节点当前使用的参数值。

原来 publisher 提供 `best_effort`，而 subscriber 请求 `reliable`。订阅端的要求高于发布端
能提供的保证，ROS2 不会在这两个端点之间建立匹配。我把 publisher 的 `reliability`
默认值改为 `reliable`，两端现在都使用 Reliable，subscriber 就可以收到 `/sensor_data`。

修改代码后需要重新编译并让当前终端加载新的安装空间：

```bash
cd lecture4/homework
colcon build
source install/setup.bash
```

## 任务二：定位和修复丢包

用下面的命令可以查看 subscriber 的参数，并在运行中改变回调延时：

```bash
ros2 param list /sensor_subscriber
ros2 param get /sensor_subscriber callback_delay_ms
ros2 param get /sensor_subscriber depth
ros2 param set /sensor_subscriber callback_delay_ms 30
ros2 param set /sensor_subscriber callback_delay_ms 0
```

publisher 默认每秒发布 100 条消息，相邻消息大约间隔 10 ms。原来 subscriber 却在每次
回调中睡眠 30 ms，单线程 executor 的消费速度跟不上发布速度。在 `KeepLast(depth)`
的有限历史中，旧消息可能在回调处理前被更新的消息取代，于是序号会出现空缺。

我把 `callback_delay_ms` 的默认值改为 0，正常运行时不再人为拖慢 subscriber。这个参数
仍然保留，可以用上面的 `ros2 param set` 命令重现慢回调现象。另外，原代码虽然计算了
每次序号空缺的 `lost`，却没有把它累加到 `lost_count_`，所以丢包率始终不对。
现在发现正向序号空缺时会执行 `lost_count_ += lost`，每秒输出的累计丢包数和丢包率
才与收到的序号一致。

## 任务三：计算接收帧率

定时器每次进入 `report()` 时，用当前 `received_count_` 减去上次保存的
`last_received_count_`，得到这一段时间实际处理的消息数。再用 `steady_clock`
计算两次报告之间的秒数，帧率为：

```text
接收帧率 = 本周期新收到的消息数 / 实际经过的秒数
```

`steady_clock` 只用于测量经过时间，不会因为系统时钟校准而突然跳变。计算完后更新上次
计数和时间点，下一次定时器就会统计新的一秒区间。运行时会每秒多输出一行：

```text
接收帧率: 99.50 Hz
```

这个数值表示 subscriber 实际执行回调的速度，不是 publisher 配置中的标称速度。

