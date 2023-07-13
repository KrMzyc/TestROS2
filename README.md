- **action_cpp**：一个actionServer和一个actionClient，client发布一个请求,请求得到 一系列的（x,y）坐标，server接受请求并发送一系列的（x,y）坐标
- **msg_srv_cpp**：ROS2的自定义消息和使用自定义消息
- **pubsub_cpp**：一个publisher随机发布一个（x,y）坐标，一个subscriber接到指令后进行保存，然后计算和上一步的距离
- **srvcli_cpp**：节点调用一个远端service，service发布（x.y）坐标

