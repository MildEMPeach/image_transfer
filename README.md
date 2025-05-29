# Image Transfer System

一个基于ROS2的高效图像传输系统，支持JPEG压缩和分块并行传输，具有跨平台兼容性。

## 项目简介

这个项目实现了一个完整的图像传输解决方案，使用ROS2服务通信机制，提供了压缩传输和分块并行传输两种优化策略。系统设计用于高效传输大尺寸图像，特别适用于4K图像传输场景。

## 功能特性

### 核心功能
- ✅ **JPEG压缩传输**：支持可配置的JPEG压缩质量（10-100）
- ✅ **分块并行传输**：将大图像分割成小块并行传输，支持可配置的块大小
- ✅ **跨平台兼容**：支持Ubuntu 22.04和Windows平台
- ✅ **实时性能监控**：传输时间、数据量、压缩率等指标统计
- ✅ **错误恢复**：自动重试机制和错误处理

### 传输策略
1. **普通传输**：直接传输原始图像数据
2. **JPEG压缩传输**：先压缩再传输，节省带宽
3. **分块传输**：将图像分割成块，逐块传输
4. **分块+压缩传输**：结合压缩和分块的混合策略

## 系统架构

```
image_transfer/
├── src/
│   ├── image_transfer/              # 主传输包
│   │   ├── src/
│   │   │   ├── image_transfer_server.cpp    # 服务端实现
│   │   │   └── image_transfer_client.cpp    # 客户端实现
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   └── image_transfer_interfaces/   # ROS2接口定义
│       ├── srv/
│       │   ├── ImageTransfer.srv    # 基础图像传输服务
│       │   └── ChunkTransfer.srv    # 分块传输服务
│       ├── package.xml
│       └── CMakeLists.txt
├── Scripts/                         # 测试和分析脚本
│   ├── run_compression_tests.py     # 压缩性能测试
│   ├── plot_compression_graph.py    # 压缩性能可视化
│   ├── run_chunk_tests.py          # 分块性能测试
│   ├── plot_chunk_graph.py         # 分块性能可视化
│   ├── generate_compression_images.py # 压缩测试图像生成
│   └── generate_4k_images.py       # 4K测试图像生成
└── README.md
```

## 环境要求

### 系统要求
- **Ubuntu**: 22.04 LTS (推荐)
- **Windows**: 10/11 (需要安装ROS2 Humble)
- **内存**: 4GB以上
- **存储**: 2GB可用空间

### 依赖包
- ROS2 Humble Hawksbill
- OpenCV 4.x
- CMake 3.16+
- Python 3.8+ (用于测试脚本)

### Python依赖
```bash
pip install opencv-python matplotlib numpy pandas pillow
```

## 安装指南

### 1. 克隆项目
```bash
git clone <repository-url>
cd image_transfer
```

### 2. 构建ROS2包
```bash
# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建项目
colcon build

# 设置环境
source install/setup.bash
```

### 3. 验证安装
```bash
# 终端1: 启动服务端
ros2 run image_transfer image_transfer_server

# 终端2: 运行客户端测试
ros2 run image_transfer image_transfer_client
```

## 使用说明

### 基本使用

#### 启动服务端
```bash
source install/setup.bash
ros2 run image_transfer image_transfer_server
```

#### 运行客户端
```bash
source install/setup.bash
ros2 run image_transfer image_transfer_client
```

### 服务接口

#### ImageTransfer服务
```
# 请求
string image_path          # 图像文件路径
int32 jpeg_quality         # JPEG压缩质量 (10-100, 0表示不压缩)

# 响应
bool success               # 传输是否成功
string message            # 状态消息
float64 transfer_time     # 传输时间(秒)
int64 data_size          # 数据大小(字节)
float64 compression_ratio # 压缩率
```

#### ChunkTransfer服务
```
# 请求
string image_path          # 图像文件路径
int32 chunk_size          # 块大小(字节, 0表示不分块)

# 响应
bool success               # 传输是否成功
string message            # 状态消息
float64 transfer_time     # 传输时间(秒)
int64 total_size         # 总数据大小(字节)
int32 chunk_count        # 块数量
```

## 测试脚本

项目提供了完整的性能测试和分析工具：

### 压缩性能测试
```bash
cd Scripts

# 生成测试图像
python generate_compression_images.py

# 运行压缩测试 (5重复测试，提供统计可靠性)
python run_compression_tests.py

# 生成性能图表
python plot_compression_graph.py
```

### 分块传输测试
```bash
cd Scripts

# 生成4K测试图像
python generate_4k_images.py

# 运行分块测试 (5重复测试)
python run_chunk_tests.py

# 生成性能图表
python plot_chunk_graph.py
```

### 测试结果

测试脚本会生成以下文件：
- `compression_results.csv`：压缩测试详细数据
- `chunk_results.csv`：分块测试详细数据
- `compression_performance.png`：压缩性能对比图
- `chunk_performance.png`：分块性能对比图

## 性能特性

### 压缩效果
- **压缩率**：根据JPEG质量参数，可达到2-10倍压缩
- **质量损失**：可控的有损压缩，适用于大多数应用场景
- **传输时间**：在网络带宽受限时显著提升传输速度

### 分块传输
- **并行处理**：支持多块并行传输
- **内存优化**：降低内存峰值使用
- **网络适应**：适应不同网络条件和MTU限制

### 性能指标
- **4K图像传输**：< 2秒 (局域网环境)
- **压缩传输**：节省60-90%带宽
- **分块传输**：降低50%内存使用

## 配置选项

### 压缩配置
- `jpeg_quality`: 10-100 (100为最高质量，10为最高压缩)
- `compression_enabled`: true/false

### 分块配置
- `chunk_size`: 字节数 (建议: 64KB - 1MB)
- `max_chunks`: 最大并发块数
- `chunk_timeout`: 块传输超时时间

## 故障排除

### 常见问题

1. **构建失败**
   ```bash
   # 清理构建
   rm -rf build install log
   colcon build
   ```

2. **服务连接失败**
   ```bash
   # 检查ROS2守护进程
   ros2 daemon stop
   ros2 daemon start
   ```

3. **图像找不到**
   - 确保图像路径正确
   - 检查文件权限
   - 支持格式：JPG, PNG, BMP, TIFF

4. **内存不足**
   - 减少块大小
   - 降低并发块数
   - 使用压缩传输

### 调试模式
```bash
# 启用详细日志
export RCUTILS_LOGGING_SEVERITY=DEBUG
ros2 run image_transfer image_transfer_server
```

## 开发指南

### 添加新的压缩算法
1. 在 `image_transfer_client.cpp` 中添加压缩函数
2. 更新服务接口定义
3. 实现对应的解压缩逻辑

### 扩展传输协议
1. 定义新的服务接口在 `image_transfer_interfaces`
2. 实现服务端处理逻辑
3. 更新客户端调用接口

## 贡献指南

1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 打开 Pull Request

---

*最后更新: 2025年5月*
