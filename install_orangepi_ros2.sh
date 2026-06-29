#!/bin/bash

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}>>> 开始自动安装 ROS2 Humble 桌面完整版及自定义依赖...${NC}"

# 1. 更新系统并安装基础依赖
echo -e "${GREEN}[1/8] 更新系统并安装基础依赖...${NC}"
sudo apt update && sudo apt upgrade -y
sudo apt install -y software-properties-common curl gnupg lsb-release python3-pip

# 2. 更换系统源为国内镜像 (以阿里云为例)
echo -e "${GREEN}[2/8] 更换系统 APT 源为阿里云镜像...${NC}"
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak
sudo sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list
sudo sed -i 's/security.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list
sudo apt update

# 3. 启用 Ubuntu Universe 仓库
echo -e "${GREEN}[3/8] 启用 Ubuntu Universe 仓库...${NC}"
sudo add-apt-repository universe -y

# 4. 添加 ROS2 GPG 密钥及软件源
echo -e "${GREEN}[4/8] 添加 ROS2 官方 GPG 密钥及软件源...${NC}"
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. 安装 ROS2 Humble 桌面完整版 + 你的自定义依赖
echo -e "${GREEN}[5/8] 正在安装 ROS2 Humble 桌面完整版及自定义依赖，请耐心等待...${NC}"
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# 6. 卸载冲突包并安装 Python 依赖
echo -e "${GREEN}[6/8] 配置 Python 依赖 (pymodbus, pyserial)...${NC}"
# brltty 可能会占用串口资源，导致通信失败，建议卸载
sudo apt remove -y brltty
pip install pymodbus pyserial --break-system-packages

# 7. 配置 USB 串口 udev 规则
echo -e "${GREEN}[7/8] 配置 USB 串口 udev 规则...${NC}"
# 假设你的 99-usb-serial.rules 文件与当前脚本在同一目录下
# 如果不在同一目录，请修改下方的源文件路径
RULES_FILE="99-usb-serial.rules"
if [ -f "$RULES_FILE" ]; then
    sudo cp "$RULES_FILE" /etc/udev/rules.d/99-usb-serial.rules
    # 重新加载 udev 规则并触发，使其立即生效
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo -e "${GREEN}    -> udev 规则已成功应用！${NC}"
else
    echo -e "${RED}    -> 警告：未在当前目录找到 $RULES_FILE，请手动复制！${NC}"
fi

# 8. 配置环境变量
echo -e "${GREEN}[8/8] 配置环境变量到 ~/.bashrc...${NC}"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo -e "${GREEN}>>> 🎉 ROS2 Humble 及所有依赖安装完成！${NC}"
echo -e "${GREEN}>>> 请打开新终端或执行 'source ~/.bashrc' 使环境变量生效。${NC}"
echo -e "${GREEN}>>> 可运行 'ros2 run turtlesim turtlesim_node' 进行测试。${NC}"
