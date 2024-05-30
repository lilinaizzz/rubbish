#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

void controlLED(int cmd, int arg)
{
    char *leds = "/dev/leds";

    int fd, ret;

    if ((fd = open(leds, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        printf("Open %s failed!\n", leds);
        return;
    }

    printf("Open %s success!\n", leds);

    ret = ioctl(fd, cmd, arg);

    close(fd);
}

int main()
{
    int fd;
    struct termios config;

    // 打开串口设备
    fd = open("/dev/ttySAC2", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("Error opening serial port");
        return 1;
    }

    // 配置串口
    memset(&config, 0, sizeof(config));
    if (tcgetattr(fd, &config) < 0) {
        perror("Error getting serial port attributes");
        close(fd);
        return 1;
    }

    // 设置波特率为115200
    cfsetispeed(&config, B115200);
    cfsetospeed(&config, B115200);

    // 8个数据位，无奇偶校验
    config.c_cflag &= ~PARENB;
    config.c_cflag &= ~CSTOPB;
    config.c_cflag &= ~CSIZE;
    config.c_cflag |= CS8;

    // 无软件流控制
    config.c_iflag &= ~(IXON | IXOFF | IXANY);
    config.c_iflag &= ~(INLCR | ICRNL);
    config.c_iflag &= ~(IGNCR | ICRNL);

    // 设置为原始模式
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 禁用输出处理
    config.c_oflag &= ~OPOST;

    // 设置新的串口属性
    if (tcsetattr(fd, TCSANOW, &config) < 0) {
        perror("Error setting serial port attributes");
        close(fd);
        return 1;
    }

    char cmd;
    while (1) {
        // 从串口读取命令
        if (read(fd, &cmd, sizeof(cmd)) > 0) {
            if (cmd == '0') {
                // 关闭LED灯
                controlLED(0, 0);
                printf("LED turned off.\n");
            } else if (cmd == '1') {
                // 打开LED灯
                controlLED(1, 1);
                printf("LED turned on.\n");
            }
        }
    }

    // 关闭串口
    close(fd);

    return 0;
}