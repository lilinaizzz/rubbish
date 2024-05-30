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

    // �򿪴����豸
    fd = open("/dev/ttySAC2", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("Error opening serial port");
        return 1;
    }

    // ���ô���
    memset(&config, 0, sizeof(config));
    if (tcgetattr(fd, &config) < 0) {
        perror("Error getting serial port attributes");
        close(fd);
        return 1;
    }

    // ���ò�����Ϊ115200
    cfsetispeed(&config, B115200);
    cfsetospeed(&config, B115200);

    // 8������λ������żУ��
    config.c_cflag &= ~PARENB;
    config.c_cflag &= ~CSTOPB;
    config.c_cflag &= ~CSIZE;
    config.c_cflag |= CS8;

    // �����������
    config.c_iflag &= ~(IXON | IXOFF | IXANY);
    config.c_iflag &= ~(INLCR | ICRNL);
    config.c_iflag &= ~(IGNCR | ICRNL);

    // ����Ϊԭʼģʽ
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // �����������
    config.c_oflag &= ~OPOST;

    // �����µĴ�������
    if (tcsetattr(fd, TCSANOW, &config) < 0) {
        perror("Error setting serial port attributes");
        close(fd);
        return 1;
    }

    char cmd;
    while (1) {
        // �Ӵ��ڶ�ȡ����
        if (read(fd, &cmd, sizeof(cmd)) > 0) {
            if (cmd == '0') {
                // �ر�LED��
                controlLED(0, 0);
                printf("LED turned off.\n");
            } else if (cmd == '1') {
                // ��LED��
                controlLED(1, 1);
                printf("LED turned on.\n");
            }
        }
    }

    // �رմ���
    close(fd);

    return 0;
}