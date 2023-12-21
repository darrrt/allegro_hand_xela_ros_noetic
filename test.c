#include <stdio.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include<linux/can.h>
#include<linux/can/raw.h>
#include<net/if.h>
int main(){
    int ch;
    ch = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (close(ch) < 0) {
        return 1;
    }
}
