#ifndef KEYBOARDBREAK_H_INCLUDED
#define KEYBOARDBREAK_H_INCLUDED

#include <unistd.h>   // UNIX standard function definitions 
#include <errno.h>    // Error number definitions 
#include <termios.h>  // POSIX terminal control definitions 
#include <sys/ioctl.h>
#include <getopt.h>
#include <stdio.h>
int kbhit(void);

#endif