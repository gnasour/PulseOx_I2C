#ifndef __client__
#define __client__

//Debug vital values sent to server
#ifndef DEBUG
#define DEBUG 0
#endif

void get_pt_info();
int recv_data(int socket_fd);
void child_init();

#endif
