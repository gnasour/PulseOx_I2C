/**
 * @file server.c
 * @author George Nassour
 * @brief Server application using POSIX sockets to receive patient data from an associated medical device
 * @version 0.1
 * @date 2021-10-18
 *       2025-02-10
 * 
 * 
 */
#include <syslog.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/wait.h>
#include <string.h>
#include <netdb.h>
#include <errno.h>
#include <signal.h>

#include "client.h"
#include "dbcon.h"


void sigint_handler(int signum){
  //kill
}


int main(int argc, char* argv[]){


  //Command line option handling
  int opt = 0;
  while((opt = getopt(argc, argv, "f")) != -1){
    switch(opt){
      //-f: forking the Heart Rate data processing process
      case 'f':
        if(vfork()==0)
          execv("./data_process.py", argv);
        break;
    }
  }
  

  //Server Socket to handle incoming, new clients
  int sockfd;
  //Server child socket to handle client requests
  int newfd;
  //Finding the address of the ESP8266
  struct addrinfo hints, *res;
  struct sockaddr_storage their_addr;
  int addr_size = sizeof their_addr;


  //Packing addrinfo struct to retrieve necessary address information
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  //Retrieving necessary address information of the local server
  if(getaddrinfo(NULL, "1025", &hints, &res) != 0){
    perror("ERROR AT GETADDRINFO");
    exit(2);
  }

  //Creating server socket to listen for connections
  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if(sockfd < 0){
    exit(-1);
  }

  //Naming socket and listening
  bind(sockfd, res->ai_addr, res->ai_addrlen);
  listen(sockfd, 20);

  //Initialize database connection for clients to store data
  init_db();

  //openlog(argv[0], LOG_CONS|LOG_PID, LOG_USER);
  
  //Main loop
  while(1){
    newfd = accept(sockfd, (struct sockaddr *)&their_addr, (socklen_t*)&addr_size);
    printf("Connection made with new device\n");
    //Create child process to handle the accepted connection
    pid_t proc_id = fork();
    if(proc_id < 0){
      perror("Error on main loop fork");
    }else if(proc_id == 0){
      child_init();
      //get_pt_info();
	    recv_data(newfd);
      _exit(EXIT_SUCCESS);
    }else{
      printf("Parent process, closing socket\n");
      close(newfd);
      int wstatus;
      waitpid(proc_id, &wstatus, 0);
      //syslog(LOG_USER|LOG_DEBUG, "Process: %zd exited with status: %d\n", proc_id, wstatus);
      fprintf(stderr, "Process: %d exited with status: %d\n", proc_id, wstatus);
    }
  }
}

