// MIT License
// Copyright (c) 2021 Panagiotis Repouskos

#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <inttypes.h>
#include <pthread.h>

/* max number of interrupt  service routines */
#define _MAX_ISR                        10
/* how long to wait till check for events again */
#define _INTERRUPT_LATENCY_USEC         20

#define BCM2708_PERI_BASE       0x20000000 /* Physical addresses range from 0x20000000 to 0x20FFFFFF for peripherals */
#define GPIO_BASE               (BCM2708_PERI_BASE + 0x200000)	// GPIO controller 

#define BLOCK_SIZE 		(4*1024)


#define OUTPUT       1
#define INPUT        0
#define HIGH         1
#define LOW          0

/*pins in pinout available for general IO*/
#define GPIO4        4
#define GPIO17       17
#define GPIO18       18
#define GPIO21       21
#define GPIO22       22
#define GPIO23       23
#define GPIO24       24
#define GPIO25       25


/*constants used to set an interrupt*/
enum events {
  _HIGH, //pin is high
  _LOW //pin is low
};


/* errors that may occur */
enum errors {
  WRONG_ARGUMENTS = -1,
  EVENT_LISTENER_FULL = -2,
  EVENT_LISTENER_ERROR_LISTENING = -3,
  ERROR_OPENING_FILE = -4,
  ERROR_MAPPING = -5,
  ERROR_ALLOCATING_MEMORY = -6,
  ERROR_MUTEX = -7,
  ERROR_THREAD = -8,
  MUTEX_NOT_INITIALIZED = -9
};


//for IO access
typedef struct peripheral peripheral;
struct peripheral {
  unsigned long _addr_p;
  int _mem_fd;
  void *_map;
  volatile unsigned int *_addr;
};


typedef struct EventListener EventListener;
typedef struct ISR ISR;

struct ISR { //mapping ISR - interrupt number
  void (*function)(void*);
  uint8_t _id; //1.._MAX_ISR
  uint8_t _pin;  
  uint8_t _event;
  pthread_t _thread;
  pthread_mutex_t _lockUser;   //to be used by user
  int8_t _listening;
  uint8_t _hasLock; //1 if mutex initialized, else 0
};

//for interrupts
struct EventListener {
  uint8_t _listening; //listen while _listening is set to 1, stop when its 0
  ISR _ISRarray[_MAX_ISR]; //array with all ISRs 
  pthread_t _listener; //interrupt handler thread
  pthread_mutex_t _lock;
};




extern peripheral gpio;

/* ---------------------------------------
   -------------  FOR USER  --------------
   ---------------------------------------
*/


/*initialize*/
int8_t initializePeripheral();
void closePeripheral();

/* port manipulation */
int8_t digitalRead(uint8_t pin);
int8_t pinMode(uint8_t pin, uint8_t val);
int8_t digitalWrite(uint8_t pin, uint8_t val);

/*set interrupts*/
int8_t initializeListener();
int8_t attachEventListener(uint8_t pin, uint8_t event, void (*function)(void*), int8_t useMutex);
int8_t beginListening();
void stopListening();
void destroyListener();

/*helpful functions*/
int8_t lock_start(uint8_t key);
int8_t lock_end(uint8_t key);




/* 
   FOR SYSTEM - user doesn't need to use these functions
*/

void thread_listening(void*); //listening while EventListener->_listening == 1


