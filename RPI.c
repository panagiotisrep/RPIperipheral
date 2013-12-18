#include "RPI.h"
#include <fcntl.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>

peripheral gpio = {GPIO_BASE};
EventListener* interrupts;

int initializePeripheral() {
  peripheral* p = &gpio;

  /* physical address using mmap on dev/mem
  */
  
  //open dev/mem
  if ((p->_mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
    return ERROR_OPENING_FILE;
  }

  p->_map = mmap(
		 NULL,
		 BLOCK_SIZE,
		 PROT_READ|PROT_WRITE,
		 MAP_SHARED,
		 p->_mem_fd,
		 p->_addr_p
		 );

  if (p->_map == MAP_FAILED) {
    return ERROR_MAPPING;
  }

  p->_addr = (volatile unsigned int*)p->_map;
  return 0;
}



void unmap_peripheral(struct peripheral* p) {
  munmap(p->_map, BLOCK_SIZE);
  close(p->_mem_fd);
}





int8_t digitalRead(uint8_t pin) {
  /* the register (GPLEV0) which reads the state of the GPIO
     pins is at virtual address 0x7E200034, that's to say 13 * 32 bits
     registers away from gpio base resgister, so we add int 13 to pointer 
     and perform a bitwise AND to get the value of the pin 
  */
  if (pin != GPIO4 && pin != GPIO17 && pin != GPIO18 && pin != GPIO21 && pin != GPIO22 && pin != GPIO23 && pin != GPIO24 && pin != GPIO25)
    return WRONG_ARGUMENTS;

  if (*(gpio._addr + 13) & (1<<pin) )
    return HIGH;
  else
    return LOW;
}





int8_t pinMode(uint8_t pin, uint8_t val) {
  /* each GPIO pin's operation is defined by 3 bits, stored in 
     GPFSEL0 - GPFSEL5, which begin at address 0x7E200000. 
     "000" sets the pin as input, "001" as output. To get the correct
     register we divide pin's number with 10, since each register holds the bits
     for 10 pins. Then to set the correct bits we get the modulo of the division 
     of pin by 10 and multiply it by 3. First we must set all bits to zero 

     This function will fail if val isn't INPUT or OUTPUT or if the pin isn't one 
     of the GPIO pins available at the pinout, for safety 
  */
  
  
  
  if (val != OUTPUT && val != INPUT)
    return WRONG_ARGUMENTS;
  
  if (pin != GPIO4 && pin != GPIO17 && pin != GPIO18 && pin != GPIO21 && pin != GPIO22 && pin != GPIO23 && pin != GPIO24 && pin != GPIO25)
    return WRONG_ARGUMENTS;


  /*set all bits to zero
    7(10) = 111(2), ~111(2) = 000(2), then left shift */

  *(gpio._addr + pin/10) &= ~(7<<((pin%10)*3));

  /* if val = INPUT, finished! */
  if (val == INPUT)
    return 0;

  /*val is OUTPUT, set bits to "001". Now they are "000" so we need to set the last
    to 1.*/

  *(gpio._addr + pin/10) |= 1<<((pin%10)*3);

  return 0;
}





int8_t digitalWrite(uint8_t pin, uint8_t val) {
  /* To write LOW or HIGH to a pin, we must address
     different registers. Separating these functions removes the need for
     read-modify-write operations.

     To write HIGH to a pin, we must acess the GPSET0 register, at address
     0x7E20001C. For n=0...31, writing 1 to bit n, will set the pin n to HIGH.
     Writing 0 will have no effect. If the pin is set as INPUT, the value is ignored.

     To write LOW, we must access the GPCLR0 register, at address 0x7E200028.
     For n=0...31, , writing 1 to bit n, sets pin n to LOW.
     Writing 0 has no effect. If the pin is defined as INPUT, the value is ignored.

     This function will fail if val isn't HIGH or LOW or if the pin isn't one 
     of the GPIO pins available at the pinout, for safety 
  */
  
 


  if (pin != GPIO4 && pin != GPIO17 && pin != GPIO18 && pin != GPIO21 && pin != GPIO22 && pin != GPIO23 && pin != GPIO24 && pin != GPIO25)
    return WRONG_ARGUMENTS;
  


  if (val == HIGH) {

    /*set the pin to high for pin n w can write to register 00..1..00
      since writing zero has no effect. No bitwise operations are needed. 
    */

    *(gpio._addr + 7) = (1<<pin);
    return 0;
  }
  else if (val == LOW) {

    //set the pin to low
    *(gpio._addr + 10) = (1<<pin);
    return 0;
  }
  else
    //if wrong val entered as parameter
    return WRONG_ARGUMENTS;
}




int8_t initializeListener() {
  /*allocate memory and set all eventlisteners id to zero */
  if ((interrupts = (EventListener*)malloc(sizeof(EventListener))) == NULL) {
      return ERROR_ALLOCATING_MEMORY;
    }

  uint8_t i;
  
  if (pthread_mutex_init(&(interrupts->_lock), NULL)) {
    free(interrupts);
    return ERROR_MUTEX;
  }

  for (i=0 ; i<_MAX_ISR ; i++) {
    interrupts->_ISRarray[i]._id = 0;
    interrupts->_listening = 0;
  }
  return 0;
}



int8_t attachEventListener(uint8_t pin, uint8_t event, void (*task)(void*), int8_t useMutex) {
  
  if (pin != GPIO4 && pin != GPIO17 && pin != GPIO18 && pin != GPIO21 && pin != GPIO22 && pin != GPIO23 && pin != GPIO24 && pin != GPIO25)
    return WRONG_ARGUMENTS;

  if (event != _HIGH && event != _LOW)
    return WRONG_ARGUMENTS;

  if (task == NULL)
    return WRONG_ARGUMENTS;
  
  uint8_t i;

  for (i = 0 ; i<_MAX_ISR ; i++) {
    
    if (interrupts->_ISRarray[i]._id == 0) {
      
      interrupts->_ISRarray[i]._id = i + 1;
      interrupts->_ISRarray[i].function = task;
      interrupts->_ISRarray[i]._event = event;
      interrupts->_ISRarray[i]._pin = pin;
      interrupts->_ISRarray[i]._listening = 0;
      

      /*if useMutex == 1*/

      if (pthread_mutex_init(&(interrupts->_ISRarray[i]._lockUser), NULL)) {
	interrupts->_ISRarray[i]._id = 0;
	return ERROR_MUTEX;
      }

      return i+1; //return id
    }
  }
  
  return EVENT_LISTENER_FULL;
}




int8_t beginListening() {
  /* create and call a thread that will listen for any interrupt
   */
  interrupts->_listening = 1;
  
  if (pthread_create(&(interrupts->_listener), NULL, (void*)thread_listening, NULL) != 0)
    return ERROR_THREAD;

  return 0;
}



void thread_listening(void* arg) {

  uint8_t i;
  
  for (;;) {

    /*check EventListener._listening, if 0 exit*/
    pthread_mutex_lock(&(interrupts->_lock));
    if (interrupts->_listening == 0)
      break;
    pthread_mutex_unlock(&(interrupts->_lock));



    /*check in EventListener._ISRarray._pin which pins we are monitoring*/
    for (i=0 ; i<_MAX_ISR ; i++) {

      /*if not monitoring this pin continue*/
      if (interrupts->_ISRarray[i]._id == 0)
	continue;

      

      if (*(gpio._addr + 13) & (1<<(interrupts->_ISRarray[i]._pin)) ) {
	if (interrupts->_ISRarray[i]._event != _HIGH)
	  continue;
	/*event detected*/

	
	/*check if an ISR for this interrupt is already active*/

	if (interrupts->_ISRarray[i]._listening)
	  /*if called with invalid pid, pthread_kill has undefined behavour*/
	  if (!pthread_kill(interrupts->_ISRarray[i]._thread, 0) )
	    /*thread active, continue*/
	  continue;

	/*no ISR ishandling the interrupt - create a thread and call ISR*/
	pthread_create(&(interrupts->_ISRarray[i]._thread), NULL, (void*)interrupts->_ISRarray[i].function, NULL);
	interrupts->_ISRarray[i]._listening = 1;
      }
      else {
	if (interrupts->_ISRarray[i]._event != _LOW)
	  continue;

	/*event detected*/

	/*check if an ISR for this interrupt is already active*/
	
	if (interrupts->_ISRarray[i]._listening)
	  /*if called with invalid pid, pthread_kill has undefined behavour*/
	  if (!pthread_kill(interrupts->_ISRarray[i]._thread, 0) )
	    /*thread active, continue*/
	    continue;

	/*no ISR ishandling the interrupt - create a thread and call ISR*/
	pthread_create(&(interrupts->_ISRarray[i]._thread), NULL, (void*)interrupts->_ISRarray[i].function, NULL);
	interrupts->_ISRarray[i]._listening = 1;
      }


    }// for (i=0 ; i<_MAX_ISR ; i++)

    usleep(_INTERRUPT_LATENCY_USEC);
  }//for (;;)

  pthread_mutex_unlock(&(interrupts->_lock));
  pthread_exit(NULL);
}




void stopListener() {
  /*set the EventListener to 0 and wait the thread to exit*/

  pthread_mutex_lock(&(interrupts->_lock));
  interrupts->_listening = 0;
  pthread_mutex_unlock(&(interrupts->_lock));

  pthread_join(interrupts->_listener, NULL);
}


void destroyListener() {
  stopListener();
  pthread_mutex_destroy(&(interrupts->_lock));
}




