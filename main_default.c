/*
 * Default main.c for rtos lab.
 * @Instructor Andrew Morton, 2018
 */
#include <LPC17xx.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

uint32_t msTicks = 0;

uint32_t prevTaskStackPointer;
uint32_t nextTaskStackPointer;

uint32_t idleTaskStackPointer;



uint32_t* vector0 = 0x0; 

typedef void (*rtosTaskFunc_t)(void *args);

typedef struct{
	uint32_t RESERVED[240];
	uint32_t R4;
	uint32_t R5;
	uint32_t R6;
	uint32_t R7;
	uint32_t R8;
	uint32_t R9;
	uint32_t R10;
	uint32_t R11;
	uint32_t R0;
	uint32_t R1;
	uint32_t R2;
	uint32_t R3;
	uint32_t R12;
	uint32_t LR;
	uint32_t PC;
	uint32_t PSR;
}TaskCB;


bool TaskFree[6] = {true,true,true,true,true,true};
TaskCB* TASKCB[6];
uint32_t TaskStackPointers[6];


typedef struct osThread_struct osThread_struct;

struct osThread_struct{
	uint32_t task_stack_pointer;
	char state;
	char priority;
	osThread_struct* next;
};

osThread_struct osThreads[6];

osThread_struct* prevThread;
osThread_struct* currThread;
osThread_struct* nextThread;
osThread_struct* idleThread;

#define PRIORITY_HIGH 7
#define PRIORITY_MED 4
#define PRIORITY_LOW 2
#define PRIORITY_IDLE 0

#define TASK_STATE_RUNNING 1
#define TASK_STATE_READY 2
#define TASK_STATE_BLOCKING 3

typedef struct{
	osThread_struct* head;
	osThread_struct* tail;
}Priority_Queue;

Priority_Queue priority_high_list;
Priority_Queue priority_med_list;
Priority_Queue priority_low_list;

void enqueue(Priority_Queue* q, osThread_struct* t){
	t->next = NULL;
	if(q->head != NULL){
		q->tail->next = t;
		q->tail = t;
	}
	else{
		q->head = t;
		q->tail = t;
	}
}

osThread_struct* dequeue(Priority_Queue* q){
	osThread_struct* dq;
	dq = q->head;
	if(q->head->next == NULL){
		q->head = NULL;
		q->tail = NULL;
	}
	else{
		q->head = q->head->next;
	}
	dq->next = NULL;
	return dq;
}

typedef struct{
	volatile uint32_t lock;
}mutex;

mutex mutexMem[2];

void osMutexNew(unsigned char id){
	mutexMem[id].lock = 1;
}

void acquire_mutex(unsigned char id){
	currThread->state = TASK_STATE_BLOCKING;
	while(mutexMem[id].lock == 0);
	currThread->state = TASK_STATE_READY;
	mutexMem[id].lock = 0;
}

void release_mutex(unsigned char id){
	mutexMem[id].lock = 1;
	
}

typedef struct{
	uint8_t state;
	uint8_t max;
}semaphore;

semaphore semaphoreMem[2];

void semaphore_init(unsigned char id, uint8_t max, uint8_t state){
	if(state < 0){
		semaphoreMem[id].state = 0;
	}
	else{
		semaphoreMem[id].state = state;
	}
	semaphoreMem[id].max = max;
}

void signal(unsigned char id){
	semaphoreMem[id].state++;
	if(semaphoreMem[id].state > semaphoreMem[id].max){
		semaphoreMem[id].state = semaphoreMem[id].max;
	}
}

void wait(unsigned char id){
	currThread->state = TASK_STATE_BLOCKING;
	while(semaphoreMem[id].state < 1){}
	currThread->state = TASK_STATE_READY;
	semaphoreMem[id].state--;
}


__asm void PendSV_Handler(void) {
  //Store current SP to global var
	
	CPSID I /* Disable interrupts */
	
	MRS R0,PSP
	STMFD R0!,{R4-R11}
	
	//PUSH {r4-r11}
	//update curr
	LDR R1, =__cpp(&prevTaskStackPointer)
	STR R0, [R1]
	
	//Change SP to next Task SP
	LDR R1, =__cpp(&nextTaskStackPointer)
	LDR R0, [R1]
	LDMFD R0!, {r4-r11}
	MSR PSP, R0
	
	CPSIE I /* Enable interrupts */
	
	//return from handler
	BX		LR
}





int previous_task_index = 5;

bool high_blocking = false;
bool med_blocking = false;
bool low_blocking = false;

void SysTick_Handler(void) {
	__disable_irq();
	bool found_high = false;
	bool found_med = false;
	bool found_low = false;
	
	prevThread->task_stack_pointer = prevTaskStackPointer;
	
	// Check highest priority
	osThread_struct* temp = priority_high_list.head;
	
	if(temp != NULL){
		if(temp->state != TASK_STATE_READY){
			temp = dequeue(&priority_high_list);
			enqueue(&priority_high_list, temp);
			if(temp->state == TASK_STATE_BLOCKING){
				high_blocking = true;
			}
			found_high = true;
			nextThread = dequeue(&priority_high_list);
		}
		else if(!high_blocking){
			found_high = true;
			nextThread = dequeue(&priority_high_list);
		}
		else{
			high_blocking = false;
		}
	}
	
	if(!found_high){
		//check medium priority
		temp = priority_med_list.head;
		if(temp != NULL){
			if(temp->state != TASK_STATE_READY){
				temp = dequeue(&priority_med_list);
				enqueue(&priority_med_list, temp);
				if(temp->state == TASK_STATE_BLOCKING){
					med_blocking = true;
				}
				found_med = true;
				nextThread = dequeue(&priority_med_list);
			}
			else if(!med_blocking){
				found_med = true;
				nextThread = dequeue(&priority_med_list);
			}
			else{
				med_blocking = false;
			}
		}
		if(!found_med){
			//check low priority
			temp = priority_low_list.head;
			if(temp != NULL){
				if(temp->state != TASK_STATE_READY){
					temp = dequeue(&priority_low_list);
					enqueue(&priority_low_list, temp);
					if(temp->state == TASK_STATE_BLOCKING){
						low_blocking = true;
					}
					found_low = true;
					nextThread = dequeue(&priority_low_list);
				}
				else if(!low_blocking){
					found_low = true;
					nextThread = dequeue(&priority_low_list);
				}
				else{
					low_blocking = false;
				}
			}
			if(!found_low){
				nextThread = idleThread;
			}
		}
	}
	
	prevThread = currThread;
	
	
	if(currThread != nextThread){
		
		switch(currThread->priority){
			case PRIORITY_HIGH:
					enqueue(&priority_high_list, currThread);
				break;
			case PRIORITY_MED:
					enqueue(&priority_med_list, currThread);
				break;
			case PRIORITY_LOW:
					enqueue(&priority_low_list, currThread);
				break;
			case PRIORITY_IDLE:
				break;
		}
		
		currThread = nextThread;
		
		
		nextTaskStackPointer = nextThread->task_stack_pointer;
		SCB->ICSR |= (0x1 << 28);
	}
	__enable_irq();
}


void idleTask(void*args){
	while(true){
	}
}

void osKernelStart(void){
	__set_MSP(*vector0);
	uint32_t control = __get_CONTROL();
	__set_CONTROL(control |= (0x1 << 1));
	__set_PSP(idleTaskStackPointer);
	prevTaskStackPointer = idleTaskStackPointer;
	prevThread = idleThread;
	currThread = idleThread;
	nextThread = idleThread;
	__enable_irq();
	idleTask(NULL);
}

void osNewTask(rtosTaskFunc_t func, void *args, char priority){
	int freeIndex = -1;
	for(int i=0; i<6;i++){
		if(TaskFree[i] == true){
			freeIndex = i;
		}
	}
	if(freeIndex < 0)
		return;
	TaskFree[freeIndex] = false;
	
	osThreads[freeIndex].priority = priority;
	osThreads[freeIndex].state = TASK_STATE_READY;
	
	 switch(priority){
		case PRIORITY_HIGH:
			if(priority_high_list.head == 0){
				priority_high_list.head = &osThreads[freeIndex];
				priority_high_list.tail = &osThreads[freeIndex];
			}
			else{
				enqueue(&priority_high_list, &osThreads[freeIndex]);
			}
			break;
		case PRIORITY_MED:
			if(priority_med_list.head == 0){
				priority_med_list.head = &osThreads[freeIndex];
				priority_med_list.tail = &osThreads[freeIndex];
			}
			else{
				enqueue(&priority_med_list, &osThreads[freeIndex]);
			}
			break;
		case PRIORITY_LOW:
			if(priority_low_list.head == 0){
				priority_low_list.head = &osThreads[freeIndex];
				priority_low_list.tail = &osThreads[freeIndex];
			}
			else{
				enqueue(&priority_low_list, &osThreads[freeIndex]);
			}
			break;
		case PRIORITY_IDLE:
			break;
	}
	
	TASKCB[freeIndex]->R0 = (uint32_t)args;
	TASKCB[freeIndex]->PC = (uint32_t)func;
	TASKCB[freeIndex]->PSR = 0x01000000;
	
	TASKCB[freeIndex]->R4 = 0xDEADBEEF;
	TASKCB[freeIndex]->R5 = 0xDEADBEEF;
	TASKCB[freeIndex]->R6 = 0xDEADBEEF;
	TASKCB[freeIndex]->R7 = 0xDEADBEEF;
	TASKCB[freeIndex]->R8 = 0xDEADBEEF;
	TASKCB[freeIndex]->R9 = 0xDEADBEEF;
	TASKCB[freeIndex]->R10 = 0xDEADBEEF;
	TASKCB[freeIndex]->R11 = 0xDEADBEEF;
	
	TASKCB[freeIndex]->R1 = 0xDEADBEEF;
	TASKCB[freeIndex]->R2 = 0xDEADBEEF;
	TASKCB[freeIndex]->R3 = 0xDEADBEEF;
	
	TASKCB[freeIndex]->R12 = 0x00000001;
	
	TASKCB[freeIndex]->LR = 0x00000001;
}

void osKernelInit(void){
	NVIC_SetPriority(15, 0x00);
	NVIC_SetPriority(14, 0xff);
	uint32_t TCBStartAddress = *vector0 - 3*sizeof(TaskCB);
	for(int i=0; i<6; i++){
		TASKCB[i] = (TaskCB*)(TCBStartAddress - sizeof(TaskCB)*(i));
		TaskStackPointers[i] = (uint32_t)TASKCB[i] + 960;
		
		osThreads[i].task_stack_pointer = TaskStackPointers[i];
	}
	idleTaskStackPointer = (uint32_t)osThreads[5].task_stack_pointer;
	
	osNewTask(idleTask, NULL, PRIORITY_IDLE);
	
	idleThread = &osThreads[5];
}


unsigned char testsem = 0;
unsigned char mutexTest = 0;





//mutex demonstration

void TaskMutex1(void*args){
	while(true){
		acquire_mutex(mutexTest);
		printf("t1: AQ\n");
		release_mutex(mutexTest);
		printf("t1: RE\n");
		for (int i = 0; i < 1000000; i++){}
	}
}
void TaskMutex2(void*args){
	while(true){
		acquire_mutex(mutexTest);
		printf("t2: AQ\n");
		release_mutex(mutexTest);
		printf("t2: RE\n");
		for (int i = 0; i < 1000000; i++){}
	}
}

//int main(void) {
//	printf("Setup\n");
//	
//	osMutexNew(mutexTest);
//	
//	SysTick_Config(SystemCoreClock/1000);
//	
//	uint32_t period = 1000; // 1s
//	uint32_t prev = -period;
//	
//	
//	osKernelInit();
//	osNewTask(TaskMutex1, NULL, PRIORITY_HIGH);
//	osNewTask(TaskMutex2, NULL, PRIORITY_HIGH);
//	osKernelStart();

//}


//semaphore demonstration

void TaskSemaphore1(void*args){
	LPC_GPIO1->FIODIR |= 0xB0000000;
	LPC_GPIO2->FIODIR |= 0x0000007C;
	long read = LPC_GPIO2->FIOPIN;
	long read2 = LPC_GPIO1->FIOPIN;
	while(1)
	{
		while((read & 0x00000400) != 0)
		{
			read = LPC_GPIO2->FIOPIN;
		}
		
		read2 = LPC_GPIO1->FIOPIN;

		if((read2 & 0x10000000) > 0)
		{
			LPC_GPIO1->FIOCLR |= 0x10000000;
		}
		else
		{
			signal(testsem);
			LPC_GPIO1->FIOSET |= 0x10000000;
		}
		
		
		while((read & 0x00000400) == 0)
		{
			read = LPC_GPIO2->FIOPIN;
		}
		
		
	}
}
void TaskSemphore2(void*args){
	while(true){
		wait(testsem);
		printf("t2: after wait\n");
		for (int i = 0; i < 1000000; i++){}
	}
}

//int main(void) {
//	printf("Setup\n");
//	
//	semaphore_init(testsem,1,1);
//	
//	SysTick_Config(SystemCoreClock/1000);
//	
//	uint32_t period = 1000; // 1s
//	uint32_t prev = -period;
//	
//	
//	osKernelInit();
//	osNewTask(TaskSemaphore1, NULL, PRIORITY_LOW);
//	osNewTask(TaskSemphore2, NULL, PRIORITY_HIGH);
//	osKernelStart();

//}

//FPP demonstration

void TaskFPP1(void*args){
	while(true){
		printf("t1: low\n");
		for (int i = 0; i < 1000000; i++){}
	}
}
void TaskFPP2(void*args){
	while(true){
		printf("t2: med\n");
		for (int i = 0; i < 1000000; i++){}
	}
}
void TaskFPP3(void*args){
	while(true){
		printf("t3: med\n");
		for (int i = 0; i < 1000000; i++){}
	}
}
void TaskFPP4(void*args){
	while(1){
		wait(testsem);
		printf("t4: high\n");
		
		signal(testsem);
		for (int i = 0; i < 1000000; i++){}
	}
}
void TaskFPP5(void*args){
	LPC_GPIO1->FIODIR |= 0xB0000000;
	LPC_GPIO2->FIODIR |= 0x0000007C;
	long read = LPC_GPIO2->FIOPIN;
	long read2 = LPC_GPIO1->FIOPIN;
	while(1)
	{
		while((read & 0x00000400) != 0)
		{
			read = LPC_GPIO2->FIOPIN;
		}
		
		read2 = LPC_GPIO1->FIOPIN;

		if((read2 & 0x10000000) > 0)
		{
			LPC_GPIO1->FIOCLR |= 0x10000000;
		}
		else
		{
			wait(testsem);
			LPC_GPIO1->FIOSET |= 0x10000000;
			for (int i = 0; i < 1000000; i++){}
			signal(testsem);
		}
		
		
		while((read & 0x00000400) == 0)
		{
			read = LPC_GPIO2->FIOPIN;
		}
		
	}
}


int main(void) {
	printf("Setup\n");
	
	SysTick_Config(SystemCoreClock/1000);
	
	semaphore_init(testsem,1,1);
	
	osKernelInit();
	osNewTask(TaskFPP1, NULL, PRIORITY_LOW);
	osNewTask(TaskFPP2, NULL, PRIORITY_MED);
	osNewTask(TaskFPP3, NULL, PRIORITY_MED);
	osNewTask(TaskFPP4, NULL, PRIORITY_HIGH);
	osNewTask(TaskFPP5, NULL, PRIORITY_HIGH);
	osKernelStart();

}
