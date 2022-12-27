# STM32_FreeRTOS_project

### It provides support for resource management, communication, precise scheduling, synchronization and planning. RTOS has a deadline associated with tasks. and must complete their tasks before this date passes.

   FreeRTOS is an operating system designed to run on a microcontroller. Embedded systems also have different operating systems such as FreeRTOS. A few of them are VxWorks, QNX, Integrity. FreeRTOS is one of the unlicensed ones. There are also licensed software (OpenRTOS, SafeRTOS) that share the same code base as FreeRTOS.
 
  Ideal for real-time embedded system applications using FreeRTOS microcontrollers or small microprocessors. Such applications normally have hard real-time and soft real-time requirements. In soft real-time requirement, not performing a job on time does not cause an error in the system.
     
   FreeRTOS is a real-time kernel created to meet the hard real-time requirements of embedded applications. FreeRTOS allows multiple jobs (tasks or threads) to run in an organized manner. This is also called multitasking. There are threads called task (or thread) in RTOS. The processor runs these threads at certain times according to a certain priority, that is, it does more than one job at the same time (it looks like it is doing it).

   A single thread is run at any one time on a single core processor. In RTOS, these threads run on the processor as if they were running simultaneously, depending on priority and time. This is where the concept of real time comes from because it is time dependent. The programmer should choose higher priority for threads with hard real-time requirements. Here we can say: In real-time systems, threads with hard real-time requirements are run more priority than threads with soft real-time requirements.

### FreeRTOS Features
- Pre-emptive or co-operative work.
- Very flexible task priority assignment.
- Flexible and fast task notification mechanism.
- Queues
- Binary and Counting Semaphore
- Mutex and Recursive Mutex
- Software Timers
- Event groups
- Tick hook and idle hook function
- Stack overflow control
- Optional commercial license versions
