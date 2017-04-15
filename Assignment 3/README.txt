I, Mitchell Hamm, implimented the entirety of this assignment; creating the 
button interupt handler and various tasks to handle brewing and led toggling.
The button interupt handler is locked via a semaphore, once the semaphore is 
available it performs a crude loop to avoid button debouncing and unlocks the 
semaphore for future interupts afterwards. All tasks are blocked from executing
by various semaphores and are unlocked to perform their normal execution.
The button interupt then unlocks the double click semaphore which allows the 
task to run. It delays for a short time and then checks the number of button 
presses that ocurred. For one button press, it unlocks the led cycle semaphore 
that lets the led cycle task execute once and change the current led. For two 
button presses,it unlocks the brew semaphore for the currently selected led. 
The brew tasks use a busy wait signal so that only one coffee is being "brewed" 
by the CPU at a time. When multiple coffees are being brewed they each receive 
a time slice of the CPU to brew; adding more brewing coffees increases the total 
brew time as a result.

/********/
PART 2
/********/

1.) The default pre-emptive scheduler brews the coffees as expected, giving each
a chance to use the CPU in a round robin fashion. Coffees can be brewed at the 
same time and will complete after the total brew time has elapsed. With 
co-operative scheduling however, only one coffee can be brewed at a time. This
is to be expected since the brew tasks are not yielding the CPU. As a result,
only one brew task can be running at a time since it will not relinquish the 
CPU during its wait period, nor will the CPU switch tasks given that it's in
co-operative scheduling mode.

2.) There is no difference in the behaviour of the application when run with
time slicing on or off. This is to be expected as the default scheduler already
switches tasks periodically.

3.) Using a slightly lower processor clock speed only impacts the actual brew
times. Since the lower processor clock means less instructions can pass through,
the brew times become slightly slower.