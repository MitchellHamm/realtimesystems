I, Mitchell Hamm, implimented the entirety of this assignment; creating the
scheduler task and the various scheduling policies. The fixed priority scheduler
uses 3 priority queues checking the queues for ready tasks in order of their 
priority. Each brew task maintains a state variable to indicate if the task is
ready or blocked. If the task is blocked it's simply pushed to the back of the 
queue. Otherwise, it's paramaters are pushed to the brew task which toggles the
led a decriments the time remaining on the task and places it back on the correct
queue. If the task has finished running it is set to the blocked state. Both EDF
and LLF use an array of brew tasks, looping through to find which task to run.
EDF finds the task in the ready state with the earliest deadline and runs it in
the brew task and LLF calculates the laxity of a task and selects the task with
the lowest laxity that's in the ready state. The scheduler and brew tasks are 
synced via a semaphore to ensure only one brew task runs to completion per "cycle",
blocking until they're finished so that the scheduler doesn't start trying to brew
another task while the brew task delay finishes.

/********/
INVESTIGATION
/********/

FPS.) The task with the highest priority that is ready is always run first. This
leads to the highest priority task never missing deadlines because it will always
get CPU time first. Since the orange and red task have the same priority, they are
scheduled round robin which will cause the orange task to miss the deadline but the
red task will meet the deadlilne since both tasks will finish by 13 seconds. The task
with the lowest priority will usually miss its deadline unless the ticks are a multiple
of 30 and not 20. In this case the task will be ready while no other tasks are and will
get to run right away. If the task is ready at the same time as the other 3 tasks
(tick % 30 == 0 and ticks % 40 == 0) it will miss it's deadline since the other tasks
will be scheduled to run first. Using the delayed start, if a low priority task is the 
only task currently running, it will get use of the CPU, however when new tasks are added
with higher priorities, those tasks will be run before the low priority task.

EDF.) The task with the earliest deadline that is ready is always run first. This
leads to the tighter deadline tasks never missing their deadlines. Here, the green
task has an earlier deadline and will we scheduled sooner than it was in FPS. The
blue task will always meet it's deadline, the green task will meet its deadline if the
orange task isn't ready at the same time, as the two will be scheduled round robin and
both will miss their deadlines by 1 and 2 seconds as a result. The red task will always
miss it's deadline unless the green task is not ready to run, in that case, all tasks will
meet their deadlines. Using the dealyed start, the first scheduled task will alawys run,
once the tasks similar behaviour will occur as above if multiple tasks are ready to run at
the same time. 

LLF.) The task with the least laxity (deadline - time left) will be run first. This leads 
to a lot more context switching as the laxity increases as a task is run and new lower
laxities are likely to be found. The blue task initially gets to run to completion and
make it's deadline, then the orange and green task will be alternated between similar to 
how they were in EDF, finally the red task will be scheduled once the green and orange
task reach a laxity of 9. This will cause the orange, red and green task to all miss their
deadlines. As in EDF and FPS, if all tasks are ready at the same time this will occur. If
a task is not ready at the same time as the others it will make it's deadline. If the blue, 
orange and red tasks are all scheduled, the blue will finish first, the orange next at 10s
followed by the red which meets it's deadline. Using the delayed start, the first ready task
will be scheduled first and once the other tasks are added to the scheduler array the behaviour
will be similar to as above, with multiple tasks being ready to run at the same time.