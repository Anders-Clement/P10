uint8 TASKSTART = 0
uint8 ENQUEUED = 1
uint8 ENTERWORKCELL = 2
uint8 TASKDONE = 3
uint8 ERROR = 4

uint8 task_state

Id robot_id
Task task
builtin_interfaces/Time stamp
int32 total_time
