string TASKSTART = 'Task_start'
string ENQUEUED = 'Enqueue_start'
string ENTERWORKCELL = 'Enqueue_done'
string TASKDONE = 'Task_done'
string ERROR = 'Error'

string task_state

Id robot_id
uint8 task_id
builtin_interfaces/Time stamp
int32 total_time
