uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_SUCCEEDED=2
uint8 STATE_FAILED=3

std_msgs/Header header
string workflow_name
string current_step
uint32 step_index
uint32 step_count
uint8 state
string message
