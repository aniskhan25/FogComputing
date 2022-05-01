MAX_QUEUE_LEN = 100000

VEH_RANGE = 100
RSU_RANGE = 200

TASK_LOWER = 0.2
TASK_UPPER = 1.0

# ARM Cortex R4 (600 MHz; 2011) and R7 (1.0 GHz; 2011); supports dual-core configuration
# https://www.silabs.com/documents/public/white-papers/Which-ARM-Cortex-Core-Is-Right-for-Your-Application.pdf
FREQ_LOWER = 0.6
FREQ_UPPER = 1.0

VEH_CORES_LOWER = 1
VEH_CORES_UPPER = 2

RSU_CORES_LOWER = 4
RSU_CORES_UPPER = 8

LAMBDA = 1.5

SIM_STEP_SIZE = 0.1

# 0.8 MHz x 1.5 cores x 260 units = 310 total V2V capacity
# 0.8 MHz x 6   cores x   6 units =  30 total V2I capacity

# (310 + 30) x 10 slots x 100 iters = 340,000 total system capacity
# (310 + 30) = 340 mean service rate

# 10 slots x 100 iters x 260 units x 0.6 bits = 156000 arrival rate
# 260 units x 0.6 bits = 156 mean arrival rate

# 156 / 340 = 0.45 system utilization


