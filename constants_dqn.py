# Main constants

LOCAL	= 0
OFFLOAD = 1

# Actions dictionary

actions_dict = {
	LOCAL: 'local',
	OFFLOAD: 'offload',	
}

num_actions = len(actions_dict)

# Exploration factor
epsilon = 0.1

data_size = 50