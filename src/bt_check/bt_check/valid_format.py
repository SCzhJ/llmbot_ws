class valid_format:
    # Define valid action nodes and their required attributes
    valid_actions = {
        "MoveTo": ["x", "y"],
        "Delay": ["delay_msec"],
    }
    
    # Define valid control nodes (BehaviorTree.CPP control nodes) and expected attributes
    valid_control_nodes = {
        "ROOOOOT": [],
        "Sequence": [],
        "Repeat": ["num_cycles"],
        # Add other control nodes as necessary
    }