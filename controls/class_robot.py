class robot:
    def __init__(self):
        # Connect to motors
        
        # Link lengths (might not use this lmao)
        self.L = [0.04, 0.04, 0.065, 0.065]     # TODO: update these values
        # initial/home position
        self.ini_pos = [0, 0, 0.00]     # TODO: adjust the height
        self.pz_max = 0.0            # TODO: determine max height
        self.pz_min = 0.0            # TODO: determine min height
        self.pitch_max = 20         # rotation in the x-axis
        self.roll_max = 20          # rotation in the y-axis
    
    def homing_logic(self, marker_center):
        """Example homing logic to adjust platform based on marker position."""
        if marker_center:
            center_x, center_y = marker_center
            platform_center_x = self.width // 2
            platform_center_y = self.height // 2

            # Compute errors
            error_x = platform_center_x - center_x
            error_y = platform_center_y - center_y

            print(f"Error X: {error_x}, Error Y: {error_y}")
            # Use this error to send control signals to the platform motors
            
    def compute_kinematics(self):
        pass 
    
    def control_posture(self, targetPosture):
        pitch = targetPosture[0]
        roll = targetPosture[1]
        pass