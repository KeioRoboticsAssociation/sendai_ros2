import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from image_detector.msg import LineSegmentArray, BallPositionArray # Assuming BallPositionArray will be used later
import math
import time # For rclpy.duration if needed explicitly, but rclpy.duration.Duration should work
import rclpy.duration

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Define states
        self.STATE_SEARCHING_LINE = "SEARCHING_LINE"
        self.STATE_FOLLOWING_LINE = "FOLLOWING_LINE"
        self.STATE_APPROACHING_BALL = "APPROACHING_BALL"
        self.STATE_COLLECTING_BALL = "COLLECTING_BALL"
        self.STATE_REALIGNING_TO_LINE = "REALIGNING_TO_LINE"
        self.STATE_STOPPED = "STOPPED"

        # New Mission States
        self.STATE_INITIAL_COLLECTION_PHASE = "INITIAL_COLLECTION_PHASE"
        self.STATE_NAVIGATING_TO_DISCHARGE = "NAVIGATING_TO_DISCHARGE"
        self.STATE_DISCHARGING_BALLS = "DISCHARGING_BALLS"
        self.STATE_SECOND_COLLECTION_PHASE = "SECOND_COLLECTION_PHASE"
        self.STATE_MISSION_COMPLETE = "MISSION_COMPLETE"

        self.current_state = self.STATE_INITIAL_COLLECTION_PHASE # Initial state for the mission
        # self.get_logger().info(f"Initial state: {self.current_state}")

        # Parameters for line following
        self.declare_parameter('line_following.kp_angle', 0.5)
        self.declare_parameter('line_following.kp_lateral', 0.01)
        self.declare_parameter('line_following.target_x_position', 320.0) # Assuming image center for 640px width
        self.declare_parameter('line_following.nominal_forward_speed', 0.05) # m/s
        self.declare_parameter('line_following.verticality_factor', 2.0) # dy must be 'factor' times larger than dx
        self.declare_parameter('line_following.lost_timeout_s', 3.0)
        self.declare_parameter('line_following.search_rotation_speed_rad_s', 0.25) # rad/s

        # Initialize variables from line following parameters
        self.kp_angle = self.get_parameter('line_following.kp_angle').get_parameter_value().double_value
        self.kp_lateral = self.get_parameter('line_following.kp_lateral').get_parameter_value().double_value
        self.target_line_x = self.get_parameter('line_following.target_x_position').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('line_following.nominal_forward_speed').get_parameter_value().double_value
        self.verticality_factor = self.get_parameter('line_following.verticality_factor').get_parameter_value().double_value
        self.search_rotation_speed = self.get_parameter('line_following.search_rotation_speed_rad_s').get_parameter_value().double_value
        self.line_lost_timeout_duration = rclpy.duration.Duration(seconds=self.get_parameter('line_following.lost_timeout_s').get_parameter_value().double_value)
        self.last_line_seen_time = self.get_clock().now()

        # Parameters for ball approach
        self.declare_parameter('ball_approach.target_ball_x', 320.0)  # Center of 640px image
        self.declare_parameter('ball_approach.target_ball_y', 450.0)  # Bottom area of 480px image (adjust as needed)
        self.declare_parameter('ball_approach.kp_ball_x', 0.005)
        self.declare_parameter('ball_approach.kp_ball_y', 0.005)
        self.declare_parameter('ball_approach.approach_speed_vx_max', 0.08) # Max speed when approaching ball
        self.declare_parameter('ball_approach.approach_speed_vy_max', 0.08) # Max speed when approaching ball
        self.declare_parameter('ball_approach.collection_threshold_x', 15.0) # Pixel tolerance for collection
        self.declare_parameter('ball_approach.collection_threshold_y', 15.0) # Pixel tolerance for collection
        self.declare_parameter('ball_approach.ball_lost_timeout_s', 2.0)

        # Initialize variables from ball approach parameters
        self.target_ball_x = self.get_parameter('ball_approach.target_ball_x').get_parameter_value().double_value
        self.target_ball_y = self.get_parameter('ball_approach.target_ball_y').get_parameter_value().double_value
        self.kp_ball_x = self.get_parameter('ball_approach.kp_ball_x').get_parameter_value().double_value
        self.kp_ball_y = self.get_parameter('ball_approach.kp_ball_y').get_parameter_value().double_value
        self.approach_speed_vx_max = self.get_parameter('ball_approach.approach_speed_vx_max').get_parameter_value().double_value
        self.approach_speed_vy_max = self.get_parameter('ball_approach.approach_speed_vy_max').get_parameter_value().double_value
        self.collection_threshold_x = self.get_parameter('ball_approach.collection_threshold_x').get_parameter_value().double_value
        self.collection_threshold_y = self.get_parameter('ball_approach.collection_threshold_y').get_parameter_value().double_value
        self.ball_lost_timeout_duration = rclpy.duration.Duration(seconds=self.get_parameter('ball_approach.ball_lost_timeout_s').get_parameter_value().double_value)
        self.last_ball_seen_time = self.get_clock().now() # Initialize

        # Parameters for ball collection
        self.declare_parameter('ball_collection.collection_wait_timeout_s', 3.0) # Max time to wait for ball to disappear
        self.declare_parameter('ball_collection.post_collection_delay_s', 0.5) # Brief pause after collection confirmation

        # Initialize variables from ball collection parameters
        self.collection_wait_timeout_duration = rclpy.duration.Duration(seconds=self.get_parameter('ball_collection.collection_wait_timeout_s').get_parameter_value().double_value)
        self.post_collection_delay_duration = rclpy.duration.Duration(seconds=self.get_parameter('ball_collection.post_collection_delay_s').get_parameter_value().double_value)
        self.collection_state_start_time = None # To track timeout in COLLECTING_BALL state

        # Parameters for re-alignment
        self.declare_parameter('realign.timeout_s', 10.0) # Max time to search for line during realignment

        # Initialize variables from re-alignment parameters
        self.realign_timeout_duration = rclpy.duration.Duration(seconds=self.get_parameter('realign.timeout_s').get_parameter_value().double_value)
        self.realign_state_start_time = None # To track timeout in REALIGNING_TO_LINE state

        # Parameters for kinematics
        self.declare_parameter('kinematics.robot_radius_m', 0.15)  # L, distance from center to wheel
        self.declare_parameter('kinematics.max_wheel_speed_mps', 0.5) # Max tangential speed of a wheel (m/s) at 1.0 duty cycle

        # Initialize variables from kinematics parameters
        self.robot_radius = self.get_parameter('kinematics.robot_radius_m').get_parameter_value().double_value
        # Radius from robot center to each wheel for kinematics calculations
        self.wheel_base_radius = self.get_parameter('kinematics.robot_radius_m').get_parameter_value().double_value
        self.max_wheel_speed = self.get_parameter('kinematics.max_wheel_speed_mps').get_parameter_value().double_value
        if self.max_wheel_speed <= 0:
            self.get_logger().error(f"kinematics.max_wheel_speed_mps must be positive. Value: {self.max_wheel_speed}. Defaulting to 0.5 m/s.")
            self.max_wheel_speed = 0.5

        # Mission Parameters
        self.declare_parameter('mission.balls_for_first_phase', 2) # Default: 2 for testing
        self.declare_parameter('mission.total_balls_to_collect', 4) # Default: 4 for testing
        self.declare_parameter('mission.discharge_wait_duration_s', 5.0)

        # Initialize Mission Variables
        self.balls_for_first_phase = self.get_parameter('mission.balls_for_first_phase').get_parameter_value().integer_value
        self.total_balls_to_collect = self.get_parameter('mission.total_balls_to_collect').get_parameter_value().integer_value
        self.discharge_wait_duration = rclpy.duration.Duration(seconds=self.get_parameter('mission.discharge_wait_duration_s').get_parameter_value().double_value)
        self.discharge_phase_start_time = None
        self.first_discharge_complete = False
        self.collected_balls_this_run = [] # Tracks balls collected in the current phase
        # self.total_collected_ever = 0 # If needed for more complex logic, but current plan relies on collected_balls_this_run and first_discharge_complete
        self.current_main_line_image_x = None # Added for side-specific ball collection

        # General state variables
        self.current_target_vx = 0.0
        self.current_target_vy = 0.0
        self.current_target_v_omega = 0.0

        self.latest_lines_msg = None
        self.latest_balls_msg = None
        self.target_ball_info = None # Stores info of the ball being approached { 'color': str, 'position': Point }
        # self.collected_balls = [] # Replaced by collected_balls_this_run for phase logic

        # Subscribers
        self.line_subscriber = self.create_subscription(
            LineSegmentArray,
            '/detection/lines',
            self.lines_callback,
            10)

        self.ball_subscriber = self.create_subscription(
            BallPositionArray,
            '/detection/balls',
            self.balls_callback,
            10)

        # Publisher
        self.motor_efforts_publisher_ = self.create_publisher( # From previous step
            Float32MultiArray,
            '/motor_control_efforts',
            10)

        # Timer for the main control loop
        self.control_loop_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Robot Control Node Initialized with Line Following Logic')

    def lines_callback(self, msg):
        # self.get_logger().debug(f'Received lines: {len(msg.lines)} segments')
        self.latest_lines_msg = msg

    def balls_callback(self, msg):
        # self.get_logger().debug(f'Received balls: {len(msg.positions)} balls')
        self.latest_balls_msg = msg
        # Optionally, if balls are continuously published, update last_ball_seen_time here
        # if self.target_ball_info and self.is_target_ball_visible(self.target_ball_info, msg):
        #    self.last_ball_seen_time = self.get_clock().now()

    def control_loop(self):
        # self.get_logger().debug(f"Control Loop - State: {self.current_state}")
        if self.current_state == self.STATE_SEARCHING_LINE:
            self.handle_searching_line()
        elif self.current_state == self.STATE_FOLLOWING_LINE:
            self.handle_following_line()
        elif self.current_state == self.STATE_APPROACHING_BALL:
            self.handle_approaching_ball()
        elif self.current_state == self.STATE_COLLECTING_BALL:
            self.handle_collecting_ball()
        elif self.current_state == self.STATE_REALIGNING_TO_LINE:
            self.handle_realigning_to_line()
        elif self.current_state == self.STATE_STOPPED:
            self.current_target_vx = 0.0
            self.current_target_vy = 0.0
            self.current_target_v_omega = 0.0
        # New mission state handlers
        elif self.current_state == self.STATE_INITIAL_COLLECTION_PHASE:
            self.handle_initial_collection_phase()
        elif self.current_state == self.STATE_NAVIGATING_TO_DISCHARGE:
            self.handle_navigating_to_discharge()
        elif self.current_state == self.STATE_DISCHARGING_BALLS:
            self.handle_discharging_balls()
        elif self.current_state == self.STATE_SECOND_COLLECTION_PHASE:
            self.handle_second_collection_phase()
        elif self.current_state == self.STATE_MISSION_COMPLETE:
            self.handle_mission_complete()
        else:
            self.get_logger().error(f"Unknown state: {self.current_state}. Stopping robot.")
            self.current_state = self.STATE_STOPPED


        # self.get_logger().debug(f"Calculated Target Vels: Vx={self.current_target_vx:.2f}, Vy={self.current_target_vy:.2f}, Vomega={self.current_target_v_omega:.2f}")

        # Convert chassis target velocities to motor efforts using inverse kinematics
        motor_efforts = self.calculate_wheel_efforts(
            self.current_target_vx,
            self.current_target_vy,
            self.current_target_v_omega
        )

        # Publish the calculated motor efforts
        self.publish_efforts(motor_efforts)

    def handle_searching_line(self):
        # self.get_logger().debug("Handling SEARCHING_LINE state.")
        if not self.latest_lines_msg:
            # self.get_logger().debug("Searching: No line data yet. Commanding rotation.")
            self.current_target_vx = 0.0
            self.current_target_vy = 0.0
            self.current_target_v_omega = self.search_rotation_speed
            return

        vertical_line, _ = self.find_target_line(self.latest_lines_msg)

        if vertical_line:
            self.current_state = self.STATE_FOLLOWING_LINE
            self.last_line_seen_time = self.get_clock().now()
            self.get_logger().info("Line found. Transitioning to FOLLOWING_LINE.")
            self.current_target_v_omega = 0.0 # Stop search rotation
        else:
            self.current_main_line_image_x = None # Line not found
            self.current_target_vx = 0.0
            self.current_target_vy = 0.0
            self.current_target_v_omega = self.search_rotation_speed
            # self.get_logger().debug("Still searching for a line by rotating...")

    def handle_following_line(self):
        # self.get_logger().debug("Handling FOLLOWING_LINE state.")
        current_time = self.get_clock().now()
        if not self.latest_lines_msg or \
           (self.latest_lines_msg and not self.latest_lines_msg.lines and (current_time - self.last_line_seen_time > self.line_lost_timeout_duration)) or \
           (self.latest_lines_msg and self.latest_lines_msg.lines and (current_time - self.last_line_seen_time > self.line_lost_timeout_duration) and not self.find_target_line(self.latest_lines_msg)[0]):
            # More robust check for line lost:
            # 1. No lines_msg at all
            # 2. lines_msg exists but has no lines, AND timeout passed since last valid line
            # 3. lines_msg exists, has lines, but no vertical line found, AND timeout passed

            # Simplified check for timeout based on last_line_seen_time update
            if current_time - self.last_line_seen_time > self.line_lost_timeout_duration:
                self.current_state = self.STATE_SEARCHING_LINE
                self.get_logger().info("Line lost (timeout). Transitioning to SEARCHING_LINE.")
                self.current_target_vx = 0.0
                self.current_target_vy = 0.0
                self.current_target_v_omega = 0.0
                self.latest_lines_msg = None # Clear old message
                self.current_main_line_image_x = None # Line lost
                return

        vertical_line, has_only_horizontal = self.find_target_line(self.latest_lines_msg)

        if vertical_line:
            self.last_line_seen_time = current_time
            # Update current_main_line_image_x
            self.current_main_line_image_x = (vertical_line.start.x + vertical_line.end.x) / 2.0

            p1 = vertical_line.start
            p2 = vertical_line.end
            # Ensure p1 is the "upper" point (smaller y) for consistent angle calculation if needed,
            # though atan2 handles quadrants. For dx/dy, image y typically increases downwards.
            # For angle_error = atan2(dx, dy), dy is "run", dx is "rise" if considering line against Y-axis.
            # If p1.y > p2.y, swap them so dy = p2.y - p1.y is negative (line goes "up" in image coords)
            if p1.y > p2.y:
                p1, p2 = p2, p1 # Swap points

            dx = p2.x - p1.x
            dy = p2.y - p1.y # Should be positive if p2.y > p1.y (lower point)

            angle_error = 0.0
            if abs(dy) < 1e-3:
                angle_error = math.pi / 2 if dx > 0 else (-math.pi / 2 if dx < 0 else 0)
                # self.get_logger().warn(f"Line segment has very small dy ({dy:.3f}), dx ({dx:.3f}). Angle error set to: {angle_error:.2f}")
            else:
                # Angle of the line relative to the image's positive Y-axis.
                # Positive Y is downwards. Positive X is to the right.
                # If dx > 0, line leans to the right. atan2(dx, dy) will be positive.
                # Target is a perfectly vertical line, so dx = 0, angle = 0.
                angle_error = math.atan2(dx, dy)

            line_center_x = (p1.x + p2.x) / 2.0
            lateral_error = self.target_line_x - line_center_x # Target is image center

            # Corrective omega: if angle_error is positive (line leans to right, dx > 0), robot needs to turn CW (negative omega).
            self.current_target_v_omega = -self.kp_angle * angle_error
            # Corrective vy: if lateral_error is positive (line is to the left of target_line_x), robot needs to move right (positive vy).
            self.current_target_vy = self.kp_lateral * lateral_error
            self.current_target_vx = self.forward_speed
            # self.get_logger().debug(f"Line Following: angle_err={angle_error:.2f} (rad), lat_err={lateral_error:.1f} (px), V_omega={self.current_target_v_omega:.2f}, Vy={self.current_target_vy:.2f}")

        elif has_only_horizontal:
            self.get_logger().info("Only horizontal lines detected. Transitioning to STOPPED.")
            self.current_state = self.STATE_STOPPED
            self.current_target_vx = 0.0
            self.current_target_vy = 0.0
            self.current_target_v_omega = 0.0
            self.latest_lines_msg = None # Clear message
            self.current_main_line_image_x = None # Line lost/irrelevant
        else:
            # No vertical line found, but not "only_horizontal". This could be no lines at all.
            # The timeout check at the beginning of the function should handle persistent lack of usable lines.
            self.current_main_line_image_x = None # Line not reliably found
            # If we are here, it means we are within the timeout.
            # self.get_logger().warn("Following_line: No usable vertical line in current frame. Holding previous commands or slowing down. Timeout will eventually trigger search.")
            # Let's slow down if no vertical line is seen, but we are not yet in timeout.
            self.current_target_vx = self.forward_speed * 0.5 # Reduced speed
            # Keep previous omega and vy or zero them? Let's zero them to be safe.
            self.current_target_v_omega = 0.0
            self.current_target_vy = 0.0
            # The last_line_seen_time is NOT updated here. If this persists, timeout will occur.

        # Check for balls if following line
        # Only look for balls if we haven't met the collection target for the current phase
        # This check is implicitly handled by the fact that handle_collecting_ball will transition
        # to NAVIGATING_TO_DISCHARGE, so we won't be in FOLLOWING_LINE if the target was met.
        if self.latest_balls_msg and self.latest_balls_msg.positions:
            # Check if we should even be looking for balls based on mission phase
            # This logic is a bit tricky here, as handle_collecting_ball is the primary decider.
            # For now, we allow selecting a ball, and handle_collecting_ball will sort it out.
            # Pass current_main_line_image_x for side-specific selection
            selected_ball = self.select_target_ball(self.latest_balls_msg, self.current_main_line_image_x)
            if selected_ball:
                self.target_ball_info = {'color': selected_ball.color, 'position': selected_ball.position}
                self.current_state = self.STATE_APPROACHING_BALL
                self.last_ball_seen_time = self.get_clock().now() # Mark time when we start approaching
                self.get_logger().info(f"Ball detected (color: {selected_ball.color}). Transitioning to APPROACHING_BALL.")
                # Reset velocities for ball approach; they will be set in handle_approaching_ball
                self.current_target_vx = 0.0
                self.current_target_vy = 0.0
                self.current_target_v_omega = 0.0
                return # Exit to allow handle_approaching_ball to take over in next loop iteration

    def select_target_ball(self, balls_msg, main_line_center_x):
        if not balls_msg or not balls_msg.positions:
            return None

        candidate_balls = []

        if self.current_state == self.STATE_INITIAL_COLLECTION_PHASE:
            if main_line_center_x is None:
                self.get_logger().warn("INITIAL_COLLECTION_PHASE: Trying to select LEFT ball, but line position is unknown.")
                return None
            for ball in balls_msg.positions:
                if ball.position.x < main_line_center_x:
                    candidate_balls.append(ball)
            if not candidate_balls:
                self.get_logger().info("INITIAL_COLLECTION_PHASE: No balls found on the LEFT side of the line.")
                return None
        elif self.current_state == self.STATE_SECOND_COLLECTION_PHASE:
            if main_line_center_x is None:
                self.get_logger().warn("SECOND_COLLECTION_PHASE: Trying to select RIGHT ball, but line position is unknown.")
                return None
            for ball in balls_msg.positions:
                if ball.position.x >= main_line_center_x: # Greater than or equal to for the right side
                    candidate_balls.append(ball)
            if not candidate_balls:
                self.get_logger().info("SECOND_COLLECTION_PHASE: No balls found on the RIGHT side of the line.")
                return None
        else:
            # Fallback or other states not requiring side-specific collection
            # For current mission logic, ball selection should primarily happen in collection phases.
            # If called from another state, it might just use all balls.
            self.get_logger().debug(f"select_target_ball called from state {self.current_state}, not a primary collection phase. Considering all balls.")
            candidate_balls = list(balls_msg.positions)


        if not candidate_balls:
            # This case should ideally be caught by the phase-specific empty checks above.
            # self.get_logger().debug("No candidate balls after filtering.")
            return None

        # Select ball with largest y (closest to bottom of image), then smallest x distance from target_ball_x (center of image)
        best_ball = None
        max_y = -1.0

        for ball in candidate_balls:
            if ball.position.y > max_y:
                max_y = ball.position.y
                best_ball = ball
            elif ball.position.y == max_y: # Tie-break with x distance to target_ball_x (image center for alignment)
                if best_ball is None or \
                   abs(ball.position.x - self.target_ball_x) < abs(best_ball.position.x - self.target_ball_x):
                    best_ball = ball
        
        if best_ball:
            side = "LEFT" if self.current_state == self.STATE_INITIAL_COLLECTION_PHASE else ("RIGHT" if self.current_state == self.STATE_SECOND_COLLECTION_PHASE else "ANY")
            self.get_logger().info(f"Selected target ball on {side} side: {best_ball.color} at ({best_ball.position.x:.1f}, {best_ball.position.y:.1f}) from {len(candidate_balls)} candidates.")
        # else:
            # self.get_logger().debug(f"No best ball selected from {len(candidate_balls)} candidates on the target side.")

        return best_ball

    def handle_approaching_ball(self):
        # self.get_logger().debug("Handling APPROACHING_BALL state.")
        current_time = self.get_clock().now()

        if not self.target_ball_info:
            self.get_logger().warn("Approaching ball: No target_ball_info. Returning to SEARCHING_LINE.")
            self.current_state = self.STATE_SEARCHING_LINE # Or REALIGNING_TO_LINE if implemented
            return

        # Check if the target ball is still visible
        current_ball_position = None
        if self.latest_balls_msg:
            for ball in self.latest_balls_msg.positions:
                # Assuming color is a unique enough identifier for now, or use a more robust tracking ID if available
                if ball.color == self.target_ball_info['color']: # Simple check, might need improvement if multiple balls of same color
                    # A more robust check would be proximity to last known position if IDs aren't available
                    current_ball_position = ball.position
                    self.target_ball_info['position'] = ball.position # Update position
                    self.last_ball_seen_time = current_time
                    break

        if current_ball_position is None:
            if current_time - self.last_ball_seen_time > self.ball_lost_timeout_duration:
                self.get_logger().warn(f"Target ball (color: {self.target_ball_info['color']}) lost for too long. Returning to SEARCHING_LINE.")
                self.target_ball_info = None
                self.current_state = self.STATE_SEARCHING_LINE # Or REALIGNING_TO_LINE
                self.current_target_vx = 0.0
                self.current_target_vy = 0.0
                self.current_target_v_omega = 0.0
                return
            else:
                # Ball temporarily not seen, maintain last command or stop
                self.get_logger().debug(f"Target ball (color: {self.target_ball_info['color']}) temporarily not visible. Holding position.")
                self.current_target_vx = 0.0
                self.current_target_vy = 0.0
                self.current_target_v_omega = 0.0 # Or maintain last small movement
                return

        # Calculate error to target position (center-bottom of image)
        error_x = self.target_ball_x - current_ball_position.x
        error_y = self.target_ball_y - current_ball_position.y # Positive error_y means ball is above target (needs to move "down" in image, so robot moves forward)

        # self.get_logger().debug(f"Ball ({self.target_ball_info['color']}): Img Pos ({current_ball_position.x:.1f}, {current_ball_position.y:.1f}), Error ({error_x:.1f}, {error_y:.1f})")

        # Check if ball is in collection zone
        if abs(error_x) < self.collection_threshold_x and abs(error_y) < self.collection_threshold_y:
            self.current_state = self.STATE_COLLECTING_BALL
            self.collection_state_start_time = self.get_clock().now() # Mark start of collection attempt
            self.get_logger().info(f"Ball ({self.target_ball_info['color']}) in collection zone. Transitioning to COLLECTING_BALL.")
            self.current_target_vx = 0.0
            self.current_target_vy = 0.0
            self.current_target_v_omega = 0.0
            return

        # Proportional control for ball approach
        # Image: X right, Y down. Robot: Vx forward, Vy strafe left.
        # If ball.y < target_ball_y (ball is "above" target in image), error_y = target_ball_y - ball.y > 0. Robot needs to move forward (positive Vx).
        target_vx = self.kp_ball_y * error_y
        # If ball.x < target_ball_x (ball is to "left" of target in image), error_x = target_ball_x - ball.x > 0. Robot needs to strafe right (negative Vy).
        target_vy = -self.kp_ball_x * error_x

        # Clamp speeds
        self.current_target_vx = max(min(target_vx, self.approach_speed_vx_max), -self.approach_speed_vx_max)
        self.current_target_vy = max(min(target_vy, self.approach_speed_vy_max), -self.approach_speed_vy_max)
        self.current_target_v_omega = 0.0 # Keep heading, adjust later if needed

        # self.get_logger().debug(f"Approaching Ball ({self.target_ball_info['color']}): TargetVel Vx={self.current_target_vx:.2f}, Vy={self.current_target_vy:.2f}")

    def find_target_line(self, lines_msg):
        # self.get_logger().debug(f"find_target_line called with {len(lines_msg.lines) if lines_msg and lines_msg.lines else 'None or empty'} lines.")
        if not lines_msg or not lines_msg.lines:
            return None, False

    def handle_collecting_ball(self):
        # self.get_logger().debug("Handling COLLECTING_BALL state.")
        current_time = self.get_clock().now()

        if not self.target_ball_info:
            self.get_logger().error("COLLECTING_BALL state reached without target_ball_info. This should not happen. Transitioning to SEARCHING_LINE.")
            self.current_state = self.STATE_SEARCHING_LINE
            return

        # Stop the robot during collection attempt
        self.current_target_vx = 0.0
        self.current_target_vy = 0.0
        self.current_target_v_omega = 0.0

        ball_is_still_visible = False
        if self.latest_balls_msg and self.latest_balls_msg.positions:
            for ball in self.latest_balls_msg.positions:
                # Simple check by color. Could be enhanced if balls of same color exist often
                # or by checking proximity to the last known position of self.target_ball_info['position']
                if ball.color == self.target_ball_info['color']:
                    # A more robust check would be:
                    # if ball.color == self.target_ball_info['color'] and \
                    #    math.hypot(ball.position.x - self.target_ball_info['position'].x, ball.position.y - self.target_ball_info['position'].y) < SOME_SMALL_PIXEL_THRESHOLD_FOR_SAME_BALL:
                    ball_is_still_visible = True
                    break

        if not ball_is_still_visible:
            collected_color = self.target_ball_info['color']
            self.collected_balls_this_run.append(collected_color)
            # self.total_collected_ever +=1 # If using this variable

            self.get_logger().info(f"Successfully collected ball of color: {collected_color}. Balls in this run: {len(self.collected_balls_this_run)} ({self.collected_balls_this_run}).")

            self.target_ball_info = None # Clear the target ball info

            # Mission logic for transitioning after collection
            if not self.first_discharge_complete and \
               len(self.collected_balls_this_run) >= self.balls_for_first_phase:
                self.get_logger().info(f"First phase collection target ({self.balls_for_first_phase} balls) reached.")
                self.current_state = self.STATE_NAVIGATING_TO_DISCHARGE
            elif self.first_discharge_complete and \
                 len(self.collected_balls_this_run) >= (self.total_balls_to_collect - self.balls_for_first_phase):
                # This assumes total_balls_to_collect is the grand total, and balls_for_first_phase were already collected and "discharged"
                # So, we are checking if we collected the remaining amount for the second phase.
                self.get_logger().info(f"Second phase collection target reached (collected {len(self.collected_balls_this_run)} out of remaining {(self.total_balls_to_collect - self.balls_for_first_phase)}).")
                self.current_state = self.STATE_NAVIGATING_TO_DISCHARGE
            # Safety check: if total_balls_to_collect is met by collected_balls_this_run, even if phase logic didn't catch it.
            # This can happen if total_balls_to_collect < balls_for_first_phase or similar configurations.
            elif len(self.collected_balls_this_run) >= self.total_balls_to_collect and not self.first_discharge_complete:
                 self.get_logger().info(f"Total collection target ({self.total_balls_to_collect} balls) met within the first phase.")
                 self.current_state = self.STATE_NAVIGATING_TO_DISCHARGE
            elif self.first_discharge_complete and sum([self.balls_for_first_phase, len(self.collected_balls_this_run)]) >= self.total_balls_to_collect :
                 self.get_logger().info(f"Total collection target ({self.total_balls_to_collect} balls) met after starting second phase.")
                 self.current_state = self.STATE_NAVIGATING_TO_DISCHARGE
            else:
                # Collection target for the current phase not yet reached, continue collecting.
                self.current_state = self.STATE_REALIGNING_TO_LINE
                self.realign_state_start_time = self.get_clock().now()
                self.get_logger().info("Collection target not met for this phase. Transitioning to REALIGNING_TO_LINE.")

            # Reset line tracking variables to force re-evaluation in REALIGNING or if returning to line following
            self.latest_lines_msg = None
            self.last_line_seen_time = self.get_clock().now()
            return

        # If ball is still visible, check for timeout
        if self.collection_state_start_time and \
           (current_time - self.collection_state_start_time) > self.collection_wait_timeout_duration:
            self.get_logger().warn(f"Ball (color: {self.target_ball_info['color']}) did not disappear within timeout. Returning to APPROACHING_BALL.")
            # Optionally, slightly nudge the robot or try something else
            self.current_state = self.STATE_APPROACHING_BALL # Try to re-center or re-evaluate
            self.collection_state_start_time = None # Reset timer for next attempt
            return

        # self.get_logger().debug(f"Waiting for ball ({self.target_ball_info['color']}) to disappear. Still visible.")

    # New Mission State Handler Implementations
    def handle_initial_collection_phase(self):
        self.get_logger().info("Entering INITIAL_COLLECTION_PHASE.")
        self.collected_balls_this_run.clear()
        # self.collection_target_achieved_this_phase = False # Not using this flag based on current logic
        self.current_state = self.STATE_SEARCHING_LINE
        self.get_logger().info("Transitioning from INITIAL_COLLECTION_PHASE to SEARCHING_LINE.")

    def handle_second_collection_phase(self):
        self.get_logger().info("Entering SECOND_COLLECTION_PHASE.")
        self.collected_balls_this_run.clear()
        # self.collection_target_achieved_this_phase = False # Not using this flag
        # Potentially turn around or a more specific realignment strategy here.
        # For now, just search for the line again.
        self.current_state = self.STATE_SEARCHING_LINE
        self.get_logger().info("Transitioning from SECOND_COLLECTION_PHASE to SEARCHING_LINE.")

    def handle_navigating_to_discharge(self):
        self.get_logger().info("Entering NAVIGATING_TO_DISCHARGE.")
        self.current_target_vx = 0.0
        self.current_target_vy = 0.0
        self.current_target_v_omega = 0.0
        # In a real scenario, this state would involve navigation logic.
        # For now, we simulate arrival.
        self.get_logger().info("Simulating navigation to discharge area... Arrived.")
        self.current_state = self.STATE_DISCHARGING_BALLS
        self.discharge_phase_start_time = self.get_clock().now()
        self.get_logger().info("Transitioning to DISCHARGING_BALLS.")

    def handle_discharging_balls(self):
        self.get_logger().debug(f"Handling DISCHARGING_BALLS state. Time started: {self.discharge_phase_start_time}")
        self.current_target_vx = 0.0
        self.current_target_vy = 0.0
        self.current_target_v_omega = 0.0

        if self.discharge_phase_start_time is None: # Should have been set in NAVIGATING_TO_DISCHARGE
            self.get_logger().error("Discharge phase start time not set! Transitioning to STOPPED.")
            self.current_state = self.STATE_STOPPED
            return

        if (self.get_clock().now() - self.discharge_phase_start_time) > self.discharge_wait_duration:
            self.get_logger().info(f"Discharge complete. Balls cleared during this phase: {self.collected_balls_this_run}")
            self.collected_balls_this_run.clear() # Clear balls from this specific run

            if not self.first_discharge_complete:
                self.first_discharge_complete = True
                self.get_logger().info("First discharge complete. Transitioning to SECOND_COLLECTION_PHASE.")
                self.current_state = self.STATE_SECOND_COLLECTION_PHASE
            else:
                # This was the second discharge (or a discharge after total balls collected)
                self.get_logger().info("Final discharge complete. Transitioning to MISSION_COMPLETE.")
                self.current_state = self.STATE_MISSION_COMPLETE
            self.discharge_phase_start_time = None # Reset for next potential discharge
        else:
            self.get_logger().debug("Waiting for discharge timer...")


    def handle_mission_complete(self):
        self.get_logger().info("Entering MISSION_COMPLETE.")
        self.current_target_vx = 0.0
        self.current_target_vy = 0.0
        self.current_target_v_omega = 0.0
        self.get_logger().info("MISSION ACCOMPLISHED. Robot stopped.")
        # Robot stays in this state. Further actions might require external reset or new commands.


    def handle_realigning_to_line(self):
        # self.get_logger().debug("Handling REALIGNING_TO_LINE state.")
        current_time = self.get_clock().now()

        # Attempt to find a vertical line
        vertical_line_found = False
        if self.latest_lines_msg: # Ensure there's fresh data to process
            target_line, _ = self.find_target_line(self.latest_lines_msg) # We only care if a vertical line exists
            if target_line:
                vertical_line_found = True

        if vertical_line_found:
            self.get_logger().info("Line found during realignment. Transitioning to FOLLOWING_LINE.")
            self.current_state = self.STATE_FOLLOWING_LINE
            self.last_line_seen_time = self.get_clock().now() # Update time for the found line
            self.current_target_vx = 0.0 # Will be set by handle_following_line
            self.current_target_vy = 0.0
            self.current_target_v_omega = 0.0 # Stop rotation
            self.realign_state_start_time = None # Clear realignment timer
            return
        else:
            # No vertical line found yet, continue rotating
            # self.get_logger().debug("Realigning: No vertical line found yet. Commanding rotation.")
            self.current_target_vx = 0.0
            self.current_target_vy = 0.0
            self.current_target_v_omega = self.search_rotation_speed

            # Check for timeout
            if self.realign_state_start_time and \
               (current_time - self.realign_state_start_time) > self.realign_timeout_duration:
                self.get_logger().warn(f"Could not find line within realignment timeout ({self.realign_timeout_duration.nanoseconds / 1e9:.1f}s). Transitioning to SEARCHING_LINE for a broader search.")
                self.current_state = self.STATE_SEARCHING_LINE
                self.realign_state_start_time = None # Clear realignment timer
                # Velocities will be set by handle_searching_line in the next loop
                return
        # self.get_logger().debug(f"Realigning: Target Vels: Vx={self.current_target_vx:.2f}, Vy={self.current_target_vy:.2f}, Vomega={self.current_target_v_omega:.2f}")

    def calculate_wheel_efforts(self, vx_mps, vy_mps, v_omega_radps):
        # ホイールの配置角度（ラジアン）
        angle_w1 = math.pi / 3      # ホイール1: 60°
        angle_w2 = math.pi          # ホイール2: 180°
        angle_w3 = 5 * math.pi / 3  # ホイール3: 300°

        # 個々のホイール速度[m/s]を逆運動学で算出
        v_w1 = (
            -math.sin(angle_w1) * vx_mps
            + math.cos(angle_w1) * vy_mps
            + v_omega_radps * self.wheel_base_radius
        )
        v_w2 = (
            -math.sin(angle_w2) * vx_mps
            + math.cos(angle_w2) * vy_mps
            + v_omega_radps * self.wheel_base_radius
        )
        v_w3 = (
            -math.sin(angle_w3) * vx_mps
            + math.cos(angle_w3) * vy_mps
            + v_omega_radps * self.wheel_base_radius
        )

        # 各ホイールの最大回転速度[rad/s]
        max_wheel_speed = self.max_wheel_speed  # パラメータで設定済み
        if max_wheel_speed <= 0:
            raise ValueError("max_wheel_speed must be positive")

        # 正規化: effort = wheel_speed / max_wheel_speed を [-1, 1] にクランプ
        def clamp(val, min_val=-1.0, max_val=1.0):
            return max(min(val, max_val), min_val)

        effort_w1 = clamp(v_w1 / max_wheel_speed)
        effort_w2 = clamp(v_w2 / max_wheel_speed)
        effort_w3 = clamp(v_w3 / max_wheel_speed)

        return [effort_w1, effort_w2, effort_w3]


    def find_target_line(self, lines_msg):
        # self.get_logger().debug(f"find_target_line called with {len(lines_msg.lines) if lines_msg and lines_msg.lines else 'None or empty'} lines.")
        if not lines_msg or not lines_msg.lines:
            return None, False

        picked_vertical_line = None
        # Simple strategy: pick the first "sufficiently vertical" line found.
        # More sophisticated: pick longest, or closest to center, or average of vertical lines.
        for line_segment in lines_msg.lines:
            p_start = line_segment.start
            p_end = line_segment.end
            dx = p_end.x - p_start.x
            dy = p_end.y - p_start.y

            # Check for verticality: dy should be significantly larger than dx
            if abs(dy) > self.verticality_factor * abs(dx):
                picked_vertical_line = line_segment
                # self.get_logger().debug(f"Found vertical line: start=({p_start.x:.1f},{p_start.y:.1f}), end=({p_end.x:.1f},{p_end.y:.1f})")
                break # Found one, use it for now.

        if picked_vertical_line:
            return picked_vertical_line, False # Found and picked a vertical line

        # If no vertical line was picked (picked_vertical_line is None)
        # Check if all lines currently in the message are non-vertical (horizontal or too oblique)
        # This is to handle the "only horizontal lines" scenario.
        has_at_least_one_line = bool(lines_msg.lines) # We know this is true if we are here from the first check.
        if not has_at_least_one_line:
             return None, False # Should not happen if first check passed and lines_msg.lines was not empty.

        all_lines_are_non_vertical = True
        for line_segment in lines_msg.lines: # Iterate again to check all lines
            p_start = line_segment.start
            p_end = line_segment.end
            dx = p_end.x - p_start.x
            dy = p_end.y - p_start.y
            if abs(dy) > self.verticality_factor * abs(dx): # Found a vertical line
                all_lines_are_non_vertical = False # So, not ALL are non-vertical
                # self.get_logger().debug("Vertical line exists, but was not picked. All lines are NOT non-vertical.")
                break

        if all_lines_are_non_vertical:
            # self.get_logger().debug("No vertical line picked, AND all available lines are confirmed non-vertical.")
            return None, True # No vertical line picked, and all available lines are indeed non-vertical

        # self.get_logger().debug("No vertical line picked, and it's NOT the case that all lines are non-vertical (e.g., mix, or none).")
        return None, False # No vertical line picked, and it's not true that all lines were non-vertical.
                           # This means either no lines at all (handled at start), or a mix, or some vertical ones were missed by 'pick first'.

    def publish_efforts(self, efforts_list): # NEW signature
        msg = Float32MultiArray()
        # NEW: Assumes efforts_list already contains clipped floats
        if len(efforts_list) == 3:
            msg.data = [float(efforts_list[0]), float(efforts_list[1]), float(efforts_list[2])]
            # self.get_logger().debug(f"Publishing motor efforts: {msg.data}")
            self.motor_efforts_publisher_.publish(msg)
        else:
            self.get_logger().error(f"publish_efforts called with list of length {len(efforts_list)}, expected 3.")

def main(args=None):
    rclpy.init(args=args)
    robot_control_node = RobotControlNode()
    try:
        rclpy.spin(robot_control_node)
    except KeyboardInterrupt:
        # self.get_logger().info('Keyboard interrupt, shutting down.') # Node logger not accessible here
        print('Keyboard interrupt, shutting down.')
    finally:
        if rclpy.ok():
            robot_control_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
