import pytest

# Mocking ROS Point and BallPosition for testing select_target_ball
class MockPoint:
    def __init__(self, x, y, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

class MockBallPosition: # Matches BallPosition.msg
    def __init__(self, color, x, y):
        self.color = color
        self.position = MockPoint(x, y)

class MockBallPositionArray: # Matches BallPositionArray.msg
    def __init__(self, ball_positions=None): # Use 'ball_positions' as argument for clarity
        self.balls = ball_positions if ball_positions is not None else [] # Field name is 'balls'

# This class would mimic a minimal RobotControlNode or provide static methods
class BallSelectionTester:
    def __init__(self, target_ball_x):
        self.target_ball_x = target_ball_x

    def select_target_ball(self, balls_msg): # Simplified from node's method
        if not balls_msg or not balls_msg.balls: # Check .balls
            return None

        best_ball = None
        # Initialize max_y to a very small number to correctly find the largest y
        max_y = -float('inf')

        for ball in balls_msg.balls: # Iterate over .balls
            if ball.position.y > max_y:
                max_y = ball.position.y
                best_ball = ball
            elif ball.position.y == max_y: # Tie in y-coordinate
                # If best_ball is None here, it means this is the first ball considered at this max_y
                if best_ball is None or \
                   abs(ball.position.x - self.target_ball_x) < abs(best_ball.position.x - self.target_ball_x):
                    best_ball = ball
        return best_ball

@pytest.fixture
def ball_selector():
    # Default target_ball_x from parameters was 320.0
    return BallSelectionTester(target_ball_x=320.0)

def test_select_ball_no_balls(ball_selector):
    balls_msg = MockBallPositionArray()
    assert ball_selector.select_target_ball(balls_msg) is None

def test_select_ball_one_ball(ball_selector):
    balls_data = [MockBallPosition("red", 100, 200)]
    balls_msg = MockBallPositionArray(balls_data)
    assert ball_selector.select_target_ball(balls_msg) == balls_data[0]

def test_select_ball_closest_to_bottom(ball_selector):
    ball_A = MockBallPosition("red", 100, 200)
    ball_B = MockBallPosition("blue", 150, 300) # Lower Y (larger value), should be chosen
    ball_C = MockBallPosition("green", 120, 100)
    balls_data = [ball_A, ball_B, ball_C]
    balls_msg = MockBallPositionArray(balls_data)
    assert ball_selector.select_target_ball(balls_msg) == ball_B

def test_select_ball_tie_y_closest_to_center_x(ball_selector):
    # target_ball_x is 320
    ball_A = MockBallPosition("red", 300, 200)  # dx = abs(300-320) = 20
    ball_B = MockBallPosition("blue", 330, 200) # dx = abs(330-320) = 10 (closer)
    ball_C = MockBallPosition("green", 100, 200)# dx = abs(100-320) = 220
    balls_data = [ball_A, ball_B, ball_C]
    balls_msg = MockBallPositionArray(balls_data)
    assert ball_selector.select_target_ball(balls_msg) == ball_B

def test_select_ball_tie_y_different_order(ball_selector):
    # target_ball_x is 320
    ball_A = MockBallPosition("red", 300, 200)  # dx = 20
    ball_B = MockBallPosition("blue", 330, 200) # dx = 10 (closer)
    ball_C = MockBallPosition("green", 100, 200)# dx = 220
    # Change order of presentation
    balls_data = [ball_C, ball_B, ball_A]
    balls_msg = MockBallPositionArray(balls_data)
    assert ball_selector.select_target_ball(balls_msg) == ball_B


def test_select_ball_tie_y_negative_x(ball_selector):
    # target_ball_x is 0 for this specific test case for simplicity
    custom_selector = BallSelectionTester(target_ball_x=0.0)
    ball_A_closer = MockBallPosition("red", -5, 200) # dist abs(-5 - 0) = 5
    ball_B_further = MockBallPosition("blue", 10, 200) # dist abs(10 - 0) = 10

    balls_data_1 = [ball_A_closer, ball_B_further]
    balls_msg_1 = MockBallPositionArray(balls_data_1)
    assert custom_selector.select_target_ball(balls_msg_1) == ball_A_closer

    balls_data_2 = [ball_B_further, ball_A_closer] # Order shouldn't matter
    balls_msg_2 = MockBallPositionArray(balls_data_2)
    assert custom_selector.select_target_ball(balls_msg_2) == ball_A_closer

def test_select_ball_all_same_y_and_x_dist_picks_first(ball_selector):
    # target_ball_x is 320
    # Ball A: x=310, y=200. dx = abs(310-320) = 10
    # Ball B: x=330, y=200. dx = abs(330-320) = 10
    ball_A = MockBallPosition("red", 310, 200)
    ball_B = MockBallPosition("blue", 330, 200)

    balls_data_1 = [ball_A, ball_B]
    balls_msg_1 = MockBallPositionArray(balls_data_1)
    # In a perfect tie of y and distance to target_x, the one encountered first in the list is picked.
    assert ball_selector.select_target_ball(balls_msg_1) == ball_A

    balls_data_2 = [ball_B, ball_A]
    balls_msg_2 = MockBallPositionArray(balls_data_2)
    assert ball_selector.select_target_ball(balls_msg_2) == ball_B

def test_select_ball_y_coordinates_are_float(ball_selector):
    ball_A = MockBallPosition("red", 100, 200.1)
    ball_B = MockBallPosition("blue", 150, 200.5) # Lower Y (larger value)
    ball_C = MockBallPosition("green", 120, 200.0)
    balls_data = [ball_A, ball_B, ball_C]
    balls_msg = MockBallPositionArray(balls_data)
    assert ball_selector.select_target_ball(balls_msg) == ball_B
