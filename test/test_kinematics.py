import math
import pytest
# It's tricky to import RobotControlNode directly without a ROS environment.
# For pure function testing, if kinematics were a static method or free function, it'd be easier.
# Assuming we can instantiate a minimal RobotControlNode for testing its method.
# This might require rclpy.init() if the node's __init__ needs it for parameters.
# For simplicity, we'll define a helper class or directly test the formulas.

# Let's define the core logic here to test it directly,
# mimicking what RobotControlNode.calculate_wheel_efforts would do.
# This avoids needing a full ROS node initialization for this specific unit test.

def calculate_expected_wheel_efforts(vx_mps, vy_mps, v_omega_radps, robot_radius, max_wheel_speed):
    if max_wheel_speed <= 0:
        raise ValueError("max_wheel_speed must be positive")

    L = robot_radius

    v_w1 = vy_mps + L * v_omega_radps
    v_w2 = (-math.sqrt(3)/2.0 * vx_mps) - (0.5 * vy_mps) + (L * v_omega_radps)
    v_w3 = (math.sqrt(3)/2.0 * vx_mps) - (0.5 * vy_mps) + (L * v_omega_radps)

    effort1 = v_w1 / max_wheel_speed
    effort2 = v_w2 / max_wheel_speed
    effort3 = v_w3 / max_wheel_speed

    effort1 = max(min(effort1, 1.0), -1.0)
    effort2 = max(min(effort2, 1.0), -1.0)
    effort3 = max(min(effort3, 1.0), -1.0)

    return [effort1, effort2, effort3]

@pytest.fixture
def kinematics_params():
    return {
        "robot_radius": 0.15,  # L
        "max_wheel_speed": 0.5 # Max tangential speed of a wheel (m/s) at 1.0 duty cycle
    }

def test_forward_motion(kinematics_params):
    vx, vy, v_omega = 0.1, 0.0, 0.0
    efforts = calculate_expected_wheel_efforts(vx, vy, v_omega, **kinematics_params)
    # v_w1 = 0, v_w2 = -sqrt(3)/2*0.1 = -0.0866, v_w3 = sqrt(3)/2*0.1 = 0.0866
    # e1 = 0, e2 = -0.0866/0.5 = -0.1732, e3 = 0.0866/0.5 = 0.1732
    assert efforts[0] == pytest.approx(0.0)
    assert efforts[1] == pytest.approx(-0.1732, abs=1e-4)
    assert efforts[2] == pytest.approx(0.1732, abs=1e-4)

def test_strafe_left_motion(kinematics_params):
    vx, vy, v_omega = 0.0, 0.1, 0.0
    efforts = calculate_expected_wheel_efforts(vx, vy, v_omega, **kinematics_params)
    # v_w1 = 0.1, v_w2 = -0.5*0.1 = -0.05, v_w3 = -0.5*0.1 = -0.05
    # e1 = 0.1/0.5 = 0.2, e2 = -0.05/0.5 = -0.1, e3 = -0.05/0.5 = -0.1
    assert efforts[0] == pytest.approx(0.2)
    assert efforts[1] == pytest.approx(-0.1)
    assert efforts[2] == pytest.approx(-0.1)

def test_rotate_ccw_motion(kinematics_params):
    vx, vy, v_omega = 0.0, 0.0, 0.2
    L = kinematics_params["robot_radius"] # 0.15
    efforts = calculate_expected_wheel_efforts(vx, vy, v_omega, **kinematics_params)
    # v_w1 = L*0.2 = 0.15*0.2 = 0.03
    # v_w2 = L*0.2 = 0.03
    # v_w3 = L*0.2 = 0.03
    # e1, e2, e3 = 0.03 / 0.5 = 0.06
    assert efforts[0] == pytest.approx(0.06)
    assert efforts[1] == pytest.approx(0.06)
    assert efforts[2] == pytest.approx(0.06)

def test_zero_motion(kinematics_params):
    vx, vy, v_omega = 0.0, 0.0, 0.0
    efforts = calculate_expected_wheel_efforts(vx, vy, v_omega, **kinematics_params)
    assert efforts == [0.0, 0.0, 0.0]

def test_clipping_motion(kinematics_params):
    # This vx value should cause v_w2 and v_w3 to exceed max_wheel_speed before scaling by it
    # max_wheel_speed = 0.5. sqrt(3)/2 is approx 0.866.
    # If vx = 0.7, then sqrt(3)/2 * 0.7 = 0.866 * 0.7 = 0.6062, which is > 0.5
    vx, vy, v_omega = 0.7, 0.0, 0.0
    efforts = calculate_expected_wheel_efforts(vx, vy, v_omega, **kinematics_params)
    # v_w1 = 0
    # v_w2 = -0.6062 => e2_unclipped = -0.6062 / 0.5 = -1.2124 => e2_clipped = -1.0
    # v_w3 = 0.6062  => e3_unclipped = 0.6062 / 0.5 = 1.2124  => e3_clipped = 1.0
    assert efforts[0] == pytest.approx(0.0)
    assert efforts[1] == pytest.approx(-1.0)
    assert efforts[2] == pytest.approx(1.0)

    # Test with strafe that should clip
    vx_strafe, vy_strafe, v_omega_strafe = 0.0, kinematics_params["max_wheel_speed"] * 1.5, 0.0 # vy = 0.5 * 1.5 = 0.75
    efforts_strafe = calculate_expected_wheel_efforts(vx_strafe, vy_strafe, v_omega_strafe, **kinematics_params)
    # v_w1 = 0.75 => e1_unclipped = 0.75/0.5 = 1.5 => e1_clipped = 1.0
    # v_w2 = -0.5 * 0.75 = -0.375 => e2 = -0.375/0.5 = -0.75
    # v_w3 = -0.5 * 0.75 = -0.375 => e3 = -0.375/0.5 = -0.75
    assert efforts_strafe[0] == pytest.approx(1.0)
    assert efforts_strafe[1] == pytest.approx(-0.75)
    assert efforts_strafe[2] == pytest.approx(-0.75)
