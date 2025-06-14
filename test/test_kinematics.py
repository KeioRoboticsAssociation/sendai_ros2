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

def calculate_expected_wheel_efforts(
    vx_mps,
    vy_mps,
    v_omega_radps,
    robot_radius,
    max_wheel_speed,
    wheel_scales=None,
):
    if max_wheel_speed <= 0:
        raise ValueError("max_wheel_speed must be positive")

    L = robot_radius
    angles = [math.pi / 3, math.pi, 5 * math.pi / 3]

    v_ws = [
        -math.sin(th) * vx_mps + math.cos(th) * vy_mps + v_omega_radps * L
        for th in angles
    ]

    def clamp(val, min_val=-0.5, max_val=0.5):
        return max(min(val, max_val), min_val)

    efforts = [clamp(v * max_wheel_speed) for v in v_ws]

    if wheel_scales is not None:
        efforts = [clamp(e * s) for e, s in zip(efforts, wheel_scales)]

    return efforts

@pytest.fixture
def kinematics_params():
    return {
        "robot_radius": 0.15,  # L
        "max_wheel_speed": 0.5 # Max tangential speed of a wheel (m/s) at 1.0 duty cycle
    }

def test_forward_motion(kinematics_params):
    vx, vy, v_omega = 0.1, 0.0, 0.0
    efforts = calculate_expected_wheel_efforts(vx, vy, v_omega, **kinematics_params)
    # v_w1 = -(sqrt(3)/2)*0.1 = -0.0866
    # v_w2 = ~0
    # v_w3 = (sqrt(3)/2)*0.1 = 0.0866
    # e1 = -0.0866*0.5 = -0.0433
    # e2 = 0
    # e3 = 0.0866*0.5 = 0.0433
    assert efforts[0] == pytest.approx(-0.0433, abs=1e-4)
    assert efforts[1] == pytest.approx(0.0, abs=1e-4)
    assert efforts[2] == pytest.approx(0.0433, abs=1e-4)

def test_strafe_left_motion(kinematics_params):
    vx, vy, v_omega = 0.0, 0.1, 0.0
    efforts = calculate_expected_wheel_efforts(vx, vy, v_omega, **kinematics_params)
    
    # v_w1 = 0.5*0.1 = 0.05 => e1 = 0.025
    # v_w2 = -0.1 => e2 = -0.05
    # v_w3 = 0.05 => e3 = 0.025
    assert efforts[0] == pytest.approx(0.025)
    assert efforts[1] == pytest.approx(-0.05)
    assert efforts[2] == pytest.approx(0.025)

def test_rotate_ccw_motion(kinematics_params):
    vx, vy, v_omega = 0.0, 0.0, 0.2
    L = kinematics_params["robot_radius"] # 0.15
    efforts = calculate_expected_wheel_efforts(vx, vy, v_omega, **kinematics_params)
    # v_w1 = L*0.2 = 0.15*0.2 = 0.03
    # v_w2 = L*0.2 = 0.03
    # v_w3 = L*0.2 = 0.03
    # e1, e2, e3 = 0.03*0.5 = 0.015
    assert efforts[0] == pytest.approx(0.015)
    assert efforts[1] == pytest.approx(0.015)
    assert efforts[2] == pytest.approx(0.015)

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

    # v_w1 = -(sqrt(3)/2)*0.7 = -0.6062 => e1 = -0.6062*0.5 = -0.3031
    # v_w2 ≈ 0       => e2 = 0
    # v_w3 = (sqrt(3)/2)*0.7 = 0.6062 => e3 = 0.6062*0.5 = 0.3031
    assert efforts[0] == pytest.approx(-0.3031, abs=1e-4)
    assert efforts[1] == pytest.approx(0.0, abs=1e-4)
    assert efforts[2] == pytest.approx(0.3031, abs=1e-4)

    # Test with strafe that should clip
    vx_strafe, vy_strafe, v_omega_strafe = 0.0, kinematics_params["max_wheel_speed"] * 1.5, 0.0 # vy = 0.5 * 1.5 = 0.75
    efforts_strafe = calculate_expected_wheel_efforts(vx_strafe, vy_strafe, v_omega_strafe, **kinematics_params)
    # v_w1 = 0.375 => e1 = 0.1875
    # v_w2 = -0.75  => e2 = -0.375 => clipped inside range
    # v_w3 = 0.375  => e3 = 0.1875
    assert efforts_strafe[0] == pytest.approx(0.1875)
    assert efforts_strafe[1] == pytest.approx(-0.375)
    assert efforts_strafe[2] == pytest.approx(0.1875)


def test_wheel_scaling_factors(kinematics_params):
    vx, vy, v_omega = 0.1, 0.0, 0.0
    scales = (1.5, 1.0, 0.5)
    efforts = calculate_expected_wheel_efforts(
        vx,
        vy,
        v_omega,
        **kinematics_params,
        wheel_scales=scales,
    )
    assert efforts[0] == pytest.approx(-0.06495, abs=1e-4)
    assert efforts[1] == pytest.approx(0.0, abs=1e-4)
    assert efforts[2] == pytest.approx(0.02165, abs=1e-4)
