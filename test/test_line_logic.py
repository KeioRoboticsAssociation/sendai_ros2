import math
import pytest

# Mocking ROS Point and LineSegment for testing find_target_line
class MockPoint:
    def __init__(self, x, y, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

class MockLineSegment:
    def __init__(self, start_x, start_y, end_x, end_y):
        self.start = MockPoint(start_x, start_y)
        self.end = MockPoint(end_x, end_y)

class MockLineSegmentArray:
    def __init__(self, lines=None):
        # Assumes 'lines' is the field name in the actual message
        self.lines = lines if lines is not None else []

# This class would mimic a minimal RobotControlNode or provide static methods
class LineLogicTester:
    def __init__(self, verticality_factor):
        self.verticality_factor = verticality_factor
        # Other params like target_line_x could be added if testing more parts

    def find_target_line(self, lines_msg): # Simplified from node's method
        if not lines_msg or not lines_msg.lines:
            return None, False

        picked_vertical_line = None
        for line_segment in lines_msg.lines:
            p_start = line_segment.start
            p_end = line_segment.end
            dx = p_end.x - p_start.x
            dy = p_end.y - p_start.y
            if abs(dy) > self.verticality_factor * abs(dx):
                picked_vertical_line = line_segment
                break

        if picked_vertical_line:
            return picked_vertical_line, False

        has_at_least_one_line = bool(lines_msg.lines)
        if not has_at_least_one_line: # Should be caught by the first check
             return None, False

        all_lines_are_non_vertical = True
        for line_segment in lines_msg.lines: # Check all lines
            p_start = line_segment.start
            p_end = line_segment.end
            dx = p_end.x - p_start.x
            dy = p_end.y - p_start.y
            if abs(dy) > self.verticality_factor * abs(dx): # Found a vertical one
                all_lines_are_non_vertical = False # Contradiction
                break

        if all_lines_are_non_vertical:
            return None, True # No vertical line picked, and all available lines are non-vertical

        return None, False # No vertical picked, but not all lines were non-vertical

def test_line_angle_calculation():
    # Test atan2(dx, dy) behavior, assuming image Y-axis is "run" for vertical lines
    # For a line from (x1,y1) to (x2,y2), dx = x2-x1, dy = y2-y1
    # Angle relative to Y-axis: atan2(dx, dy)
    assert math.atan2(0, 100) == pytest.approx(0.0)  # Vertical line (dx=0, positive dy)
    assert math.atan2(10, 100) == pytest.approx(0.09966, abs=1e-4)  # Leans right (dx>0)
    assert math.atan2(-10, 100) == pytest.approx(-0.09966, abs=1e-4) # Leans left (dx<0)
    assert math.atan2(100, 0) == pytest.approx(math.pi / 2) # Horizontal line (dy=0, positive dx)
    assert math.atan2(0, -100) == pytest.approx(math.pi) # Vertical line pointing up image (negative dy)
                                                      # atan2(0, -1) is pi. For robot, this may mean it passed the line.
                                                      # Or depending on point order, could be valid.
                                                      # The node code swaps points to ensure p1.y < p2.y for dy calculation.
                                                      # Let's assume dy is typically positive in image coords for "downward" segment.


@pytest.fixture
def line_tester():
    return LineLogicTester(verticality_factor=2.0) # Default factor used in node

def test_find_target_line_no_lines(line_tester):
    lines_msg = MockLineSegmentArray()
    line, only_non_vertical = line_tester.find_target_line(lines_msg)
    assert line is None
    assert not only_non_vertical

def test_find_target_line_one_vertical(line_tester):
    lines = [MockLineSegment(10, 0, 10, 100)] # Vertical line dx=0, dy=100. 100 > 2.0 * 0
    lines_msg = MockLineSegmentArray(lines)
    line, only_non_vertical = line_tester.find_target_line(lines_msg)
    assert line == lines[0]
    assert not only_non_vertical

def test_find_target_line_one_horizontal(line_tester):
    # dx=100, dy=10. dy (10) not > 2.0 * abs(dx) (100*2=200)
    lines = [MockLineSegment(0, 10, 100, 20)]
    lines_msg = MockLineSegmentArray(lines)
    line, only_non_vertical = line_tester.find_target_line(lines_msg)
    assert line is None
    assert only_non_vertical

def test_find_target_line_mixed_picks_vertical(line_tester):
    lines = [
        MockLineSegment(0, 10, 100, 20),  # Horizontal: dx=100, dy=10. 10 not > 2*100
        MockLineSegment(10, 0, 12, 100)   # Vertical-ish: dx=2, dy=100. 100 > 2*2 (4)
    ]
    lines_msg = MockLineSegmentArray(lines)
    line, only_non_vertical = line_tester.find_target_line(lines_msg)
    assert line == lines[1]
    assert not only_non_vertical

def test_find_target_line_all_oblique_non_vertical(line_tester):
    # dx=50, dy=60. dy (60) not > 2.0 * dx (50), i.e., 60 not > 100
    lines = [MockLineSegment(0, 0, 50, 60)]
    lines_msg = MockLineSegmentArray(lines)
    line, only_non_vertical = line_tester.find_target_line(lines_msg)
    assert line is None
    assert only_non_vertical # "only_non_vertical" is the correct term

def test_find_target_line_multiple_verticals_picks_first(line_tester):
    lines = [
        MockLineSegment(10, 0, 10, 100), # First vertical
        MockLineSegment(50, 0, 50, 100)  # Second vertical
    ]
    lines_msg = MockLineSegmentArray(lines)
    line, only_non_vertical = line_tester.find_target_line(lines_msg)
    assert line == lines[0]
    assert not only_non_vertical

def test_find_target_line_oblique_but_vertical_enough(line_tester):
    # dx=10, dy=30. dy (30) > 2.0 * dx (10), i.e. 30 > 20. This IS vertical.
    lines = [MockLineSegment(0,0, 10, 30)]
    lines_msg = MockLineSegmentArray(lines)
    line, only_non_vertical = line_tester.find_target_line(lines_msg)
    assert line == lines[0]
    assert not only_non_vertical

def test_find_target_line_oblique_and_horizontal(line_tester):
    # Oblique: dx=50, dy=60. 60 not > 2*50=100. Not vertical.
    # Horizontal: dx=100, dy=0. 0 not > 2*100=200. Not vertical.
    lines = [
        MockLineSegment(0,0, 50, 60),
        MockLineSegment(0,0, 100, 0)
    ]
    lines_msg = MockLineSegmentArray(lines)
    line, only_non_vertical = line_tester.find_target_line(lines_msg)
    assert line is None
    assert only_non_vertical
