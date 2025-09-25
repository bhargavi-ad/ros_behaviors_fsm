#!/usr/bin/env python3
"""
Test script for the Finite State Machine
This script demonstrates how to use the FSM controller
"""

import rclpy
from finite_state_controller import FiniteStateController, RobotState


def test_fsm():
    """Test the finite state machine functionality"""
    print("Testing Finite State Machine Controller...")

    rclpy.init()

    try:
        # Create the FSM controller
        controller = FiniteStateController()

        print(f"Initial state: {controller.get_current_state().value}")

        # Test state transitions using public methods
        print("\nTesting state transitions...")

        # Test teleop transition
        controller.set_state(RobotState.TELEOP)
        print(f"After teleop transition: {controller.get_current_state().value}")

        # Test wall follow transition
        controller.set_state(RobotState.WALL_FOLLOW)
        print(f"After wall follow transition: {controller.get_current_state().value}")

        # Test triangle transition
        controller.set_state(RobotState.DRAW_TRIANGLE)
        print(f"After triangle transition: {controller.get_current_state().value}")

        # Test emergency stop
        controller.set_state(RobotState.EMERGENCY_STOP)
        print(f"After emergency stop: {controller.get_current_state().value}")

        # Test return to idle
        controller.set_state(RobotState.IDLE)
        print(f"After return to idle: {controller.get_current_state().value}")

        print("\nAll state transitions successful!")

    except Exception as e:
        print(f"Error during testing: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    test_fsm()
