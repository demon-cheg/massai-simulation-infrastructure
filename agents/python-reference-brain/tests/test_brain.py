from __future__ import annotations

import unittest

from networked_simulation_agent.brain import ReferenceBrain
from networked_simulation_agent.harness import KinematicHarness, SimulatedTarget
from networked_simulation_agent.model import (
    MissionCommand,
    MissionState,
    MissionType,
    SensorPacket,
    TargetObservation,
    Vec3,
    VehicleState,
)


class VisibilityTests(unittest.TestCase):
    def make_harness(self, target: Vec3, heading: float = 0.0) -> KinematicHarness:
        return KinematicHarness(
            tick_hz=20,
            spawn=Vec3(),
            heading_deg=heading,
            target=SimulatedTarget("beacon", target, "inspection_beacon"),
            camera_pitch_deg=0.0,
        )

    def test_ground_truth_adapter_hides_target_behind_camera(self) -> None:
        packet = self.make_harness(Vec3(0.0, -10.0, 0.0)).packet()
        self.assertEqual(packet.targets, ())

    def test_ground_truth_adapter_emits_only_image_space_observation(self) -> None:
        packet = self.make_harness(Vec3(2.0, 10.0, 0.0)).packet()
        self.assertEqual(len(packet.targets), 1)
        observation = packet.targets[0]
        self.assertGreater(observation.center_x_normalized, 0.0)
        self.assertLessEqual(observation.center_x_normalized, 1.0)
        self.assertEqual(observation.estimated_position_enu_m, Vec3(2.0, 10.0, 0.0))


class MissionTests(unittest.TestCase):
    def test_takeoff_reaches_requested_altitude(self) -> None:
        harness = self.make_harness()
        brain = ReferenceBrain()
        brain.submit(MissionCommand("takeoff", MissionType.TAKEOFF, target_altitude_m=5.0))

        succeeded = False
        for _ in range(400):
            output = brain.step(harness.packet())
            succeeded = bool(
                output.mission_update
                and output.mission_update.state is MissionState.SUCCEEDED
            )
            harness.apply(output.setpoint)
            if succeeded:
                break

        self.assertTrue(succeeded)
        self.assertAlmostEqual(harness.position.z, 5.0, delta=0.4)

    def test_tag_requires_continuous_centered_dwell(self) -> None:
        brain = ReferenceBrain()
        brain.submit(
            MissionCommand(
                "tag",
                MissionType.TAG_OBJECT,
                target_object_id="beacon",
                desired_range_m=10.0,
                dwell_seconds=0.15,
            )
        )

        def packet(tick: int, centered: bool) -> SensorPacket:
            observation = TargetObservation(
                object_id="beacon",
                semantic_class="inspection_beacon",
                center_x_normalized=0.0 if centered else 0.8,
                center_y_normalized=0.0,
                width_normalized=0.1,
                height_normalized=0.1,
                simulated_range_m=10.0,
            )
            return SensorPacket(
                sequence=tick,
                simulation_tick=tick,
                simulation_time_s=tick * 0.05,
                dt_s=0.05,
                vehicle=VehicleState(Vec3(0.0, 0.0, 10.0), Vec3(), 0.0, False),
                targets=(observation,),
            )

        brain.step(packet(0, True))
        brain.step(packet(1, True))
        brain.step(packet(2, False))
        self.assertFalse(brain.idle)
        brain.step(packet(3, True))
        brain.step(packet(4, True))
        output = brain.step(packet(5, True))
        self.assertEqual(output.events, ("object.tagged:beacon",))
        self.assertTrue(brain.idle)

    @staticmethod
    def make_harness() -> KinematicHarness:
        return KinematicHarness(
            tick_hz=20,
            spawn=Vec3(),
            heading_deg=0.0,
            target=SimulatedTarget("beacon", Vec3(0.0, 20.0, 0.0), "inspection_beacon"),
        )


if __name__ == "__main__":
    unittest.main()
