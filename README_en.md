# Astra · ROS 2 autonomous flight simulation

This branch builds on [ShuWei-2024/XT-Drone2](https://github.com/ShuWei-2024/XT-Drone2) with ROS 2 Jazzy, Gazebo Harmonic and pinned PX4 v1.17.0. It integrates simulated LiDAR/RGB/IMU, FAST-LIVO2, guarded local mapping, EGO-Planner and a guarded Offboard execution bridge. Optional External Vision fuses XY/Z position; visual yaw and velocity fusion remain disabled.

The canonical documentation is maintained in Chinese:

- [Project overview](README.md)
- [Documentation index](docs/README.md)
- [Installation and quick start](docs/QUICKSTART.md)
- [Architecture and interfaces](docs/ARCHITECTURE.md)
- [Operations and troubleshooting](docs/OPERATIONS.md)
- [Development and licensing](docs/DEVELOPMENT.md)
- [Project context for another developer or AI](docs/PROJECT_CONTEXT.md)
- [Validation index](astra/validation/README.md)

Selected indoor and outdoor closed-loop routes have passed. Latest testing includes one fully audited 64.91 m outdoor route, but same-configuration repetition, all-region drift, long-duration mapping memory and hardware flight remain unverified. Read the validation reports before interpreting a completed mission as certification.

Original framework attribution is preserved in [the upstream introduction](docs/README.upstream.md). Framework and bundled modules/assets retain their respective licenses.
