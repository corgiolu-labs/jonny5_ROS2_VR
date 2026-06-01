"""
IMU signal processing, validation, and diagnostic analytics (self-test support, not WS routing).

Includes optional CLIs:
- ``python -m controller.imu_analytics.validate_imu_vs_ee``
- ``python -m controller.imu_analytics.run_real_imu_ee_validation``
- ``python -m controller.imu_analytics.test_roll_direction_consistency``

Run them from ``raspberry5/`` with ``PYTHONPATH`` pointing at that root.
"""
