import os
import unittest

from database.load_imu_data import imu_data_df_to_imu_calibration_data, load_imu_data_df


class TestDatabaseImuCalibrationData(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_imu_data_df_to_imu_calibration_data(self):
        df = load_imu_data_df(self.db_path)
        data = imu_data_df_to_imu_calibration_data(df)

        self.assertEqual(len(data.keys()), 1)
        # At this time it only contains the raw measurement data (imu_measurement) and nothing else
        self.assertEqual(len(data["/imu0"]), 1)
        self.assertEqual(len(data["/imu0"]["frames"]), 8770)

        # Check one random imu measurement
        self.assertListEqual(
            data["/imu0"]["frames"][1520528314236489759]["imu_measurement"],
            [
                0.1360140701,
                0.070485813,
                -0.1766831676,
                -0.4123734266,
                -0.4820176527,
                10.2490537933,
            ],
        )


if __name__ == "__main__":
    unittest.main()
