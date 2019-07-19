from gnss_coordinate_transform import LocalGeographicCS, DoublePair
import unittest


class LocalGeographicCSTest(unittest.TestCase):
    def test(self):
        cs = LocalGeographicCS(49.01439, 8.41722)
        x = 0.
        y = 0.
        dp = DoublePair
        dp = cs.xy2ll(x, y)
        lat = dp.first
        lon = dp.second
        self.assertAlmostEqual(lat, 49.01439)
        self.assertAlmostEqual(lon, 8.41722)

if __name__ == '__main__':
    unittest.main()
