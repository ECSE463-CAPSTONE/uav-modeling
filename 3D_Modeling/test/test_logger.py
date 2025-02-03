import unittest
from utilities.logger import log, consolidate_logs

def dummy_function():
    a = 10
    b = 20
    c = a + b
    return log([a, b, c])

class TestLogger(unittest.TestCase):
    def test_log_function(self):
        """Test if the logger correctly logs variable names and values."""
        logged_data = dummy_function()
        expected_data = {"a": 10, "b": 20, "c": 30}
        self.assertEqual(logged_data, expected_data)
    
    def test_consolidate_logs(self):
        """Test if logs are correctly consolidated over multiple timesteps."""
        logs = {
            0: {"V": 5, "Re": 1000},
            1: {"V": 6, "Re": 1200},
            2: {"V": 7, "Re": 1400}
        }
        consolidated = consolidate_logs(logs)
        expected_consolidation = {"V": [5, 6, 7], "Re": [1000, 1200, 1400]}
        self.assertEqual(consolidated, expected_consolidation)

if __name__ == "__main__":
    unittest.main()
