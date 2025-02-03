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
            0: {"V": 5.2, "Re": 1200, "Cl": 0.3},
            1: {"V": 5.5, "Re": 1300},  # Missing "Cl"
            2: {"V": 5.7, "Cl": 0.34}   # Missing "Re"
        }
        
        consolidated = consolidate_logs(logs)
        expected_consolidation = {
            "timestep": [0, 1, 2],
            "V": [5.2, 5.5, 5.7],
            "Re": [1200, 1300, None],  # Fill missing with None
            "Cl": [0.3, None, 0.34]   # Fill missing with None
        }
        
        self.assertEqual(consolidated, expected_consolidation)

if __name__ == "__main__":
    unittest.main()