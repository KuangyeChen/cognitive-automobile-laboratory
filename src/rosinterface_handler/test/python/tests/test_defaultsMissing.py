import unittest
from rosinterface_handler.interface.DefaultsMissingInterface import DefaultsMissingInterface


class TestDefaultsMissingInterface(unittest.TestCase):
    def test_defaults_missing(self):
        with self.assertRaises(KeyError):
            params = DefaultsMissingInterface()
