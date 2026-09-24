import importlib.util
import unittest
from pathlib import Path
p=Path(__file__).resolve().parents[1] / 'config/ros/domain.py'
s=importlib.util.spec_from_file_location('domain',p); m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
def addresses(*ips):
 return [{'addr_info':[{'family':'inet','local':ip} for ip in ips]}]
class Domain(unittest.TestCase):
 def test_select_robot_network(self):
  self.assertEqual(m.resolve_domain(addresses('127.0.0.1','192.168.71.51','10.0.0.9'),'192.168.71.0/24'),51)
 def test_reject_missing_ambiguous_and_outside_range(self):
  for ips in [[],['192.168.71.51','192.168.71.52'],['192.168.71.0'],['192.168.71.233']]:
   with self.assertRaises(ValueError):m.resolve_domain(addresses(*ips),'192.168.71.0/24')
 def test_explicit(self):
  self.assertEqual(m.resolve_domain([], '192.168.71.0/24','51'),51)
  self.assertEqual(m.resolve_domain([], '192.168.71.0/24','105'),105)
  self.assertEqual(m.resolve_domain([], '192.168.71.0/24','135'),135)
  for value in ['0','233','bad']:
   with self.assertRaises(ValueError):m.resolve_domain([], '192.168.71.0/24',value)
if __name__ == '__main__':
 unittest.main()
