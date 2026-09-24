import unittest

from agent.capabilities.common import Hand
from agent.layouts.basket import (
    agv_nav_id,
    agv_side,
    basket_nav_id,
    validate_agv_position,
    validate_basket_position,
)


class LayoutTest(unittest.TestCase):
    def test_agv_navigation_depends_on_column_and_hand(self):
        self.assertEqual(agv_nav_id("1", Hand.LEFT), "AGV_C")
        self.assertEqual(agv_nav_id("1", Hand.RIGHT), "AGV_L")
        self.assertEqual(agv_nav_id("2", Hand.LEFT), "AGV_R")
        self.assertEqual(agv_nav_id("2", Hand.RIGHT), "AGV_C")
        self.assertEqual(agv_side("1"), "LEFT")
        self.assertEqual(agv_side("2"), "RIGHT")

    def test_basket_sides_use_separate_navigation_points(self):
        self.assertEqual(basket_nav_id("4"), "SORTING_BASKET_4")
        self.assertEqual(basket_nav_id("4", review=True), "REVIEW_BASKET_4")

    def test_validates_independent_rows_and_columns(self):
        validate_agv_position("L5", "2")
        validate_basket_position("L4", "5")
        for call in ((validate_agv_position, "L6", "1"), (validate_basket_position, "L1", "6")):
            with self.assertRaises(ValueError):
                call[0](call[1], call[2])


if __name__ == "__main__":
    unittest.main()
