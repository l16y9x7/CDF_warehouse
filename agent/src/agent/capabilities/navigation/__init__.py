from .contract import NAVIGATION_TARGETS, NavigationCapability
from .http_adapter import HttpNavigationCapability
from .mock import MockNavigationCapability

# Short aliases retained for existing skills.
Navigation = NavigationCapability
MockNavigation = MockNavigationCapability

__all__ = [
    "NAVIGATION_TARGETS",
    "HttpNavigationCapability",
    "MockNavigation",
    "MockNavigationCapability",
    "Navigation",
    "NavigationCapability",
]
