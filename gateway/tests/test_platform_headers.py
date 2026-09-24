import os
import sys
import unittest


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from gateway.platform_headers import json_request_headers


class TestPlatformHeaders(unittest.TestCase):
    def test_json_only_when_platform_api_is_missing(self) -> None:
        self.assertEqual(
            json_request_headers({}),
            {"Content-Type": "application/json"},
        )

    def test_adds_resource_and_token_from_config(self) -> None:
        self.assertEqual(
            json_request_headers(
                {
                    "platform_api": {
                        "resource": "robot_dog_service",
                        "resource_token": "test-resource-token",
                    }
                }
            ),
            {
                "Content-Type": "application/json",
                "resource": "robot_dog_service",
                "resource-token": "test-resource-token",
            },
        )

    def test_accepts_hyphenated_token_key(self) -> None:
        headers = json_request_headers(
            {
                "platform_api": {
                    "resource": "robot_dog_service",
                    "resource-token": "hyphen-token",
                }
            }
        )
        self.assertEqual(headers["resource-token"], "hyphen-token")

    def test_accepts_legacy_map_auth_config(self) -> None:
        self.assertEqual(
            json_request_headers(
                {
                    "map_api_auth": {
                        "resource": "robot-map-service",
                        "resource_token": "legacy-token",
                    }
                }
            ),
            {
                "Content-Type": "application/json",
                "resource": "robot-map-service",
                "resource-token": "legacy-token",
            },
        )

    def test_reads_nested_map_sync_platform_api(self) -> None:
        headers = json_request_headers(
            {
                "map_sync": {
                    "platform_api": {
                        "resource": "robot_dog_service",
                        "resource_token": "nested-token",
                    }
                }
            }
        )
        self.assertEqual(headers["resource"], "robot_dog_service")
        self.assertEqual(headers["resource-token"], "nested-token")

    def test_platform_api_takes_precedence_over_legacy_config(self) -> None:
        headers = json_request_headers(
            {
                "platform_api": {
                    "resource": "robot_dog_service",
                    "resource_token": "current-token",
                },
                "map_api_auth": {
                    "resource": "robot-map-service",
                    "resource_token": "legacy-token",
                },
            }
        )
        self.assertEqual(headers["resource"], "robot_dog_service")
        self.assertEqual(headers["resource-token"], "current-token")

    def test_ignores_blank_resource_and_token(self) -> None:
        self.assertEqual(
            json_request_headers(
                {"platform_api": {"resource": "  ", "resource_token": ""}}
            ),
            {"Content-Type": "application/json"},
        )

    def test_rejects_partial_or_unsafe_auth(self) -> None:
        for config_key in ("platform_api", "map_api_auth"):
            for auth in (
                {"resource": "robot_dog_service"},
                {"resource_token": "test-resource-token"},
                {
                    "resource": "robot_dog_service",
                    "resource_token": "bad\nvalue",
                },
            ):
                with self.subTest(config_key=config_key, auth=auth):
                    with self.assertRaises(ValueError):
                        json_request_headers({config_key: auth})

    def test_rejects_non_object_auth(self) -> None:
        for config_key in ("platform_api", "map_api_auth"):
            with self.subTest(config_key=config_key):
                with self.assertRaises(ValueError):
                    json_request_headers({config_key: "invalid"})


if __name__ == "__main__":
    unittest.main()
