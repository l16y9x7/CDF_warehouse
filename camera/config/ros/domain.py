"""Resolve the robot domain at process startup; stdout is only the domain ID."""
import ipaddress
import json
import os
import subprocess


def resolve_domain(addresses, network, explicit="auto"):
    if explicit != "auto":
        value = int(explicit)
    else:
        subnet = ipaddress.IPv4Network(network)
        candidates = {ipaddress.IPv4Address(item["local"])
                      for interface in addresses for item in interface.get("addr_info", [])
                      if item.get("family") == "inet"
                      and ipaddress.IPv4Address(item["local"]) in subnet}
        if len(candidates) != 1:
            raise ValueError(f"Expected one robot IPv4 address in {network}; found {len(candidates)}")
        value = int(str(candidates.pop()).rsplit(".", 1)[1])
    # ROS 2 DDS domain IDs are 0..232. Keep 0 reserved; allow host overrides
    # such as 105/135 from /etc (not from vision-tracked config).
    if not 1 <= value <= 232:
        raise ValueError("Choose a unique ROS domain in 1..232")
    return value


if __name__ == "__main__":
    explicit = os.environ.get("VISION_ROS_DOMAIN_ID", "auto")
    addresses = json.loads(subprocess.check_output(["ip", "-j", "-4", "addr", "show"], text=True)) if explicit == "auto" else []
    print(resolve_domain(addresses, os.environ["VISION_ROS_NETWORK"], explicit))
