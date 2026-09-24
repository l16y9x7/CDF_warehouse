"""Resolve Agent target IDs to navigation station IDs."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

_PRODUCT_SLOT_PATTERN = re.compile(
    r"^H(?P<shelf>[12])_(?P<face>[FB])_L(?P<level>[1-5])_C(?P<column>\d{2})$"
)


class TargetResolutionError(ValueError):
    pass


@dataclass(frozen=True)
class FaceMapping:
    level_column_counts: Mapping[str, int]
    station_ids: tuple[str, ...]
    # Two-point faces: explicit slots for station_ids[0]; remaining columns → [1].
    # Four-point faces leave this empty and use equal-segment centers.
    point_1_slots: frozenset[str]


class TargetResolver:
    def __init__(
        self,
        *,
        station_ids: set[str],
        aliases: Mapping[str, str],
        faces: Mapping[str, FaceMapping],
    ) -> None:
        self._station_ids = frozenset(station_ids)
        self._aliases = dict(aliases)
        self._faces = dict(faces)
        self._validate()

    def _validate(self) -> None:
        for alias, station_id in self._aliases.items():
            if not alias or station_id not in self._station_ids:
                raise ValueError(
                    f"invalid target alias {alias!r} -> {station_id!r}"
                )
        for face_id, mapping in self._faces.items():
            expected_levels = {f"L{level}" for level in range(1, 6)}
            if set(mapping.level_column_counts) != expected_levels:
                raise ValueError(
                    f"{face_id}.levels must define L1 through L5"
                )
            if any(
                count < 1 for count in mapping.level_column_counts.values()
            ):
                raise ValueError(
                    f"{face_id}.levels column counts must be positive"
                )

            station_count = len(mapping.station_ids)
            if station_count == 2:
                if not mapping.point_1_slots:
                    raise ValueError(
                        f"{face_id}.point_1_slots must list point-1 product slots"
                    )
                covered_levels: set[str] = set()
                for slot in mapping.point_1_slots:
                    match = _PRODUCT_SLOT_PATTERN.fullmatch(slot)
                    if match is None:
                        raise ValueError(
                            f"{face_id}.point_1_slots has invalid slot {slot!r}"
                        )
                    slot_face = f"H{match.group('shelf')}_{match.group('face')}"
                    if slot_face != face_id:
                        raise ValueError(
                            f"{face_id}.point_1_slots contains foreign slot {slot!r}"
                        )
                    level_id = f"L{match.group('level')}"
                    column = int(match.group("column"))
                    column_count = mapping.level_column_counts[level_id]
                    if column < 1 or column > column_count:
                        raise ValueError(
                            f"{face_id}.point_1_slots {slot} is outside "
                            f"1..{column_count}"
                        )
                    covered_levels.add(level_id)
                if covered_levels != expected_levels:
                    missing = sorted(expected_levels - covered_levels)
                    raise ValueError(
                        f"{face_id}.point_1_slots missing levels: {missing}"
                    )
                for level_id, column_count in mapping.level_column_counts.items():
                    point_1_count = 0
                    for slot in mapping.point_1_slots:
                        m = _PRODUCT_SLOT_PATTERN.fullmatch(slot)
                        assert m is not None
                        if f"L{m.group('level')}" == level_id:
                            point_1_count += 1
                    if point_1_count < 1 or point_1_count >= column_count:
                        raise ValueError(
                            f"{face_id}.{level_id} point_1_slots must leave "
                            f"at least one column for point 2"
                        )
            elif station_count == 4:
                if mapping.point_1_slots:
                    raise ValueError(
                        f"{face_id} four-point faces must not set point_1_slots"
                    )
            else:
                raise ValueError(
                    f"{face_id} must contain two or four station IDs"
                )

            unknown = [
                station_id
                for station_id in mapping.station_ids
                if station_id not in self._station_ids
            ]
            if unknown:
                raise ValueError(f"{face_id} contains unknown stations: {unknown}")

    def resolve(self, target_id: str) -> str:
        target_id = str(target_id or "").strip()
        if target_id in self._station_ids:
            return target_id
        if target_id in self._aliases:
            return self._aliases[target_id]

        match = _PRODUCT_SLOT_PATTERN.fullmatch(target_id)
        if match is None:
            raise TargetResolutionError(f"invalid target_id: {target_id!r}")

        face_id = f"H{match.group('shelf')}_{match.group('face')}"
        mapping = self._faces.get(face_id)
        if mapping is None:
            raise TargetResolutionError(f"no navigation mapping for {face_id}")

        level_id = f"L{match.group('level')}"
        column_count = mapping.level_column_counts.get(level_id)
        if column_count is None:
            raise TargetResolutionError(
                f"no navigation mapping for {face_id}_{level_id}"
            )
        column = int(match.group("column"))
        if column < 1 or column > column_count:
            raise TargetResolutionError(
                f"{target_id} column is outside 1..{column_count}"
            )

        if len(mapping.station_ids) == 2:
            return mapping.station_ids[
                0 if target_id in mapping.point_1_slots else 1
            ]

        # Four equal segments: nearest center; boundary ties pick lower segment.
        segment_number = (4 * column + column_count - 3) // column_count
        segment_index = min(3, max(0, segment_number - 1))
        return mapping.station_ids[segment_index]


def _load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def load_target_resolver(
    mapping_path: str | Path,
    stations_path: str | Path,
) -> TargetResolver:
    mapping_data = _load_yaml_mapping(mapping_path)
    station_data = _load_yaml_mapping(stations_path)
    raw_stations = station_data.get("stations") or {}
    if not isinstance(raw_stations, dict):
        raise ValueError(f"stations must be a mapping: {stations_path}")

    raw_aliases = mapping_data.get("aliases") or {}
    raw_faces = mapping_data.get("faces") or {}
    if not isinstance(raw_aliases, dict) or not isinstance(raw_faces, dict):
        raise ValueError("aliases and faces must be mappings")

    faces: dict[str, FaceMapping] = {}
    for face_id, raw in raw_faces.items():
        if not isinstance(raw, dict):
            raise ValueError(f"{face_id} must be a mapping")
        raw_station_ids = raw.get("station_ids") or []
        if not isinstance(raw_station_ids, list) or len(raw_station_ids) not in (
            2,
            4,
        ):
            raise ValueError(
                f"{face_id}.station_ids must contain two or four items"
            )
        raw_levels = raw.get("levels") or {}
        if not isinstance(raw_levels, dict):
            raise ValueError(f"{face_id}.levels must be a mapping")
        raw_point_1_slots = raw.get("point_1_slots") or []
        if raw_point_1_slots and not isinstance(raw_point_1_slots, list):
            raise ValueError(f"{face_id}.point_1_slots must be a list")
        if len(raw_station_ids) == 2 and not raw_point_1_slots:
            raise ValueError(
                f"{face_id}.point_1_slots is required for two-point faces"
            )
        faces[str(face_id)] = FaceMapping(
            level_column_counts={
                str(level_id): int(column_count)
                for level_id, column_count in raw_levels.items()
            },
            station_ids=tuple(str(item) for item in raw_station_ids),
            point_1_slots=frozenset(str(item) for item in raw_point_1_slots),
        )

    return TargetResolver(
        station_ids={str(station_id) for station_id in raw_stations},
        aliases={
            str(alias): str(station_id)
            for alias, station_id in raw_aliases.items()
        },
        faces=faces,
    )
