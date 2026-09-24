#!/usr/bin/env python3
"""Assemble categorized sources into the existing AGX runtime layout.

This command only writes a new directory. It never installs files, imports robot
code, connects to hardware, or starts/stops any services.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath


REPO = Path(__file__).resolve().parents[1]
MANIFEST = REPO / 'tools' / 'rokae_bundle_manifest.json'


def relative_path(value: str) -> PurePosixPath:
    path = PurePosixPath(value)
    if not value or '\\' in value or ':' in value or path.is_absolute() or '..' in path.parts or str(path) == '.':
        raise ValueError(f'Expected a safe relative path: {value!r}')
    return path


def build(output: Path, *, verify_import: bool = False, repo: Path = REPO,
          manifest_path: Path = MANIFEST) -> dict:
    repo = repo.resolve()
    output = output.resolve()
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite an existing directory: {output}')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest['schema_version'] != 1:
        raise ValueError('Unsupported manifest version')
    prepared, destinations = [], set()
    for entry in manifest['entries']:
        source_rel = relative_path(entry['source'])
        destination = relative_path(entry['destination'])
        source = (repo / source_rel).resolve()
        if not source.is_relative_to(repo) or not source.is_file():
            raise ValueError(f'Source must be a file inside the repository: {source_rel}')
        if output.is_relative_to(repo / source_rel.parts[0]):
            raise ValueError('Output must not be inside a source module')
        key = str(destination).casefold()
        if key in destinations or key == 'bundle_manifest.json':
            raise ValueError(f'Duplicate/reserved destination: {destination}')
        destinations.add(key)
        data = source.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        if verify_import and digest != entry['imported_sha256']:
            raise ValueError(f'Source differs from the imported AGX snapshot: {source_rel}')
        mode = int(entry.get('mode', '0o644'), 8) & 0o777
        prepared.append((entry, destination, data, digest, mode))
    # Validate all paths and source bytes before creating anything.
    output.mkdir(parents=True, exist_ok=False)
    records = []
    for entry, destination, data, digest, mode in prepared:
        target = output / destination
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        target.chmod(mode)
        records.append({'source': entry['source'], 'path': str(destination),
                        'sha256': digest, 'size': len(data)})
    result = {'schema_version': 1, 'source_snapshot': manifest['captured_at'],
              'file_count': len(records), 'files': records,
              'deployment_files_not_included': manifest['excluded']}
    (output / 'BUNDLE_MANIFEST.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=REPO / '.build' / 'rokae_web_control')
    parser.add_argument('--verify-import', action='store_true',
                        help='Additionally require byte-identical initial AGX source files.')
    args = parser.parse_args()
    result = build(args.output, verify_import=args.verify_import)
    print(json.dumps({'output': str(args.output.resolve()), 'file_count': result['file_count']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
