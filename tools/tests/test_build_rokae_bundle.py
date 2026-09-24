import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('builder', Path(__file__).resolve().parents[1] / 'build_rokae_bundle.py')
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo = self.root / 'repo'
        self.repo.mkdir()
        source = self.repo / 'camera' / 'driver.py'
        source.parent.mkdir()
        source.write_bytes(b'# sample\n')
        self.manifest = self.repo / 'manifest.json'
        self.entry = {'source': 'camera/driver.py', 'destination': 'driver.py',
                      'imported_sha256': hashlib.sha256(source.read_bytes()).hexdigest(), 'mode': '0o755'}
        self.output = self.root / 'bundle'

    def write_manifest(self, entries):
        self.manifest.write_text(json.dumps({'schema_version': 1, 'captured_at': 'test',
                                            'entries': entries, 'excluded': []}))

    def build(self, **kwargs):
        return builder.build(self.output, repo=self.repo, manifest_path=self.manifest, **kwargs)

    def test_round_trip_and_no_overwrite(self):
        self.write_manifest([self.entry])
        self.assertEqual(self.build(verify_import=True)['file_count'], 1)
        self.assertEqual((self.output / 'driver.py').read_bytes(), b'# sample\n')
        (self.output / 'keep.txt').write_text('keep')
        with self.assertRaises(FileExistsError):
            self.build()
        self.assertEqual((self.output / 'keep.txt').read_text(), 'keep')

    def test_modified_sources_require_opt_out_of_import_verification(self):
        self.write_manifest([self.entry])
        (self.repo / 'camera' / 'driver.py').write_bytes(b'# edited\n')
        with self.assertRaises(ValueError):
            self.build(verify_import=True)
        self.assertFalse(self.output.exists())
        self.build()
        self.assertEqual((self.output / 'driver.py').read_bytes(), b'# edited\n')

    def test_rejects_source_and_destination_escape_before_writing(self):
        for field in ('source', 'destination'):
            for path in ('../outside', '/tmp/outside', 'C:/outside', '..\\outside'):
                with self.subTest(field=field, path=path):
                    self.write_manifest([dict(self.entry, **{field: path})])
                    with self.assertRaises(ValueError):
                        self.build()
                    self.assertFalse(self.output.exists())

    def test_duplicate_targets_and_missing_sources_fail_before_writing(self):
        for entries in ([self.entry, self.entry], [dict(self.entry, source='camera/missing.py')]):
            self.write_manifest(entries)
            with self.assertRaises(ValueError):
                self.build()
            self.assertFalse(self.output.exists())

    def test_output_cannot_be_inside_source_module(self):
        self.write_manifest([self.entry])
        self.output = self.repo / 'camera' / 'build'
        with self.assertRaises(ValueError):
            self.build()
        self.assertFalse(self.output.exists())


if __name__ == '__main__':
    unittest.main()
