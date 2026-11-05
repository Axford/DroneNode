import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('release', ROOT / 'tools/release.py')
release = importlib.util.module_from_spec(spec)
spec.loader.exec_module(release)


class ReleaseTests(unittest.TestCase):
    def test_complete_legacy_catalog_with_reviewed_annotation_corrections(self):
        fixture = ROOT / 'tests/fixtures/sep05-legacy-catalog.json'
        provenance = json.loads(fixture.with_suffix('.provenance.json').read_text())
        self.assertEqual(hashlib.sha256(fixture.read_bytes()).hexdigest(), provenance['fixtureSha256'])
        expected = json.loads(fixture.read_text())
        for module in expected.values():
            module['filename'] = [p.removeprefix('../DroneNode/') for p in module['filename']]
        expected['ADS1015']['pub'][-1]['name'] = 'max'
        expected['LSM9DS1']['inherits'] = ['I2CBase']
        expected['ODrive']['pub'][-1]['address'] = 20
        for name in ['PanSerial', 'PanTilt']:
            expected[name]['sub'][1] = dict(address=12, addrAddress=13, type='f', numValues=1, name='heading', description='Current heading (e.g. from Compass)')
        for entry in expected['TankSteer']['pub']:
            if entry['name'] in ['mode', 'trim', 'limits', 'threshold']:
                entry['address'] = {'mode': 10, 'trim': 11, 'limits': 13, 'threshold': 14}[entry['name']]
        self.assertEqual(release.metadata(), expected)
        self.assertEqual(len(expected), 51)

    def test_malformed_ambiguous_and_multiline_metadata(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp) / 'module.h'
            with patch.object(release, 'ROOT', Path(temp)), patch.object(release, 'annotation_files', return_value=[p]):
                for text in ['@type Test\n@sub 12;13;f;1;missing', '@type Test\n@pub nope;u8;1;w;x;desc', '@type Test\n@guide >>>\nunfinished', '@type Test\n@inherits Test', '@type ../escape', '@type Test\n@pub 1;f;1;w;x;desc\n@pub 1;f;1;w;y;desc']:
                    p.write_text(text)
                    with self.subTest(text=text), self.assertRaises(ValueError): release.metadata()
                p.write_text('@type Test\n@guide >>>\n<h2>Guide</h2>\n<<<\n@pub 1;u8;1;r;value;Description')
                self.assertEqual(release.metadata()['Test']['guide'], ['<h2>Guide</h2>'])

    def test_immutable_deterministic_archive_and_checksum_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            payload = {'moduleInfo.json': release.canonical({'Test': {'type': ['Test'], 'filename': ['src/Test.h']}})}
            first = release.publish('metadata', 'a' * 40, {'src/a.h': 'b' * 64}, payload, temp)
            before = first.read_bytes()
            self.assertEqual(release.publish('metadata', 'a' * 40, {'src/a.h': 'b' * 64}, payload, temp), first)
            self.assertEqual(first.read_bytes(), before)
            first.write_bytes(b'bad bytes')
            with self.assertRaises(ValueError): release.publish('metadata', 'a' * 40, {'src/a.h': 'b' * 64}, payload, temp)
            self.assertEqual(first.read_bytes(), b'bad bytes')
            with self.assertRaises(ValueError): release.verify_archive(first)

    def test_export_rejects_links_duplicates_traversal_and_extra_files(self):
        with tempfile.TemporaryDirectory() as temp:
            for kind in ('symlink', 'hardlink', 'duplicate', 'traversal', 'extra'):
                archive = Path(temp) / 'unsafe.tgz'
                with tarfile.open(archive, 'w:gz') as tar:
                    entry = tarfile.TarInfo('../escape' if kind == 'traversal' else 'moduleInfo.json')
                    entry.size = 2
                    if kind in ('symlink', 'hardlink'):
                        entry.type = tarfile.SYMTYPE if kind == 'symlink' else tarfile.LNKTYPE
                        entry.linkname = '/tmp/elsewhere'
                        entry.size = 0
                    tar.addfile(entry, io.BytesIO(b'{}'))
                    if kind == 'duplicate': tar.addfile(entry, io.BytesIO(b'{}'))
                Path(str(archive) + '.sha256').write_text(release.digest(archive.read_bytes()))
                with self.subTest(kind=kind), self.assertRaises((ValueError, KeyError)): release.verify_archive(archive)

    def test_dirty_source_cannot_export(self):
        with patch.object(release, 'git', return_value=' M src/module.h'):
            with self.assertRaisesRegex(ValueError, 'clean committed'): release.export_metadata('/tmp/not-published')

    def test_build_hook_has_no_upload_or_build_publication_action(self):
        actions = []
        namespace = {'Import': lambda _: None, 'env': type('Env', (), {'AddPostAction': lambda *args: actions.append(args)})()}
        exec((ROOT / 'post_build.py').read_text(), namespace)
        self.assertEqual(actions, [])


if __name__ == '__main__': unittest.main()
