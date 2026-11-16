#!/usr/bin/env python3
"""Explicit local firmware/metadata exports. No install, upload or sibling writes."""
import argparse
import gzip
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SAFE = re.compile(r'^[A-Za-z][A-Za-z0-9_-]*$')
HEX = re.compile(r'^[0-9a-f]{64}$')


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()


def digest(data):
    return hashlib.sha256(data).hexdigest()


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args]).decode().strip()


def clean_revision():
    if git('status', '--porcelain', '--untracked-files=normal'):
        raise ValueError('Release export requires clean committed source (including untracked files)')
    return git('rev-parse', 'HEAD')


def source_digest(path):
    # Git tracks uthash/include as a relative directory symlink. Hash the link
    # itself, as Git does, rather than reading a directory as source bytes.
    if path.is_symlink():
        return digest(b'symlink:' + os.readlink(path).encode())
    return digest(path.read_bytes())


def hashes(paths):
    return {p.relative_to(ROOT).as_posix(): source_digest(p) for p in sorted(paths)}


def tree_entries(revision):
    entries = {}
    for row in git('ls-tree', '-rz', revision).split('\0'):
        if row:
            header, name = row.split('\t', 1)
            mode, kind, checksum = header.split()
            if kind != 'blob':
                raise ValueError(f'Unsupported source tree entry: {name}')
            entries[name] = (mode, checksum)
    return entries


def assert_committed(revision, paths):
    entries = tree_entries(revision)
    for p in paths:
        name = p.relative_to(ROOT).as_posix()
        mode, expected = entries.get(name, ('', ''))
        if p.is_symlink():
            if not p.resolve().is_relative_to(ROOT):
                raise ValueError(f'Source link escapes project: {name}')
            raw = os.readlink(p).encode()
        else:
            raw = p.read_bytes()
        actual = hashlib.sha1(b'blob ' + str(len(raw)).encode() + b'\0' + raw).hexdigest()
        if actual != expected or p.is_symlink() != (mode == '120000'):
            raise ValueError(f'Source differs from committed revision: {name}')


def annotation_files():
    names = tree_entries(git('rev-parse', 'HEAD'))
    modules = sorted(p for p in names if p.startswith('src/droneModules/') and p.count('/') == 2 and Path(p).suffix in ('.h', '.cpp'))
    return [ROOT / 'src/DroneModule.h', *(ROOT / p for p in modules)]


def metadata():
    catalog = {}
    for path in annotation_files():
        text = path.read_text()
        # A source without @type was never a catalog entry (e.g. unfinished Proa).
        if not re.search(r"@type\s+", text):
            continue
        tags = {}
        pending = None
        for line_no, line in enumerate(text.splitlines(), 1):
            location = f'{path.relative_to(ROOT)}:{line_no}'
            if pending:
                if '<<<' not in line:
                    pending[1] += line + '\n'
                    continue
                name, value, location = pending
                pending = None
            else:
                match = re.search(r'@([A-Za-z][A-Za-z0-9]*)\s+(.*)', line)
                if not match:
                    continue
                name, value = match.groups()
                if value.endswith('>>>'):
                    pending = [name, value[:-3] + '\n', location]
                    continue
            value = value.strip()
            if name in ('pub', 'sub'):
                parts = value.split(';')
                if len(parts) != 6:
                    raise ValueError(f'{location}: @{name} requires six semicolon-separated fields: {value}')
                try:
                    if name == 'pub':
                        address, typ, count, writable, pname, description = parts
                        if writable not in ('r', 'w'):
                            raise ValueError('access must be r or w')
                        value = dict(address=int(address), type=typ, numValues=int(count),
                                     writeable=writable == 'w', name=pname, description=description)
                    else:
                        address, addr, typ, count, pname, description = parts
                        value = dict(address=int(address), addrAddress=int(addr), type=typ,
                                     numValues=int(count), name=pname, description=description)
                    if not 0 <= value['address'] <= 255 or not 1 <= value['numValues'] <= 16 or not pname or not description or typ not in ('u8', 'u32', 'f', 'c', 'addr'):
                        raise ValueError('invalid address, count or empty required field')
                    if name == 'sub' and not 0 <= value['addrAddress'] <= 255:
                        raise ValueError('invalid subscription address')
                except ValueError as error:
                    raise ValueError(f'{location}: invalid @{name}: {error}') from error
            elif name == 'default':
                parts = value.split('=')
                if len(parts) != 2 or not parts[0].strip() or not parts[1].strip():
                    raise ValueError(f'{location}: invalid @default')
                value = dict(param=parts[0].strip(), values=[v.strip() for v in parts[1].split(',')])
            tags.setdefault(name, []).append(value)
        if pending:
            raise ValueError(f'{pending[2]}: unterminated multiline annotation')
        # Implementation files without a module declaration are not catalog entries,
        # matching the legacy scanner. Their tagged parameter fields still validate.
        if 'type' not in tags:
            continue
        if len(tags['type']) != 1 or not SAFE.fullmatch(tags['type'][0]):
            raise ValueError(f'{path}: ambiguous or unsafe @type')
        kind = tags['type'][0]
        tags['filename'] = [path.relative_to(ROOT).as_posix()]
        if kind in catalog:
            for name, values in tags.items():
                if name != 'type':
                    catalog[kind].setdefault(name, []).extend(values)
        else:
            catalog[kind] = tags
    return validate_catalog(catalog)


def validate_catalog(catalog):
    if not isinstance(catalog, dict) or not catalog:
        raise ValueError("Invalid catalog")
    for kind, tags in catalog.items():
        if not SAFE.fullmatch(kind) or kind in ("constructor", "prototype", "hasOwnProperty") or not isinstance(tags, dict) or tags.get("type") != [kind] or not tags.get("filename") or not all(path_ok(p) for p in tags["filename"]):
            raise ValueError(f"Invalid catalog entry: {kind}")
        for name in ('pub', 'sub'):
            entries = tags.get(name, [])
            if not isinstance(entries, list) or any(not isinstance(e, dict) or not isinstance(e.get("address"), int) or not 0 <= e["address"] <= 255 or not isinstance(e.get("numValues"), int) or not 1 <= e["numValues"] <= 16 or not all(isinstance(e.get(f), str) and e[f] for f in ("name", "description", "type")) or e["type"] not in ("u8", "u32", "f", "c", "addr") or (not isinstance(e.get("writeable"), bool) if name == "pub" else not isinstance(e.get("addrAddress"), int) or not 0 <= e["addrAddress"] <= 255) for e in entries):
                raise ValueError(f"{kind}: malformed @{name}")
            if len({e['address'] for e in entries}) != len(entries) or len({e['name'] for e in entries}) != len(entries):
                raise ValueError(f'{kind}: ambiguous duplicate @{name} address/name')
        for parent in tags.get('inherits', []):
            if parent not in catalog or parent == kind:
                raise ValueError(f'{kind}: invalid parent {parent}')
        visited = {kind}
        parent = tags
        while parent.get('inherits'):
            parents = parent['inherits']
            if len(parents) != 1 or parents[0] in visited:
                raise ValueError(f'{kind}: ambiguous/cyclic inheritance')
            visited.add(parents[0])
            parent = catalog[parents[0]]
    return catalog


def path_ok(name):
    p = PurePosixPath(name)
    return bool(name) and not p.is_absolute() and '\\' not in name and str(p) == name and all(x not in ('', '.', '..') for x in p.parts)


def verify_archive(path):
    path = Path(path)
    raw = path.read_bytes()
    if Path(str(path) + '.sha256').read_text().split()[0] != digest(raw):
        raise ValueError('Archive checksum mismatch')
    files = {}
    with tarfile.open(fileobj=io.BytesIO(raw), mode='r:gz') as archive:
        for member in archive:
            if not member.isfile() or not path_ok(member.name) or member.name in files or member.size > 64 * 1024 * 1024:
                raise ValueError('Unsafe, duplicate or oversized archive member')
            files[member.name] = archive.extractfile(member).read()
            if sum(map(len, files.values())) > 128 * 1024 * 1024:
                raise ValueError('Export is too large')
    manifest = json.loads(files.pop('release-manifest.json'))
    identity = manifest.pop('identity')
    if identity != digest(canonical(manifest)):
        raise ValueError('Manifest identity mismatch')
    manifest['identity'] = identity
    if manifest['schemaVersion'] != 1 or manifest['kind'] not in ('metadata', 'firmware'):
        raise ValueError('Unsupported export kind/schema')
    if not re.fullmatch(r'[0-9a-f]{40}', manifest['sourceRevision']) or not manifest['sourceFiles'] or any(not path_ok(p) or not HEX.fullmatch(h) for p, h in manifest['sourceFiles'].items()):
        raise ValueError('Invalid source provenance')
    expected = {'moduleInfo.json'} if manifest['kind'] == 'metadata' else {'firmware.bin', 'firmware.ver'}
    if set(files) != expected or manifest['files'] != {p: digest(v) for p, v in sorted(files.items())}:
        raise ValueError('Payload file set/hash mismatch')
    if manifest['kind'] == 'metadata':
        validate_catalog(json.loads(files['moduleInfo.json']))
    elif not HEX.fullmatch(manifest['metadataIdentity']) or manifest['environment'] != 'esp32doit-devkit-v1' or manifest['board'] != 'esp32doit-devkit-v1' or not files['firmware.bin'] or not re.fullmatch(r'[0-9a-f]{4,40}', manifest['buildMarker']) or not manifest['sourceRevision'].startswith(manifest['buildMarker']) or files['firmware.ver'].decode() != manifest['buildMarker']:
        raise ValueError('Invalid firmware metadata/environment/version')
    return manifest, files


def publish(kind, revision, source_files, payload, output, **extra):
    manifest = dict(schemaVersion=1, kind=kind, sourceRevision=revision, sourceFiles=source_files,
                    files={p: digest(v) for p, v in sorted(payload.items())}, **extra)
    manifest['identity'] = digest(canonical(manifest))
    files = {**payload, 'release-manifest.json': canonical(manifest)}
    buffer = io.BytesIO()
    with gzip.GzipFile(fileobj=buffer, mode='wb', mtime=0, filename='') as zipped:
        with tarfile.open(fileobj=zipped, mode='w', format=tarfile.USTAR_FORMAT) as archive:
            for name, data in sorted(files.items()):
                info = tarfile.TarInfo(name)
                info.size, info.mode, info.mtime = len(data), 0o644, 0
                archive.addfile(info, io.BytesIO(data))
    raw = buffer.getvalue()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    destination = output / f'dronelink-{kind}-{revision}-{manifest["identity"]}.tgz'
    for path, data in ((destination, raw), (Path(str(destination) + '.sha256'), (digest(raw) + '  ' + destination.name + '\n').encode())):
        # Write complete private files before atomic no-overwrite publication.
        with tempfile.NamedTemporaryFile(dir=output, delete=False) as staged:
            staged.write(data)
            staged.flush()
            os.fsync(staged.fileno())
            temporary = Path(staged.name)
        try:
            try:
                os.link(temporary, path)
            except FileExistsError:
                if path.is_symlink() or path.read_bytes() != data:
                    raise ValueError(f'Release identity already exists with different bytes: {path}')
        finally:
            temporary.unlink()
    verify_archive(destination)
    return destination


def export_metadata(output):
    revision = clean_revision()
    paths = [*annotation_files(), Path(__file__).resolve()]
    assert_committed(revision, paths)
    inputs = hashes(paths)
    catalog = metadata()
    if revision != clean_revision() or inputs != hashes([*annotation_files(), Path(__file__).resolve()]):
        raise ValueError('Source changed during metadata export')
    assert_committed(revision, paths)
    return publish('metadata', revision, inputs, {'moduleInfo.json': canonical(catalog)}, output)


def export_firmware(environment, metadata_path, output):
    revision = clean_revision()
    manifest, payload = verify_archive(metadata_path)
    metadata_paths = [*annotation_files(), Path(__file__).resolve()]
    assert_committed(revision, metadata_paths)
    expected_inputs = hashes(metadata_paths)
    if manifest['kind'] != 'metadata' or manifest['sourceRevision'] != revision or manifest['sourceFiles'] != expected_inputs or payload['moduleInfo.json'] != canonical(metadata()):
        raise ValueError('Metadata does not match current committed firmware source')
    if environment != 'esp32doit-devkit-v1':
        raise ValueError('Unsupported firmware environment')
    # Fresh build from a frozen Git snapshot: no stale build image can be selected.
    tracked = git('ls-files', '-z').split('\0')
    paths = [ROOT / p for p in tracked if p and (p.startswith(('src/', 'lib/')) or p in ('platformio.ini', 'git_rev_macro.py', 'post_build.py', 'tools/release.py'))]
    assert_committed(revision, paths)
    inputs = hashes(paths)
    with tempfile.TemporaryDirectory(prefix='dronelink-release-') as work:
        snapshot = Path(work) / 'DroneNode'
        subprocess.run(['git', 'clone', '--quiet', '--local', '--no-hardlinks', str(ROOT), str(snapshot)], check=True)
        subprocess.run(['git', '-C', str(snapshot), 'checkout', '--quiet', '--detach', revision], check=True)
        pio = os.environ.get('DRONELINK_PIO', 'pio')
        # Dependency caches can be configured explicitly via standard PlatformIO
        # environment variables. The snapshot never gets a Server sibling.
        config = json.loads(subprocess.check_output([pio, 'project', 'config', '--json-output'], cwd=snapshot))
        effective = dict(dict(config).get('env:' + environment, []))
        board = effective.get('board')
        if board != 'esp32doit-devkit-v1':
            raise ValueError(f'Unsupported effective board: {board}')
        marker = subprocess.check_output(['git', 'log', '-n1', '--format=%h'], cwd=snapshot).decode().strip()
        subprocess.run([pio, 'run', '-e', environment], cwd=snapshot, check=True)
        image = snapshot / '.pio/build' / environment / 'firmware.bin'
        resolved = subprocess.check_output([pio, 'pkg', 'list', '-e', environment, '--verbose'], cwd=snapshot).decode().strip()
        tools = subprocess.check_output([pio, '--version']).decode().strip()
        data = image.read_bytes()
        # A build may create ignored dependency files, but must not alter inputs.
        if hashes(paths) != inputs or clean_revision() != revision:
            raise ValueError('Source changed during firmware export')
        assert_committed(revision, paths)
        for relative, checksum in inputs.items():
            if source_digest(snapshot / relative) != checksum:
                raise ValueError(f'Build changed source input: {relative}')
        return publish('firmware', revision, inputs,
                       {'firmware.bin': data, 'firmware.ver': marker.encode()}, output,
                       environment=environment, board=board, buildMarker=marker, metadataIdentity=manifest['identity'],
                       buildFlags=config, buildTools={'platformio': tools, 'resolvedPackages': resolved})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    m = commands.add_parser('export-metadata'); m.add_argument('--output-dir', required=True)
    f = commands.add_parser('export-firmware'); f.add_argument('--output-dir', required=True); f.add_argument('--metadata', required=True); f.add_argument('--environment', required=True)
    v = commands.add_parser('verify'); v.add_argument('archive')
    args = parser.parse_args()
    try:
        if args.command == 'export-metadata':
            print(export_metadata(args.output_dir))
        elif args.command == 'export-firmware':
            print(export_firmware(args.environment, args.metadata, args.output_dir))
        else:
            print(json.dumps(verify_archive(args.archive)[0], indent=2))
    except (ValueError, KeyError, OSError, subprocess.CalledProcessError, tarfile.TarError) as error:
        parser.exit(1, f'Release failed: {error}\n')


if __name__ == '__main__':
    main()
