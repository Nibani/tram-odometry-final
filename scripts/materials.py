"""Verify organizer release assets and safely unpack the supplied nested ZIPs."""
from pathlib import Path, PurePosixPath
import argparse, hashlib, json, stat, zipfile

ROOT = Path(__file__).resolve().parents[1]

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

def unpack(archive, destination):
    destination.mkdir(parents=True, exist_ok=False)
    base = destination.resolve()
    with zipfile.ZipFile(archive) as zipped:
        members = zipped.infolist()
        for member in members:
            name = PurePosixPath(member.filename.replace('\\', '/'))
            target = (base / str(name)).resolve()
            if name.is_absolute() or '..' in name.parts or ':' in str(name) or not target.is_relative_to(base):
                raise ValueError('Unsafe archive path: ' + member.filename)
            if stat.S_ISLNK(member.external_attr >> 16):
                raise ValueError('Symbolic links are not accepted: ' + member.filename)
        zipped.extractall(base)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['verify', 'extract'])
    parser.add_argument('--dir', type=Path, default=ROOT / 'data/raw')
    parser.add_argument('--out', type=Path, default=ROOT / 'data/unpacked')
    args = parser.parse_args()
    manifest = json.loads((ROOT / 'data/assets.json').read_text(encoding='utf-8'))
    for item in manifest['assets']:
        path = args.dir / item['asset']
        if not path.is_file():
            raise SystemExit('Missing ' + str(path) + '; download release assets as described in data/README.md')
        if path.stat().st_size != item['bytes'] or digest(path) != item['sha256']:
            raise SystemExit('Archive checksum mismatch: ' + str(path))
        print('VERIFIED', item['asset'], flush=True)
    if args.action == 'extract':
        if args.out.exists():
            raise SystemExit('Choose a new --out directory; existing outputs are not overwritten.')
        for item in manifest['assets']:
            target = args.out / item['id']
            unpack(args.dir / item['asset'], target)
            for depth in range(4):
                nested = [p for p in target.rglob('*.zip') if p.name in ('dataset.zip', 'data.zip') and not p.with_suffix('').exists()]
                if not nested:
                    break
                for path in nested:
                    unpack(path, path.with_suffix(''))
            bags = sorted(target.rglob('*.db3'))
            print(json.dumps({'id': item['id'], 'bag_count': len(bags), 'bag_parent_directories': sorted({str(p.parent.parent.resolve()) for p in bags})}, ensure_ascii=False))
    print('PASS')

if __name__ == '__main__':
    main()
