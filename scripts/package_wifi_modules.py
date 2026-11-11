"""Preserve and check PC-built candidate modules; does not deploy anything."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import tarfile

root = Path(__file__).resolve().parents[1] / 'maintenance/wifi'
build = Path('/tmp/terasic-wifi-build')
dest = root / 'candidate-modules'
modules = sorted((build / 'crypto').glob('*.ko')) + [
    build / 'net/wireless/cfg80211.ko',
    build / 'net/mac80211/mac80211.ko',
    build / 'drivers/net/wireless/mediatek/mt7601u/mt7601u.ko',
]


def output(*args):
    return subprocess.check_output(args, text=True)


required, provided, manifest = set(), set(), []
for module in modules:
    relative = module.relative_to(build)
    target = dest / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(module, target)
    subprocess.run(['arm-linux-gnueabihf-strip', '--strip-debug', str(target)], check=True)
    vermagic = output('modinfo', '-k', '4.5.0', '-F', 'vermagic', str(target)).strip()
    if vermagic != '4.5.0 SMP mod_unload ARMv7 p2v8':
        raise RuntimeError(f'Unexpected vermagic: {relative}: {vermagic}')
    relocations = output('arm-linux-gnueabihf-readelf', '-rW', str(target))
    if re.search(r'R_ARM_(?:GOT\w*|BASE_PREL)', relocations):
        raise RuntimeError(f'PIC relocation in {relative}; rebuild without PIC/PIE')
    for line in output('arm-linux-gnueabihf-nm', '-u', str(target)).splitlines():
        fields = line.split()
        if fields and fields[0] == 'U':
            required.add(fields[-1])
    for line in output('arm-linux-gnueabihf-nm', '--defined-only', str(target)).splitlines():
        fields = line.split()
        if fields and fields[-1].startswith('__ksymtab_'):
            provided.add(fields[-1][len('__ksymtab_'):])
    manifest.append({'path': str(relative), 'sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                     'vermagic': vermagic})

required -= provided
names = {line.split()[2] for line in (root / 'board-kallsyms.txt').read_text().splitlines()
         if len(line.split()) >= 3}
exports = {name[len('__ksymtab_'):] for name in names if name.startswith('__ksymtab_')}
missing = sorted(required - exports)
report = {'required_symbols': len(required), 'kernel_export_records': len(exports),
          'missing_exports': missing, 'pic_relocations': False,
          'result': 'FAIL' if missing else 'PASS',
          'limitation': 'Export/metadata checks do not prove full ABI or runtime compatibility.'}
(root / 'symbol-check.json').write_text(json.dumps(report, indent=2) + '\n')
if missing:
    raise RuntimeError(f'Missing kernel exports: {missing}')
(root / 'required-kernel-symbols.txt').write_text('\n'.join(sorted(required)) + '\n')
(root / 'candidate-modules.json').write_text(json.dumps({
    'status': 'Candidates only; consult hardware-trial.log for on-board result',
    'compiler': output('arm-linux-gnueabihf-gcc', '-dumpfullversion').strip(),
    'modules': manifest}, indent=2) + '\n')
shutil.copy2(build / '.config', root / 'wifi-build.config')
original = {Path(line).name for line in (root / 'board-module-files.txt').read_text().splitlines()}
new = sorted(module for module in dest.rglob('*.ko') if module.name not in original)
with tarfile.open(root / 'wifi-new-modules-4.5.0.tar.gz', 'w:gz') as archive:
    for module in new:
        archive.add(module, arcname=str(module.relative_to(dest)))
print(f'{len(new)} new modules packaged; {len(required)} required kernel exports verified; no PIC relocations.')
