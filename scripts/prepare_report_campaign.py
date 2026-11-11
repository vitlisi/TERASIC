#!/usr/bin/env python3
"""Create the report campaign folders without acquiring data or running motors."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def prepare(data_root, figure_root):
    plan_path = ROOT/'config/report_campaign.json'
    model_path = ROOT/'config/final_campaign_model.json'
    plan = json.loads(plan_path.read_text())
    model = json.loads(model_path.read_text())
    for name, expected in model['sha256'].items():
        actual = hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError('Frozen input changed: '+name)
    for folder in (data_root, figure_root):
        folder.mkdir(parents=True, exist_ok=True)
    # Never replace the input snapshot of an existing campaign silently.
    for source in (plan_path, model_path):
        target = data_root/source.name
        if target.exists() and target.read_bytes() != source.read_bytes():
            raise ValueError('Campaign snapshot differs; select a new --data-root: '+str(target))
        if not target.exists():
            target.write_bytes(source.read_bytes())
    index = ['# Final report campaign', '',
             'Prepared folders are not completed experiments. No acquisition runs automatically.', '',
             'Figures: English; full recorded traces from t=0; separate additional zooms.', '',
             '| Section | Test | Input requirement |', '|---|---|---|']
    status = {}
    count = 0
    for section in plan['sections']:
        for case in section['cases']:
            key = section['id']+'/'+case['id']
            folder = data_root/key
            figures = figure_root/key
            for platform in case['platforms']:
                (folder/platform/'runs').mkdir(parents=True, exist_ok=True)
            (folder/'analysis').mkdir(parents=True, exist_ok=True)
            figures.mkdir(parents=True, exist_ok=True)
            readme = folder/'README.md'
            if not readme.exists():
                lines = ['# '+case['title'], '',
                         'Status: '+case.get('status', 'awaiting new inputs and protocol verification.'), '',
                         '## Required figures', '']
                lines += ['- '+x for x in case['figures']]
                lines += ['', '## Required inputs and checks', '']
                lines += ['- '+x for x in case['requirements']]
                lines += ['', '## Existing tools', '']
                lines += ['- `'+x+'`' for x in case['existing_tools']]
                if case['reuse']:
                    lines += ['', '## Reused acquisition references', '']
                    lines += ['- `'+x+'`' for x in case['reuse']]
                lines += ['', 'Record every run in a fresh subdirectory of `<platform>/runs/`.',
                          'Keep raw CSV, status, model/controller snapshots, measured voltage,',
                          'startup/fixture, compensation setting, encoder/filter details and stop evidence.',
                          'A reusable source is referenced by path and SHA256, not copied and relabeled.',
                          'A CSV file is an input candidate, not an automatically validated result.',
                          '', 'Figure output: `'+str(figures)+'`.', '']
                readme.write_text('\n'.join(lines))
            status[key] = {}
            for platform in case['platforms']:
                records = sorted(str(p.relative_to(data_root)) for p in
                                 (folder/platform/'runs').rglob('*.csv'))
                status[key][platform] = dict(
                    status='input_candidates_require_review' if records else 'awaiting_inputs_or_source_reference',
                    csv_candidates=records)
                reference=case.get('input_source')
                if reference and (ROOT/reference).is_file():
                    status[key][platform]['source_reference']=reference
                    status[key][platform]['status']=('unavailable_no_new_acquisition_planned' if case.get('status','').startswith('No complete') else 'historical_source_reference_available')
            index.append('| '+section['title']+' | `'+key+'` | '+', '.join(case['platforms'])+' |')
            count += 1
    (data_root/'README.md').write_text('\n'.join(index)+'\n')
    (data_root/'input_status.json').write_text(json.dumps(status, indent=2)+'\n')
    (figure_root/'README.md').write_text(
        '# Final report figures\n\nFolders mirror `'+str(data_root)+'`. '
        'No measured-data figures are available until the required inputs are supplied. '
        'Export SVG/PDF and PNG; keep full traces and label optional zooms explicitly.\n')
    print('Prepared '+str(count)+' test folders across '+str(len(plan['sections']))+' report sections.')
    print('Data: '+str(data_root))
    print('Figures: '+str(figure_root))
    print('No simulation or hardware acquisition was started.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path, default=ROOT/'maintenance/report_plan')
    parser.add_argument('--figure-root', type=Path, default=ROOT/'deliverables/final_campaign')
    args = parser.parse_args()
    prepare(args.data_root.resolve(), args.figure_root.resolve())


if __name__ == '__main__':
    main()
