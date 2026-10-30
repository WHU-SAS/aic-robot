#!/usr/bin/env python3
"""Local difficult-image archive and precise-vision replay (no hardware)."""
import argparse
import json
from pathlib import Path

from host.vision.hard_dataset import DEFAULT_DATASET, REASONS, add_case, import_evidence, label_case, list_cases, replay


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=DEFAULT_DATASET, help='local ignored archive root')
    commands = parser.add_subparsers(dest='command', required=True)
    add = commands.add_parser('add', help='archive one PC run directory or original image')
    add.add_argument('--source', type=Path, required=True)
    add.add_argument('--reason', choices=REASONS, required=True)
    add.add_argument('--note', default='')
    add.add_argument('--vision-config', type=Path, help='explicit comparison calibration; never claimed as original')
    board = commands.add_parser('import-evidence', help='import aic.vision_evidence.v1 board manifest')
    board.add_argument('--source', type=Path, required=True)
    board.add_argument('--reason', choices=REASONS, help='required for successful-but-inaccurate or manually selected frames')
    board.add_argument('--note', default='')
    commands.add_parser('list', help='list cases, labels and metadata gaps')
    label = commands.add_parser('label', help='append human labels; never replace original evidence')
    label.add_argument('case_id')
    label.add_argument('--reason', choices=REASONS, required=True)
    label.add_argument('--note', default='')
    label.add_argument('--expected-status', choices=('localized', 'no_complete_face', 'failed'))
    label.add_argument('--expected-face', choices=('top', 'side'))
    label.add_argument('--expected-center', type=float, nargs=3, metavar=('X', 'Y', 'Z'), help='human measured Base center, never previous predictions')
    label.add_argument('--tolerance-mm', type=float, default=6)
    run = commands.add_parser('replay', help='run current shared precise vision on archived originals')
    run.add_argument('--check', action='store_true', help='exit 1 on explicit expectation failure, runtime error or empty dataset; uncertain cases remain reported')
    run.add_argument('--case', action='append', dest='case_ids', help='repeat to restrict cases')
    run.add_argument('--vision-config', type=Path, help='comparison only for cases without any saved calibration')
    run.add_argument('--output', type=Path, help='new directory; existing reports are never overwritten')
    args = parser.parse_args(argv)
    try:
        if args.command == 'add':
            result = dict(case_id=add_case(args.source, args.reason, args.note, args.dataset, args.vision_config))
        elif args.command == 'import-evidence':
            result = import_evidence(args.source, args.dataset, args.reason, args.note)
        elif args.command == 'list':
            result = list_cases(args.dataset)
        elif args.command == 'label':
            expected = {}
            for name, value in (('status', args.expected_status), ('face_kind', args.expected_face), ('center_Base_mm', args.expected_center)):
                if value is not None:
                    expected[name] = value
            if args.expected_center is not None:
                expected['tolerance_mm'] = args.tolerance_mm
            result = label_case(args.dataset, args.case_id, args.reason, args.note, expected)
        else:
            result = replay(args.dataset, args.case_ids, args.vision_config, args.output)
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(2, str(exc) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.command == 'replay' and args.check and result['check_status'] == 'fail':
        raise SystemExit(1)
    return result


if __name__ == '__main__':
    main()
