#!/usr/bin/env python3
"""
Generate single-motor EL05 DBC from a source DBC.

Default use case:
- Source file is strict single-motor DBC for motor id 0x01.
- Generate another single-motor DBC for target id (e.g. 0x02).

This script updates:
1) BO_ message IDs (extended ID encoded as DBC ID with bit31 set)
2) Motor node names like Motor01 -> Motor02
3) Message names like _M01 -> _M02

Examples:
1) Generate id=02 from default source id=01:
   python generate_el05_single_motor_dbc.py --input EL05_protocol_full.dbc --target-id 2

2) Generate id=11 and specify output filename:
   python generate_el05_single_motor_dbc.py --input EL05_protocol_full.dbc --target-id 11 --output EL05_protocol_full_id11.dbc

3) If source DBC is id=11 and want id=12:
   python generate_el05_single_motor_dbc.py --input EL05_protocol_full_id11.dbc --source-id 11 --target-id 12

4) Batch generation in PowerShell (11/12/13/21...):
   $ids = 11,12,13,21,22,23,51,52,53,61,62,63
   foreach ($id in $ids) {
     python .\\generate_el05_single_motor_dbc.py --input .\\EL05_protocol_full.dbc --target-id $id
   }
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


BO_RE = re.compile(r"^(BO_\s+)(\d+)(\s+\S+:\s+\d+\s+\S+\s*)$")
VAL_RE = re.compile(r"^(VAL_\s+)(\d+)(\s+.+)$")
CM_BO_RE = re.compile(r"^(CM_\s+BO_\s+)(\d+)(\s+.+)$")
BA_BO_RE = re.compile(r'^(BA_\s+".*?"\s+BO_\s+)(\d+)(\s+.+)$')


def to_dbc_ext_id(raw_29bit_id: int) -> int:
    return raw_29bit_id | (1 << 31)


def from_dbc_ext_id(dbc_id: int) -> int:
    return dbc_id & 0x1FFFFFFF


def remap_bo_id(dbc_id: int, source_id: int, target_id: int) -> int:
    raw = from_dbc_ext_id(dbc_id)
    low8 = raw & 0xFF
    mid8 = (raw >> 8) & 0xFF

    # Request-like IDs in this protocol: motor id in low 8 bits.
    # Example: (cmd<<24) | (field<<8) | motor_id
    if low8 == source_id:
        raw = (raw & ~0xFF) | target_id
        return to_dbc_ext_id(raw)

    # Response-like IDs in this protocol: low8 is master(0xFD), motor id in bits 8..15.
    # Example: (cmd<<24) | (mode/err<<16..23) | motor_id<<8 | 0xFD
    if low8 == 0xFD and mid8 == source_id:
        raw = (raw & ~(0xFF << 8)) | (target_id << 8)
        return to_dbc_ext_id(raw)

    return dbc_id


def motor_token(mid: int) -> str:
    return f"{mid:02d}"


def transform_text(text: str, source_id: int, target_id: int) -> str:
    src = motor_token(source_id)
    dst = motor_token(target_id)
    out_lines = []

    for line in text.splitlines():
        m = BO_RE.match(line)
        if m:
            prefix, id_text, suffix = m.groups()
            old_id = int(id_text)
            new_id = remap_bo_id(old_id, source_id, target_id)
            line = f"{prefix}{new_id}{suffix}"
        else:
            m_val = VAL_RE.match(line)
            if m_val:
                prefix, id_text, suffix = m_val.groups()
                old_id = int(id_text)
                new_id = remap_bo_id(old_id, source_id, target_id)
                line = f"{prefix}{new_id}{suffix}"
            else:
                m_cm_bo = CM_BO_RE.match(line)
                if m_cm_bo:
                    prefix, id_text, suffix = m_cm_bo.groups()
                    old_id = int(id_text)
                    new_id = remap_bo_id(old_id, source_id, target_id)
                    line = f"{prefix}{new_id}{suffix}"
                else:
                    m_ba_bo = BA_BO_RE.match(line)
                    if m_ba_bo:
                        prefix, id_text, suffix = m_ba_bo.groups()
                        old_id = int(id_text)
                        new_id = remap_bo_id(old_id, source_id, target_id)
                        line = f"{prefix}{new_id}{suffix}"

        # Update node/message tokens for single-motor naming convention.
        # Keep this conservative to avoid touching unrelated numbers.
        line = line.replace(f"Motor{src}", f"Motor{dst}")
        line = line.replace(f"_M{src}", f"_M{dst}")

        out_lines.append(line)

    return "\n".join(out_lines) + "\n"


def default_output_path(input_path: Path, target_id: int) -> Path:
    stem = input_path.stem
    suffix = f"_id{target_id:02d}"
    return input_path.with_name(f"{stem}{suffix}{input_path.suffix}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate EL05 single-motor DBC for another motor id."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("EL05_protocol_full.dbc"),
        help="Source DBC path (default: EL05_protocol_full.dbc)",
    )
    parser.add_argument(
        "--target-id",
        type=int,
        required=True,
        help="Target motor id in decimal, e.g. 2",
    )
    parser.add_argument(
        "--source-id",
        type=int,
        default=1,
        help="Source motor id in DBC (default: 1)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output DBC path (optional)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not (0 <= args.source_id <= 255 and 0 <= args.target_id <= 255):
        raise SystemExit("source-id and target-id must be in [0, 255]")

    input_path = args.input
    if not input_path.is_file():
        raise SystemExit(f"Input file not found: {input_path}")

    output_path = args.output or default_output_path(input_path, args.target_id)
    text = input_path.read_text(encoding="utf-8")
    transformed = transform_text(text, args.source_id, args.target_id)
    output_path.write_text(transformed, encoding="utf-8")

    print(f"Generated: {output_path}")
    print(f"source-id={args.source_id:02d} -> target-id={args.target_id:02d}")


if __name__ == "__main__":
    main()

