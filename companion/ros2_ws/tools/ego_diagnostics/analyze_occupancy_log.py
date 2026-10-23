#!/usr/bin/env python3
"""汇总 BB_OCCUPANCY 失败记录；没有记录或格式错误时不输出成功结论。"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re

MARKER = re.compile(r"BB_OCCUPANCY reason=(\S+) data=(\{.*\})")


def _finiteVector(value, length):
    return (isinstance(value, list) and len(value) == length
            and all(type(v) in (int, float) and math.isfinite(v) for v in value))


def classify(data):
    if not isinstance(data, dict) or type(data.get("finite")) is not bool:
        raise ValueError("missing finite flag")
    if not data["finite"]:
        return "nonfinite_input"
    if type(data.get("in_map")) is not bool or not _finiteVector(data.get("position"), 3):
        raise ValueError("invalid map membership or position")
    if not data["in_map"]:
        return "outside_map"
    if (data.get("raw") not in (0, 1) or data.get("inflated") not in (0, 1)
            or type(data.get("raw_unknown")) is not bool
            or type(data.get("virtual_ceiling")) is not bool):
        raise ValueError("invalid occupancy fields")
    sources = data.get("contributors")
    total = data.get("contributors_total")
    if (not isinstance(sources, list) or type(total) is not int or total < len(sources)
            or len(sources) > 16 or len(sources) != min(total, 16)
            or data.get("contributors_truncated") is not (total > 16)):
        raise ValueError("inconsistent contributor counts")
    for source in sources:
        if (not isinstance(source, dict) or not _finiteVector(source.get("index"), 3)
                or not _finiteVector(source.get("center"), 3)
                or type(source.get("log_odds")) not in (int, float)
                or not math.isfinite(source["log_odds"])):
            raise ValueError("invalid contributor")
    if data["raw"] == 1:
        return "raw_occupied"
    if data["virtual_ceiling"] and data["inflated"]:
        return "virtual_ceiling"
    if data["inflated"] and total:
        return "inflated_from_neighbors"
    if data["inflated"]:
        return "inflated_without_current_contributor"
    return "unknown_without_inflation" if data["raw_unknown"] else "known_free"


def analyze(lines, maxRecords=10000):
    records = []
    for lineNumber, line in enumerate(lines, 1):
        if "BB_OCCUPANCY" not in line:
            continue
        match = MARKER.search(line)
        if not match:
            raise ValueError(f"line {lineNumber}: malformed BB_OCCUPANCY record")
        if len(records) >= maxRecords:
            raise ValueError("record limit exceeded; select a shorter failure window")
        def rejectConstant(value):
            raise ValueError(f"nonfinite JSON value: {value}")
        try:
            data = json.loads(match[2], parse_constant=rejectConstant)
            category = classify(data)
        except (ValueError, TypeError, KeyError) as exc:
            raise ValueError(f"line {lineNumber}: {exc}") from exc
        records.append({"line": lineNumber, "reason": match[1],
                        "category": category, "data": data})
    if not records:
        raise ValueError("no BB_OCCUPANCY records; diagnostics may be disabled or unavailable")
    return {"record_count": len(records),
            "categories": dict(Counter(r["category"] for r in records)),
            "records": records,
            "limits": "Contributor cells explain current grid geometry, not historical depth-pixel provenance or physical obstacle identity."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-records", type=int, default=10000)
    args = parser.parse_args()
    if args.max_records < 1:
        parser.error("--max-records must be positive")
    if args.output.resolve() == args.log.resolve():
        parser.error("output must differ from input")
    try:
        with args.log.open(encoding="utf-8", errors="strict") as stream:
            result = analyze(stream, args.max_records)
    except (OSError, UnicodeError, ValueError) as exc:
        parser.exit(2, f"error: {exc}\n")
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"records={result['record_count']} categories={result['categories']}")


if __name__ == "__main__":
    main()
