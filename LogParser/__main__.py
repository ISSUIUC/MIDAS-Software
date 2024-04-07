import enum
import json
import pathlib
import struct
import time
import zlib
from argparse import ArgumentParser
from sys import stderr
from typing import Any, IO

from LogParser.cpp_parser import Context, parse_file, Type, Struct, Enum, Integer, Union
from LogParser import cpp_parser as types


def construct_values_mapping(ctxt: Context) -> dict[int, tuple[str, Type]]:
    if "LoggedReading" not in ctxt.types:
        raise Exception("'LoggedReading' not found")

    format_struct = ctxt.types["LoggedReading"]
    if not isinstance(format_struct, Struct):
        raise Exception("'LoggedReading' is not a struct")

    for field in ("discriminant", "timestamp_ms", "data"):
        if field not in format_struct.members:
            raise Exception(f"'LoggedReading' is missing field '{field}'")

    discriminant_type = format_struct.members["discriminant"]
    if not isinstance(discriminant_type, Enum):
        raise Exception("'LoggedReading.discriminant' is not a enum")

    timestamp_type = format_struct.members["timestamp_ms"]
    if not isinstance(timestamp_type, Integer):
        raise Exception("'LoggedReading.timestamp_ms' is not a uint32_t")

    if timestamp_type.signed or timestamp_type.size != 4:
        raise Exception("'LoggedReading.timestamp_ms' is not a uint32_t")

    variants_type = format_struct.members["data"]
    if not isinstance(variants_type, Union):
        raise Exception("'LoggedReading.data' is not a union")

    discriminants_normalized = {d_value: d_name.lower().removeprefix("id_") for d_value, d_name in discriminant_type.variants.items()}
    variants_normalized = {v_name.lower().replace("_", ""): v_type for v_name, v_type in variants_type.variants.items()}

    discriminant_set = set(discriminants_normalized.values())
    variant_set = set(variants_normalized.keys())

    diff = variant_set ^ discriminant_set
    if len(diff) > 0:
        raise Exception(f"Some items not matched between discriminant enum and variants union: {', '.join(diff)}")

    variants: dict[int, tuple[str, Type]] = {}
    for d_value, d_name in discriminants_normalized.items():
        variants[d_value] = (d_name, variants_normalized[d_name])

    return variants


def flatten_headers(prev: str, this: Type) -> list[str]:
    items = []
    if isinstance(this, types.Union):
        raise NotImplementedError()
    elif isinstance(this, types.Struct):
        for member_name, member_type in this.members.items():
            items.extend(flatten_headers(f"{prev}.{member_name}", member_type))
    elif isinstance(this, types.Array):
        for i in range(this.count):
            items.extend(flatten_headers(f"{prev}[{i}]", this.item))
    else:
        items.append(prev)
    return items

def flatten_items(prev: str, data: Any) -> dict[str, str]:
    items = {}
    if isinstance(data, dict):
        for key, item in data.items():
            items |= flatten_items(f"{prev}.{key}", item)
    elif isinstance(data, list):
        for i, item in enumerate(data):
            items |= flatten_items(f"{prev}[{i}]", item)
    else:
        items[prev] = repr(data)
    return items


class Format(enum.Enum):
    JSON = "json"
    CSV = "csv"

    def write_header(self, file: IO, headers: dict[int, tuple[str, Type]]) -> list[str]:
        match self:
            case Format.JSON:
                file.write("[\n".encode("utf-8"))
                return []
            case Format.CSV:
                items = ["type", "timestamp"]
                for _, (name, ty) in headers.items():
                    items.extend(flatten_headers(name, ty))
                file.write(",".join(items).encode("utf-8"))
                file.write("\n".encode("utf-8"))
                return items

    def write(self, file: IO, data: Any, index: int, headers: list[str]):
        match self:
            case Format.JSON:
                if index != 0:
                    file.write(",\n".encode("utf-8"))
                file.write(json.dumps(data).encode("utf-8"))
            case Format.CSV:
                flatted = flatten_items(data["type"], data["data"]) | {"type": data["type"], "timestamp": str(data["timestamp"])}
                rows = []
                for header in headers:
                    if header in flatted:
                        rows.append(flatted[header])
                    else:
                        rows.append("")
                file.write(",".join(rows).encode("utf-8"))
                file.write("\n".encode("utf-8"))

    def write_footer(self, file: IO):
        match self:
            case Format.JSON:
                file.write("\n]".encode("utf-8"))
            case Format.CSV:
                pass


def format_validator(s: str) -> Format:
    return Format(s.lower())


def pretty_size(size: int) -> str:
    ooms = ["B", "kB", "MB", "GB"]
    i = 0
    while size > 1000 and i < len(ooms):
        size /= 1000
        i += 1

    return f"{size:.4}{ooms[i]}"


def calculate_checksum(format_file: pathlib.Path) -> int:
    src = format_file.parent
    text = (src / "log_format.h").read_text(errors="replace") + (src / "sensor_data.h").read_text(errors="replace")
    expected_checksum = zlib.crc32(text.encode("utf-8"))
    return expected_checksum

def main():
    parser = ArgumentParser()
    parser.add_argument("raw", type=pathlib.Path)
    parser.add_argument("-o", "--out", type=pathlib.Path)
    parser.add_argument("-f", "--format", type=pathlib.Path)
    parser.add_argument("-d", "--data", type=format_validator)
    args = parser.parse_args()
    if args.out is None and args.data is None:
        args.out = args.raw.with_suffix(".json")
        args.data = Format.JSON
    elif args.out is None and args.data is not None:
        args.out = args.raw.with_suffix(f".{args.data.value}")
    elif args.out is not None and args.data is None:
        args.data = Format(args.out.suffix[1:])

    if args.format is None:
        format_file = pathlib.Path(__file__).parent.parent / "MIDAS" / "src" / "log_format.h"
    else:
        format_file = args.format
    ctxt: Context = parse_file(format_file)

    expected_checksum = calculate_checksum(format_file)

    mapping = construct_values_mapping(ctxt)

    with (open(args.raw, "rb") as data_file,
          open(args.out, "wb") as out_file):
        print(f"Writing output to '{args.out}'")

        found_checksum = int.from_bytes(data_file.read(4), byteorder="little")
        if found_checksum != expected_checksum:
            print(f"Warning: Format file has checksum 0x{hex(expected_checksum)[2:]}, but input file has checksum 0x{hex(found_checksum)[2:]}", file=stderr, flush=True)

        start_time = time.time()

        total_size = args.raw.stat().st_size
        processed = 0

        headers = args.data.write_header(out_file, mapping)
        index = 0
        while True:
            disc_bytes = data_file.read(4)
            if len(disc_bytes) == 0:
                break
            discriminant = int.from_bytes(disc_bytes, byteorder='little')
            timestamp_ms = int.from_bytes(data_file.read(4), byteorder="little")
            data_name, data_type = mapping[discriminant]
            try:
                data = data_type.parse(data_file.read(data_type.size))
            except struct.error:
                break
            args.data.write(out_file, {"type": data_name, "timestamp": timestamp_ms, "data": data}, index, headers)
            index += 1
            processed += 4 + 4 + data_type.size
            if index % 100 == 0 or processed == total_size:
                diff = time.time() - start_time
                remaining = diff / (processed / total_size) - diff
                print(f"\rParsed {pretty_size(processed)}/{pretty_size(total_size)} ({processed/total_size*100:.2f}%) in {diff:.2f} sec. Est {remaining:.2f} sec remaining.   ", end="", flush=True)

        args.data.write_footer(out_file)


if __name__ == '__main__':
    main()
