import pathlib
import struct
from argparse import ArgumentParser
from typing import Any

from .cpp_parser import Context, parse_file, Type, Struct, Enum, Integer, Union
from . import cpp_parser as types


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
    if not isinstance(timestamp_type, Integer) or timestamp_type.signed or timestamp_type.size != 4:
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
        raise Exception(f"Some items did not match between the discriminant enum and the variants union: {', '.join(diff)}")

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


class TypeId:
    INTEGER = 0b000
    BOOLEAN = 0b001
    FLOAT   = 0b010
    STRUCT  = 0b011
    ARRAY   = 0b100
    ENUM    = 0b101
    UNION   = 0b110


def encode_string(s: str) -> bytes:
    return struct.pack("<B", len(s)) + s.encode("utf-8")

def encode_type(typ: Type) -> bytes:
    if isinstance(typ, types.Integer):
        assert 0 < typ.size <= 16
        return struct.pack("<B", (TypeId.INTEGER << 5) + ((1 if typ.signed else 0) << 4) + (typ.size - 1))
    elif isinstance(typ, types.Boolean):
        return struct.pack("<B", TypeId.BOOLEAN << 5)
    elif isinstance(typ, types.Float):
        assert 0 < typ.size <= 32
        return struct.pack("<B", (TypeId.FLOAT << 5) + (typ.size - 1))
    elif isinstance(typ, types.Struct):
        assert len(typ.members) < 32
        encoded = bytearray(struct.pack("<B", (TypeId.STRUCT << 5) + len(typ.members)))
        for name, member in typ.members.items():
            encoded += encode_string(name)
            encoded += encode_type(member)
        return encoded
    elif isinstance(typ, types.Array):
        assert 0 <= typ.count < 32
        encoded = bytearray(struct.pack("<B", (TypeId.ARRAY << 5) + typ.count))
        encoded += encode_type(typ.item)
        return encoded
    elif isinstance(typ, types.Enum):
        assert len(typ.variants) < 32
        encoded = bytearray(struct.pack("<B", (TypeId.ENUM << 5) + len(typ.variants)))
        for (discriminant, variant) in typ.variants.items():
            encoded += struct.pack("<I", discriminant)
            encoded += encode_string(variant)
        return encoded
    elif isinstance(typ, types.Union):
        raise NotImplementedError()
    else:
        raise NotImplementedError()

def create_format_header(mapping: dict[int, tuple[str, Type]]) -> bytes:
    header = bytearray()
    header += struct.pack("<B", len(mapping))
    for (discriminant, (name, typ)) in mapping.items():
        header += struct.pack("<B", discriminant)
        header += encode_string(name)
        header += encode_type(typ)
    return bytes([0xDE, 0xAD, 0xBE, 0xEF]) + struct.pack("<H", len(header)) + bytes(header)


def main():
    parser = ArgumentParser()
    parser.add_argument("format", type=pathlib.Path)
    args = parser.parse_args()

    format_file = args.format
    ctxt = parse_file(format_file)

    mapping = construct_values_mapping(ctxt)

    fmt_bytes = create_format_header(mapping)

    formatted = "{" + ", ".join(hex(byte) for byte in list(fmt_bytes)) + "}"
    print(f"""\
#pragma once

constexpr int LOG_FORMAT_HEADER_LENGTH = {len(fmt_bytes)};
constexpr uint8_t LOG_FORMAT_HEADER[LOG_FORMAT_HEADER_LENGTH] = {formatted};
""")


if __name__ == '__main__':
    main()
