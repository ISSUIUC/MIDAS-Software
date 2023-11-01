import json
import pathlib
import zlib
from argparse import ArgumentParser
from LogParser.cpp_parser import Context, parse_file, Type, Struct, Enum, Integer, Union


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


def calculate_checksum() -> int:
    src = pathlib.Path(__file__).parent.parent / "MIDAS" / "src"
    text = (src / "log_format.h").read_text() + (src / "sensor_data.h").read_text()
    expected_checksum = zlib.crc32(text.encode("utf-8"))
    return expected_checksum


def main():
    parser = ArgumentParser()
    parser.add_argument("raw", type=pathlib.Path)
    parser.add_argument("-o", "--out", type=pathlib.Path)
    args = parser.parse_args()
    if args.out is None:
        args.out = args.raw.with_suffix(".json")

    format_file = pathlib.Path(__file__).parent.parent / "MIDAS" / "src" / "log_format.h"

    expected_checksum = calculate_checksum()

    ctxt: Context = parse_file(format_file)

    mapping = construct_values_mapping(ctxt)

    json_out = []
    with open(args.raw, "rb") as data_file:
        found_checksum = int.from_bytes(data_file.read(4), byteorder="little")
        # if found_checksum != expected_checksum:
        #     raise Exception(f"Trying to parse file with checksum {expected_checksum}, found file with checksum {found_checksum}")
        
        while True:
            disc_bytes = data_file.read(4)
            if len(disc_bytes) == 0:
                break
            discriminant = int.from_bytes(disc_bytes, byteorder='little')
            timestamp_ms = int.from_bytes(data_file.read(4), byteorder="little")
            data_name, data_type = mapping[discriminant]
            data = data_type.parse(data_file.read(data_type.size))
            json_out.append({"type": data_name, "timestamp": timestamp_ms, "data": data})

        data_file.read()

    with open(args.out, "w") as out_file:
        json.dump(json_out, out_file)


if __name__ == '__main__':
    main()
