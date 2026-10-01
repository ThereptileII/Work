#!/usr/bin/env python3
"""Extract the exact patched peer response buffer for a standalone test TU."""
import argparse
from pathlib import Path

STRUCT_BEGIN = "// OPENNAV_PEER_BUFFER_STRUCT_BEGIN"
STRUCT_END = "// OPENNAV_PEER_BUFFER_STRUCT_END"
CALLBACK_START = "static size_t WriteMemoryCallback"
CALLBACK_END = "// OPENNAV_PEER_BUFFER_CALLBACK_END"
INIT_BEGIN = "// OPENNAV_PEER_REQUEST_INIT_BEGIN"
INIT_END = "// OPENNAV_PEER_REQUEST_INIT_END"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    text = args.source.read_text(encoding="utf-8")
    markers = (STRUCT_BEGIN, STRUCT_END, CALLBACK_START, CALLBACK_END,
               INIT_BEGIN, INIT_END)
    if any(text.count(marker) != 1 for marker in markers):
        raise SystemExit("patched peer buffer markers must occur exactly once")
    struct_start = text.index(STRUCT_BEGIN) + len(STRUCT_BEGIN)
    struct_end = text.index(STRUCT_END, struct_start)
    callback_start = text.index(CALLBACK_START, struct_end)
    callback_end = text.index(CALLBACK_END, callback_start)
    init_start = text.index(INIT_BEGIN, callback_end) + len(INIT_BEGIN)
    init_end = text.index(INIT_END, init_start)
    extracted = (text[struct_start:struct_end].strip() + "\n\n" +
                 text[callback_start:callback_end].strip() + "\n\n" +
                 text[init_start:init_end].strip() + "\n")
    required = ("kMaxPeerResponseBytes", "struct MemoryStruct",
                "WriteMemoryCallback", "InitPeerRequest")
    if not all(token in extracted for token in required):
        raise SystemExit("patched peer buffer extraction is incomplete")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(extracted, encoding="utf-8")


if __name__ == "__main__":
    main()
