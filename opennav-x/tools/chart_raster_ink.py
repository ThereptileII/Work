"""Lossless neutral-ink derivation for three hash-locked RGBA8 S-52 sheets.

This is not a general image recoloring operation. Only corresponding pixels
which are Day neutral ink, the reviewed theme neutral RGB and identical alpha
may change. All other pixels, dimensions and alpha are preserved exactly.
"""
import struct
import zlib

SIGNATURE = b'\x89PNG\r\n\x1a\n'


def decode(content):
    assert content.startswith(SIGNATURE)
    chunks = []
    offset = 8
    while offset < len(content):
        size = struct.unpack_from('>I', content, offset)[0]
        kind = content[offset+4:offset+8]
        data = content[offset+8:offset+8+size]
        assert len(data) == size and size <= 16 * 1024 * 1024
        crc = struct.unpack_from('>I', content, offset+8+size)[0]
        assert zlib.crc32(kind+data) == crc
        chunks.append((kind, data))
        offset += size+12
    assert offset == len(content) and chunks[0][0] == b'IHDR' and chunks[-1][0] == b'IEND'
    width, height, depth, color, compression, filtering, interlace = struct.unpack('>IIBBBBB', chunks[0][1])
    # Exact pinned format/dimensions, never an unbounded external input path.
    assert (width, height, depth, color, compression, filtering, interlace) == (1500, 1200, 8, 6, 0, 0, 0)
    stride = width * 4
    expected = (stride+1)*height
    stream = zlib.decompressobj()
    raw = stream.decompress(b''.join(d for k,d in chunks if k == b'IDAT'), expected+1)
    assert len(raw) == expected and stream.eof and not stream.unused_data and not stream.unconsumed_tail
    pixels = bytearray(stride*height)
    for y in range(height):
        start = y*(stride+1)
        filter_type = raw[start]
        assert 0 <= filter_type <= 4
        for x, value in enumerate(raw[start+1:start+1+stride]):
            i = y*stride+x
            a = pixels[i-4] if x >= 4 else 0
            b = pixels[i-stride] if y else 0
            c = pixels[i-stride-4] if y and x >= 4 else 0
            if filter_type == 1: value += a
            elif filter_type == 2: value += b
            elif filter_type == 3: value += (a+b)//2
            elif filter_type == 4:
                p = a+b-c
                pa,pb,pc = abs(p-a),abs(p-b),abs(p-c)
                value += a if pa <= pb and pa <= pc else b if pb <= pc else c
            pixels[i] = value & 255
    return chunks, pixels


def encode(chunks, pixels):
    stride = 1500*4
    assert len(pixels) == stride*1200
    encoded = zlib.compress(b''.join(b'\0'+pixels[y*stride:(y+1)*stride]
                                     for y in range(1200)), 9)
    result = bytearray(SIGNATURE)
    written = False
    for kind, data in chunks:
        if kind == b'IDAT':
            if written: continue
            data = encoded
            written = True
        result.extend(struct.pack('>I', len(data))+kind+data+struct.pack('>I', zlib.crc32(kind+data)))
    assert written
    return bytes(result)


def derive(day, themed, before_rgb, after_rgb):
    chunks, original = decode(themed)
    assert len(day) == len(original)
    before, after = bytes(before_rgb), bytes(after_rgb)
    assert len(before) == len(after) == 3 and len(set(before)) == 1
    changed = bytearray(original)
    count = 0
    for i in range(0, len(day), 4):
        if day[i:i+3] == b'\x07\x07\x07' and day[i+3] and day[i+3] == original[i+3] and original[i:i+3] == before:
            changed[i:i+3] = after
            count += 1
    # Independently counted against pinned sheets; catches wrong inputs/masks.
    assert count == 42100
    assert changed[3::4] == original[3::4]
    return encode(chunks, changed), count
