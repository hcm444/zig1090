//! Raw TCP output (`*HEX;\\n`) compatible with common ADS-B raw feed consumers (without MLAT timestamps).

const std = @import("std");

/// `*<hex>;\n` — no `@timestamp` prefix (no MLAT clock in zig1090). Returns wire length.
pub fn encodeRawLine(buf: []u8, msg: []const u8) error{NoSpace}!usize {
    const need = 1 + msg.len * 2 + 2;
    if (need > buf.len) return error.NoSpace;
    const hex = "0123456789ABCDEF";
    var i: usize = 0;
    buf[i] = '*';
    i += 1;
    for (msg) |b| {
        buf[i] = hex[b >> 4];
        buf[i + 1] = hex[b & 0xf];
        i += 2;
    }
    buf[i] = ';';
    buf[i + 1] = '\n';
    return i + 2;
}

pub fn encodeRawHeartbeat(buf: []u8) error{NoSpace}!usize {
    const s = "*0000;\n";
    if (s.len > buf.len) return error.NoSpace;
    @memcpy(buf[0..s.len], s);
    return s.len;
}
