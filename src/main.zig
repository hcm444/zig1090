//! zig1090: RTL-SDR ADS-B / Mode S demodulator (entrypoint).

const std = @import("std");
const cli = @import("app/cli.zig");
const run = @import("app/run.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    try run.runRtl1090(allocator, cli.parseCli(args));
}
