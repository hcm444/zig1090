//! TCP listeners for common ADS-B feed formats: raw (30002), SBS (30003), Beast cooked (30005), plus heartbeats.

const std = @import("std");
const beast_encode = @import("beast_encode.zig");
const raw_encode = @import("raw_encode.zig");
const sbs_encode = @import("sbs_encode.zig");
const aircraft_table = @import("../ui/aircraft_table.zig");

pub const Config = struct {
    bind_addr: []const u8,
    ro_port: ?u16 = null,
    sbs_port: ?u16 = null,
    bo_port: ?u16 = null,
    /// Seconds between heartbeats (common default ~60). Use 0 to disable.
    heartbeat_s: f64 = 60.0,
};

pub const ClientList = struct {
    mutex: std.Thread.Mutex = .{},
    conns: std.ArrayListUnmanaged(std.net.Stream) = .{},

    pub fn deinit(self: *ClientList, gpa: std.mem.Allocator) void {
        self.mutex.lock();
        for (self.conns.items) |s| s.close();
        self.conns.deinit(gpa);
        self.mutex.unlock();
    }

    pub fn broadcast(self: *ClientList, data: []const u8) void {
        if (data.len == 0) return;
        self.mutex.lock();
        defer self.mutex.unlock();
        var i: usize = 0;
        while (i < self.conns.items.len) {
            self.conns.items[i].writeAll(data) catch {
                self.conns.items[i].close();
                _ = self.conns.swapRemove(i);
                continue;
            };
            i += 1;
        }
    }
};

pub const NetFeed = struct {
    gpa: std.mem.Allocator,
    ro: ClientList,
    sbs: ClientList,
    bo: ClientList,
    heartbeat_ns: u64,

    pub fn init(self: *NetFeed, gpa: std.mem.Allocator, cfg: Config) !void {
        const hb_ns: u64 = if (cfg.heartbeat_s <= 0.0)
            0
        else
            @intFromFloat(cfg.heartbeat_s * @as(f64, @floatFromInt(std.time.ns_per_s)));

        self.* = .{
            .gpa = gpa,
            .ro = .{},
            .sbs = .{},
            .bo = .{},
            .heartbeat_ns = hb_ns,
        };

        if (cfg.ro_port) |p| {
            _ = try std.Thread.spawn(.{}, acceptLoop, .{ gpa, cfg.bind_addr, p, &self.ro });
        }
        if (cfg.sbs_port) |p| {
            _ = try std.Thread.spawn(.{}, acceptLoop, .{ gpa, cfg.bind_addr, p, &self.sbs });
        }
        if (cfg.bo_port) |p| {
            _ = try std.Thread.spawn(.{}, acceptLoop, .{ gpa, cfg.bind_addr, p, &self.bo });
        }
        if (hb_ns > 0 and (cfg.ro_port != null or cfg.sbs_port != null or cfg.bo_port != null)) {
            _ = try std.Thread.spawn(.{}, heartbeatLoop, .{self});
        }
    }

    pub fn deinit(self: *NetFeed) void {
        self.ro.deinit(self.gpa);
        self.sbs.deinit(self.gpa);
        self.bo.deinit(self.gpa);
    }

    /// Beast (cooked) + raw — call without holding the aircraft table lock. Matches dump1090 filters for cooked/raw.
    pub fn emitBeastAndRaw(
        self: *NetFeed,
        now_ns: i128,
        msg: []const u8,
        crc_repair_bits: u8,
        conf64: f64,
    ) void {
        if (crc_repair_bits >= 2) return;

        var buf_beast: [256]u8 = undefined;
        const ts = beast_encode.timestamp12MHz(now_ns);
        const n_beast = beast_encode.encodeModeSFrame(&buf_beast, ts, conf64 * conf64, msg) catch return;
        self.bo.broadcast(buf_beast[0..n_beast]);

        var buf_raw: [256]u8 = undefined;
        const n_raw = raw_encode.encodeRawLine(&buf_raw, msg) catch return;
        self.ro.broadcast(buf_raw[0..n_raw]);
    }

    /// BaseStation line — call with the same mutex held as `aircraft_table` updates (see `processDecoded` in main).
    pub fn emitSbs(
        self: *NetFeed,
        table: *aircraft_table.Table,
        msg: []const u8,
        bit_len: usize,
        df: u32,
        now_ns: i128,
        crc_repair_bits: u8,
        icao: u32,
    ) void {
        const ac = table.getAircraftPtr(icao) orelse return;
        var line_buf: [512]u8 = undefined;
        const line = sbs_encode.formatSbsLine(line_buf[0..], msg, bit_len, df, now_ns, crc_repair_bits, ac) orelse return;
        self.sbs.broadcast(line);
    }
};

fn parseAddress(bind: []const u8, port: u16) !std.net.Address {
    return std.net.Address.parseIp(bind, port);
}

fn acceptLoop(gpa: std.mem.Allocator, bind: []const u8, port: u16, clients: *ClientList) void {
    const addr = parseAddress(bind, port) catch |err| {
        std.log.err("net listen {s}:{d}: {s}", .{ bind, port, @errorName(err) });
        return;
    };
    var listener = addr.listen(.{ .reuse_address = true }) catch |err| {
        std.log.err("net listen {s}:{d}: {s}", .{ bind, port, @errorName(err) });
        return;
    };
    defer listener.deinit();

    std.log.info("net: listening on {s}:{d}", .{ bind, port });

    while (true) {
        const conn = listener.accept() catch |err| {
            std.log.warn("accept: {s}", .{@errorName(err)});
            continue;
        };
        clients.mutex.lock();
        clients.conns.append(gpa, conn.stream) catch {
            clients.mutex.unlock();
            conn.stream.close();
            continue;
        };
        clients.mutex.unlock();
    }
}

fn heartbeatLoop(net: *NetFeed) void {
    while (true) {
        std.Thread.sleep(net.heartbeat_ns);

        var buf_b: [64]u8 = undefined;
        const n_b = beast_encode.encodeHeartbeat(&buf_b) catch continue;
        net.bo.broadcast(buf_b[0..n_b]);

        var buf_r: [64]u8 = undefined;
        const n_r = raw_encode.encodeRawHeartbeat(&buf_r) catch continue;
        net.ro.broadcast(buf_r[0..n_r]);

        net.sbs.broadcast("\r\n");
    }
}
