//! Command-line options for the RTL-SDR `zig1090` entrypoint.

const std = @import("std");

pub const DEFAULT_CENTER_HZ: f64 = 1090e6;
pub const DEFAULT_SAMPLE_RATE: f64 = 2.0e6;

pub const Cli = struct {
    verbose: bool,
    /// Print periodic decode counters to stderr (does not affect demod).
    stats: bool,
    center_hz: f64,
    sample_rate_hz: f64,
    receiver_lat: ?f64,
    receiver_lon: ?f64,
    /// When set, serve map + `/aircraft.json` on this TCP port (`--http` or `--http <port>`).
    http_port: ?u16,
    /// TCP outputs (raw 30002, SBS 30003, Beast 30005 by default).
    net: bool,
    net_bind_storage: [64]u8 = undefined,
    net_bind_len: usize,
    net_ro_port: ?u16,
    net_sbs_port: ?u16,
    net_bo_port: ?u16,
    net_heartbeat_s: f64,
    preamble_score_min: f32,
    timing_search_halfspan_samples: f64,
    rescue_conf_scale: f32,
    crc_two_bit_max_pairs: usize,
    overlap_rescan_samples: usize,
    phase_enhance_weight: f32,
    conf_min_sigma: f32,

    pub fn netBindSlice(self: *const Cli) []const u8 {
        return self.net_bind_storage[0..self.net_bind_len];
    }
};

pub fn parseCli(argv: []const []const u8) Cli {
    var verbose = false;
    var stats = false;
    var center: ?f64 = null;
    var rate: ?f64 = null;
    var rlat: ?f64 = null;
    var rlon: ?f64 = null;
    var http_port: ?u16 = null;
    var net = false;
    var net_bind_storage: [64]u8 = undefined;
    @memcpy(net_bind_storage[0.."0.0.0.0".len], "0.0.0.0");
    var net_bind_len: usize = "0.0.0.0".len;
    var net_ro_port: ?u16 = null;
    var net_sbs_port: ?u16 = null;
    var net_bo_port: ?u16 = null;
    var net_heartbeat_s: f64 = 60.0;
    var preamble_score_min: f32 = 1.6;
    var timing_search_halfspan_samples: f64 = 0.20;
    var rescue_conf_scale: f32 = 1.00;
    var crc_two_bit_max_pairs: usize = 180;
    var overlap_rescan_samples: usize = 48;
    var phase_enhance_weight: f32 = 0.35;
    var conf_min_sigma: f32 = 0.10;

    var i: usize = 1;
    while (i < argv.len) {
        const a = argv[i];
        if (std.mem.eql(u8, a, "-v") or std.mem.eql(u8, a, "--verbose")) {
            verbose = true;
            i += 1;
            continue;
        }
        if (std.mem.eql(u8, a, "--stats")) {
            stats = true;
            i += 1;
            continue;
        }
        if (std.mem.eql(u8, a, "--http")) {
            if (i + 1 < argv.len) {
                if (std.fmt.parseInt(u16, argv[i + 1], 10)) |p| {
                    if (p != 0) {
                        http_port = p;
                        i += 2;
                        continue;
                    }
                } else |_| {}
            }
            http_port = 8080;
            i += 1;
            continue;
        }
        if (std.mem.eql(u8, a, "--lat")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --lat <deg> requires a value\n", .{});
                std.posix.exit(2);
            }
            rlat = std.fmt.parseFloat(f64, argv[i + 1]) catch {
                std.debug.print("invalid --lat\n", .{});
                std.posix.exit(2);
            };
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--lon")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --lon <deg> requires a value\n", .{});
                std.posix.exit(2);
            }
            rlon = std.fmt.parseFloat(f64, argv[i + 1]) catch {
                std.debug.print("invalid --lon\n", .{});
                std.posix.exit(2);
            };
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--preamble-score-min")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --preamble-score-min <value>\n", .{});
                std.posix.exit(2);
            }
            preamble_score_min = std.fmt.parseFloat(f32, argv[i + 1]) catch {
                std.debug.print("invalid --preamble-score-min\n", .{});
                std.posix.exit(2);
            };
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--timing-search-halfspan")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --timing-search-halfspan <samples>\n", .{});
                std.posix.exit(2);
            }
            timing_search_halfspan_samples = std.fmt.parseFloat(f64, argv[i + 1]) catch {
                std.debug.print("invalid --timing-search-halfspan\n", .{});
                std.posix.exit(2);
            };
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--rescue-conf-scale")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --rescue-conf-scale <factor>\n", .{});
                std.posix.exit(2);
            }
            rescue_conf_scale = std.fmt.parseFloat(f32, argv[i + 1]) catch {
                std.debug.print("invalid --rescue-conf-scale\n", .{});
                std.posix.exit(2);
            };
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--crc-two-bit-max-pairs")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --crc-two-bit-max-pairs <count>\n", .{});
                std.posix.exit(2);
            }
            crc_two_bit_max_pairs = std.fmt.parseInt(usize, argv[i + 1], 10) catch {
                std.debug.print("invalid --crc-two-bit-max-pairs\n", .{});
                std.posix.exit(2);
            };
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--overlap-rescan-samples")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --overlap-rescan-samples <count>\n", .{});
                std.posix.exit(2);
            }
            overlap_rescan_samples = std.fmt.parseInt(usize, argv[i + 1], 10) catch {
                std.debug.print("invalid --overlap-rescan-samples\n", .{});
                std.posix.exit(2);
            };
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--phase-enhance-weight")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --phase-enhance-weight <0.0..1.0>\n", .{});
                std.posix.exit(2);
            }
            phase_enhance_weight = std.fmt.parseFloat(f32, argv[i + 1]) catch {
                std.debug.print("invalid --phase-enhance-weight\n", .{});
                std.posix.exit(2);
            };
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--conf-min-sigma")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --conf-min-sigma <0.02..0.50>\n", .{});
                std.posix.exit(2);
            }
            conf_min_sigma = std.fmt.parseFloat(f32, argv[i + 1]) catch {
                std.debug.print("invalid --conf-min-sigma\n", .{});
                std.posix.exit(2);
            };
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--net")) {
            net = true;
            i += 1;
            continue;
        }
        if (std.mem.eql(u8, a, "--net-bind-address")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --net-bind-address <ip>\n", .{});
                std.posix.exit(2);
            }
            const s = argv[i + 1];
            if (s.len > net_bind_storage.len) {
                std.debug.print("--net-bind-address too long\n", .{});
                std.posix.exit(2);
            }
            @memcpy(net_bind_storage[0..s.len], s);
            net_bind_len = s.len;
            net = true;
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--net-ro-port")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --net-ro-port <port>\n", .{});
                std.posix.exit(2);
            }
            net_ro_port = std.fmt.parseInt(u16, argv[i + 1], 10) catch {
                std.debug.print("invalid --net-ro-port\n", .{});
                std.posix.exit(2);
            };
            net = true;
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--net-sbs-port")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --net-sbs-port <port>\n", .{});
                std.posix.exit(2);
            }
            net_sbs_port = std.fmt.parseInt(u16, argv[i + 1], 10) catch {
                std.debug.print("invalid --net-sbs-port\n", .{});
                std.posix.exit(2);
            };
            net = true;
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--net-bo-port")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --net-bo-port <port>\n", .{});
                std.posix.exit(2);
            }
            net_bo_port = std.fmt.parseInt(u16, argv[i + 1], 10) catch {
                std.debug.print("invalid --net-bo-port\n", .{});
                std.posix.exit(2);
            };
            net = true;
            i += 2;
            continue;
        }
        if (std.mem.eql(u8, a, "--net-heartbeat")) {
            if (i + 1 >= argv.len) {
                std.debug.print("usage: --net-heartbeat <seconds>\n", .{});
                std.posix.exit(2);
            }
            net_heartbeat_s = std.fmt.parseFloat(f64, argv[i + 1]) catch {
                std.debug.print("invalid --net-heartbeat\n", .{});
                std.posix.exit(2);
            };
            net = true;
            i += 2;
            continue;
        }
        const f = std.fmt.parseFloat(f64, a) catch {
            std.debug.print("unknown argument: {s}\n", .{a});
            std.posix.exit(2);
        };
        if (center == null) {
            center = f;
        } else if (rate == null) {
            rate = f;
        } else {
            std.debug.print("extra positional argument (expected at most center_hz sample_rate_hz)\n", .{});
            std.posix.exit(2);
        }
        i += 1;
    }

    if ((rlat == null) != (rlon == null)) {
        std.debug.print("use both --lat and --lon (WGS84 degrees), or neither\n", .{});
        std.posix.exit(2);
    }
    if (rlat) |la| {
        if (la < -90.0 or la > 90.0) {
            std.debug.print("--lat must be between -90 and 90\n", .{});
            std.posix.exit(2);
        }
    }
    if (rlon) |lo| {
        if (lo < -180.0 or lo > 180.0) {
            std.debug.print("--lon must be between -180 and 180\n", .{});
            std.posix.exit(2);
        }
    }
    if (timing_search_halfspan_samples < 0.0 or timing_search_halfspan_samples > 0.75) {
        std.debug.print("--timing-search-halfspan must be between 0.0 and 0.75 samples\n", .{});
        std.posix.exit(2);
    }
    if (rescue_conf_scale < 1.0 or rescue_conf_scale > 3.0) {
        std.debug.print("--rescue-conf-scale must be between 1.0 and 3.0\n", .{});
        std.posix.exit(2);
    }
    if (crc_two_bit_max_pairs > 4000) {
        std.debug.print("--crc-two-bit-max-pairs must be <= 4000\n", .{});
        std.posix.exit(2);
    }
    if (phase_enhance_weight < 0.0 or phase_enhance_weight > 1.0) {
        std.debug.print("--phase-enhance-weight must be between 0.0 and 1.0\n", .{});
        std.posix.exit(2);
    }
    if (conf_min_sigma < 0.02 or conf_min_sigma > 0.50) {
        std.debug.print("--conf-min-sigma must be between 0.02 and 0.50\n", .{});
        std.posix.exit(2);
    }
    if (net_heartbeat_s < 0.0) {
        std.debug.print("--net-heartbeat must be >= 0 (0 disables heartbeats)\n", .{});
        std.posix.exit(2);
    }

    if (net) {
        if (net_ro_port == null) net_ro_port = 30002;
        if (net_sbs_port == null) net_sbs_port = 30003;
        if (net_bo_port == null) net_bo_port = 30005;
    }

    return .{
        .verbose = verbose,
        .stats = stats,
        .center_hz = center orelse DEFAULT_CENTER_HZ,
        .sample_rate_hz = rate orelse DEFAULT_SAMPLE_RATE,
        .receiver_lat = rlat,
        .receiver_lon = rlon,
        .http_port = http_port,
        .net = net,
        .net_bind_storage = net_bind_storage,
        .net_bind_len = net_bind_len,
        .net_ro_port = net_ro_port,
        .net_sbs_port = net_sbs_port,
        .net_bo_port = net_bo_port,
        .net_heartbeat_s = net_heartbeat_s,
        .preamble_score_min = preamble_score_min,
        .timing_search_halfspan_samples = timing_search_halfspan_samples,
        .rescue_conf_scale = rescue_conf_scale,
        .crc_two_bit_max_pairs = crc_two_bit_max_pairs,
        .overlap_rescan_samples = overlap_rescan_samples,
        .phase_enhance_weight = phase_enhance_weight,
        .conf_min_sigma = conf_min_sigma,
    };
}
