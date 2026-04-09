const std = @import("std");
const rtl_iq = @import("rtl_iq.zig");
const adsb = @import("adsb/adsb_payload.zig");
const crc = @import("mode_s/mode_s_crc.zig");
const demod = @import("mode_s/mode_s_demod.zig");
const msdec = @import("mode_s/mode_s_decode.zig");
const aircraft_table = @import("ui/aircraft_table.zig");
const web = @import("ui/web.zig");
const net_feed = @import("net/net_feed.zig");

const C32 = std.math.Complex(f32);

const DEFAULT_CENTER_HZ: f64 = 1090e6;
const DEFAULT_SAMPLE_RATE: f64 = 2.0e6;

const Cli = struct {
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

    fn netBindSlice(self: *const Cli) []const u8 {
        return self.net_bind_storage[0..self.net_bind_len];
    }
};

fn parseCli(argv: []const []const u8) Cli {
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
const FRAME_N: usize = 8192;
comptime {
    std.debug.assert(FRAME_N == rtl_iq.frame_len);
}
const TABLE_REFRESH_NS: i128 = 250 * std.time.ns_per_ms;

fn icaoFromMsg(msg: []const u8, bit_len: usize, df: u32) ?u32 {
    if (bit_len == 112) {
        return (@as(u32, msg[1]) << 16) | (@as(u32, msg[2]) << 8) | @as(u32, msg[3]);
    }
    if (bit_len != 56) return null;
    if (df == 11) {
        return (@as(u32, msg[1]) << 16) | (@as(u32, msg[2]) << 8) | @as(u32, msg[3]);
    }
    if (df == 0 or df == 4 or df == 5) {
        var msg7: [7]u8 = undefined;
        @memcpy(&msg7, msg[0..7]);
        return @as(u32, crc.modesChecksum56(&msg7));
    }
    return null;
}

fn processDecodedMessage(
    web_sh_ptr: ?*web.Shared,
    net_ptr: ?*net_feed.NetFeed,
    table: *aircraft_table.Table,
    now_ns: i128,
    msg: []const u8,
    bit_len: usize,
    df: u32,
    conf64: f64,
    crc_repair_bits: u8,
    cpr_air: *std.AutoHashMap(u32, adsb.CprAirborneState),
    cpr_surf: *std.AutoHashMap(u32, adsb.CprSurfaceState),
) !void {
    if (net_ptr) |n| {
        n.emitBeastAndRaw(now_ns, msg, crc_repair_bits, conf64);
    }

    const m = web_sh_ptr;
    if (m) |ws| ws.mutex.lock();
    defer if (m) |ws| ws.mutex.unlock();
    try table.updateFromModeSMessage(now_ns, msg, bit_len, df, conf64, cpr_air, cpr_surf);
    if (net_ptr) |n| {
        if (icaoFromMsg(msg, bit_len, df)) |icao| {
            if (icao != 0) {
                n.emitSbs(table, msg, bit_len, df, now_ns, crc_repair_bits, icao);
            }
        }
    }
}

fn renderTableLocked(
    web_sh_ptr: ?*web.Shared,
    table: *aircraft_table.Table,
    w: *std.Io.Writer,
    now_ns: i128,
    center_mhz: f64,
) !void {
    const m = web_sh_ptr;
    if (m) |ws| ws.mutex.lock();
    defer if (m) |ws| ws.mutex.unlock();
    try table.render(w, now_ns, center_mhz);
}
const MIN_SAMPLE_RATE_HZ: f64 = 1.8e6;

fn fillEnergy(samples: []const C32, energy: []f32) void {
    std.debug.assert(samples.len == energy.len);
    var i: usize = 0;
    while (i + 8 <= samples.len) : (i += 8) {
        inline for (0..8) |k| {
            const s = samples[i + k];
            energy[i + k] = s.re * s.re + s.im * s.im;
        }
    }
    while (i < samples.len) : (i += 1) {
        const s = samples[i];
        energy[i] = s.re * s.re + s.im * s.im;
    }
}

const DecodeStats = struct {
    preambles_passed: u64 = 0,
    full_decodes: u64 = 0,
    conf_rejected: u64 = 0,
    accepted_56: u64 = 0,
    accepted_112: u64 = 0,
    rescue_redecodes: u64 = 0,
    crc112_no_repair: u64 = 0,
    crc112_1bit: u64 = 0,
    crc112_2bit: u64 = 0,
};

const NoiseEstimator = struct {
    mean: f32 = 0.0,
    var_pop: f32 = 1.0,
    initialized: bool = false,
    /// EMA blend; lower = steadier, higher = more responsive.
    alpha: f32 = 0.05,
    /// Subsample stride for per-frame estimation.
    stride: usize = 16,

    fn update(self: *NoiseEstimator, energy: []const f32) void {
        // Subsample Welford over the frame to avoid full-frame passes.
        var mean: f32 = 0.0;
        var m2: f32 = 0.0;
        var n: f32 = 0.0;

        var i: usize = 0;
        const step = @max(@as(usize, 1), self.stride);
        while (i < energy.len) : (i += step) {
            const x = energy[i];
            n += 1.0;
            const delta = x - mean;
            mean += delta / n;
            const delta2 = x - mean;
            m2 += delta * delta2;
        }

        const var_pop = m2 / @max(1.0, n);

        if (!self.initialized) {
            self.mean = mean;
            self.var_pop = var_pop;
            self.initialized = true;
            return;
        }

        const a = self.alpha;
        self.mean = self.mean * (1.0 - a) + mean * a;
        self.var_pop = self.var_pop * (1.0 - a) + var_pop * a;
    }

    fn stddev(self: *const NoiseEstimator) f32 {
        return @sqrt(self.var_pop + 1e-12);
    }
};

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    const cli = parseCli(args);
    const verbose = cli.verbose;
    const center_hz = cli.center_hz;
    const sample_rate_hz = cli.sample_rate_hz;

    if (sample_rate_hz < MIN_SAMPLE_RATE_HZ) {
        std.debug.print("sample rate must be at least 1.8e6 Hz (Mode S Manchester timing).\n", .{});
        std.posix.exit(1);
    }

    const dparams = demod.DemodParams.init(sample_rate_hz);

    crc.initTables();

    // Keep stderr quiet: driver chatter draws below the fullscreen table on the same TTY.
    var iq = rtl_iq.RtlIqStream.init(allocator, center_hz, sample_rate_hz, .{ .debug = false }) catch |err| switch (err) {
        error.SdrNotDetected => {
            std.debug.print("SDR not detected.\n", .{});
            std.posix.exit(1);
        },
        else => |e| return e,
    };
    defer iq.deinit();
    try iq.start();

    var stdout_buf: [16384]u8 = undefined;

    // Alternate screen: table redraws on stdout only; stderr (libusb, etc.) won't sit under it.
    if (!verbose) {
        try std.fs.File.stdout().writeAll("\x1b[?1049h");
    }
    defer if (!verbose) {
        _ = std.fs.File.stdout().writeAll("\x1b[?1049l") catch {};
    };

    if (verbose) {
        std.debug.print(
            "Verbose: per-message lines on stderr; fullscreen table disabled. Center={d:.3} MHz\n",
            .{center_hz / 1e6},
        );
    }

    var energy: [FRAME_N]f32 = undefined;
    var bits: [112]u1 = undefined;
    var bits_scratch: [112]u1 = undefined;
    var peek_byte: [1]u8 = undefined;

    var cpr_map = std.AutoHashMap(u32, adsb.CprAirborneState).init(allocator);
    defer cpr_map.deinit();
    var cpr_surface_map = std.AutoHashMap(u32, adsb.CprSurfaceState).init(allocator);
    defer cpr_surface_map.deinit();

    var ac_table = aircraft_table.Table.init(allocator, cli.receiver_lat, cli.receiver_lon);
    defer ac_table.deinit();

    var web_sh: web.Shared = undefined;
    var web_sh_ptr: ?*web.Shared = null;
    var http_thread: ?std.Thread = null;
    if (cli.http_port) |hp| {
        web_sh = .{
            .table = &ac_table,
            .center_mhz = center_hz / 1e6,
            .json_cache = &.{},
            .json_cache_at_ns = 0,
        };
        web_sh_ptr = &web_sh;
        http_thread = try web.spawnListener(&web_sh, hp);
    }
    defer if (http_thread) |t| t.join();

    var net_feed_storage: net_feed.NetFeed = undefined;
    var net_feed_ptr: ?*net_feed.NetFeed = null;
    if (cli.net) {
        try net_feed_storage.init(allocator, .{
            .bind_addr = cli.netBindSlice(),
            .ro_port = cli.net_ro_port,
            .sbs_port = cli.net_sbs_port,
            .bo_port = cli.net_bo_port,
            .heartbeat_s = cli.net_heartbeat_s,
        });
        net_feed_ptr = &net_feed_storage;
    }
    defer if (net_feed_ptr) |p| p.deinit();

    var last_table_draw_ns: i128 = 0;
    var noise = NoiseEstimator{};
    var last_accept_i: usize = std.math.maxInt(usize);
    var last_accept_len: usize = 0;
    var last_accept_bits: usize = 0;
    var last_accept_hash: u64 = 0;

    var decode_stats = DecodeStats{};
    var last_stats_print_ns: i128 = 0;

    while (true) {
        iq.wait(FRAME_N, 200 * std.time.ns_per_ms) catch |err| switch (err) {
            error.Timeout => continue,
            error.EndOfStream => break,
        };

        const available = iq.get();
        if (available.len < FRAME_N) continue;
        const frame = available[0..FRAME_N];
        fillEnergy(frame, &energy);
        noise.update(&energy);
        const noise_mean = noise.mean;
        const noise_std = noise.stddev();
        const now_ns = std.time.nanoTimestamp();

        var i: usize = 0;
        while (i + dparams.messageExtentSamples(112) < FRAME_N) {
            const preamble_score = demod.preambleScore(&energy, i, noise_mean, dparams);
            if (preamble_score < cli.preamble_score_min) {
                i += 1;
                continue;
            }
            if (cli.stats) decode_stats.preambles_passed += 1;
            if (last_accept_len != 0) {
                const overlap_end = last_accept_i + last_accept_len;
                if (i > last_accept_i and i < overlap_end and i > last_accept_i + cli.overlap_rescan_samples) {
                    i += 1;
                    continue;
                }
            }

            const data_start = @as(f64, @floatFromInt(i + dparams.preamble_samples));
            const phase_ref = demod.estimatePreamblePhaseRef(frame, i, dparams);
            const timing_halfspan = cli.timing_search_halfspan_samples;
            const offsets_primary = [_]f64{
                -timing_halfspan,
                0.0,
                timing_halfspan,
            };

            _ = demod.decodeBitsManchesterBestOffsetPhaseEnhanced(
                &energy,
                frame,
                data_start,
                bits[0..5],
                bits_scratch[0..5],
                dparams.samples_per_half_chip,
                &offsets_primary,
                phase_ref,
                cli.phase_enhance_weight,
            );
            demod.bitsToBytes(bits[0..5], &peek_byte);
            const df = (peek_byte[0] >> 3) & 0x1f;
            const bit_len = msdec.modeSBitLength(df);

            if (i + dparams.messageExtentSamples(bit_len) > FRAME_N) {
                i += 1;
                continue;
            }

            var conf = demod.decodeBitsManchesterBestOffsetPhaseEnhanced(
                &energy,
                frame,
                data_start,
                bits[0..bit_len],
                bits_scratch[0..bit_len],
                dparams.samples_per_half_chip,
                &offsets_primary,
                phase_ref,
                cli.phase_enhance_weight,
            );
            if (cli.stats) decode_stats.full_decodes += 1;
            if (conf < noise_std * cli.conf_min_sigma) {
                if (cli.stats) decode_stats.conf_rejected += 1;
                i += 1;
                continue;
            }
            const rescue_threshold = (noise_std * cli.conf_min_sigma) * cli.rescue_conf_scale;
            const offsets_rescue = [_]f64{
                -timing_halfspan,
                -timing_halfspan * 0.5,
                0.0,
                timing_halfspan * 0.5,
                timing_halfspan,
            };

            if (bit_len == 56) {
                var msg7: [7]u8 = undefined;
                demod.bitsToBytes(bits[0..56], &msg7);
                var crc_ok = crc.accept56Short(df, &msg7, true);
                if (!crc_ok and df == 11 and conf >= rescue_threshold) {
                    if (cli.stats) decode_stats.rescue_redecodes += 1;
                    conf = demod.decodeBitsManchesterBestOffsetPhaseEnhanced(
                        &energy,
                        frame,
                        data_start,
                        bits[0..56],
                        bits_scratch[0..56],
                        dparams.samples_per_half_chip,
                        &offsets_rescue,
                        phase_ref,
                        cli.phase_enhance_weight,
                    );
                    demod.bitsToBytes(bits[0..56], &msg7);
                    crc_ok = crc.accept56Short(df, &msg7, true);
                }
                if (!crc_ok) {
                    i += 1;
                    continue;
                }
                const msg_hash = std.hash.Wyhash.hash(0, &msg7);
                if (last_accept_bits == 56 and last_accept_hash == msg_hash and i <= last_accept_i + cli.overlap_rescan_samples) {
                    i += 1;
                    continue;
                }
                const conf64: f64 = @as(f64, conf);
                try processDecodedMessage(web_sh_ptr, net_feed_ptr, &ac_table, now_ns, msg7[0..], 56, df, conf64, 0, &cpr_map, &cpr_surface_map);
                if (cli.stats) decode_stats.accepted_56 += 1;
                if (verbose) msdec.printVerbose(&msg7, 56, df, conf64);
                last_accept_hash = msg_hash;
            } else {
                var msg14: [14]u8 = undefined;
                demod.bitsToBytes(bits[0..112], &msg14);
                const crc_before_ok = crc.modesChecksum(&msg14) == 0;
                var crc_repair_bits: u8 = 0;
                var crc_ok = crc_before_ok;
                if (!crc_ok) {
                    crc_ok = crc.acceptOrFixSingleBit(&msg14);
                    if (crc_ok) {
                        crc_repair_bits = 1;
                        if (cli.stats) decode_stats.crc112_1bit += 1;
                    }
                }
                if (crc_before_ok and cli.stats) decode_stats.crc112_no_repair += 1;
                if (!crc_ok and conf >= rescue_threshold) {
                    if (cli.stats) decode_stats.rescue_redecodes += 1;
                    conf = demod.decodeBitsManchesterBestOffsetPhaseEnhanced(
                        &energy,
                        frame,
                        data_start,
                        bits[0..112],
                        bits_scratch[0..112],
                        dparams.samples_per_half_chip,
                        &offsets_rescue,
                        phase_ref,
                        cli.phase_enhance_weight,
                    );
                    demod.bitsToBytes(bits[0..112], &msg14);
                    crc_ok = crc.acceptOrFixTwoBit(&msg14, cli.crc_two_bit_max_pairs);
                    if (crc_ok) {
                        crc_repair_bits = 2;
                        if (cli.stats) decode_stats.crc112_2bit += 1;
                    }
                }
                if (!crc_ok) {
                    i += 1;
                    continue;
                }
                const msg_hash = std.hash.Wyhash.hash(0, &msg14);
                if (last_accept_bits == 112 and last_accept_hash == msg_hash and i <= last_accept_i + cli.overlap_rescan_samples) {
                    i += 1;
                    continue;
                }
                const icao: u32 = (@as(u32, msg14[1]) << 16) | (@as(u32, msg14[2]) << 8) | @as(u32, msg14[3]);
                if (icao == 0) {
                    i += 1;
                    continue;
                }
                const conf64: f64 = @as(f64, conf);
                try processDecodedMessage(web_sh_ptr, net_feed_ptr, &ac_table, now_ns, msg14[0..], 112, df, conf64, crc_repair_bits, &cpr_map, &cpr_surface_map);
                if (cli.stats) decode_stats.accepted_112 += 1;
                if (verbose) msdec.printVerbose(&msg14, 112, df, conf64);
                last_accept_hash = msg_hash;
            }

            const accepted_extent = dparams.messageExtentSamples(bit_len);
            last_accept_i = i;
            last_accept_len = accepted_extent;
            last_accept_bits = bit_len;
            const advance = if (accepted_extent > cli.overlap_rescan_samples) accepted_extent - cli.overlap_rescan_samples else 1;
            i += advance;
        }

        if (!verbose and now_ns - last_table_draw_ns >= TABLE_REFRESH_NS) {
            last_table_draw_ns = now_ns;
            var stdout_wr = std.fs.File.stdout().writer(&stdout_buf);
            try renderTableLocked(web_sh_ptr, &ac_table, &stdout_wr.interface, now_ns, center_hz / 1e6);
            try stdout_wr.interface.flush();
        }

        iq.update(FRAME_N);

        if (cli.stats) {
            const stats_now = std.time.nanoTimestamp();
            if (stats_now - last_stats_print_ns >= 5 * std.time.ns_per_s) {
                last_stats_print_ns = stats_now;
                std.debug.print(
                    "zig1090 stats: preambles={d} full_decodes={d} conf_rej={d} acc56={d} acc112={d} rescue={d} crc112_ok={d} crc112_1b={d} crc112_2b={d}\n",
                    .{
                        decode_stats.preambles_passed,
                        decode_stats.full_decodes,
                        decode_stats.conf_rejected,
                        decode_stats.accepted_56,
                        decode_stats.accepted_112,
                        decode_stats.rescue_redecodes,
                        decode_stats.crc112_no_repair,
                        decode_stats.crc112_1bit,
                        decode_stats.crc112_2bit,
                    },
                );
            }
        }
    }
}
