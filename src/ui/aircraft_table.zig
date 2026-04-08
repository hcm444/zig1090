//! Per-ICAO state and periodic table render (dump1090-style).

const std = @import("std");
const adsb = @import("../adsb/adsb_payload.zig");
const crc = @import("../mode_s/mode_s_crc.zig");
const msdec = @import("../mode_s/mode_s_decode.zig");

/// Column widths for the terminal table (header, rule line, and `render` data row must stay in sync).
const term_table_widths = [_]u8{ 6, 8, 6, 6, 2, 5, 5, 5, 5, 2, 10, 10, 4, 6, 3, 4, 2, 2, 5, 2, 2, 5, 4, 1, 7 };
comptime {
    std.debug.assert(term_table_widths.len == 25);
}

const TRAIL_RETENTION_NS: i128 = 24 * 60 * 60 * std.time.ns_per_s;
const TRAIL_SAMPLE_MIN_NS: i128 = 10 * std.time.ns_per_s;
const TRAIL_MOVE_MIN_NM: f64 = 0.05;
const COVERAGE_BINS: usize = 72;
const RING_HALFLIFE_S: f64 = 15.0 * 60.0;
/// `range_nm` from `greatCircleNm` is nautical miles; multiply to compare with statute-mile rings.
const NM_TO_STATUTE_MI: f64 = 1.150779448;

pub const TrailPoint = struct {
    at_ns: i128,
    lat: f64,
    lon: f64,
    /// Barometric altitude (ft) when known at this point; surface tracks may be null.
    alt_ft: ?i32 = null,
    range_nm: ?f64,
};

/// Receiver position for map / `/aircraft.json`.
pub const ReceiverPos = struct {
    lat: f64,
    lon: f64,
};

pub const NetTrailPt = struct {
    lat: f64,
    lon: f64,
    alt: ?i32,
    t: f64,
};

pub const NetAircraft = struct {
    hex: []const u8,
    flight: []const u8,
    lat: ?f64,
    lon: ?f64,
    track: ?f32,
    alt_baro: ?i32,
    /// True = 25 ft steps (Q); false = Gillham Mode C (100 ft) when `alt_baro` set from airborne position.
    baro_alt_is_q: ?bool,
    /// GNSS / geometric height (ft) from TC 20–22 when decoded.
    geom_alt_ft: ?i32,
    /// Q-bit for `geom_alt_ft` when set from TC 20–22.
    geom_alt_is_q: ?bool,
    /// Mode A squawk 0000–7777 (ADS-B TC 28 or future Mode S identity).
    squawk: ?u16,
    gs: ?f32,
    vert_rate: ?i32,
    /// Matches terminal `Vb`: baro (B) vs geom (G) vertical rate source when known.
    vert_rate_is_baro: ?bool,
    /// DF17/18 CA (capability) bits 6–8.
    capability_ca: u8,
    /// ADS-B Aircraft Identification EC (emitter category) for TC 1–4 (ME bits 6–8).
    emitter_category_ec: ?u8,
    on_ground: bool,
    seen: f64,
    trail: []const NetTrailPt,
    /// TC 9–18 surveillance status / NIC supplement B; null if surface or unknown.
    pos_ss: ?u8,
    pos_nic_b: ?u8,
    vel_subtype: ?u8,
    nac_v: ?u8,
    geom_delta_ft: ?i32,
    msg_count: u32,
    last_conf: f64,
    /// Range to receiver (NM) from latest trail point, if known.
    range_nm: ?f64,
    max_range_24h_nm: ?f64,
    /// TC 31: ADS-B version (ME bits 41–43).
    ads_b_version: ?u8,
    /// TC 31: subtype 0 = airborne op, 1 = surface op.
    op_mesub: ?u8,
    op_nic_a: ?u8,
    op_nic_c: ?u8,
    op_nac_p: ?u8,
    op_sil: ?u8,
    /// TC 31 v2: SIL reporting type (true = per-sample, false = per-hour).
    op_sil_per_sample: ?bool,
    op_gva: ?u8,
    op_nic_baro: ?u8,
    op_nac_v_surface: ?u8,
};

pub const NetSnapshot = struct {
    now: f64,
    center_mhz: f64,
    receiver: ?ReceiverPos,
    /// Maximum observed range from receiver (nautical miles), based on trail points kept in memory.
    /// Null when receiver position is unknown or no ranged trail points have been recorded.
    coverage_nm: ?f64 = null,
    /// Farthest observed point per bearing bin (receiver-centered), for coverage visualization.
    coverage_farthest_points: []const ReceiverPos = &.{},
    /// Ring radii (statute miles), same as map range rings.
    ring_mi: [4]u16 = .{ 50, 100, 150, 200 },
    ring_hits: [4]u64 = .{ 0, 0, 0, 0 },
    ring_total_ranged_hits: u64 = 0,
    /// EWMA of ranged fixes per minute within each ring (adapts to traffic volume).
    ring_rate_ewma_per_min: [4]f64 = .{ 0, 0, 0, 0 },
    aircraft: []const NetAircraft,
};

pub const Aircraft = struct {
    icao: u32,
    callsign: [8]u8 = [_]u8{' '} ** 8,
    callsign_len: u8 = 0,
    /// DF17/18 CA (capability) bits 6–8; updated on every ES message.
    capability_ca: u8 = 0,
    /// ADS-B Aircraft Identification EC (emitter category) for TC 1–4; null until seen.
    emitter_category_ec: ?u8 = null,
    baro_alt_ft: ?i32 = null,
    /// Set from TC 9–18 AC12 Q bit when baro alt updated from airborne position.
    baro_alt_is_q: ?bool = null,
    /// TC 20–22 geometric / GNSS height (ft).
    geom_alt_ft: ?i32 = null,
    geom_alt_is_q: ?bool = null,
    /// Mode A code (squawk), 4-digit 0000–7777.
    squawk: ?u16 = null,
    ground_speed_kt: ?f32 = null,
    track_deg: ?f32 = null,
    vert_rate_fpm: ?i32 = null,
    /// From TC 19 ME bit 36 when vertical rate is present.
    vert_rate_is_baro: ?bool = null,
    lat: ?f64 = null,
    lon: ?f64 = null,
    /// TC 9–18: surveillance status bits (0–3); null if last pos was surface / not airborne pos.
    pos_ss: ?u8 = null,
    /// TC 9–18: NIC supplement B (0–1); null if not from airborne position.
    pos_nic_b: ?u8 = null,
    /// TC 19 subtype 1–4; null if last velocity update not seen.
    vel_subtype: ?u8 = null,
    /// TC 19 NAC velocity (0–7).
    nac_v: ?u8 = null,
    /// TC 19 geometric height diff vs baro altitude (ft).
    geom_delta_ft: ?i32 = null,
    /// TC 31 operational status (last decoded v1/v2 fields).
    ads_b_version: ?u8 = null,
    op_mesub: ?u8 = null,
    op_nic_a: ?u8 = null,
    op_nic_c: ?u8 = null,
    op_nac_p: ?u8 = null,
    op_sil: ?u8 = null,
    op_sil_per_sample: ?bool = null,
    op_gva: ?u8 = null,
    op_nic_baro: ?u8 = null,
    op_nac_v_surface: ?u8 = null,
    on_ground: bool = false,
    msg_count: u32 = 0,
    last_seen_ns: i128 = 0,
    last_conf: f64 = 0,
    trail: std.ArrayList(TrailPoint) = .{},
    max_range_24h_nm: ?f64 = null,

    fn setCallsign(ac: *Aircraft, raw: []const u8) void {
        const n = @min(raw.len, @as(usize, 8));
        @memcpy(ac.callsign[0..n], raw[0..n]);
        ac.callsign_len = @as(u8, @intCast(n));
        if (n < 8) @memset(ac.callsign[n..8], ' ');
    }

    fn copyCallsignFromBuf(ac: *Aircraft, buf: *[9]u8) void {
        const trimmed = adsb.trimCallsignPadding(buf);
        // Only update when the decoded callsign looks meaningful.
        // In weak reception, TC 1–4 frames can decode to mostly spaces/'#' which would erase a good callsign.
        var ok = false;
        for (trimmed) |c| {
            if (c == ' ' or c == '#') continue;
            ok = true;
            break;
        }
        if (ok) ac.setCallsign(trimmed);
    }
};

pub const Table = struct {
    alloc: std.mem.Allocator,
    map: std.AutoHashMap(u32, Aircraft),
    keys: std.ArrayList(u32),
    keys_dirty: bool = false,
    start_ns: i128,
    ring_hits: [4]u64 = .{ 0, 0, 0, 0 },
    ring_total_ranged_hits: u64 = 0,
    ring_counts_since_rate_update: [4]u64 = .{ 0, 0, 0, 0 },
    ring_rate_ewma_per_min: [4]f64 = .{ 0, 0, 0, 0 },
    ring_rate_last_update_ns: i128 = 0,
    /// Antenna / receiver WGS84 position for CPR reference (surface global + relative, airborne relative).
    receiver_lat: ?f64,
    receiver_lon: ?f64,

    pub fn init(gpa: std.mem.Allocator, receiver_lat: ?f64, receiver_lon: ?f64) Table {
        return .{
            .alloc = gpa,
            .map = std.AutoHashMap(u32, Aircraft).init(gpa),
            .keys = .{},
            .keys_dirty = false,
            .start_ns = std.time.nanoTimestamp(),
            .ring_hits = .{ 0, 0, 0, 0 },
            .ring_total_ranged_hits = 0,
            .ring_counts_since_rate_update = .{ 0, 0, 0, 0 },
            .ring_rate_ewma_per_min = .{ 0, 0, 0, 0 },
            .ring_rate_last_update_ns = 0,
            .receiver_lat = receiver_lat,
            .receiver_lon = receiver_lon,
        };
    }

    pub fn deinit(self: *Table) void {
        var it = self.map.iterator();
        while (it.next()) |e| {
            e.value_ptr.trail.deinit(self.alloc);
        }
        self.map.deinit();
        self.keys.deinit(self.alloc);
    }

    pub fn getAircraftPtr(self: *Table, icao: u32) ?*Aircraft {
        return self.map.getPtr(icao);
    }

    /// JSON / HTTP snapshot (caller **must** hold `Web.Shared.mutex` if concurrent).
    pub fn snapshotForNet(self: *Table, arena: std.mem.Allocator, now_ns: i128, center_mhz: f64) !NetSnapshot {
        if (self.keys_dirty) {
            std.mem.sort(u32, self.keys.items, {}, std.sort.asc(u32));
            self.keys_dirty = false;
        }
        var list = std.ArrayList(NetAircraft).empty;
        var coverage_nm: ?f64 = null;
        var coverage_farthest_points: []const ReceiverPos = &.{};
        var bin_best_nm: [COVERAGE_BINS]f64 = [_]f64{-1.0} ** COVERAGE_BINS;
        var bin_best_pos: [COVERAGE_BINS]ReceiverPos = undefined;
        for (self.keys.items) |icao| {
            const ac_ptr = self.map.getPtr(icao) orelse continue;
            // Keep 24h rolling coverage independent of "currently seen" aircraft.
            self.pruneTrail(ac_ptr, now_ns);
            const ac = ac_ptr.*;

            if (ac.max_range_24h_nm) |r| {
                if (coverage_nm == null or r > coverage_nm.?) coverage_nm = r;
            }
            if (self.receiver_lat != null and self.receiver_lon != null) {
                for (ac.trail.items) |p| {
                    if (p.range_nm) |r| {
                        const b = bearingDeg(self.receiver_lat.?, self.receiver_lon.?, p.lat, p.lon);
                        const bin = @min(COVERAGE_BINS - 1, @as(usize, @intFromFloat(@floor(b / (360.0 / @as(f64, COVERAGE_BINS))))));
                        if (r > bin_best_nm[bin]) {
                            bin_best_nm[bin] = r;
                            bin_best_pos[bin] = .{ .lat = p.lat, .lon = p.lon };
                        }
                    }
                }
            }

            const seen_s: f64 = @as(f64, @floatFromInt(now_ns - ac.last_seen_ns)) / 1e9;
            if (seen_s > 120.0) continue;

            var hex_buf: [6]u8 = undefined;
            _ = try std.fmt.bufPrint(&hex_buf, "{x:0>6}", .{icao});
            const hex = try arena.dupe(u8, hex_buf[0..6]);

            const fl = trimTrailingSpaces(ac.callsign[0..ac.callsign_len]);
            const flight = if (fl.len == 0)
                try arena.dupe(u8, "........")
            else
                try arena.dupe(u8, fl);

            const trail = try arena.alloc(NetTrailPt, ac.trail.items.len);
            for (ac.trail.items, 0..) |p, ti| {
                trail[ti] = .{
                    .lat = p.lat,
                    .lon = p.lon,
                    .alt = p.alt_ft,
                    .t = @as(f64, @floatFromInt(p.at_ns)) / 1e9,
                };
            }
            try list.append(arena, .{
                .hex = hex,
                .flight = flight,
                .lat = ac.lat,
                .lon = ac.lon,
                .track = ac.track_deg,
                .alt_baro = ac.baro_alt_ft,
                .baro_alt_is_q = ac.baro_alt_is_q,
                .geom_alt_ft = ac.geom_alt_ft,
                .geom_alt_is_q = ac.geom_alt_is_q,
                .squawk = ac.squawk,
                .gs = ac.ground_speed_kt,
                .vert_rate = ac.vert_rate_fpm,
                .vert_rate_is_baro = ac.vert_rate_is_baro,
                .capability_ca = ac.capability_ca,
                .emitter_category_ec = ac.emitter_category_ec,
                .on_ground = ac.on_ground,
                .seen = seen_s,
                .trail = trail,
                .pos_ss = ac.pos_ss,
                .pos_nic_b = ac.pos_nic_b,
                .vel_subtype = ac.vel_subtype,
                .nac_v = ac.nac_v,
                .geom_delta_ft = ac.geom_delta_ft,
                .msg_count = ac.msg_count,
                .last_conf = ac.last_conf,
                .range_nm = latestRangeNm(&ac),
                .max_range_24h_nm = ac.max_range_24h_nm,
                .ads_b_version = ac.ads_b_version,
                .op_mesub = ac.op_mesub,
                .op_nic_a = ac.op_nic_a,
                .op_nic_c = ac.op_nic_c,
                .op_nac_p = ac.op_nac_p,
                .op_sil = ac.op_sil,
                .op_sil_per_sample = ac.op_sil_per_sample,
                .op_gva = ac.op_gva,
                .op_nic_baro = ac.op_nic_baro,
                .op_nac_v_surface = ac.op_nac_v_surface,
            });
        }

        const recv: ?ReceiverPos = if (self.receiver_lat) |la|
            .{ .lat = la, .lon = self.receiver_lon.? }
        else
            null;

        if (recv != null) {
            var pts = std.ArrayList(ReceiverPos).empty;
            for (0..COVERAGE_BINS) |i| {
                if (bin_best_nm[i] >= 0.0) try pts.append(arena, bin_best_pos[i]);
            }
            coverage_farthest_points = try pts.toOwnedSlice(arena);
        }

        return .{
            .now = @as(f64, @floatFromInt(now_ns)) / 1e9,
            .center_mhz = center_mhz,
            .receiver = recv,
            .coverage_nm = if (recv != null) coverage_nm else null,
            .coverage_farthest_points = coverage_farthest_points,
            .ring_mi = .{ 50, 100, 150, 200 },
            .ring_hits = self.ring_hits,
            .ring_total_ranged_hits = self.ring_total_ranged_hits,
            .ring_rate_ewma_per_min = self.ring_rate_ewma_per_min,
            .aircraft = try list.toOwnedSlice(arena),
        };
    }

    /// DF17/18 extended squitter; DF11 all-call (AA); DF 0/4/5 only **update** known ICAOs (see below).
    ///
    /// Address-parity short replies (DF 0/4/5) carry the ICAO in the CRC remainder, not in plaintext.
    /// Without a readsb-style `icaoFilter`, creating new rows from those remainders floods the table with
    /// noise addresses. We therefore only apply alt/squawk from DF 0/4/5 when that ICAO already exists
    /// (usually from DF17/18 or DF11).
    pub fn updateFromModeSMessage(
        self: *Table,
        now_ns: i128,
        msg: []const u8,
        bit_len: usize,
        df: u32,
        conf: f64,
        cpr_air: *std.AutoHashMap(u32, adsb.CprAirborneState),
        cpr_surf: *std.AutoHashMap(u32, adsb.CprSurfaceState),
    ) !void {
        if (bit_len == 112 and (df == 17 or df == 18)) {
            var msg14: [14]u8 = undefined;
            @memcpy(&msg14, msg[0..14]);
            return try self.updateFromEsMessage(now_ns, &msg14, conf, cpr_air, cpr_surf);
        }
        if (bit_len != 56) return;

        if (df == 11) {
            const icao: u32 = (@as(u32, msg[1]) << 16) | (@as(u32, msg[2]) << 8) | @as(u32, msg[3]);
            if (icao == 0) return;

            const gac = try self.map.getOrPut(icao);
            if (!gac.found_existing) {
                gac.value_ptr.* = .{ .icao = icao };
                try self.keys.append(self.alloc, icao);
                self.keys_dirty = true;
            }
            const ac = gac.value_ptr;
            ac.msg_count += 1;
            ac.last_seen_ns = now_ns;
            ac.last_conf = conf;
            return;
        }

        if (df == 0 or df == 4 or df == 5) {
            var msg7: [7]u8 = undefined;
            @memcpy(&msg7, msg[0..7]);
            const icao: u32 = @as(u32, crc.modesChecksum56(&msg7));
            if (icao == 0) return;

            const ac = self.map.getPtr(icao) orelse return;

            ac.msg_count += 1;
            ac.last_seen_ns = now_ns;
            ac.last_conf = conf;

            if (df == 0 or df == 4) {
                const ac13 = msdec.bitsInclusive1(msg, 20, 32);
                if (adsb.decodeAc13Feet(ac13)) |alt| {
                    ac.baro_alt_ft = alt;
                    ac.baro_alt_is_q = (ac13 & 0x0010) != 0;
                }
            }
            if (df == 5) {
                const id13 = msdec.bitsInclusive1(msg, 20, 32);
                if (adsb.squawkDecimalFromId13(id13)) |sq| {
                    ac.squawk = sq;
                }
            }
        }
    }

    pub fn updateFromEsMessage(
        self: *Table,
        now_ns: i128,
        msg: *const [14]u8,
        conf: f64,
        cpr_air: *std.AutoHashMap(u32, adsb.CprAirborneState),
        cpr_surf: *std.AutoHashMap(u32, adsb.CprSurfaceState),
    ) !void {
        const icao: u32 = (@as(u32, msg[1]) << 16) | (@as(u32, msg[2]) << 8) | @as(u32, msg[3]);

        const gop = try cpr_air.getOrPut(icao);
        if (!gop.found_existing) gop.value_ptr.* = .{};
        const cpr_state = gop.value_ptr;

        const sgop = try cpr_surf.getOrPut(icao);
        if (!sgop.found_existing) sgop.value_ptr.* = .{};
        const cpr_surf_st = sgop.value_ptr;

        const gac = try self.map.getOrPut(icao);
        if (!gac.found_existing) {
            gac.value_ptr.* = .{ .icao = icao };
            try self.keys.append(self.alloc, icao);
            self.keys_dirty = true;
        }
        const ac = gac.value_ptr;

        const tc = (msg[4] >> 3) & 0x1f;
        const me: u56 = if (adsb.typeCodeUsesMe(tc)) adsb.me56FromBytes(msg) else 0;

        ac.capability_ca = msg[0] & 0x07;
        ac.msg_count += 1;
        ac.last_seen_ns = now_ns;
        ac.last_conf = conf;

        if (tc >= 1 and tc <= 4) {
            var buf: [9]u8 = undefined;
            adsb.decodeCallsign(me, &buf);
            ac.copyCallsignFromBuf(&buf);
            if (adsb.parseAircraftIdentificationEmitterCategoryEc(me, tc)) |ec| {
                ac.emitter_category_ec = @intCast(ec);
            }
        }

        if (tc >= 5 and tc <= 8) {
            if (adsb.parseSurfacePosition(me, tc)) |sp| {
                ac.on_ground = true;
                ac.baro_alt_is_q = null;
                ac.pos_ss = null;
                ac.pos_nic_b = null;
                if (sp.movement > 0 and sp.movement < 125) {
                    ac.ground_speed_kt = adsb.decodeMovementFieldV0(sp.movement);
                }
                if (sp.heading_valid) {
                    ac.track_deg = sp.heading_deg;
                }
                if (adsb.updateCprSurfaceAndMaybeDecode(cpr_surf_st, sp, sp.f_odd, now_ns, self.receiver_lat, self.receiver_lon)) |fix| {
                    ac.lat = fix.lat;
                    ac.lon = fix.lon;
                    try self.maybeAppendTrailPoint(ac, now_ns, fix.lat, fix.lon);
                }
            }
        }

        if ((tc >= 9 and tc <= 18) or (tc >= 20 and tc <= 22)) {
            if (adsb.parseAirbornePosition(me, tc)) |pos| {
                ac.on_ground = false;
                ac.pos_ss = @intCast(pos.ss);
                ac.pos_nic_b = @intCast(pos.nic_b);
                if (adsb.airbornePositionIsBaroAltitude(tc)) {
                    ac.baro_alt_is_q = (pos.alt12 & 0x10) != 0;
                    if (adsb.decodeAc12Feet(pos.alt12)) |alt| {
                        ac.baro_alt_ft = alt;
                    }
                } else {
                    ac.geom_alt_is_q = (pos.alt12 & 0x10) != 0;
                    if (adsb.decodeAc12Feet(pos.alt12)) |alt| {
                        ac.geom_alt_ft = alt;
                    }
                }
                if (adsb.updateCprAndMaybeDecode(cpr_state, pos, pos.f_odd, now_ns, self.receiver_lat, self.receiver_lon)) |fix| {
                    ac.lat = fix.lat;
                    ac.lon = fix.lon;
                    try self.maybeAppendTrailPoint(ac, now_ns, fix.lat, fix.lon);
                }
            }
        }

        if (tc == 31) {
            ac.op_mesub = null;
            ac.ads_b_version = null;
            ac.op_nic_a = null;
            ac.op_nic_c = null;
            ac.op_nac_p = null;
            ac.op_sil = null;
            ac.op_sil_per_sample = null;
            ac.op_gva = null;
            ac.op_nic_baro = null;
            ac.op_nac_v_surface = null;
            if (adsb.parseOperationalStatus(me, tc)) |op| {
                ac.op_mesub = @intCast(op.mesub);
                ac.ads_b_version = @intCast(op.version);
                if (op.nic_a) |v| ac.op_nic_a = @intCast(v);
                if (op.nic_c) |v| ac.op_nic_c = @intCast(v);
                if (op.nac_p) |v| ac.op_nac_p = @intCast(v);
                if (op.sil) |v| ac.op_sil = @intCast(v);
                ac.op_sil_per_sample = op.sil_per_sample;
                if (op.gva) |v| ac.op_gva = @intCast(v);
                if (op.nic_baro) |v| ac.op_nic_baro = @intCast(v);
                if (op.nac_v_surface) |v| ac.op_nac_v_surface = @intCast(v);
            }
        }

        if (tc == 28) {
            if (adsb.parseTc28EmergencySquawkMe(me)) |sq| {
                ac.squawk = sq;
            }
        }

        if (tc == 19) {
            if (adsb.parseVelocity(me, tc)) |v| {
                ac.vel_subtype = @intCast(v.st);
                ac.nac_v = @intCast(v.nac_v);
                ac.geom_delta_ft = v.geom_delta_ft;
                if (v.vert_rate_fpm) |r| {
                    ac.vert_rate_fpm = r;
                    ac.vert_rate_is_baro = v.vert_rate_is_baro;
                }
                if (v.st == 1 or v.st == 2 or v.st == 3 or v.st == 4) {
                    ac.ground_speed_kt = v.ground_speed_kt;
                    ac.track_deg = v.track_deg;
                }
            }
        }
    }

    pub fn render(self: *Table, w: *std.Io.Writer, now_ns: i128, center_mhz: f64) !void {
        try w.writeAll("\x1b[2J\x1b[H");
        if (self.receiver_lat) |rla| {
            const rlo = self.receiver_lon.?;
            try w.print(
                "ADS-B  {d:.3} MHz  -  receiver {d:.4},{d:.4}  -  libusb: 2>/dev/null to hide driver stderr\n",
                .{ center_mhz, rla, rlo },
            );
        } else {
            try w.print(
                "ADS-B  {d:.3} MHz  -  no --lat/--lon (CPR ref weaker)  -  libusb: 2>/dev/null to hide driver stderr\n",
                .{center_mhz},
            );
        }
        // Same layout as each data row: `{x:0>6} {s:<8.8} … {d:>7.3}` (see `term_table_widths`).
        try writeTermTableHeader(w);
        try writeTermTableRule(w);

        if (self.keys_dirty) {
            std.mem.sort(u32, self.keys.items, {}, std.sort.asc(u32));
            self.keys_dirty = false;
        }

        var flight_buf: [9]u8 = undefined;
        var b_alt: [16]u8 = undefined;
        var b_galt: [16]u8 = undefined;
        var b_sqk: [8]u8 = undefined;
        var b_gs: [16]u8 = undefined;
        var b_trk: [16]u8 = undefined;
        var b_vr: [16]u8 = undefined;
        var b_lat: [16]u8 = undefined;
        var b_lon: [16]u8 = undefined;
        var b_gd: [16]u8 = undefined;
        var b_ssnb: [8]u8 = undefined;
        var b_st: [8]u8 = undefined;
        var b_nv: [8]u8 = undefined;
        var b_ec: [8]u8 = undefined;
        var b_rng: [16]u8 = undefined;
        var b_max_rng: [16]u8 = undefined;
        var b_trl: [8]u8 = undefined;

        for (self.keys.items) |icao| {
            const ac = self.map.getPtr(icao) orelse continue;
            const seen_s: f64 = @as(f64, @floatFromInt(now_ns - ac.last_seen_ns)) / 1e9;
            if (seen_s > 120.0) continue;
            self.pruneTrail(ac, now_ns);

            @memcpy(flight_buf[0..8], ac.callsign[0..8]);
            flight_buf[8] = 0;
            const fl = trimTrailingSpaces(flight_buf[0..ac.callsign_len]);

            const alt_s = fmtOptI32(&b_alt, ac.baro_alt_ft);
            const galt_s = fmtOptI32(&b_galt, ac.geom_alt_ft);
            const ac_enc_s = baroAltEncodingStr(ac.baro_alt_is_q);
            const sqk_s = fmtSquawk(&b_sqk, ac.squawk);
            const gs_s = fmtOptF32(&b_gs, ac.ground_speed_kt, "{d:.0}");
            const trk_s = fmtOptF32(&b_trk, ac.track_deg, "{d:.0}");
            const vr_s = fmtOptI32(&b_vr, ac.vert_rate_fpm);
            const vb_s = vertRateBaroStr(ac.vert_rate_fpm, ac.vert_rate_is_baro);
            const lat_s = fmtOptF64(&b_lat, ac.lat, "{d:.4}");
            const lon_s = fmtOptF64(&b_lon, ac.lon, "{d:.4}");
            const ss_nb_s = fmtSsNic(&b_ssnb, ac.pos_ss, ac.pos_nic_b);
            const st_s = fmtOptU8Dash(&b_st, ac.vel_subtype);
            const nv_s = fmtOptU8Dash(&b_nv, ac.nac_v);
            const gd_s = fmtOptI32(&b_gd, ac.geom_delta_ft);
            const ec_s = fmtOptU8Dash(&b_ec, ac.emitter_category_ec);
            const rng_s = fmtOptF64(&b_rng, latestRangeNm(ac), "{d:.1}");
            const max_rng_s = fmtOptF64(&b_max_rng, ac.max_range_24h_nm, "{d:.1}");
            const trl_s = std.fmt.bufPrint(&b_trl, "{d}", .{ac.trail.items.len}) catch "---";

            const flight_disp = if (fl.len == 0) "........" else fl;

            // NOTE: widths in Zig are minimum widths; precision caps max width.
            // The `.N` precisions below keep the table columns aligned even if any field is longer.
            var row_buf: [512]u8 = undefined;
            const row = std.fmt.bufPrint(&row_buf, "{x:0>6} {s:<8.8} {s:>6.6} {s:>6.6} {s:>2.2} {s:>5.5} {s:>5.5} {s:>5.5} {s:>5.5} {s:>2.2} {s:>10.10} {s:>10.10} {s:>4.4} {s:>6.6} {s:>3.3} {s:>4.4} {s:>2.2} {s:>2.2} {s:>5.5} {d:>2} {s:>2.2} {d:>5} {d:>4.0} {s:>1.1} {d:>7.3}\n", .{
                icao,
                flight_disp,
                alt_s,
                galt_s,
                ac_enc_s,
                sqk_s,
                gs_s,
                trk_s,
                vr_s,
                vb_s,
                lat_s,
                lon_s,
                rng_s,
                max_rng_s,
                trl_s,
                ss_nb_s,
                st_s,
                nv_s,
                gd_s,
                ac.capability_ca,
                ec_s,
                ac.msg_count,
                seen_s,
                if (ac.on_ground) "G" else " ",
                ac.last_conf,
            }) catch {
                try w.print("{x:0>6} {s:<8.8} {s:>6.6} {s:>6.6} {s:>2.2} {s:>5.5} {s:>5.5} {s:>5.5} {s:>5.5} {s:>2.2} {s:>10.10} {s:>10.10} {s:>4.4} {s:>6.6} {s:>3.3} {s:>4.4} {s:>2.2} {s:>2.2} {s:>5.5} {d:>2} {s:>2.2} {d:>5} {d:>4.0} {s:>1.1} {d:>7.3}\n", .{
                    icao,
                    flight_disp,
                    alt_s,
                    galt_s,
                    ac_enc_s,
                    sqk_s,
                    gs_s,
                    trk_s,
                    vr_s,
                    vb_s,
                    lat_s,
                    lon_s,
                    rng_s,
                    max_rng_s,
                    trl_s,
                    ss_nb_s,
                    st_s,
                    nv_s,
                    gd_s,
                    ac.capability_ca,
                    ec_s,
                    ac.msg_count,
                    seen_s,
                    if (ac.on_ground) "G" else " ",
                    ac.last_conf,
                });
                continue;
            };
            try w.writeAll(row);
        }
    }

    fn maybeAppendTrailPoint(self: *Table, ac: *Aircraft, now_ns: i128, lat: f64, lon: f64) !void {
        self.pruneTrail(ac, now_ns);
        const range_nm = if (self.receiver_lat != null and self.receiver_lon != null)
            greatCircleNm(self.receiver_lat.?, self.receiver_lon.?, lat, lon)
        else
            null;

        if (ac.trail.items.len > 0) {
            const prev = ac.trail.items[ac.trail.items.len - 1];
            if (now_ns - prev.at_ns < TRAIL_SAMPLE_MIN_NS) {
                return;
            }
            const moved_nm = greatCircleNm(prev.lat, prev.lon, lat, lon);
            if (moved_nm < TRAIL_MOVE_MIN_NM) {
                return;
            }
        }

        try ac.trail.append(self.alloc, .{
            .at_ns = now_ns,
            .lat = lat,
            .lon = lon,
            .alt_ft = ac.baro_alt_ft orelse ac.geom_alt_ft,
            .range_nm = range_nm,
        });

        if (range_nm) |r| {
            // Simple ring stats (since startup): cumulative <= 50/100/150/200 **statute miles**.
            const r_mi = r * NM_TO_STATUTE_MI;
            self.ring_total_ranged_hits +%= 1;
            if (r_mi <= 50.0) {
                self.ring_hits[0] +%= 1;
                self.ring_counts_since_rate_update[0] +%= 1;
            }
            if (r_mi <= 100.0) {
                self.ring_hits[1] +%= 1;
                self.ring_counts_since_rate_update[1] +%= 1;
            }
            if (r_mi <= 150.0) {
                self.ring_hits[2] +%= 1;
                self.ring_counts_since_rate_update[2] +%= 1;
            }
            if (r_mi <= 200.0) {
                self.ring_hits[3] +%= 1;
                self.ring_counts_since_rate_update[3] +%= 1;
            }
            self.maybeUpdateRingRates(now_ns);
            if (ac.max_range_24h_nm == null or r > ac.max_range_24h_nm.?) {
                ac.max_range_24h_nm = r;
            }
        }
    }

    fn maybeUpdateRingRates(self: *Table, now_ns: i128) void {
        if (self.ring_rate_last_update_ns == 0) {
            self.ring_rate_last_update_ns = now_ns;
            return;
        }
        const dt_ns: i128 = now_ns - self.ring_rate_last_update_ns;
        // Update at most ~2 Hz to reduce overhead.
        if (dt_ns < 500 * std.time.ns_per_ms) return;

        const dt_s: f64 = @as(f64, @floatFromInt(dt_ns)) / 1e9;
        if (dt_s <= 0) return;

        // Convert half-life to time constant for continuous-time EMA:
        // alpha = 1 - exp(-dt/tau), tau = halflife / ln(2)
        const tau: f64 = RING_HALFLIFE_S / std.math.ln2;
        const alpha: f64 = 1.0 - std.math.exp(-dt_s / tau);

        for (0..4) |i| {
            const c: f64 = @as(f64, @floatFromInt(self.ring_counts_since_rate_update[i]));
            const rate_per_min: f64 = (c / dt_s) * 60.0;
            self.ring_rate_ewma_per_min[i] = self.ring_rate_ewma_per_min[i] * (1.0 - alpha) + rate_per_min * alpha;
            self.ring_counts_since_rate_update[i] = 0;
        }
        self.ring_rate_last_update_ns = now_ns;
    }

    fn pruneTrail(self: *Table, ac: *Aircraft, now_ns: i128) void {
        // Keep order, but prune in bulk to avoid O(n^2) repeated head removals.
        var drop: usize = 0;
        while (drop < ac.trail.items.len and now_ns - ac.trail.items[drop].at_ns > TRAIL_RETENTION_NS) {
            drop += 1;
        }
        if (drop > 0) {
            const keep = ac.trail.items.len - drop;
            std.mem.copyForwards(TrailPoint, ac.trail.items[0..keep], ac.trail.items[drop..]);
            ac.trail.items = ac.trail.items[0..keep];
        }

        var max_nm: ?f64 = null;
        for (ac.trail.items) |p| {
            if (p.range_nm) |r| {
                if (max_nm == null or r > max_nm.?) {
                    max_nm = r;
                }
            }
        }
        ac.max_range_24h_nm = max_nm;
        _ = self;
    }
};

fn writeTermTableHeader(w: *std.Io.Writer) !void {
    try w.print(
        "{s:>6} {s:<8} {s:>6} {s:>6} {s:>2} {s:>5} {s:>5} {s:>5} {s:>5} {s:>2} {s:>10} {s:>10} {s:>4} {s:>6} {s:>3} {s:>4} {s:>2} {s:>2} {s:>5} {s:>2} {s:>2} {s:>5} {s:>4} {s:>1} {s:>7}\n",
        .{ "ICAO", "Flight", "Alt", "GAlt", "AC", "Sqk", "GS", "Trk", "Vrate", "Vb", "Lat", "Lon", "Rng", "Max24h", "Trl", "SS/n", "ST", "Nv", "Gd", "CA", "EC", "Msgs", "Seen", "G", "Conf" },
    );
}

fn writeTermTableRule(w: *std.Io.Writer) !void {
    // One contiguous line: avoid ~100+ tiny `writeAll("-")` calls per refresh (was measurable vs. demod loop).
    const RULE_LINE_LEN: usize = comptime blk: {
        var n: usize = 0;
        for (term_table_widths, 0..) |wd, i| {
            n += wd;
            if (i != 0) n += 1; // spaces between columns
        }
        n += 1; // trailing newline
        break :blk n;
    };
    var buf: [RULE_LINE_LEN]u8 = undefined;
    var pos: usize = 0;
    for (term_table_widths, 0..) |wd, i| {
        if (i != 0) {
            buf[pos] = ' ';
            pos += 1;
        }
        @memset(buf[pos..][0..wd], '-');
        pos += wd;
    }
    buf[pos] = '\n';
    pos += 1;
    try w.writeAll(buf[0..pos]);
}

fn trimTrailingSpaces(s: []const u8) []const u8 {
    var end: usize = s.len;
    while (end > 0 and s[end - 1] == ' ') end -= 1;
    return s[0..end];
}

fn baroAltEncodingStr(is_q: ?bool) []const u8 {
    if (is_q) |q| return if (q) "Q" else "M";
    return "-";
}

fn fmtSquawk(buf: []u8, v: ?u16) []const u8 {
    if (v) |x| {
        return std.fmt.bufPrint(buf, "{d:0>4}", .{x}) catch return "----";
    }
    return "----";
}

fn fmtOptI32(buf: []u8, v: ?i32) []const u8 {
    if (v) |x| {
        return std.fmt.bufPrint(buf, "{d}", .{x}) catch return "----";
    }
    return "----";
}

fn fmtOptF32(buf: []u8, v: ?f32, comptime fmt: []const u8) []const u8 {
    if (v) |x| {
        return std.fmt.bufPrint(buf, fmt, .{x}) catch return "---";
    }
    return "---";
}

fn fmtOptF64(buf: []u8, v: ?f64, comptime fmt: []const u8) []const u8 {
    if (v) |x| {
        return std.fmt.bufPrint(buf, fmt, .{x}) catch return "----";
    }
    return "----";
}

fn fmtOptU8Dash(buf: []u8, v: ?u8) []const u8 {
    if (v) |x| {
        return std.fmt.bufPrint(buf, "{d}", .{x}) catch return "-";
    }
    return "-";
}

fn fmtSsNic(buf: []u8, ss: ?u8, nic_b: ?u8) []const u8 {
    if (ss == null or nic_b == null) return " -- ";
    return std.fmt.bufPrint(buf, "{d}/{d}", .{ ss.?, nic_b.? }) catch return "--/--";
}

fn vertRateBaroStr(vr: ?i32, is_baro: ?bool) []const u8 {
    if (vr == null) return "-";
    if (is_baro) |b| return if (b) "B" else "G";
    return "?";
}

fn latestRangeNm(ac: *const Aircraft) ?f64 {
    if (ac.trail.items.len == 0) return null;
    return ac.trail.items[ac.trail.items.len - 1].range_nm;
}

fn greatCircleNm(lat1_deg: f64, lon1_deg: f64, lat2_deg: f64, lon2_deg: f64) f64 {
    const d2r = std.math.pi / 180.0;
    const lat1 = lat1_deg * d2r;
    const lat2 = lat2_deg * d2r;
    const dlat = (lat2_deg - lat1_deg) * d2r;
    const dlon = (lon2_deg - lon1_deg) * d2r;

    const s_dlat = std.math.sin(dlat / 2.0);
    const s_dlon = std.math.sin(dlon / 2.0);
    const a = s_dlat * s_dlat + std.math.cos(lat1) * std.math.cos(lat2) * s_dlon * s_dlon;
    const c = 2.0 * std.math.atan2(@sqrt(a), @sqrt(@max(0.0, 1.0 - a)));
    return 3440.065 * c;
}

fn bearingDeg(lat1_deg: f64, lon1_deg: f64, lat2_deg: f64, lon2_deg: f64) f64 {
    const d2r = std.math.pi / 180.0;
    const r2d = 180.0 / std.math.pi;
    const lat1 = lat1_deg * d2r;
    const lat2 = lat2_deg * d2r;
    const dlon = (lon2_deg - lon1_deg) * d2r;
    const y = std.math.sin(dlon) * std.math.cos(lat2);
    const x = std.math.cos(lat1) * std.math.sin(lat2) - std.math.sin(lat1) * std.math.cos(lat2) * std.math.cos(dlon);
    var brng = std.math.atan2(y, x) * r2d;
    if (brng < 0.0) brng += 360.0;
    return brng;
}
