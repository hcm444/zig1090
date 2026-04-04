const std = @import("std");
const aircraft_table = @import("aircraft_table.zig");

pub const Shared = struct {
    mutex: std.Thread.Mutex = .{},
    table: *aircraft_table.Table,
    center_mhz: f64,
};

const index_html = @embedFile("web/index.html");

const static_leaflet_css = @embedFile("web/static/leaflet/leaflet.css");
const static_leaflet_js = @embedFile("web/static/leaflet/leaflet.js");
const static_img_marker_icon = @embedFile("web/static/leaflet/images/marker-icon.png");
const static_img_marker_icon_2x = @embedFile("web/static/leaflet/images/marker-icon-2x.png");
const static_img_marker_shadow = @embedFile("web/static/leaflet/images/marker-shadow.png");
const static_img_layers = @embedFile("web/static/leaflet/images/layers.png");
const static_img_layers_2x = @embedFile("web/static/leaflet/images/layers-2x.png");

fn targetPath(target: []const u8) []const u8 {
    if (std.mem.indexOfScalar(u8, target, '?')) |i| return target[0..i];
    return target;
}

pub fn spawnListener(shared: *Shared, port: u16) !std.Thread {
    return std.Thread.spawn(.{}, listenerMain, .{ shared, port });
}

fn listenerMain(shared: *Shared, port: u16) void {
    listenerMainErr(shared, port) catch |err| {
        std.log.err("HTTP server failed: {s}", .{@errorName(err)});
    };
}

fn listenerMainErr(shared: *Shared, port: u16) !void {
    const addr = try std.net.Address.parseIp4("0.0.0.0", port);
    var listener = try addr.listen(.{ .reuse_address = true });
    defer listener.deinit();

    std.log.info("map UI: http://127.0.0.1:{d}/  (JSON: /aircraft.json)", .{port});

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    while (true) {
        const conn = listener.accept() catch |err| {
            std.log.warn("accept: {s}", .{@errorName(err)});
            continue;
        };
        handleConnection(allocator, shared, conn) catch |err| {
            std.log.warn("connection: {s}", .{@errorName(err)});
        };
    }
}

fn respondStatic(req: *std.http.Server.Request, body: []const u8, content_type: []const u8) !void {
    try req.respond(body, .{
        .extra_headers = &.{
            .{ .name = "content-type", .value = content_type },
            .{ .name = "cache-control", .value = "public, max-age=86400" },
        },
    });
}

fn handleConnection(gpa: std.mem.Allocator, shared: *Shared, conn: std.net.Server.Connection) !void {
    defer conn.stream.close();

    var read_buf: [16384]u8 = undefined;
    var write_buf: [65536]u8 = undefined;
    var nr = conn.stream.reader(&read_buf);
    var nw = conn.stream.writer(&write_buf);
    var http_srv = std.http.Server.init(nr.interface(), &nw.interface);

    while (true) {
        var req = http_srv.receiveHead() catch break;
        const path = targetPath(req.head.target);

        if (req.head.method != .GET and req.head.method != .HEAD) {
            try req.respond("Method Not Allowed", .{ .status = .method_not_allowed, .keep_alive = false });
            break;
        }

        if (std.mem.eql(u8, path, "/") or std.mem.eql(u8, path, "/index.html")) {
            try req.respond(index_html, .{
                .extra_headers = &.{.{ .name = "content-type", .value = "text/html; charset=utf-8" }},
            });
        } else if (std.mem.eql(u8, path, "/aircraft.json") or std.mem.eql(u8, path, "/data/aircraft.json")) {
            var arena = std.heap.ArenaAllocator.init(gpa);
            defer arena.deinit();
            const now_ns = std.time.nanoTimestamp();

            shared.mutex.lock();
            const snap = shared.table.snapshotForNet(arena.allocator(), now_ns, shared.center_mhz) catch |e| {
                shared.mutex.unlock();
                return e;
            };
            shared.mutex.unlock();

            const json_bytes = try std.json.Stringify.valueAlloc(gpa, snap, .{});
            defer gpa.free(json_bytes);
            try req.respond(json_bytes, .{
                .extra_headers = &.{
                    .{ .name = "content-type", .value = "application/json; charset=utf-8" },
                    .{ .name = "cache-control", .value = "no-store" },
                },
            });
        } else if (std.mem.eql(u8, path, "/static/leaflet/leaflet.css")) {
            try respondStatic(&req, static_leaflet_css, "text/css; charset=utf-8");
        } else if (std.mem.eql(u8, path, "/static/leaflet/leaflet.js")) {
            try respondStatic(&req, static_leaflet_js, "application/javascript; charset=utf-8");
        } else if (std.mem.eql(u8, path, "/static/leaflet/images/marker-icon.png")) {
            try respondStatic(&req, static_img_marker_icon, "image/png");
        } else if (std.mem.eql(u8, path, "/static/leaflet/images/marker-icon-2x.png")) {
            try respondStatic(&req, static_img_marker_icon_2x, "image/png");
        } else if (std.mem.eql(u8, path, "/static/leaflet/images/marker-shadow.png")) {
            try respondStatic(&req, static_img_marker_shadow, "image/png");
        } else if (std.mem.eql(u8, path, "/static/leaflet/images/layers.png")) {
            try respondStatic(&req, static_img_layers, "image/png");
        } else if (std.mem.eql(u8, path, "/static/leaflet/images/layers-2x.png")) {
            try respondStatic(&req, static_img_layers_2x, "image/png");
        } else if (std.mem.eql(u8, path, "/favicon.ico")) {
            try req.respond("", .{ .status = .not_found, .keep_alive = true });
        } else {
            try req.respond("Not Found", .{ .status = .not_found, .keep_alive = true });
        }

        if (!req.head.keep_alive) break;
    }
}
