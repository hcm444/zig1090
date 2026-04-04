//! RTL-SDR IQ capture for zig1090 without ZigRadio.
//! Device setup, IQ scaling, and read sizing follow ZigRadio's `RtlSdrSource`
//! (MIT License, Copyright (c) Ivan (Vanya) A. Sergeev).
const std = @import("std");

const C32 = std.math.Complex(f32);

/// Must match the demod frame length in `main.zig`.
pub const frame_len: usize = 8192;

pub const DirectSamplingMode = enum { I, Q };

pub const Options = struct {
    bias_tee: bool = false,
    direct_sampling: ?DirectSamplingMode = null,
    bandwidth: ?f32 = null,
    rf_gain: ?f32 = null,
    freq_correction: isize = 0,
    device_index: usize = 0,
    device_serial: ?[]const u8 = null,
    debug: bool = false,
};

pub const RtlSdrError = error{
    /// No RTL-SDR at the chosen index, USB unplugged, or serial not found.
    SdrNotDetected,
    InitializationError,
    ReadError,
};

const rtlsdr_dev_t = opaque {};

extern "c" fn rtlsdr_open(out: [*c]?*rtlsdr_dev_t, index: u32) callconv(.c) c_int;
extern "c" fn rtlsdr_get_index_by_serial(serial: [*c]const u8) callconv(.c) c_int;
extern "c" fn rtlsdr_get_device_name(index: u32) callconv(.c) [*c]const u8;
extern "c" fn rtlsdr_get_usb_strings(dev: ?*rtlsdr_dev_t, manufact: [*c]u8, product: [*c]u8, serial: [*c]u8) callconv(.c) c_int;
extern "c" fn rtlsdr_set_bias_tee(dev: ?*rtlsdr_dev_t, on: c_int) callconv(.c) c_int;
extern "c" fn rtlsdr_set_direct_sampling(dev: ?*rtlsdr_dev_t, on: c_int) callconv(.c) c_int;
extern "c" fn rtlsdr_set_tuner_gain_mode(dev: ?*rtlsdr_dev_t, manual: c_int) callconv(.c) c_int;
extern "c" fn rtlsdr_set_agc_mode(dev: ?*rtlsdr_dev_t, on: c_int) callconv(.c) c_int;
extern "c" fn rtlsdr_set_tuner_gain(dev: ?*rtlsdr_dev_t, gain: c_int) callconv(.c) c_int;
extern "c" fn rtlsdr_set_freq_correction(dev: ?*rtlsdr_dev_t, ppm: c_int) callconv(.c) c_int;
extern "c" fn rtlsdr_set_center_freq(dev: ?*rtlsdr_dev_t, freq: u32) callconv(.c) c_int;
extern "c" fn rtlsdr_get_center_freq(dev: ?*rtlsdr_dev_t) callconv(.c) u32;
extern "c" fn rtlsdr_set_sample_rate(dev: ?*rtlsdr_dev_t, rate: u32) callconv(.c) c_int;
extern "c" fn rtlsdr_get_sample_rate(dev: ?*rtlsdr_dev_t) callconv(.c) u32;
extern "c" fn rtlsdr_set_tuner_bandwidth(dev: ?*rtlsdr_dev_t, bw: u32) callconv(.c) c_int;
extern "c" fn rtlsdr_reset_buffer(dev: ?*rtlsdr_dev_t) callconv(.c) c_int;
extern "c" fn rtlsdr_read_sync(dev: ?*rtlsdr_dev_t, buf: ?*anyopaque, len: c_int, n_read: [*c]c_int) callconv(.c) c_int;
extern "c" fn rtlsdr_close(dev: ?*rtlsdr_dev_t) callconv(.c) c_int;

const min_block_bytes: usize = 8192;
const read_buf_factor: usize = 16 * 2;

pub const RtlIqStream = struct {
    allocator: std.mem.Allocator,
    frequency: f64,
    rate: f64,
    options: Options,

    dev: ?*rtlsdr_dev_t = null,
    read_buf: []u8 = &.{},
    convert_buf: []C32 = &.{},

    ring: []C32 = &.{},
    ring_mask: usize = 0,
    head: usize = 0,
    tail: usize = 0,
    count: usize = 0,
    mutex: std.Thread.Mutex = .{},
    nonempty: std.Thread.Condition = .{},
    nonfull: std.Thread.Condition = .{},
    eof: bool = false,

    scratch: [frame_len]C32 = undefined,

    thread: ?std.Thread = null,
    stop: std.atomic.Value(bool) = std.atomic.Value(bool).init(false),

    pub fn init(allocator: std.mem.Allocator, frequency: f64, rate: f64, options: Options) RtlSdrError!RtlIqStream {
        var self = RtlIqStream{
            .allocator = allocator,
            .frequency = frequency,
            .rate = rate,
            .options = options,
        };

        const device_index: u32 = blk: {
            if (options.device_serial) |serial| {
                const ret = rtlsdr_get_index_by_serial(@ptrCast(serial.ptr));
                if (ret < 0) {
                    if (options.debug) std.debug.print("rtlsdr_get_index_by_serial(): {d}\n", .{ret});
                    return RtlSdrError.SdrNotDetected;
                }
                break :blk @intCast(ret);
            } else {
                break :blk @intCast(options.device_index);
            }
        };

        var ret = rtlsdr_open(&self.dev, device_index);
        if (ret != 0) {
            if (options.debug) std.debug.print("rtlsdr_open(): {d}\n", .{ret});
            return RtlSdrError.SdrNotDetected;
        }
        errdefer {
            if (self.dev) |d| _ = rtlsdr_close(d);
            self.dev = null;
        }

        if (options.debug) {
            const device_name = rtlsdr_get_device_name(@intCast(options.device_index));
            var usb_manufacturer: [256:0]u8 = undefined;
            var usb_product: [256:0]u8 = undefined;
            var usb_serial: [256:0]u8 = undefined;
            ret = rtlsdr_get_usb_strings(self.dev, &usb_manufacturer, &usb_product, &usb_serial);
            if (ret != 0) {
                std.debug.print("rtlsdr_get_usb_strings(): {d}\n", .{ret});
                return RtlSdrError.InitializationError;
            }
            std.debug.print("[RtlIqStream] Device name:       {s}\n", .{device_name});
            std.debug.print("[RtlIqStream] USB Manufacturer:  {s}\n", .{usb_manufacturer});
            std.debug.print("[RtlIqStream] USB Product:       {s}\n", .{usb_product});
            std.debug.print("[RtlIqStream] USB Serial:        {s}\n", .{usb_serial});
        }

        if (options.bias_tee) {
            ret = rtlsdr_set_bias_tee(self.dev, 1);
            if (ret != 0) {
                std.debug.print("rtlsdr_set_bias_tee(): {d}\n", .{ret});
                return RtlSdrError.InitializationError;
            }
        }

        if (options.direct_sampling) |ds| {
            ret = rtlsdr_set_direct_sampling(self.dev, if (ds == DirectSamplingMode.I) 1 else 2);
            if (ret != 0) {
                std.debug.print("rtlsdr_set_direct_sampling(): {d}\n", .{ret});
                return RtlSdrError.InitializationError;
            }
        }

        if (options.rf_gain == null) {
            ret = rtlsdr_set_tuner_gain_mode(self.dev, 0);
            if (ret != 0) {
                std.debug.print("rtlsdr_set_tuner_gain_mode(): {d}\n", .{ret});
                return RtlSdrError.InitializationError;
            }
            ret = rtlsdr_set_agc_mode(self.dev, 1);
            if (ret != 0) {
                std.debug.print("rtlsdr_set_agc_mode(): {d}\n", .{ret});
                return RtlSdrError.InitializationError;
            }
        } else {
            ret = rtlsdr_set_tuner_gain_mode(self.dev, 1);
            if (ret != 0) {
                std.debug.print("rtlsdr_set_tuner_gain_mode(): {d}\n", .{ret});
                return RtlSdrError.InitializationError;
            }
            ret = rtlsdr_set_agc_mode(self.dev, 0);
            if (ret != 0) {
                std.debug.print("rtlsdr_set_agc_mode(): {d}\n", .{ret});
                return RtlSdrError.InitializationError;
            }
            ret = rtlsdr_set_tuner_gain(self.dev, @intFromFloat(options.rf_gain.? * 10.0));
            if (ret != 0) {
                std.debug.print("rtlsdr_set_tuner_gain(): {d}\n", .{ret});
                return RtlSdrError.InitializationError;
            }
        }

        if (options.debug) {
            std.debug.print("[RtlIqStream] Frequency: {d} Hz, Sample rate: {d} Hz\n", .{ frequency, rate });
        }

        ret = rtlsdr_set_freq_correction(self.dev, @intCast(options.freq_correction));
        if (ret != 0 and ret != -2) {
            std.debug.print("rtlsdr_set_freq_correction(): {d}\n", .{ret});
            return RtlSdrError.InitializationError;
        }

        ret = rtlsdr_set_sample_rate(self.dev, @intFromFloat(rate));
        if (ret != 0) {
            std.debug.print("rtlsdr_set_sample_rate(): {d}\n", .{ret});
            return RtlSdrError.InitializationError;
        }

        ret = rtlsdr_set_tuner_bandwidth(self.dev, if (options.bandwidth) |bw| @intFromFloat(bw) else 0);
        if (ret != 0) {
            std.debug.print("rtlsdr_set_tuner_bandwidth(): {d}\n", .{ret});
            return RtlSdrError.InitializationError;
        }

        ret = rtlsdr_set_center_freq(self.dev, @intFromFloat(frequency));
        if (ret != 0) {
            std.debug.print("rtlsdr_set_center_freq(): {d}\n", .{ret});
            return RtlSdrError.InitializationError;
        }

        if (options.debug) {
            const cf = rtlsdr_get_center_freq(self.dev);
            const sr = rtlsdr_get_sample_rate(self.dev);
            std.debug.print("[RtlIqStream] Configured Frequency: {d} Hz, Configured Sample Rate: {d} Hz\n", .{ cf, sr });
        }

        ret = rtlsdr_reset_buffer(self.dev);
        if (ret != 0) {
            std.debug.print("rtlsdr_reset_buffer(): {d}\n", .{ret});
            return RtlSdrError.InitializationError;
        }

        self.read_buf = allocator.alloc(u8, read_buf_factor * min_block_bytes) catch return RtlSdrError.InitializationError;
        errdefer allocator.free(self.read_buf);

        self.convert_buf = allocator.alloc(C32, self.read_buf.len / 2) catch {
            allocator.free(self.read_buf);
            return RtlSdrError.InitializationError;
        };
        errdefer allocator.free(self.convert_buf);

        const ring_cap: usize = 262_144;
        std.debug.assert(ring_cap > frame_len * 4);
        std.debug.assert(std.math.isPowerOfTwo(ring_cap));
        self.ring = allocator.alloc(C32, ring_cap) catch {
            allocator.free(self.convert_buf);
            allocator.free(self.read_buf);
            return RtlSdrError.InitializationError;
        };
        self.ring_mask = ring_cap - 1;

        return self;
    }

    pub fn start(self: *RtlIqStream) !void {
        self.stop.store(false, .release);
        self.thread = try std.Thread.spawn(.{}, readerMain, .{self});
    }

    pub fn wait(self: *RtlIqStream, min_count: usize, timeout_ns: ?u64) error{ EndOfStream, Timeout }!void {
        self.mutex.lock();
        defer self.mutex.unlock();

        while (self.count < min_count) {
            if (self.eof) return error.EndOfStream;
            if (timeout_ns) |t| {
                self.nonempty.timedWait(&self.mutex, t) catch {
                    if (self.count < min_count) {
                        if (self.eof) return error.EndOfStream;
                        return error.Timeout;
                    }
                };
            } else {
                self.nonempty.wait(&self.mutex);
            }
        }
    }

    pub fn get(self: *RtlIqStream) []const C32 {
        self.mutex.lock();
        defer self.mutex.unlock();
        std.debug.assert(self.count >= frame_len);

        var rd = self.head;
        for (0..frame_len) |i| {
            self.scratch[i] = self.ring[rd & self.ring_mask];
            rd += 1;
        }
        return self.scratch[0..frame_len];
    }

    pub fn update(self: *RtlIqStream, n: usize) void {
        self.mutex.lock();
        defer self.mutex.unlock();
        std.debug.assert(self.count >= n);
        self.head += n;
        self.count -= n;
        self.nonfull.broadcast();
    }

    pub fn deinit(self: *RtlIqStream) void {
        self.stop.store(true, .release);

        self.mutex.lock();
        self.nonfull.broadcast();
        self.nonempty.broadcast();
        self.mutex.unlock();

        if (self.thread) |th| {
            th.join();
            self.thread = null;
        }

        if (self.options.bias_tee) {
            _ = rtlsdr_set_bias_tee(self.dev, 0);
        }
        if (self.dev) |dev| {
            _ = rtlsdr_close(dev);
            self.dev = null;
        }

        if (self.read_buf.len != 0) self.allocator.free(self.read_buf);
        if (self.convert_buf.len != 0) self.allocator.free(self.convert_buf);
        if (self.ring.len != 0) self.allocator.free(self.ring);
    }

    fn readerMain(self: *RtlIqStream) void {
        const len_bytes = self.read_buf.len & ~@as(usize, min_block_bytes - 1);

        while (!self.stop.load(.acquire)) {
            var num_read: c_int = 0;
            const ret = rtlsdr_read_sync(self.dev, self.read_buf.ptr, @intCast(len_bytes), &num_read);
            if (ret != 0) {
                std.debug.print("rtlsdr_read_sync(): {d}\n", .{ret});
                self.mutex.lock();
                self.eof = true;
                self.mutex.unlock();
                self.nonempty.broadcast();
                return;
            }

            const nread: usize = @intCast(num_read);
            if (nread == 0) continue;
            const num_pairs = nread / 2;

            for (0..num_pairs) |i| {
                const b = self.read_buf[2 * i];
                const q = self.read_buf[2 * i + 1];
                self.convert_buf[i] = C32.init(
                    (@as(f32, @floatFromInt(b)) - 127.5) * (1.0 / 127.5),
                    (@as(f32, @floatFromInt(q)) - 127.5) * (1.0 / 127.5),
                );
            }

            self.pushSlice(self.convert_buf[0..num_pairs]);
        }
    }

    fn pushSlice(self: *RtlIqStream, samples: []const C32) void {
        self.mutex.lock();
        defer self.mutex.unlock();

        for (samples) |s| {
            while (self.count >= self.ring.len) {
                if (self.stop.load(.acquire)) return;
                self.nonfull.wait(&self.mutex);
            }
            self.ring[self.tail & self.ring_mask] = s;
            self.tail += 1;
            self.count += 1;
            self.nonempty.signal();
        }
    }
};
