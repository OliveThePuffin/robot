const std = @import("std");
const Log = @import("Log").Log;
const LogConfig = @import("Log").Config;

pub const Unit = struct {
    const Self = @This();

    log: Log,
    loop_continue: bool = false,
    loop_thread: ?std.Thread = null,
    loop_mutex: std.Thread.Mutex = .{},

    frequency: f64, // if invalid (<= 0) will run as fast as possible
    ctx: *anyopaque,
    fn_start: ?*const fn (ctx: *anyopaque) void,
    fn_stop: ?*const fn (ctx: *anyopaque) void,
    fn_deinit: *const fn (ctx: *anyopaque) void,
    fn_update: *const fn (ctx: *anyopaque) anyerror!void,

    pub fn deinit(self: *Self) void {
        if (self.loop_thread) |_| {
            self.stop();
        }
        self.fn_deinit(self.ctx);
    }

    pub fn start(self: *Self) void {
        self.loop_mutex.lock();
        defer self.loop_mutex.unlock();

        if (self.loop_thread) |_| {
            self.log.warn("Loop already running...", .{});
        } else {
            if (self.fn_start) |fn_start| {
                fn_start(self.ctx);
            }
            self.loop_continue = true;
            self.loop_thread = std.Thread.spawn(.{}, loop, .{self}) catch unreachable;
        }
    }
    pub fn stop(self: *Self) void {
        self.loop_mutex.lock();
        defer self.loop_mutex.unlock();

        if (self.loop_thread) |*thread| {
            if (self.fn_stop) |fn_stop| {
                fn_stop(self.ctx);
            }
            self.loop_continue = false;
            thread.join();
            self.loop_thread = null;
        }
    }

    fn loop(self: *Self) !void {
        var timer = try std.time.Timer.start();
        while (self.loop_continue) {
            _ = timer.lap();
            try self.fn_update(self.ctx);
            const update_time = @as(f64, @floatFromInt(timer.read())) / std.time.ns_per_s;
            self.log.debug("Update time | {d:2.9} s", .{update_time});
            if (self.frequency > 0) {
                const sleep_time: f64 = (1.0 / self.frequency) - update_time;
                std.time.sleep(@intFromFloat(sleep_time * std.time.ns_per_s));
            }
            const loop_time = @as(f64, @floatFromInt(timer.read())) / std.time.ns_per_s;
            self.log.debug("Loop time   | {d:2.9} s", .{loop_time});
        }
    }
};
