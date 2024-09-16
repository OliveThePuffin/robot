const std = @import("std");
const Log = @import("Log").Log;
const LogConfig = @import("Log").Config;

pub const Unit = struct {
    const Self = @This();

    log: Log,
    loop_continue: bool = false,
    loop_thread: ?std.Thread = null,
    loop_mutex: std.Thread.Mutex = .{},

    frequency: f32, // if invalid (<= 0) will run as fast as possible
    impl: *anyopaque,
    fn_deinit: *const fn (impl: *anyopaque) void,
    fn_update: *const fn (impl: *anyopaque) anyerror!void,

    pub fn deinit(self: *Self) void {
        if (self.loop_thread) |_| {
            self.stop();
        }
        self.fn_deinit(self.impl);
    }

    pub fn start(self: *Self) void {
        self.loop_mutex.lock();
        defer self.loop_mutex.unlock();

        if (self.loop_thread) |_| {
            self.log.warn("Loop already running...", .{});
        } else {
            self.loop_continue = true;
            self.loop_thread = std.Thread.spawn(.{}, loop, .{self}) catch unreachable;
        }
    }
    pub fn stop(self: *Self) void {
        self.loop_mutex.lock();
        defer self.loop_mutex.unlock();

        if (self.loop_thread) |*thread| {
            self.loop_continue = false;
            thread.join();
            self.loop_thread = null;
        }
    }

    fn loop(self: *Self) !void {
        while (self.loop_continue) {
            try self.fn_update(self.impl);
        }
    }
};
