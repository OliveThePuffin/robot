const std = @import("std");
const Log = @import("Log").Log;
const LogConfig = @import("Log").Config;

pub const IMU = struct {
    const Self = @This();

    pub const Config = struct {
        frequency: f32,
        dry_run: bool,
        log: LogConfig,
    };

    config: Config,
    loop_continue: bool = false,
    loop_thread: ?std.Thread = null,
    loop_mutex: std.Thread.Mutex = .{},

    pub fn init(config: Config) Self {
        return .{
            .config = config,
        };
    }

    pub fn deinit(self: *Self) void {
        log.deinit();
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

    fn loop(self: *Self) void {
        while (self.loop_continue) {
            // TODO
        }
    }

    // TODO: Make this whole pattern an interface
};
