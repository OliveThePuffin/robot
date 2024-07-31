const std = @import("std");

pub const Level = enum(u8) {
    NONE,
    DEBUG,
    INFO,
    WARN,
    ERROR,
};

var log_mutex = std.Thread.Mutex{};

var log_level: Level = .INFO;

// stderr
const stderr_file = std.io.getStdErr().writer();
var err_bw = std.io.bufferedWriter(stderr_file);
const stderr = err_bw.writer();

// stdout
const stdout_file = std.io.getStdOut().writer();
var out_bw = std.io.bufferedWriter(stdout_file);
const stdout = out_bw.writer();

pub fn logLevelSet(level: Level) void {
    log_level = level;
}

pub fn log(level: Level, channel: []const u8, comptime fmt: []const u8, args: anytype) void {
    log_mutex.lock();
    defer log_mutex.unlock();
    const color = switch (level) {
        .NONE => unreachable,
        .DEBUG => "\x1b[34m",
        .INFO => "\x1b[32m",
        .WARN => "\x1b[33m",
        .ERROR => "\x1b[31m",
    };
    const fmt_prefix = "{s}{s}: {s}: ";
    const fmt_rst = fmt ++ "\x1b[0m\n";

    //const log_message = color ++ @tagName(level) ++ ": " ++ channel ++ ": " ++ fmt ++ "\x1b[0m\n";
    if (@intFromEnum(level) >= @intFromEnum(log_level) and log_level != .NONE) {
        switch (level) {
            .ERROR => {
                stderr.print(fmt_prefix, .{ color, @tagName(level), channel }) catch return;
                stderr.print(fmt_rst, args) catch return;
                err_bw.flush() catch return;
            },
            else => {
                stdout.print(fmt_prefix, .{ color, @tagName(level), channel }) catch return;
                stdout.print(fmt_rst, args) catch return;
                out_bw.flush() catch return;
            },
        }
    }
}

test "logger" {
    logLevelSet(.NONE);
    log(.INFO, "test", "hello {s}", .{"world"});
}
