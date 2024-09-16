const std = @import("std");
const Dir = std.fs.Dir;
const File = std.fs.File;

pub const Level = enum(u8) {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    NONE,
};

pub const Config = struct {
    level: Level = .INFO,
    abs_path: ?[]const u8 = null,
    channel: []const u8,
};

pub const Log = struct {
    config: Config,
    out_file: File = std.io.getStdOut(),
    err_file: File = std.io.getStdErr(),
    log_mutex: std.Thread.Mutex = .{},

    //(self: Dir, sub_path: []const u8, open_dir_options: OpenDirOptions) (MakeError || OpenError || StatFileError)!Dir {

    pub fn init(config: Config) !Log {
        if (config.abs_path) |abs_path| {
            var cwd = try std.fs.cwd().makeOpenPath(abs_path, .{});
            cwd = try cwd.makeOpenPath("log", .{});
            const file = try cwd.createFile(config.channel, .{
                .read = true,
                .truncate = true,
            });
            return .{ .out_file = file, .err_file = file, .config = config };
        }
        return .{ .config = config };
    }

    pub fn deinit(self: *Log) void {
        //self.log_mutex.lock();
        //defer self.log_mutex.unlock();
        if (self.config.abs_path) |_| {
            self.out_file.close();
            // self.err_file.close(); // Both files right now are the same
        }
    }

    pub fn debug(self: *Log, comptime fmt: []const u8, args: anytype) void {
        self.log(.DEBUG, fmt, args);
    }

    pub fn info(self: *Log, comptime fmt: []const u8, args: anytype) void {
        self.log(.INFO, fmt, args);
    }

    pub fn warn(self: *Log, comptime fmt: []const u8, args: anytype) void {
        self.log(.WARN, fmt, args);
    }

    pub fn err(self: *Log, comptime fmt: []const u8, args: anytype) void {
        self.log(.ERROR, fmt, args);
    }

    fn log(self: *Log, level: Level, comptime fmt: []const u8, args: anytype) void {
        self.log_mutex.lock();
        defer self.log_mutex.unlock();

        const out_writer = self.out_file.writer();
        const err_writer = self.err_file.writer();

        const color = switch (level) {
            .DEBUG => "\x1b[34m",
            .INFO => "\x1b[32m",
            .WARN => "\x1b[33m",
            .ERROR => "\x1b[31m",
            .NONE => unreachable,
        };
        const fmt_prefix = "{s}{s}: {s}: ";
        const fmt_rst = fmt ++ "\x1b[0m\n";

        //const log_message = color ++ @tagName(level) ++ ": " ++ channel ++ ": " ++ fmt ++ "\x1b[0m\n";
        if (@intFromEnum(level) >= @intFromEnum(self.config.level)) {
            switch (level) {
                .ERROR => {
                    err_writer.print(fmt_prefix, .{ color, @tagName(level), self.config.channel }) catch return;
                    err_writer.print(fmt_rst, args) catch return;
                },
                else => {
                    out_writer.print(fmt_prefix, .{ color, @tagName(level), self.config.channel }) catch return;
                    out_writer.print(fmt_rst, args) catch return;
                },
            }
        }
    }
};

test "log_out" {
    var log = try Log.init(.{
        .abs_path = "/tmp/CLAW/test/Log",
        .channel = "log_out",
        .level = .INFO,
    });

    const contents = "Hello, world!";
    log.info("{s}", .{contents});

    var buffer: [100]u8 = undefined;
    try log.out_file.seekTo(0);
    const bytes_read = try log.out_file.read(&buffer);

    const info_contents = "\x1b[32mINFO: log_out: " ++ contents ++ "\x1b[0m\n";
    try std.testing.expect(std.mem.eql(u8, buffer[0..bytes_read], info_contents));
}

test "log_err" {
    var log = try Log.init(.{
        .abs_path = "/tmp/CLAW/test/Log",
        .channel = "log_err",
        .level = .INFO,
    });

    const contents = "Hello, world!";
    log.err("{s}", .{contents});

    var buffer: [100]u8 = undefined;
    try log.err_file.seekTo(0);
    const bytes_read = try log.err_file.read(&buffer);

    const err_contents = "\x1b[31mERROR: log_err: " ++ contents ++ "\x1b[0m\n";
    try std.testing.expect(std.mem.eql(u8, buffer[0..bytes_read], err_contents));
}

test "log_empty" {
    var log = try Log.init(.{
        .abs_path = "/tmp/CLAW/test/Log",
        .channel = "log_empty",
        .level = .INFO,
    });

    const contents = "Hello, world!";
    log.debug("{s}", .{contents});

    var buffer: [100]u8 = undefined;
    try log.out_file.seekTo(0);
    const bytes_read = try log.out_file.read(&buffer);

    try std.testing.expectEqual(bytes_read, 0);
}
