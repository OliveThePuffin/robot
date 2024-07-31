const Slam = @import("../modules/slam/Slam.zig");
const Module = @import("../modules/Module.zig");
const LogLevel = @import("../modules/logger.zig").Level;
const Config = @This();
const std = @import("std");

modules: []const ModuleConfig,
log_level: LogLevel,

const ModuleConfig = union(enum) {
    slam: struct {
        name: []const u8,
        module_init: *const fn ([]const u8, std.mem.Allocator, Slam.Config) ?*Module,
        config: Slam.Config,
    },
};

pub const default = Config{
    .log_level = .INFO,
    .modules = &[_]ModuleConfig{
        ModuleConfig{ .slam = .{
            .name = "SLAM",
            .module_init = Slam.init,
            .config = .{
                .dry_run = false,
            },
        } },
    },
};

pub const dry_run = Config{
    .log_level = .INFO,
    .modules = &[_]ModuleConfig{
        ModuleConfig{ .slam = .{
            .name = "SLAM",
            .module_init = Slam.init,
            .config = .{
                .dry_run = true,
            },
        } },
    },
};
