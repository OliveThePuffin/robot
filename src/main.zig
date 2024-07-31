const Config = @import("config/Config.zig");
const Module = @import("modules/Module.zig");
const Slam = @import("modules/slam/Slam.zig");
const log = @import("modules/logger.zig").log;
const std = @import("std");

//const rs_depth = @import("rs-depth.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};

    // Read config and initialize modules
    const config = Config.dry_run;

    var modules: [config.modules.len]?*Module = [_]?*Module{null} ** config.modules.len;

    for (config.modules, 0..) |module, i| {
        modules[i] = switch (module) {
            .slam => |s| s.module_init(s.name, gpa.allocator(), s.config),
        };
    }

    std.time.sleep(0.5 * std.time.ns_per_s);
    try modules[0].?.send(Slam.Request.START, &void{});
    std.time.sleep(2 * std.time.ns_per_s);
    try modules[0].?.send(Slam.Request.STOP, &void{});
    std.time.sleep(0.5 * std.time.ns_per_s);

    // Deinitialize modules
    for (modules) |module| {
        if (module) |m| {
            m.deinit();
        }
    }

    if (gpa.deinit() == .leak) {
        log(.ERROR, "Main", "Memory leaks detected", .{});
    }
}
