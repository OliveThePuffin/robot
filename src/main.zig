const Config = @import("config/Config.zig");
const Slam = @import("Slam");
const logger = @import("logger");
const std = @import("std");

pub fn main() !void {

    // Read config
    const config = Config.dry_run;

    // Initialize modules
    try Slam.init(config.slam);
    defer Slam.deinit();

    // Start modules
    Slam.start();
    Slam.stop();
}

test {}
