const Config = @import("config/Config.zig");
const Slam = @import("Slam");
const logger = @import("logger");
const std = @import("std");

pub fn main() !void {

    // Read config
    const config = Config.dry_run;
    logger.logLevelSet(config.log_level);

    // Initialize modules
    try Slam.init(config.slam);
    defer {
        // Deinitialize modules
        Slam.deinit();
    }

    // Start modules
    Slam.start();
    //std.time.sleep(0.25 * std.time.ns_per_s);
    Slam.stop();
}

test {}
