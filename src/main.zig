const Config = @import("config/Config.zig");
const Slam = @import("Slam").FastLIO;
const std = @import("std");

pub fn main() !void {

    // Read config
    const config = Config.dry_run;

    // Initialize modules
    var slam = try Slam.init(config.slam);
    defer slam.deinit();

    // Start modules
    //slam.start();
    //slam.stop();
}

test {}
