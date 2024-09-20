const Config = @import("Config");
const Slam = @import("Slam").FastLIO;
const std = @import("std");

pub fn main() !void {

    // Read config
    const config = Config.default;

    // Initialize modules
    var slam = try Slam.init(config.slam);
    defer slam.deinit();

    // Start modules
    slam.start();
    std.time.sleep(2 * std.time.ns_per_s);
    slam.stop();
}

test {}
