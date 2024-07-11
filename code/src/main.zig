const Slam = @import("modules/slam/slam.zig").Slam;
const std = @import("std");

//const rs_depth = @import("rs-depth.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};

    // Initialize modules
    var module_slam = try Slam.init("SLAM", gpa.allocator());

    std.time.sleep(0.5 * std.time.ns_per_s);
    try module_slam.send(Slam.RequestEnum.START, void{});
    std.time.sleep(2 * std.time.ns_per_s);
    try module_slam.send(Slam.RequestEnum.STOP, void{});
    std.time.sleep(0.5 * std.time.ns_per_s);

    // Deinitialize modules
    module_slam.deinit();
}
