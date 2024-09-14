const std = @import("std");

pub fn main() !void {
    try @import("perf/IKDTree.zig").ikd_tree_perf();
}
