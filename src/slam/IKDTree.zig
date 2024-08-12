const std = @import("std");
const log = @import("logger").log;

// TODO: implement https://arxiv.org/pdf/2102.10808
// Section III.C.2
pub fn IKDTree(comptime Axes: type) type {
    std.debug.assert(@typeInfo(Axes) == .Enum);
    const K = std.meta.fields(Axes).len;
    const name = "IKDTree";
    const IKDError = error{
        NullRoot,
    };

    return struct {
        const Self = @This();
        const Node = struct {
            point: [K]f32,
            axis_enum: Axes,
            left: ?*Node,
            right: ?*Node,

            tree_size: usize = 1,
            invalid_num: usize = 0,
            deleted: bool = false,
            tree_deleted: bool = false,
            push_down: bool = false,
            minimums: [K]f32 = undefined,
            maximums: [K]f32 = undefined,

            // Pushes values down the tree
            // push_down, tree_deleted, deleted
            pub fn pushDown(node: *Node) void {
                if (node.left) |left| {
                    left.push_down = node.push_down;
                    left.tree_deleted = node.tree_deleted;
                    left.deleted = node.deleted;
                }
                if (node.right) |right| {
                    right.push_down = node.push_down;
                    right.tree_deleted = node.tree_deleted;
                    right.deleted = node.deleted;
                }
            }

            // Pulls up (Calculates) info from below the subtree
            // tree_size, invalid_num, range
            pub fn pullUp(node: *Node) void {
                // Base cases
                for (0..K) |k| {
                    node.minimums[k] = node.point[k];
                    node.maximums[k] = node.point[k];
                }
                node.tree_size = 1;
                if (node.tree_deleted) {
                    node.invalid_num = node.tree_size;
                }
                node.invalid_num = @intFromBool(!node.deleted);

                // pull up the subtrees
                if (node.left) |left| {
                    pullUp(left);
                }
                if (node.right) |right| {
                    pullUp(right);
                }

                // update the node
                if (node.left) |left| {
                    node.tree_size += left.tree_size;
                    node.invalid_num += left.invalid_num;
                    for (0..K) |k| {
                        node.minimums[k] = @min(node.minimums[k], left.minimums[k]);
                        node.maximums[k] = @max(node.maximums[k], left.maximums[k]);
                    }
                }
                if (node.right) |right| {
                    node.tree_size += right.tree_size;
                    node.invalid_num += right.invalid_num;
                    for (0..K) |k| {
                        node.minimums[k] = @min(node.minimums[k], right.minimums[k]);
                        node.maximums[k] = @max(node.maximums[k], right.maximums[k]);
                    }
                }
            }

            fn printSubtree(node: ?*Node, prefix: []const u8, isLeft: bool) void {
                if (node) |n| {
                    log(.DEBUG, name, "{s}{s} {d}: {s}{s}", .{
                        prefix,
                        if (isLeft) "├──" else "└──",
                        n.point,
                        @tagName(n.axis_enum),
                        if (n.deleted) " (deleted)" else "",
                    });
                    const newPrefix = std.heap.c_allocator.alloc(u8, prefix.len + 6) catch |err| {
                        log(.ERROR, name, "Failed to allocate memory for log: {s}", .{@errorName(err)});
                        return;
                    };
                    std.mem.copyForwards(u8, newPrefix, prefix);
                    std.mem.copyForwards(u8, newPrefix[prefix.len..], if (isLeft) "│   " else "    ");
                    defer std.heap.c_allocator.free(newPrefix);

                    if (n.left) |left| {
                        left.printSubtree(newPrefix, true);
                    }
                    if (n.right) |right| {
                        right.printSubtree(newPrefix, false);
                    }
                }
            }

            // For greatest efficiency, make the allocator a fixed buffer allocator
            fn buildSubtree(Vec: [][K]f32, alloc: std.mem.Allocator) ?*Node {
                if (Vec.len == 0) return null;
                const mid = Vec.len / 2;

                var max_range_axis: usize = 0;
                var max_range: f32 = 0;
                var mins: [K]f32 = undefined;
                var maxs: [K]f32 = undefined;
                for (0..K) |k| {
                    var min = Vec[0][k];
                    var max = Vec[0][k];
                    for (0..Vec.len) |i| {
                        if (Vec[i][k] < min) min = Vec[i][k];
                        if (Vec[i][k] > max) max = Vec[i][k];
                    }
                    mins[k] = min;
                    maxs[k] = max;
                    const range = max - min;
                    if (range > max_range) {
                        max_range = range;
                        max_range_axis = k;
                    }
                }
                std.mem.sort([K]f32, Vec, max_range_axis, axisOrder);

                const node = alloc.create(Node) catch |err| {
                    log(.ERROR, name, "Failed to allocate memory for node: {s}", .{@errorName(err)});
                    return null;
                };

                node.* = Node{
                    .point = Vec[mid],
                    .axis_enum = @enumFromInt(max_range_axis),
                    .left = buildSubtree(Vec[0..mid], alloc),
                    .right = buildSubtree(Vec[mid + 1 ..], alloc),

                    .tree_size = Vec.len,
                    .minimums = mins,
                    .maximums = maxs,
                };
                return node;
            }
        };

        root: *Node,
        alloc: std.mem.Allocator,

        pub fn init(Vec: [][K]f32, alloc: std.mem.Allocator) IKDError!Self {
            return .{
                .root = Node.buildSubtree(Vec, alloc) orelse return IKDError.NullRoot,
                .alloc = alloc,
            };
        }
        pub fn deinit(self: *Self) void {
            // recursively free the tree
            recursiveFree(self.root, self.alloc);
        }

        fn recursiveFree(node: ?*Node, alloc: std.mem.Allocator) void {
            if (node) |n| {
                recursiveFree(n.left, alloc);
                recursiveFree(n.right, alloc);
                alloc.destroy(n);
            }
        }

        pub fn print(self: *Self) void {
            self.root.printSubtree("", true);
        }

        // Add a point
        pub fn add(self: *Self, point: [K]f32) void {
            addRecursive(self.root, point, self.alloc);
        }

        pub fn readd(self: *Self, point: [K]f32) void {
            if (!readdRecursive(self.root, point)) {
                log(.WARN, name, "No such point {d} found", .{point});
            }
        }

        fn readdRecursive(node: *Node, point: [K]f32) bool {
            const axis = @intFromEnum(node.axis_enum);
            var found = false;
            const equal = for (0..K) |k| {
                if (node.point[k] != point[k]) break false;
            } else true;
            if (equal) {
                if (!node.deleted) {
                    log(.WARN, name, "Attempting to add already added node", .{});
                }
                node.deleted = false;
                return true;
            }

            const left_bound = axisOrder(axis, point, node.point);
            if (left_bound) {
                if (node.left) |left| {
                    found = readdRecursive(left, point);
                }
            } else {
                if (node.right) |right| {
                    found = found or readdRecursive(right, point);
                }
            }
            return found;
        }

        fn addRecursive(node: *Node, point: [K]f32, alloc: std.mem.Allocator) void {
            const axis = @intFromEnum(node.axis_enum);
            const left_bound = axisOrder(axis, point, node.point);
            if (left_bound) {
                if (node.left) |left| {
                    addRecursive(left, point, alloc);
                } else {
                    node.left = alloc.create(Node) catch |err| {
                        log(.ERROR, name, "Failed to create node: {s}", .{@errorName(err)});
                        return;
                    };
                    node.left.?.* = Node{
                        .point = point,
                        .axis_enum = node.axis_enum,
                        .left = null,
                        .right = null,
                    };
                }
            } else {
                if (node.right) |right| {
                    addRecursive(right, point, alloc);
                } else {
                    node.right = alloc.create(Node) catch |err| {
                        log(.ERROR, name, "Failed to create node: {s}", .{@errorName(err)});
                        return;
                    };
                    node.right.?.* = Node{
                        .point = point,
                        .axis_enum = node.axis_enum,
                        .left = null,
                        .right = null,
                    };
                }
            }
        }

        pub fn addBox() void {}
        pub fn remove(self: *Self, point: [K]f32) void {
            if (!removeRecursive(self.root, point)) {
                log(.WARN, name, "No such point {d} found", .{point});
            }
        }

        fn removeRecursive(node: *Node, point: [K]f32) bool {
            const axis = @intFromEnum(node.axis_enum);
            var found = false;
            const equal = for (0..K) |k| {
                if (node.point[k] != point[k]) break false;
            } else true;
            if (equal) {
                if (node.deleted) {
                    log(.WARN, name, "Attempting to remove already deleted node", .{});
                }
                node.deleted = true;
                return true;
            }

            const left_bound = axisOrder(axis, point, node.point);
            if (left_bound) {
                if (node.left) |left| {
                    found = removeRecursive(left, point);
                }
            } else {
                if (node.right) |right| {
                    found = found or removeRecursive(right, point);
                }
            }
            return found;
        }
        pub fn removeBox() void {}
        pub fn search() void {}
        pub fn nearestNeighbor() void {}

        fn axisOrder(axis_id: usize, lhs: [K]f32, rhs: [K]f32) bool {
            return (lhs[axis_id] < rhs[axis_id]);
        }
    };
}

test {}
