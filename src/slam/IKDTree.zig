const std = @import("std");
const log = @import("logger").log;

// Based on https://arxiv.org/pdf/2102.10808
// TODO: Section III.C.4 onwards
pub fn IKDTree(comptime Axes: type) type {
    std.debug.assert(@typeInfo(Axes) == .Enum);
    const K = std.meta.fields(Axes).len;
    const name = "IKDTree";

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
                node.push_down = false;
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
                    node.invalid_num += if (node.tree_deleted) left.tree_size else left.invalid_num;
                    for (0..K) |k| {
                        node.minimums[k] = @min(node.minimums[k], left.minimums[k]);
                        node.maximums[k] = @max(node.maximums[k], left.maximums[k]);
                    }
                }
                if (node.right) |right| {
                    node.tree_size += right.tree_size;
                    node.invalid_num += if (node.tree_deleted) right.tree_size else right.invalid_num;
                    for (0..K) |k| {
                        node.minimums[k] = @min(node.minimums[k], right.minimums[k]);
                        node.maximums[k] = @max(node.maximums[k], right.maximums[k]);
                    }
                }
            }

            fn printSubtree(node: ?*Node, prefix: []const u8, isLeft: bool) !void {
                if (node) |n| {
                    log(.DEBUG, name, "{s}{s} {d}: {s}{s}{s}", .{
                        prefix,
                        if (isLeft) "├──" else "└──",
                        n.point,
                        @tagName(n.axis_enum),
                        if (n.deleted) " (deleted)" else "",
                        if (n.tree_deleted) " (tree deleted)" else "",
                    });
                    const newPrefix = try std.heap.c_allocator.alloc(u8, prefix.len + 6);
                    defer std.heap.c_allocator.free(newPrefix);
                    std.mem.copyForwards(u8, newPrefix, prefix);
                    std.mem.copyForwards(u8, newPrefix[prefix.len..], if (isLeft) "│   " else "    ");

                    if (n.left) |left| {
                        try left.printSubtree(newPrefix, true);
                    }
                    if (n.right) |right| {
                        try right.printSubtree(newPrefix, false);
                    }
                } else {
                    log(.DEBUG, name, "Subtree is null", .{});
                }
            }

            // For greatest efficiency, make the allocator a fixed buffer allocator
            fn buildSubtree(Vec: [][K]f32, alloc: std.mem.Allocator) !?*Node {
                if (Vec.len == 0) {
                    return null;
                }
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

                const node = try alloc.create(Node);

                node.* = Node{
                    .point = Vec[mid],
                    .axis_enum = @enumFromInt(max_range_axis),
                    .left = try buildSubtree(Vec[0..mid], alloc),
                    .right = try buildSubtree(Vec[mid + 1 ..], alloc),

                    .tree_size = Vec.len,
                    .minimums = mins,
                    .maximums = maxs,
                };
                return node;
            }

            fn rebuildSubtree(node: *Node, alloc: std.mem.Allocator) !?*Node {
                const nodes = try node.flatten(alloc);
                defer alloc.free(nodes);

                // get only points from the nodes
                var points: [][K]f32 = try alloc.alloc([K]f32, nodes.len);
                defer alloc.free(points);
                for (nodes, 0..) |n, h| {
                    points[h] = n.point;
                }
                return try Node.buildSubtree(points, alloc);
            }

            fn recursiveFree(node: *Node, alloc: std.mem.Allocator) void {
                if (node.left) |left| {
                    recursiveFree(left, alloc);
                }
                if (node.right) |right| {
                    recursiveFree(right, alloc);
                }
                alloc.destroy(node);
            }

            // Deletes or reinserts a node in the tree
            fn recursiveUpdate(node: *Node, point: [K]f32, delete: bool) bool {
                const axis = @intFromEnum(node.axis_enum);
                var found = false;
                const equal = for (0..K) |k| {
                    if (node.point[k] != point[k]) break false;
                } else true;
                if (equal) {
                    if (node.deleted == delete) {
                        const status = if (delete) "delete" else "reinsert";
                        const status_past = if (delete) "deleted" else "inserted";
                        log(.WARN, name, "Attempting to {s} already {s} node", .{ status, status_past });
                    }
                    node.deleted = delete;
                    return true;
                }

                const left_bound = axisOrder(axis, point, node.point);
                if (left_bound) {
                    if (node.left) |left| {
                        found = recursiveUpdate(left, point, delete);
                    }
                } else {
                    if (node.right) |right| {
                        found = found or recursiveUpdate(right, point, delete);
                    }
                }
                return found;
            }

            fn recursiveInsert(node: *Node, point: [K]f32, alloc: std.mem.Allocator) !void {
                const axis = @intFromEnum(node.axis_enum);
                const left_bound = axisOrder(axis, point, node.point);
                if (left_bound) {
                    if (node.left) |left| {
                        try recursiveInsert(left, point, alloc);
                    } else {
                        node.left = try alloc.create(Node);
                        node.left.?.* = Node{
                            .point = point,
                            .axis_enum = node.axis_enum,
                            .left = null,
                            .right = null,
                        };
                    }
                } else {
                    if (node.right) |right| {
                        try recursiveInsert(right, point, alloc);
                    } else {
                        node.right = try alloc.create(Node);
                        node.right.?.* = Node{
                            .point = point,
                            .axis_enum = node.axis_enum,
                            .left = null,
                            .right = null,
                        };
                    }
                }
            }

            fn checkIntersection(node: *Node, mins: [K]f32, maxs: [K]f32) bool {
                var intersect = true;
                for (0..K) |k| {
                    intersect = intersect and (node.minimums[k] <= maxs[k] and node.maximums[k] >= mins[k]);
                }
                return intersect;
            }

            fn checkSubset(node: *Node, mins: [K]f32, maxs: [K]f32) bool {
                var subset = true;
                for (0..K) |k| {
                    subset = subset and (node.minimums[k] >= mins[k] and node.maximums[k] <= maxs[k]);
                }
                return subset;
            }

            fn checkContained(node: *Node, mins: [K]f32, maxs: [K]f32) bool {
                var contained = true;
                for (0..K) |k| {
                    contained = contained and (node.point[k] >= mins[k] and node.point[k] <= maxs[k]);
                }
                return contained;
            }

            fn flatten(node: *Node, alloc: std.mem.Allocator) ![]*Node {
                var nodes = std.ArrayList(*Node).init(alloc);
                if (!node.tree_deleted) {
                    if (!node.deleted) {
                        try nodes.append(node);
                    }

                    if (node.left) |left| {
                        const left_nodes = try left.flatten(alloc);
                        defer alloc.free(left_nodes);
                        try nodes.appendSlice(left_nodes);
                    }
                    if (node.right) |right| {
                        const right_nodes = try right.flatten(alloc);
                        defer alloc.free(right_nodes);
                        try nodes.appendSlice(right_nodes);
                    }
                }
                return try nodes.toOwnedSlice();
            }

            // Returns a list of nodes within the box
            fn recursiveSearchBox(node: *Node, mins: [K]f32, maxs: [K]f32, alloc: std.mem.Allocator) ![]*Node {
                var results = std.ArrayList(*Node).init(alloc);
                // if the node range is outside the box, return
                if (!checkIntersection(node, mins, maxs)) {
                    return results.toOwnedSlice();
                }
                // if the node range is a subset of the box, flatten the subtree and append it
                if (checkSubset(node, mins, maxs)) {
                    const nodes = try node.flatten(alloc);
                    defer alloc.free(nodes);
                    try results.appendSlice(nodes);
                    return try results.toOwnedSlice();
                }
                // if the node is contained in the box, append it
                if (checkContained(node, mins, maxs)) {
                    try results.append(node);
                }
                if (node.left) |left| {
                    const left_nodes = try left.recursiveSearchBox(mins, maxs, alloc);
                    defer alloc.free(left_nodes);
                    try results.appendSlice(left_nodes);
                }
                if (node.right) |right| {
                    const right_nodes = try right.recursiveSearchBox(mins, maxs, alloc);
                    defer alloc.free(right_nodes);
                    try results.appendSlice(right_nodes);
                }
                return try results.toOwnedSlice();
            }

            // Deletes or reinserts nodes in the tree
            fn recursiveUpdateBox(node: *Node, delete: bool, enable_parallel: bool, mins: [K]f32, maxs: [K]f32, alloc: std.mem.Allocator) !?*Node {
                pushDown(node);
                // if the node range is outside the box, return
                if (!checkIntersection(node, mins, maxs))
                    return node;
                // if the node range is a subset of the box, mark the tree and node as deleted
                if (checkSubset(node, mins, maxs)) {
                    node.tree_deleted = delete;
                    node.deleted = delete;
                    node.push_down = true;
                } else {
                    if (checkContained(node, mins, maxs)) {
                        node.deleted = delete;
                    }
                    if (node.left) |left| {
                        const new_left = try recursiveUpdateBox(left, delete, enable_parallel, mins, maxs, alloc);
                        if (new_left != node.left) {
                            recursiveFree(left, alloc);
                            node.left = new_left;
                        }
                    }
                    if (node.right) |right| {
                        const new_right = try recursiveUpdateBox(right, delete, enable_parallel, mins, maxs, alloc);
                        if (new_right != node.right) {
                            recursiveFree(right, alloc);
                            node.right = new_right;
                        }
                    }
                }
                pullUp(node);
                // TODO: finish me (rebuild to balance)
                //if (violateCriterion(node)) {
                //    if (node.tree_size < n_max or !enable_parallel) {
                //        rebuildTree(node);
                return try balanceSubtree(node, alloc);
                //    } else {
                //        ThreadSpawn(ParallelRebuild, node);
                //    }

                //}
            }

            fn axisOrder(axis_id: usize, lhs: [K]f32, rhs: [K]f32) bool {
                return (lhs[axis_id] < rhs[axis_id]);
            }

            fn balanceSubtree(node: *Node, alloc: std.mem.Allocator) !?*Node {
                // The lower these values are, the more likely it is that the tree will be rebalanced
                // α_bal ∈ (0.5, 1)
                const a_bal = 0.6;
                // α_del ∈ (0, 1)
                const a_del = 0.25;

                const treesize = @as(f32, @floatFromInt(node.tree_size));
                const treesize_left = @as(f32, @floatFromInt(if (node.left) |left| left.tree_size else 0));
                const treesize_right = @as(f32, @floatFromInt(if (node.right) |right| right.tree_size else 0));
                const num_invalid = @as(f32, @floatFromInt(node.invalid_num));

                const balanced_left = treesize_left < a_bal * (treesize - 1);
                const balanced_right = treesize_right < a_bal * (treesize - 1);
                const low_invalid = num_invalid < a_del * treesize;

                if (!balanced_left or !balanced_right or !low_invalid) {
                    return try rebuildSubtree(node, alloc);
                }
                return node;
            }
        };

        root: ?*Node,
        alloc: std.mem.Allocator,

        pub fn init(Vec: [][K]f32, alloc: std.mem.Allocator) !Self {
            return .{
                .root = try Node.buildSubtree(Vec, alloc),
                .alloc = alloc,
            };
        }
        pub fn deinit(self: *Self) void {
            // recursively free the tree
            if (self.root) |root| {
                root.recursiveFree(self.alloc);
            }
        }

        pub fn print(self: *Self) !void {
            if (self.root) |root| {
                log(.DEBUG, name, "Tree Size: {d}", .{root.tree_size});
                log(.DEBUG, name, "Num Deleted: {d}", .{root.invalid_num});
            } else {
                log(.DEBUG, name, "NULL Root", .{});
            }
            try Node.printSubtree(self.root, "", false);
        }

        pub fn insert(self: *Self, point: [K]f32) !void {
            if (self.root == null) {
                self.root = try Node.buildSubtree(@constCast(&[_][K]f32{point}), self.alloc);
            } else if (!Node.recursiveUpdate(self.root.?, point, false)) {
                try Node.recursiveInsert(self.root.?, point, self.alloc);
            }
        }

        pub fn insertMany(self: *Self, points: [][K]f32) !void {
            for (points) |point| {
                try insert(self, point);
            }
        }

        // Reinsert all points in a box
        pub fn reinsertBox(self: *Self, mins: [K]f32, maxs: [K]f32) !void {
            try Node.recursiveUpdateBox(self.root, false, true, mins, maxs, self.alloc);
        }

        // Remove all points in a box
        pub fn removeBox(self: *Self, mins: [K]f32, maxs: [K]f32) !void {
            try Node.recursiveUpdateBox(self.root, true, true, mins, maxs, self.alloc);
        }

        // Remove a point
        pub fn remove(self: *Self, point: [K]f32) void {
            if (!Node.recursiveUpdate(self.root, true, true, point, true)) {
                log(.WARN, name, "No such point {d} found", .{point});
            }
        }

        pub fn downsample(self: *Self, length: f32, point: [K]f32) !void {
            // how many hypercubes fill the space
            if (self.root == null) return;
            const root = self.root.?;
            var hypercube_lengths: [K]usize = undefined;
            var total_hypercubes: usize = 1;

            // TODO: fix hypercube length math
            // (see output: point should be in (2.4, 2.4, 2.4)-(4.8, 4.8, 4.8) not (0, 2.4, 2.4)-(2.4, 4.8, 4.8))
            // when there are no points in (0, 0, 0) - (4.8, 2.4, 2.4))
            //
            var min_hypercube: [K]f32 = undefined;
            for (0..K) |k| {
                min_hypercube[k] = @floor(root.minimums[k] / length);
            }

            inline for (0..K) |k| {
                hypercube_lengths[k] = @intFromFloat(@ceil(root.maximums[k] / length) - min_hypercube[k]);
                total_hypercubes *= hypercube_lengths[k];
            }

            // get the hypercube that the point is in
            var hypercube: [K]usize = undefined;
            inline for (0..K) |k| {
                hypercube[k] = @intFromFloat(@divFloor(point[k] - root.minimums[k], length));
            }

            var hypercube_mins: [K]f32 = undefined;
            var hypercube_maxs: [K]f32 = undefined;
            inline for (0..K) |k| {
                hypercube_mins[k] = min_hypercube[k] * length + @as(f32, @floatFromInt(hypercube[k])) * length;
                hypercube_maxs[k] = hypercube_mins[k] + length;
            }

            // get all points in the hypercube
            const hypercube_nodes = try root.recursiveSearchBox(hypercube_mins, hypercube_maxs, self.alloc);
            log(.INFO, name, "Hypercube Nodes: {} Min: {d} Max: {d}", .{ hypercube_nodes.len, hypercube_mins, hypercube_maxs });
            defer self.alloc.free(hypercube_nodes);

            // get only points from the nodes
            var points: [][K]f32 = try self.alloc.alloc([K]f32, hypercube_nodes.len);
            defer self.alloc.free(points);
            for (hypercube_nodes, 0..) |node, h| {
                points[h] = node.point;
            }

            const hypercube_tree = try Node.buildSubtree(points, self.alloc) orelse return;
            defer hypercube_tree.recursiveFree(self.alloc);

            var center = hypercube_mins;
            inline for (0..K) |k| {
                center[k] += length / 2.0;
            }
            // TODO: get nearest neighbor to the center of the hypercube
            const center_point = hypercube_tree.point;
            log(.INFO, name, "Center point: {d}", .{center_point});

            // Delete all nodes in the hypercube
            const new_node = try Node.recursiveUpdateBox(root, true, true, hypercube_mins, hypercube_maxs, self.alloc);
            if (self.root) |old_root| {
                old_root.recursiveFree(self.alloc);
            }
            self.root = new_node;
            log(.INFO, name, "New root: {?d}", .{if (new_node) |n| n.point else null});

            try self.insert(center_point);
        }

        pub fn search() void {}
        pub fn nearestNeighbor() void {}
    };
}

test {}
