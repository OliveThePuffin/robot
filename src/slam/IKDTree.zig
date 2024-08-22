const std = @import("std");
const log = @import("logger").log;

// Based on https://arxiv.org/pdf/2102.10808
// TODO: Parallel portion
//
// TODO: Error handling

/// Massive credit to @Travis from the zig discord for helping with optimizations!
/// Search functions are heavily adapted from https://github.com/travisstaloch/kd-tree
pub fn IKDTree(comptime K: usize) type {
    return struct {
        const Self = @This();
        const Point = [K]f32;
        const name = "IKDTree";
        pub const Config = struct {
            a_bal: f32,
            a_del: f32,
            enable_parallel: bool,
            max_size_single_thread_rebuild: usize,
            relative_tolrance: f32,
        };

        const Node = struct {
            point: Point,
            axis: usize,
            left: ?*Node,
            right: ?*Node,

            tree_size: usize = 1,
            invalid_num: usize = 0,
            deleted: bool = false,
            tree_deleted: bool = false,
            minimums: Point,
            maximums: Point,

            pub const NnResult = struct {
                point: Point,
                distance: f32,
            };
            const KnnQueue = struct {
                storage: std.ArrayListUnmanaged(NnResult) = .{},
                alloc: std.mem.Allocator,
                bound: usize,

                fn init(alloc: std.mem.Allocator, bound: usize) KnnQueue {
                    std.debug.assert(bound > 0);
                    return .{ .alloc = alloc, .bound = bound };
                }

                pub fn push(self: *KnnQueue, result: NnResult) !void {
                    const index = std.sort.lowerBound(
                        NnResult,
                        result,
                        self.storage.items,
                        void{},
                        nnResultCompare,
                    );
                    try self.storage.insert(self.alloc, index, result);
                    if (self.storage.items.len > self.bound) {
                        self.storage.items.len -= 1;
                        std.debug.assert(self.storage.items.len == self.bound);
                    }
                }

                pub fn count(self: KnnQueue) usize {
                    return self.storage.items.len;
                }
                pub fn last(self: KnnQueue) NnResult {
                    std.debug.assert(self.storage.items.len > 0);
                    return self.storage.items[self.storage.items.len - 1];
                }
                pub fn deinit(self: *KnnQueue) void {
                    self.storage.deinit(self.alloc);
                }

                fn nnResultCompare(_: void, a: Node.NnResult, b: Node.NnResult) bool {
                    return a.distance < b.distance;
                }
            };

            fn nnSearch(node: ?*Node, point: Point, include_deleted: bool, result: *NnResult) void {
                if (node == null or (node.?.tree_deleted and !include_deleted)) return;

                const dist = if (node.?.deleted and !include_deleted)
                    std.math.inf(f32)
                else
                    distanceSquared(node.?.point, point);
                if (dist < result.distance) {
                    result.distance = dist;
                    result.point = node.?.point;
                }

                const axis = node.?.axis;
                const dir = point[axis] < node.?.point[axis];
                nnSearch(
                    if (dir) node.?.left else node.?.right,
                    point,
                    include_deleted,
                    result,
                );

                const diff = point[axis] - node.?.point[axis];
                if ((diff * diff) < result.distance) {
                    nnSearch(
                        if (dir) node.?.right else node.?.left,
                        point,
                        include_deleted,
                        result,
                    );
                }
            }

            fn knnSearch(node: ?*Node, point: Point, include_deleted: bool, k: usize, result: *KnnQueue) !void {
                if (node == null or (node.?.tree_deleted and !include_deleted)) return;

                const dist = if (node.?.deleted and !include_deleted)
                    std.math.inf(f32)
                else
                    distanceSquared(node.?.point, point);
                try result.push(.{
                    .point = node.?.point,
                    .distance = dist,
                });

                const axis = node.?.axis;
                const dir = point[axis] < node.?.point[axis];
                try knnSearch(
                    if (dir) node.?.left else node.?.right,
                    point,
                    include_deleted,
                    k,
                    result,
                );

                const diff = point[axis] - node.?.point[axis];
                if (result.count() < k or (diff * diff) < result.last().distance) {
                    try knnSearch(
                        if (dir) node.?.right else node.?.left,
                        point,
                        include_deleted,
                        k,
                        result,
                    );
                }
            }

            fn distanceSquared(p1: Point, p2: Point) f32 {
                var sum: f32 = 0;
                inline for (0..K) |k| {
                    sum += (p1[k] - p2[k]) * (p1[k] - p2[k]);
                }
                return sum;
            }

            // Pulls up (Calculates) info from below the subtree
            // tree_size, invalid_num, range
            fn pullUp(node: *Node) void {
                // Base cases
                inline for (0..K) |k| {
                    node.minimums[k] = node.point[k];
                    node.maximums[k] = node.point[k];
                }
                node.tree_size = 1;
                node.invalid_num = @intFromBool(node.deleted);

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
                    inline for (0..K) |k| {
                        node.minimums[k] = @min(node.minimums[k], left.minimums[k]);
                        node.maximums[k] = @max(node.maximums[k], left.maximums[k]);
                    }
                }
                if (node.right) |right| {
                    node.tree_size += right.tree_size;
                    node.invalid_num += if (node.tree_deleted) right.tree_size else right.invalid_num;
                    inline for (0..K) |k| {
                        node.minimums[k] = @min(node.minimums[k], right.minimums[k]);
                        node.maximums[k] = @max(node.maximums[k], right.maximums[k]);
                    }
                }
            }

            fn printSubtree(node: ?*Node, prefix: []const u8, isLeft: bool) !void {
                if (node) |n| {
                    log(.DEBUG, name, "{s}{s} {d}: {}{s}{s}", .{
                        prefix,
                        if (isLeft) "├──" else "└──",
                        n.point,
                        n.axis,
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

            fn buildSubtree(Vec: []Point, alloc: std.mem.Allocator) !?*Node {
                if (Vec.len == 0) {
                    return null;
                }
                const low_size = Vec.len / 2;
                const mid_index = low_size;
                const high_index = mid_index + 1;

                var max_range_axis: usize = 0;
                var max_range: f32 = 0;
                var mins: Point = undefined;
                var maxs: Point = undefined;
                inline for (0..K) |k| {
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

                const median = select(Vec, mid_index, max_range_axis);

                const node = try alloc.create(Node);

                node.* = Node{
                    .point = median,
                    .axis = max_range_axis,
                    .left = try buildSubtree(Vec[0..low_size], alloc),
                    .right = try buildSubtree(Vec[high_index..], alloc),

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
                var points: []Point = try alloc.alloc(Point, nodes.len);
                defer alloc.free(points);
                for (nodes, 0..) |n, h| {
                    points[h] = n.point;
                }
                return try Node.buildSubtree(points, alloc);
            }

            fn rebuildSubtreeParallel(node: *Node, alloc: std.mem.Allocator) !?*Node {
                // TODO: implement the following
                // LockUpdates(node);
                // This makes insert, reinsert, and delete not accessible
                // queries are still possible (such as nearest neighbor)
                //
                // Flatten(node);
                //
                // UnlockUpdates(node);
                // After unlock, all incremental updates are suspended and recorded
                // in an operation logger
                //
                // buildSubtree(arr);
                // for each op in operationLogger do
                //      incrementalUpdates(node, op, disable_parallel);
                // end
                // node_temp = node;
                // LockAll(node); // This makes no operations on the tree accessible
                // node = new_node;
                // UnlockAll(node);
                // free(node_temp);

                const nodes = try node.flatten(alloc);
                defer alloc.free(nodes);

                // get only points from the nodes
                var points: []Point = try alloc.alloc(Point, nodes.len);
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

            fn findInSubtree(node: *Node, point: Point, tolerance: f32, include_deleted: bool) bool {
                var nearest = NnResult{
                    .point = [_]f32{std.math.inf(f32)} ** K,
                    .distance = std.math.inf(f32),
                };
                nnSearch(node, point, include_deleted, &nearest);
                inline for (0..K) |k| {
                    if (!std.math.approxEqRel(f32, nearest.point[k], point[k], tolerance)) {
                        return false;
                    }
                }
                return true;
            }

            // Deletes or reinserts a node in the tree
            fn recursiveUpdate(
                node: *Node,
                point: Point,
                config: Config,
                delete: bool,
                alloc: std.mem.Allocator,
            ) !?*Node {
                const equal = inline for (0..K) |k| {
                    if (node.point[k] != point[k]) break false;
                } else true;
                if (equal) {
                    if (node.deleted == delete) {
                        const status = if (delete) "delete" else "reinsert";
                        const status_past = if (delete) "deleted" else "inserted";
                        log(.WARN, name, "Attempting to {s} already {s} node", .{ status, status_past });
                    }
                    node.deleted = delete;
                    return node;
                }

                const left_bound = axisOrder(node.axis, point, node.point);
                if (left_bound) {
                    if (node.left) |left| {
                        const new_left = try recursiveUpdate(left, point, config, delete, alloc);
                        if (new_left != node.left) {
                            recursiveFree(left, alloc);
                            node.left = new_left;
                        }
                    }
                } else {
                    if (node.right) |right| {
                        const new_right = try recursiveUpdate(right, point, config, delete, alloc);
                        if (new_right != node.right) {
                            recursiveFree(right, alloc);
                            node.right = new_right;
                        }
                    }
                }
                pullUp(node);
                return try balanceSubtree(node, config, alloc);
            }

            fn recursiveInsert(node: *Node, point: Point, config: Config, alloc: std.mem.Allocator) !?*Node {
                const left_bound = axisOrder(node.axis, point, node.point);
                if (left_bound) {
                    if (node.left) |left| {
                        const new_left = try recursiveInsert(left, point, config, alloc);
                        if (new_left != node.left) {
                            recursiveFree(left, alloc);
                            node.left = new_left;
                        }
                    } else {
                        node.left = try alloc.create(Node);
                        node.left.?.* = Node{
                            .point = point,
                            .axis = node.axis,
                            .left = null,
                            .right = null,
                        };
                    }
                } else {
                    if (node.right) |right| {
                        const new_right = try recursiveInsert(right, point, config, alloc);
                        if (new_right != node.right) {
                            recursiveFree(right, alloc);
                            node.right = new_right;
                        }
                    } else {
                        node.right = try alloc.create(Node);
                        node.right.?.* = Node{
                            .point = point,
                            .axis = node.axis,
                            .left = null,
                            .right = null,
                        };
                    }
                }
                pullUp(node);
                return try balanceSubtree(node, config, alloc);
            }

            fn checkIntersection(node: *Node, mins: Point, maxs: Point) bool {
                var intersect = true;
                inline for (0..K) |k| {
                    intersect = intersect and (node.minimums[k] <= maxs[k] and node.maximums[k] >= mins[k]);
                }
                return intersect;
            }

            fn checkSubset(node: *Node, mins: Point, maxs: Point) bool {
                var subset = true;
                inline for (0..K) |k| {
                    subset = subset and (node.minimums[k] >= mins[k] and node.maximums[k] <= maxs[k]);
                }
                return subset;
            }

            fn checkContained(node: *Node, mins: Point, maxs: Point) bool {
                var contained = true;
                inline for (0..K) |k| {
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
            fn recursiveSearchBox(node: *Node, mins: Point, maxs: Point, alloc: std.mem.Allocator) ![]*Node {
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
                if (checkContained(node, mins, maxs) and !node.deleted) {
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
            fn recursiveUpdateBox(
                node: *Node,
                delete: bool,
                config: Config,
                mins: Point,
                maxs: Point,
                alloc: std.mem.Allocator,
            ) !?*Node {
                // if the node range is outside the box, return
                if (!checkIntersection(node, mins, maxs))
                    return node;
                // if the node range is a subset of the box, mark the tree and node as deleted
                if (checkSubset(node, mins, maxs)) {
                    node.tree_deleted = delete;
                    node.deleted = delete;
                } else {
                    if (checkContained(node, mins, maxs)) {
                        node.deleted = delete;
                    }
                    if (node.left) |left| {
                        const new_left = try recursiveUpdateBox(left, delete, config, mins, maxs, alloc);
                        if (new_left != node.left) {
                            recursiveFree(left, alloc);
                            node.left = new_left;
                        }
                    }
                    if (node.right) |right| {
                        const new_right = try recursiveUpdateBox(right, delete, config, mins, maxs, alloc);
                        if (new_right != node.right) {
                            recursiveFree(right, alloc);
                            node.right = new_right;
                        }
                    }
                }
                pullUp(node);
                return try balanceSubtree(node, config, alloc);
            }

            fn axisOrder(axis_id: usize, lhs: Point, rhs: Point) bool {
                return (lhs[axis_id] < rhs[axis_id]);
            }

            fn balanceSubtree(node: *Node, config: Config, alloc: std.mem.Allocator) !?*Node {
                // The lower these values are, the more likely it is that the tree will be rebalanced
                // α_bal ∈ (0.5, 1)
                //const a_bal = 0.7;
                const a_bal = config.a_bal;
                // α_del ∈ (0, 1)
                //const a_del = 0.5;
                const a_del = config.a_del;

                const treesize = @as(f32, @floatFromInt(node.tree_size));
                const treesize_left = @as(f32, @floatFromInt(if (node.left) |left| left.tree_size else 0));
                const treesize_right = @as(f32, @floatFromInt(if (node.right) |right| right.tree_size else 0));
                const num_invalid = @as(f32, @floatFromInt(node.invalid_num));

                const balanced_left = treesize_left < a_bal * (treesize - 1);
                const balanced_right = treesize_right < a_bal * (treesize - 1);
                const low_invalid = num_invalid < a_del * treesize;

                // TODO: finish me (rebuild to balance)
                //if (node.tree_size < n_max or !enable_parallel) {
                //      rebuildTree(node);
                //    } else {
                //        ThreadSpawn(ParallelRebuild, node);
                //    }
                //}
                if (!balanced_left or !balanced_right or !low_invalid) {
                    return try rebuildSubtree(node, alloc);
                }
                return node;
            }
        };

        root: ?*Node,
        config: Config,
        alloc: std.mem.Allocator,

        pub fn init(Vec: []Point, config: Config, alloc: std.mem.Allocator) !Self {
            return .{
                .root = try Node.buildSubtree(Vec, alloc),
                .alloc = alloc,
                .config = config,
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

        pub fn insert(self: *Self, point: Point) !void {
            if (self.root == null) {
                self.root = try Node.buildSubtree(@constCast(&[_]Point{point}), self.alloc);
            } else {
                if (Node.findInSubtree(self.root.?, point, self.config.relative_tolrance, true)) {}
                const new_root =
                    if (Node.findInSubtree(self.root.?, point, self.config.relative_tolrance, true))
                    try Node.recursiveUpdate(self.root.?, point, self.config, false, self.alloc)
                else
                    try Node.recursiveInsert(self.root.?, point, self.config, self.alloc);
                replaceRoot(self, new_root);
            }
        }

        pub fn insertMany(self: *Self, points: []Point) !void {
            for (points) |point| {
                try self.insert(point);
            }
        }

        // Reinsert all points in a box
        pub fn reinsertBox(self: *Self, mins: Point, maxs: Point) !void {
            if (self.root == null) return;
            const new_root = try Node.recursiveUpdateBox(self.root.?, false, self.config, mins, maxs, self.alloc);
            replaceRoot(self, new_root);
        }

        // Remove all points in a box
        pub fn removeBox(self: *Self, mins: Point, maxs: Point) !void {
            if (self.root == null) return;
            const new_root = try Node.recursiveUpdateBox(self.root.?, true, self.config, mins, maxs, self.alloc);
            replaceRoot(self, new_root);
        }

        // Remove a point
        pub fn remove(self: *Self, point: Point) !void {
            if (self.root == null) return;
            const new_root = try Node.recursiveUpdate(self.root.?, point, self.config, true, self.alloc);
            replaceRoot(self, new_root);
        }

        pub fn downsample(self: *Self, length: f32, point: Point) !void {
            // how many hypercubes fill the space
            if (self.root == null) return;
            const root = self.root.?;
            var hypercube_lengths: [K]usize = undefined;
            var total_hypercubes: usize = 1;

            // coord of minimum hypercube from origin
            var min_hypercube: Point = undefined;
            inline for (0..K) |k| {
                min_hypercube[k] = @floor(root.minimums[k] / length);
            }

            inline for (0..K) |k| {
                hypercube_lengths[k] = @intFromFloat(@ceil(root.maximums[k] / length) - min_hypercube[k]);
                total_hypercubes *= hypercube_lengths[k];
            }

            // get the hypercube that the point is in
            var hypercube: [K]usize = undefined;
            inline for (0..K) |k| {
                hypercube[k] = @intFromFloat(@divFloor(point[k], length));
            }

            var hypercube_mins: Point = undefined;
            var hypercube_maxs: Point = undefined;
            inline for (0..K) |k| {
                hypercube_mins[k] = @as(f32, @floatFromInt(hypercube[k])) * length;
                hypercube_maxs[k] = hypercube_mins[k] + length;
            }

            // get all points in the hypercube
            const hypercube_nodes = try root.recursiveSearchBox(hypercube_mins, hypercube_maxs, self.alloc);
            defer self.alloc.free(hypercube_nodes);

            // get only points from the nodes
            var points: []Point = try self.alloc.alloc(Point, hypercube_nodes.len);
            defer self.alloc.free(points);
            for (hypercube_nodes, 0..) |node, h| {
                points[h] = node.point;
            }

            const hypercube_tree = try Node.buildSubtree(points, self.alloc) orelse return;
            defer hypercube_tree.recursiveFree(self.alloc);

            // Delete all nodes in the hypercube
            const new_node = try Node.recursiveUpdateBox(
                root,
                true,
                self.config,
                hypercube_mins,
                hypercube_maxs,
                self.alloc,
            );
            replaceRoot(self, new_node);

            var center = hypercube_mins;
            inline for (0..K) |k| {
                center[k] += length / 2.0;
            }
            var center_result = Node.NnResult{ .point = undefined, .distance = std.math.inf(f32) };
            Node.nnSearch(hypercube_tree, center, false, &center_result);
            if (center_result.distance < std.math.inf(f32)) {
                try self.insert(center_result.point);
            }
        }

        pub fn nearestNeighbor(self: *Self, point: Point) ?Point {
            var nearest = Node.NnResult{ .point = undefined, .distance = std.math.inf(f32) };
            Node.nnSearch(self.root, point, false, &nearest);
            return if (nearest.distance < std.math.inf(f32)) nearest.point else null;
        }

        pub fn kNearestNeighbors(self: *Self, point: Point, k: usize) ![]Point {
            var queue = Node.KnnQueue.init(self.alloc, k);
            defer queue.deinit();

            try Node.knnSearch(self.root, point, false, k, &queue);

            const num_items = @min(k, queue.count());
            var points = try self.alloc.alloc(Point, num_items);

            for (0..num_items) |queue_idx| {
                points[queue_idx] = queue.storage.items[queue_idx].point;
            }
            return points;
        }

        fn replaceRoot(self: *Self, new_node: ?*Node) void {
            if (self.root) |old_root| {
                if (old_root != new_node) {
                    old_root.recursiveFree(self.alloc);
                    self.root = new_node;
                }
            } else {
                self.root = new_node;
            }
        }

        // Gives a point at index if array were to be sorted
        // guarantees that all points less than index are lower in the array
        // and all points greater than index are higher in the array
        fn select(arr: []Point, idx: usize, axis: usize) Point {
            if (arr.len == 1) return arr[0];

            var left: usize = 0;
            var right: usize = arr.len - 1;

            while (right > left) {
                if (right - left > 600) {
                    const idx_float: f32 = @floatFromInt(idx);
                    const n: f32 = @floatFromInt(right - left + 1);
                    const i: f32 = @floatFromInt(idx - left + 1);
                    const z: f32 = @log(n);
                    const s: f32 = 0.5 * std.math.exp(2.0 * z / 3);
                    const sd: f32 = 0.5 * @sqrt(z * s * (n - s) / n) * std.math.sign(i - n / 2);
                    const new_left: usize = @intCast(@max(
                        @as(isize, @intCast(left)),
                        @as(isize, @intFromFloat(idx_float - i * s / n + sd)),
                    ));
                    const new_right: usize = @intCast(@min(
                        @as(isize, @intCast(right)),
                        @as(isize, @intFromFloat(idx_float + (n - i) * s / n + sd)),
                    ));
                    _ = select(arr[new_left .. new_right + 1], idx - new_left, axis);
                }
                const t = arr[idx];
                var i = left;
                var j = right;
                std.mem.swap(Point, &arr[left], &arr[idx]);
                if (arr[right][axis] > t[axis]) {
                    std.mem.swap(Point, &arr[left], &arr[right]);
                }
                while (i < j) {
                    std.mem.swap(Point, &arr[i], &arr[j]);
                    i += 1;
                    j -= 1;
                    while (arr[i][axis] < t[axis]) {
                        i += 1;
                    }
                    while (arr[j][axis] > t[axis]) {
                        j -= 1;
                    }
                }

                if (arr[left][axis] == t[axis]) {
                    std.mem.swap(Point, &arr[left], &arr[j]);
                } else {
                    j += 1;
                    std.mem.swap(Point, &arr[right], &arr[j]);
                }
                if (j <= idx)
                    left = j + 1;
                if (idx <= j and j > 0)
                    right = j - 1;
            }
            return arr[idx];
        }
    };
}

test {
    @import("logger").logLevelSet(.NONE);
    const I3DTree = IKDTree(3);
    const allocator = std.testing.allocator;

    const config = I3DTree.Config{
        .a_bal = 0.75,
        .a_del = 0.5,
        .enable_parallel = false,
        .max_size_single_thread_rebuild = 1,
        .relative_tolrance = 0.00001,
    };

    var points = try allocator.alloc([3]f32, 5 * 5 * 5);
    for (0..5) |i| {
        for (0..5) |j| {
            for (0..5) |k| {
                points[i * 5 * 5 + j * 5 + k] = .{
                    @as(f32, @floatFromInt(i)),
                    @as(f32, @floatFromInt(j)),
                    @as(f32, @floatFromInt(k)),
                };
            }
        }
    }

    var ikd = try I3DTree.init(points, config, allocator);
    allocator.free(points);
    var total_size = ikd.root.?.tree_size - ikd.root.?.invalid_num;

    try std.testing.expectEqual(125, total_size);
    try ikd.insert(.{ 0.5, 0.5, 0.5 });
    total_size = ikd.root.?.tree_size - ikd.root.?.invalid_num;
    try std.testing.expectEqual(126, total_size);

    points = try allocator.alloc([3]f32, 8);
    for (0..2) |i| {
        for (0..2) |j| {
            for (0..2) |k| {
                points[i * 4 + j * 2 + k] = .{
                    @as(f32, @floatFromInt(i)) * 2.4 + 1.2,
                    @as(f32, @floatFromInt(j)) * 2.4 + 1.2,
                    @as(f32, @floatFromInt(k)) * 2.4 + 1.2,
                };
            }
        }
    }
    try ikd.insertMany(points);
    allocator.free(points);
    total_size = ikd.root.?.tree_size - ikd.root.?.invalid_num;
    try std.testing.expectEqual(134, total_size);
    try ikd.remove(.{ 0.5, 0.5, 0.5 });
    try ikd.removeBox(.{ 0, 0, 0 }, .{ 2.4, 2.4, 2.4 });

    try ikd.downsample(2.4, [3]f32{ 0, 0, 0 });
    try ikd.downsample(2.4, [3]f32{ 0, 0, 3 });
    try ikd.downsample(2.4, [3]f32{ 0, 3, 0 });
    try ikd.downsample(2.4, [3]f32{ 0, 3, 3 });
    try ikd.downsample(2.4, [3]f32{ 3, 0, 0 });
    try ikd.downsample(2.4, [3]f32{ 3, 0, 3 });
    try ikd.downsample(2.4, [3]f32{ 3, 3, 0 });
    try ikd.downsample(2.4, [3]f32{ 3, 3, 3 });
    ikd.root.?.pullUp();
    total_size = ikd.root.?.tree_size - ikd.root.?.invalid_num;
    try std.testing.expectEqual(7, total_size);
    try std.testing.expect(ikd.root.?.findInSubtree(.{ 1.2, 1.2, 3.6 }, 0.00001, false));
    try std.testing.expect(ikd.root.?.findInSubtree(.{ 1.2, 3.6, 1.2 }, 0.00001, false));
    try std.testing.expect(ikd.root.?.findInSubtree(.{ 1.2, 3.6, 3.6 }, 0.00001, false));
    try std.testing.expect(ikd.root.?.findInSubtree(.{ 3.6, 1.2, 1.2 }, 0.00001, false));
    try std.testing.expect(ikd.root.?.findInSubtree(.{ 3.6, 1.2, 3.6 }, 0.00001, false));
    try std.testing.expect(ikd.root.?.findInSubtree(.{ 3.6, 3.6, 1.2 }, 0.00001, false));
    try std.testing.expect(ikd.root.?.findInSubtree(.{ 3.6, 3.6, 3.6 }, 0.00001, false));
    var point = ikd.nearestNeighbor(.{ 1, 2, 3 }).?;
    try std.testing.expectApproxEqRel(point[0], 1.2, 0.00001);
    try std.testing.expectApproxEqRel(point[1], 1.2, 0.00001);
    try std.testing.expectApproxEqRel(point[2], 3.6, 0.00001);
    point = ikd.nearestNeighbor(.{ 4, 4, 4 }).?;
    try std.testing.expectApproxEqRel(point[0], 3.6, 0.00001);
    try std.testing.expectApproxEqRel(point[1], 3.6, 0.00001);
    try std.testing.expectApproxEqRel(point[2], 3.6, 0.00001);
    try ikd.print();
    ikd.deinit();
}
