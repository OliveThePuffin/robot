const std = @import("std");
const builtin = @import("builtin");
const log = @import("../../util/logger.zig").log;
const MessageHandler = @import("../../util/message_handler.zig").MessageHandler;
const null_platform = @import("null_platform.zig");
const wayland_platform = @import("wayland_platform.zig");

pub const WindowSystem = struct {
    const Self = @This();

    const RequestEnum = enum {
        CREATE,
        CLOSE,
        FULLSCREEN,
        WINDOWED,
        SET_DIMENSIONS,
        SET_TITLE,
        SET_ICON,
    };

    const Platform = struct {
        const PlatformType = enum {
            Win32,
            Cocoa,
            Wayland,
            X11,
            Null,
        };
        const PlatformState = union(PlatformType) {
            Win32: void,
            Cocoa: void,
            Wayland: wayland_platform.WaylandState,
            X11: void,
            Null: void,
        };
        const PlatformMethods = struct {
            // init
            init_platform: *const fn (*Self) anyerror!void = undefined,
            terminate: *const fn (*Self) void = undefined,
            // monitor
            free_monitor: *const fn () void = undefined,
            get_monitor_pos: *const fn () void = undefined,
            get_monitor_content_scale: *const fn () void = undefined,
            get_monitor_workarea: *const fn () void = undefined,
            get_video_modes: *const fn () void = undefined,
            get_video_mode: *const fn () VideoMode = undefined,
            get_gamma_ramp: *const fn () void = undefined,
            set_gamma_ramp: *const fn () void = undefined,
            // window
            create_window: *const fn () void = undefined,
            destroy_window: *const fn () void = undefined,
            set_window_title: *const fn () void = undefined,
            set_window_icon: *const fn () void = undefined,
            get_window_pos: *const fn () void = undefined,
            set_window_pos: *const fn () void = undefined,
            get_window_size: *const fn () void = undefined,
            set_window_size: *const fn () void = undefined,
            set_window_size_limits: *const fn () void = undefined,
            set_window_aspect_ratio: *const fn () void = undefined,
            get_framebuffer_size: *const fn () void = undefined,
            get_window_frame_size: *const fn () void = undefined,
            get_window_content_scale: *const fn () void = undefined,
            iconify_window: *const fn () void = undefined,
            restore_window: *const fn () void = undefined,
            maximize_window: *const fn () void = undefined,
            show_window: *const fn () void = undefined,
            hide_window: *const fn () void = undefined,
            request_window_attention: *const fn () void = undefined,
            focus_window: *const fn () void = undefined,
            set_window_monitor: *const fn () void = undefined,
            window_focused: *const fn () void = undefined,
            window_iconified: *const fn () void = undefined,
            window_visible: *const fn () void = undefined,
            window_maximized: *const fn () void = undefined,
            window_hovered: *const fn () void = undefined,
            framebuffer_transparent: *const fn () void = undefined,
            get_window_opacity: *const fn () void = undefined,
            set_window_resizable: *const fn () void = undefined,
            set_window_decorated: *const fn () void = undefined,
            set_window_floating: *const fn () void = undefined,
            set_window_opacity: *const fn () void = undefined,
            set_window_mouse_passthrough: *const fn () void = undefined,
            poll_events: *const fn () void = undefined,
            wait_events: *const fn () void = undefined,
            wait_events_timeout: *const fn () void = undefined,
            post_empty_event: *const fn () void = undefined,
            //// EGL
            //get_EGL_platform: fn () void,
            //get_EGL_native_display: fn () void,
            //get_EGL_native_window: fn () void,
            //// vulkan
            //get_required_instance_extensions: fn () void,
            //get_physical_device_present_support: fn () void,
            //create_window_surface: fn () void,
        };

        type: PlatformType,
        state: PlatformState,

        pub fn init(alloc: std.mem.Allocator) !Platform {
            const platform_type = choose_platform_type();
            return Platform{
                .type = platform_type,
                .state = switch (platform_type) {
                    .Wayland => .{ .Wayland = try wayland_platform.WaylandState.init(alloc) },
                    else => .{ .Null = {} },
                },
            };
        }

        pub fn deinit(self: *Platform) void {
            switch (self.type) {
                .Wayland => self.state.Wayland.deinit(),
                else => {},
            }
        }

        fn choose_platform_type() PlatformType {
            var platform_type: PlatformType = .Null;
            defer log(.INFO, @typeName(Self), "Platform chosen: {s}", .{@tagName(platform_type)});
            switch (builtin.os.tag) {
                .linux, .freebsd, .netbsd, .openbsd, .dragonfly, .solaris => if (std.os.getenv("XDG_SESSION_TYPE") == null) {
                    log(.WARN, @typeName(Self), "XDG_SESSION_TYPE not set, defaulting to Null", .{});
                } else if (std.os.getenv("WAYLAND_DISPLAY") != null) {
                    platform_type = .Wayland;
                } else if (std.os.getenv("DISPLAY") != null) {
                    platform_type = .X11;
                } else {
                    log(.WARN, @typeName(Self), "Neither WAYLAND_DISPLAY nor DISPLAY envar set, defaulting to Null", .{});
                },
                .windows => platform_type = .Win32,
                .macos, .ios => platform_type = .Cocoa,
                else => {
                    log(.WARN, @typeName(Self), "Unsupported Platform", .{});
                },
            }
            return platform_type;
        }
    };

    pub const VideoMode = struct {
        /// The width, in screen coordinates, of the video mode.
        width: u32,
        /// The height, in screen coordinates, of the video mode.
        height: u32,
        /// The refresh rate, in Hz, of the video mode.
        refreshRate: u32,
    };

    const WindowMessageHandler = MessageHandler(RequestEnum, void);

    allocator: std.mem.Allocator,
    message_handler: WindowMessageHandler,
    platform: Platform,

    // TODO: Add an array of windows

    // TODO: need to pass in an allocator
    pub fn init() !WindowSystem {
        var gpa = std.heap.GeneralPurposeAllocator(.{}){};
        const window_system = WindowSystem{
            .allocator = gpa.allocator(),
            .message_handler = WindowMessageHandler.init(gpa.allocator()),
            .platform = try Platform.init(gpa.allocator()),
        };
        return window_system;
    }

    pub fn deinit(self: *WindowSystem) void {
        self.platform.deinit();
    }
};
