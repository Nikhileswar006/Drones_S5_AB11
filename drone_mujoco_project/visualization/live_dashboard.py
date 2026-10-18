import os
import sys
import time
import numpy as np
import pygame

class LiveDashboard:
    """
    Full-Screen 3D UAV Flight Simulation with Floating HUD Dashboard
    matching the user's reference design:
    - 3D UAV simulation covers 100% of the screen (Full Screen 3D World).
    - Floating 'UAV LIVE TRACKING RESULTS' telemetry card on the top-left.
    - 3D flight path line connecting waypoints with 3D floating markers.
    - Top-right arena color legend (Blue Start, Amber WPs, Green Destination).
    """

    def __init__(self, 
                 width: int = 1600, 
                 height: int = 900,
                 title: str = "MuJoCo - Full Screen 3D UAV Flight Simulation & Live Dashboard"):
        self.width = width
        self.height = height
        self.title = title

        # Pygame Initialization
        pygame.init()
        pygame.font.init()
        
        # Configure Display
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption(self.title)
        self.clock = pygame.time.Clock()

        # Fonts
        mono_font = pygame.font.match_font('consolas') or pygame.font.match_font('couriernew')
        sans_font = pygame.font.match_font('segoeui') or pygame.font.match_font('arial')

        self.font_hdr = pygame.font.Font(mono_font, 14)
        self.font_titlebar = pygame.font.Font(sans_font, 12)
        self.font_data_mono = pygame.font.Font(mono_font, 13)
        self.font_data_bold = pygame.font.Font(mono_font, 13)
        self.font_label = pygame.font.Font(mono_font, 13)
        self.font_small = pygame.font.Font(sans_font, 11)
        self.font_tag = pygame.font.Font(sans_font, 11)

        # Color Palette matching user reference image
        self.c_hud_bg = (14, 18, 26, 235)      # Dark semi-transparent card
        self.c_hud_titlebar = (24, 30, 42, 245) # Window titlebar
        self.c_border = (0, 229, 255, 140)     # Glowing Cyan outline
        self.c_div = (45, 58, 78)              # Divider lines
        
        # Telemetry Data Colors (matching reference image)
        self.c_green = (0, 230, 118)           # Header green / Reached
        self.c_lime = (118, 255, 3)            # Live time / thrust
        self.c_yellow = (255, 235, 59)         # Roll / Pitch / Yaw angles
        self.c_cyan = (0, 229, 255)            # Coordinates [X, Y, Z]
        self.c_orange = (255, 112, 67)         # Position error / torques
        self.c_magenta = (224, 64, 251)        # Quaternion error
        self.c_blue = (68, 138, 255)           # Start Point Blue
        self.c_amber = (255, 171, 0)           # Intermediate WP Amber
        self.c_red = (255, 60, 60)             # Collision / Danger Red
        self.c_text_lbl = (175, 190, 210)      # Grey-blue labels
        self.c_white = (245, 250, 255)

        # State flags
        self.paused = False
        self.show_hud = True
        self.camera_mode = "Flight Path"

        # Draggable HUD Card properties (UAV Live Tracking Results)
        self.card_w = 425
        self.card_h = 565
        self.card_x = 24
        self.card_y = 24
        self.dragging_card = False
        self.drag_offset_x = 0
        self.drag_offset_y = 0

        # Draggable Legend properties
        self.leg_w = 325
        self.leg_h = 165
        self.leg_x = self.width - self.leg_w - 24
        self.leg_y = 24
        self.dragging_legend = False
        self.leg_drag_offset_x = 0
        self.leg_drag_offset_y = 0

        self.cursor_changed = False

    def draw_scene(self, sim_pixels: np.ndarray):
        """Draw full-screen 3D MuJoCo viewport across the entire window."""
        h, w, _ = sim_pixels.shape
        sim_surf = pygame.image.frombuffer(sim_pixels.tobytes(), (w, h), 'RGB')
        if w != self.width or h != self.height:
            sim_surf = pygame.transform.smoothscale(sim_surf, (self.width, self.height))
        self.screen.blit(sim_surf, (0, 0))

    def draw_paths(self, camera_info: dict, reference_waypoints: list, actual_positions: list,
                   detour_path: list = None, detour_target: np.ndarray = None, avoidance_active: bool = False):
        """
        Renders:
        1. Reference path — planned (Dashed vibrant orange line connecting waypoints)
        2. Safe detour path — avoidance (Dashed vibrant green line active during detour)
        3. Actual path — drone movement (Continuous solid electric blue line tracking flight)
        """
        if not camera_info:
            return

        c_pos = camera_info['pos']
        c_fwd = camera_info['forward']
        c_up = camera_info['up']
        c_right = np.cross(c_fwd, c_up)
        f = 0.5 * self.height / np.tan(np.deg2rad(camera_info.get('fovy', 45.0)) / 2)

        def project(pt):
            d = np.array(pt, dtype=float) - c_pos
            zc = np.dot(d, c_fwd)
            if zc <= 0.2:
                return None
            xc = np.dot(d, c_right)
            yc = np.dot(d, c_up)
            u = int(self.width / 2 + (xc / zc) * f)
            v = int(self.height / 2 - (yc / zc) * f)
            return (u, v)

        def draw_dashed_segment(p1, p2, color, dash_len=14, space_len=8, width=3):
            x1, y1 = p1
            x2, y2 = p2
            dx = x2 - x1
            dy = y2 - y1
            dist = np.hypot(dx, dy)
            if dist < 1.0:
                return
            ux, uy = dx / dist, dy / dist
            curr = 0.0
            while curr < dist:
                start_pt = (int(x1 + curr * ux), int(y1 + curr * uy))
                end_dist = min(curr + dash_len, dist)
                end_pt = (int(x1 + end_dist * ux), int(y1 + end_dist * uy))
                pygame.draw.line(self.screen, color, start_pt, end_pt, width)
                curr += dash_len + space_len

        # -------------------------------------------------------------
        # 1. Reference path — planned (Dashed Orange Line)
        # -------------------------------------------------------------
        if reference_waypoints and len(reference_waypoints) >= 2:
            col_ref = (255, 140, 0) # Vibrant Orange
            for i in range(len(reference_waypoints) - 1):
                p_start = project(reference_waypoints[i])
                p_end = project(reference_waypoints[i+1])
                if p_start and p_end:
                    draw_dashed_segment(p_start, p_end, col_ref, dash_len=14, space_len=8, width=3)

        # -------------------------------------------------------------
        # 2. Safe Detour Path — Avoidance (Dashed Green Line)
        # -------------------------------------------------------------
        if avoidance_active and detour_path and len(detour_path) >= 2:
            col_detour = (0, 230, 118) # Vibrant Neon Green
            for i in range(len(detour_path) - 1):
                p_start = project(detour_path[i])
                p_end = project(detour_path[i+1])
                if p_start and p_end:
                    draw_dashed_segment(p_start, p_end, col_detour, dash_len=10, space_len=6, width=3)

        # 3. Detour Waypoint Marker (Green glowing diamond/sphere)
        if avoidance_active and detour_target is not None:
            p_detour = project(detour_target)
            if p_detour and (-50 <= p_detour[0] <= self.width + 50 and -50 <= p_detour[1] <= self.height + 50):
                pygame.draw.circle(self.screen, (0, 255, 128, 120), p_detour, 9)
                pygame.draw.circle(self.screen, (0, 230, 118), p_detour, 5)
                txt_det = self.font_tag.render("SAFE DETOUR WP", True, (0, 255, 128))
                self.screen.blit(txt_det, (p_detour[0] + 8, p_detour[1] - 8))

        # -------------------------------------------------------------
        # 4. Actual path — drone movement (Solid Electric Blue Line)
        # -------------------------------------------------------------
        if actual_positions and len(actual_positions) >= 2:
            col_act = (0, 140, 255) # Electric Blue
            col_act_glow = (0, 80, 190)
            
            screen_pts = []
            for pt in actual_positions:
                p2d = project(pt)
                if p2d is not None:
                    if -150 <= p2d[0] <= self.width + 150 and -150 <= p2d[1] <= self.height + 150:
                        screen_pts.append(p2d)
                    elif len(screen_pts) >= 2:
                        pygame.draw.lines(self.screen, col_act_glow, False, screen_pts, 5)
                        pygame.draw.lines(self.screen, col_act, False, screen_pts, 3)
                        screen_pts = []
                else:
                    if len(screen_pts) >= 2:
                        pygame.draw.lines(self.screen, col_act_glow, False, screen_pts, 5)
                        pygame.draw.lines(self.screen, col_act, False, screen_pts, 3)
                    screen_pts = []

            if len(screen_pts) >= 2:
                pygame.draw.lines(self.screen, col_act_glow, False, screen_pts, 5)
                pygame.draw.lines(self.screen, col_act, False, screen_pts, 3)

    def draw_3d_marker_tags(self, camera_info: dict, marker_positions: dict):
        """Draw floating 3D callout tags above waypoints directly in the 3D scene."""
        if not camera_info or not marker_positions:
            return

        c_pos = camera_info['pos']
        c_fwd = camera_info['forward']
        c_up = camera_info['up']
        c_right = np.cross(c_fwd, c_up)
        f = 0.5 * self.height / np.tan(np.deg2rad(camera_info.get('fovy', 45.0)) / 2)

        for name, (pt, col, tag) in marker_positions.items():
            d = np.array(pt) - c_pos
            zc = np.dot(d, c_fwd)
            if zc > 0.5:
                xc = np.dot(d, c_right)
                yc = np.dot(d, c_up)
                u = int(self.width / 2 + (xc / zc) * f)
                v = int(self.height / 2 - (yc / zc) * f)

                if 10 < u < self.width - 10 and 10 < v < self.height - 20:
                    tag_txt = self.font_tag.render(tag, True, (255, 255, 255))
                    tw = tag_txt.get_width() + 16
                    th = 20
                    tx = max(10, min(self.width - tw - 10, u - tw // 2))
                    
                    # If marker lies near the movable HUD card area, place tag below point to avoid occlusion
                    in_hud = (self.show_hud and 
                              (self.card_x - 15 <= tx <= self.card_x + self.card_w + 15) and
                              (self.card_y - 15 <= v <= self.card_y + self.card_h + 15))
                    if in_hud:
                        ty = v + 14
                        pin_y_tag = ty
                        pin_y_marker = v + 4
                    else:
                        ty = max(10, v - 26)
                        pin_y_tag = ty + th
                        pin_y_marker = v - 4

                    # Tag background
                    tag_surf = pygame.Surface((tw, th), pygame.SRCALPHA)
                    tag_surf.fill((col[0]//6, col[1]//6, col[2]//6, 225))
                    pygame.draw.rect(tag_surf, col, (0, 0, tw, th), 1, border_radius=4)
                    tag_surf.blit(tag_txt, (8, 2))
                    self.screen.blit(tag_surf, (tx, ty))

                    # Pin line
                    pin_x = max(tx + 4, min(tx + tw - 4, u))
                    pygame.draw.line(self.screen, col, (pin_x, pin_y_tag), (pin_x, pin_y_marker), 1)

    def draw_floating_hud(self, 
                          t: float, 
                          current_wp_idx: int, 
                          total_wps: int,
                          target_name: str,
                          dist_err: float,
                          actual_pos: np.ndarray,
                          target_pos: np.ndarray,
                          euler: np.ndarray,
                          quat_err: np.ndarray,
                          total_thrust: float,
                          torques: np.ndarray,
                          switch_history: list,
                          mission_completed: bool,
                          rmse: float = None,
                          avoidance_info: dict = None):
        """
        Draw the sleek floating HUD overlay window in top-left
        matching the user's reference image structure.
        """
        if not self.show_hud:
            return

        card_x = self.card_x
        card_y = self.card_y
        card_w = self.card_w
        card_h = self.card_h

        # Main Card Surface with Glassmorphic Alpha
        card_surf = pygame.Surface((card_w, card_h), pygame.SRCALPHA)
        card_surf.fill(self.c_hud_bg)
        border_col = (0, 255, 255, 230) if self.dragging_card else self.c_border
        border_width = 2 if self.dragging_card else 1
        pygame.draw.rect(card_surf, border_col, (0, 0, card_w, card_h), border_width, border_radius=8)

        # 1. Window Title Bar
        tb_h = 24
        tb_col = (34, 44, 62, 245) if self.dragging_card else self.c_hud_titlebar
        pygame.draw.rect(card_surf, tb_col, (0, 0, card_w, tb_h), border_top_left_radius=8, border_top_right_radius=8)
        pygame.draw.line(card_surf, (60, 75, 100), (0, tb_h), (card_w, tb_h), 1)
        
        # Window buttons (Mac/Windows decorative style)
        pygame.draw.circle(card_surf, (255, 95, 87), (14, 12), 4)
        pygame.draw.circle(card_surf, (254, 188, 46), (26, 12), 4)
        pygame.draw.circle(card_surf, (40, 201, 64), (38, 12), 4)
        
        txt_wintitle = self.font_titlebar.render("UAV Live Tracking Dashboard", True, self.c_text_lbl)
        card_surf.blit(txt_wintitle, (52, 4))

        # Visual indicator showing card is draggable
        drag_txt = "[Drag to Move]" if not self.dragging_card else "[Moving...]"
        drag_col = (0, 229, 255) if self.dragging_card else (130, 160, 200)
        txt_drag = self.font_small.render(drag_txt, True, drag_col)
        card_surf.blit(txt_drag, (card_w - txt_drag.get_width() - 12, 5))

        # 2. Main Header: ■ UAV LIVE TRACKING RESULTS ■
        y = 36
        header_text = "■  UAV LIVE TRACKING RESULTS  ■"
        txt_hdr = self.font_hdr.render(header_text, True, self.c_green)
        hdr_x = (card_w - txt_hdr.get_width()) // 2
        card_surf.blit(txt_hdr, (hdr_x, y))

        y += 24
        pygame.draw.line(card_surf, (0, 229, 255, 90), (16, y), (card_w - 16, y), 1)

        # 3. Telemetry Rows
        def draw_row(lbl: str, val: str, val_col: tuple, curr_y: int) -> int:
            txt_l = self.font_label.render(lbl, True, self.c_text_lbl)
            txt_v = self.font_data_mono.render(val, True, val_col)
            card_surf.blit(txt_l, (20, curr_y))
            card_surf.blit(txt_v, (170, curr_y))
            return curr_y + 21

        def draw_separator(curr_y: int) -> int:
            curr_y += 2
            pygame.draw.line(card_surf, self.c_div, (16, curr_y), (card_w - 16, curr_y), 1)
            return curr_y + 5

        y += 8
        # Time & Waypoint
        y = draw_row("Time:", f"{t:6.2f} s", self.c_lime, y)
        wp_str = f"WP {current_wp_idx + 1} / {total_wps}" if current_wp_idx < total_wps - 1 else f"Destination ({total_wps}/{total_wps})"
        y = draw_row("Waypoint:", wp_str, self.c_yellow, y)

        y = draw_separator(y)

        # Position Error & Tracking RMSE
        err_color = self.c_green if dist_err < 0.25 else (self.c_yellow if dist_err < 1.0 else self.c_orange)
        y = draw_row("Position Error:", f"{dist_err:6.3f} m", err_color, y)
        if rmse is not None:
            rmse_color = self.c_green if rmse < 0.50 else (self.c_yellow if rmse < 1.5 else self.c_orange)
            y = draw_row("Tracking RMSE:", f"{rmse:6.3f} m", rmse_color, y)

        act_str = f"[ {actual_pos[0]:+5.2f}, {actual_pos[1]:+5.2f}, {actual_pos[2]:+5.2f} ] m"
        tgt_str = f"[ {target_pos[0]:+5.2f}, {target_pos[1]:+5.2f}, {target_pos[2]:+5.2f} ] m"
        y = draw_row("Actual Position:", act_str, self.c_cyan, y)
        y = draw_row("Target Position:", tgt_str, self.c_cyan, y)

        y = draw_separator(y)

        # 4. Obstacle Avoidance Telemetry Section
        if avoidance_info:
            is_avoiding = avoidance_info.get('avoidance_active', False)
            obs_state = avoidance_info.get('state', 'CLEAR')
            active_obs = avoidance_info.get('active_obs_name', 'None')
            nearest_d = avoidance_info.get('nearest_obs_dist', 999.0)
            nearest_name = avoidance_info.get('nearest_obs_id', 'None')
            detour_wp = avoidance_info.get('detour_target', None)
            coll_detected = avoidance_info.get('collision_detected', False)
            coll_count = avoidance_info.get('collision_count', 0)

            if is_avoiding:
                av_txt = f"ACTIVE ({active_obs})"
                av_col = self.c_amber
            elif obs_state == "RESUMING":
                av_txt = "RESUMING NOMINAL"
                av_col = self.c_lime
            else:
                av_txt = "INACTIVE (Path Clear)"
                av_col = (130, 160, 200)
            y = draw_row("Obstacle Avoidance:", av_txt, av_col, y)

            d_col = self.c_red if nearest_d < 0.85 else (self.c_yellow if nearest_d < 1.8 else self.c_green)
            nearest_label = f"{nearest_d:5.2f} m ({nearest_name})" if nearest_d < 50.0 else "All Clear"
            y = draw_row("Nearest Obstacle:", nearest_label, d_col, y)

            if detour_wp is not None:
                det_str = f"[{detour_wp[0]:+4.1f}, {detour_wp[1]:+4.1f}, {detour_wp[2]:+4.1f}] m"
                y = draw_row("Avoidance Target:", det_str, (0, 230, 118), y)
            else:
                y = draw_row("Avoidance Target:", "Nominal Path", (130, 160, 200), y)

            if coll_detected or coll_count > 0:
                coll_str = f"COLLISION ({coll_count})"
                coll_col = self.c_red
            else:
                coll_str = "Collision-Free (0)"
                coll_col = self.c_green
            y = draw_row("Collision Status:", coll_str, coll_col, y)

            y = draw_separator(y)

        # 5. Euler Angles (Roll, Pitch, Yaw in degrees)
        roll_deg = np.rad2deg(euler[0])
        pitch_deg = np.rad2deg(euler[1])
        yaw_deg = np.rad2deg(euler[2])
        y = draw_row("Roll:", f"{roll_deg:+5.1f} deg", self.c_yellow, y)
        y = draw_row("Pitch:", f"{pitch_deg:+5.1f} deg", self.c_yellow, y)
        y = draw_row("Yaw:", f"{yaw_deg:+5.1f} deg", self.c_yellow, y)

        y = draw_separator(y)

        # 6. Quaternion Error [q0, q1, q2, q3]
        if quat_err is not None and len(quat_err) == 4:
            q_str = f"[ {quat_err[0]:5.3f}, {quat_err[1]:5.3f}, {quat_err[2]:5.3f}, {quat_err[3]:5.3f} ]"
        else:
            q_str = "[ 1.000, 0.000, 0.000, 0.000 ]"
        y = draw_row("Quaternion Error:", q_str, self.c_magenta, y)

        y = draw_separator(y)

        # 7. Thrust & Torques
        y = draw_row("Thrust:", f"{total_thrust:6.2f} N", self.c_lime, y)
        tau_str = f"[ {torques[0]:+5.2f}, {torques[1]:+5.2f}, {torques[2]:+5.2f} ] N*m"
        y = draw_row("Torques:", tau_str, self.c_orange, y)

        y = draw_separator(y)

        # 8. Mission Status Banner
        y += 4
        status_text = "★ DESTINATION REACHED ★" if mission_completed else ("PAUSED" if self.paused else "● ACTIVE TRACKING")
        status_col = self.c_cyan if mission_completed else (self.c_amber if self.paused else self.c_green)
        txt_stat = self.font_data_bold.render(status_text, True, status_col)
        stat_x = (card_w - txt_stat.get_width()) // 2
        card_surf.blit(txt_stat, (stat_x, y))

        # Waypoint progress indicator circles
        y += 24
        cw = card_w - 40
        dot_step = cw / max(1, total_wps)
        dot_x0 = 26
        for w_i in range(total_wps + 1):
            cx = int(dot_x0 + w_i * (cw / total_wps))
            cy = y + 4
            if w_i < current_wp_idx + 1 or mission_completed:
                pygame.draw.circle(card_surf, self.c_green, (cx, cy), 5)
            elif w_i == current_wp_idx + 1:
                pygame.draw.circle(card_surf, self.c_amber, (cx, cy), 6)
                pygame.draw.circle(card_surf, (255, 255, 255), (cx, cy), 2)
            else:
                pygame.draw.circle(card_surf, (60, 75, 100), (cx, cy), 4, 1)

        # Blit Card onto Main Screen
        self.screen.blit(card_surf, (card_x, card_y))

    def draw_top_right_legend(self):
        """Draw Arena Trajectory, Detour & Obstacle Legend (Draggable)."""
        if not self.show_hud:
            return

        leg_w, leg_h = self.leg_w, self.leg_h
        leg_x, leg_y = self.leg_x, self.leg_y
        leg_surf = pygame.Surface((leg_w, leg_h), pygame.SRCALPHA)
        leg_surf.fill((14, 18, 26, 235))
        border_col = (0, 255, 255, 220) if self.dragging_legend else (0, 229, 255, 120)
        border_w = 2 if self.dragging_legend else 1
        pygame.draw.rect(leg_surf, border_col, (0, 0, leg_w, leg_h), border_w, border_radius=8)

        txt_hdr = self.font_small.render("ARENA TRAJECTORY & OBSTACLE LEGEND", True, self.c_cyan)
        leg_surf.blit(txt_hdr, (14, 8))
        pygame.draw.line(leg_surf, (45, 58, 78), (12, 24), (leg_w - 12, 24), 1)

        # 1. Reference path — planned (Orange dashed line)
        y = 30
        pygame.draw.line(leg_surf, (255, 140, 0), (14, y + 6), (21, y + 6), 3)
        pygame.draw.line(leg_surf, (255, 140, 0), (25, y + 6), (32, y + 6), 3)
        txt_ref = self.font_data_bold.render("Reference path", True, (255, 150, 0))
        txt_ref_desc = self.font_small.render("— planned", True, self.c_white)
        leg_surf.blit(txt_ref, (40, y))
        leg_surf.blit(txt_ref_desc, (148, y + 2))

        # 2. Actual path — drone movement (Blue solid line)
        y += 22
        pygame.draw.line(leg_surf, (0, 140, 255), (14, y + 6), (32, y + 6), 3)
        txt_act = self.font_data_bold.render("Actual path", True, (0, 180, 255))
        txt_act_desc = self.font_small.render("— drone movement", True, self.c_white)
        leg_surf.blit(txt_act, (40, y))
        leg_surf.blit(txt_act_desc, (126, y + 2))

        # 3. Safe detour path — avoidance (Green dashed line)
        y += 22
        pygame.draw.line(leg_surf, (0, 230, 118), (14, y + 6), (21, y + 6), 3)
        pygame.draw.line(leg_surf, (0, 230, 118), (25, y + 6), (32, y + 6), 3)
        txt_det = self.font_data_bold.render("Safe detour", True, (0, 230, 118))
        txt_det_desc = self.font_small.render("— obstacle avoidance", True, self.c_white)
        leg_surf.blit(txt_det, (40, y))
        leg_surf.blit(txt_det_desc, (126, y + 2))

        # Divider
        y += 23
        pygame.draw.line(leg_surf, (45, 58, 78), (12, y), (leg_w - 12, y), 1)

        # 4. Markers: Start (Blue), Waypoints (Amber), Destination (Green)
        y += 6
        pygame.draw.circle(leg_surf, self.c_blue, (20, y + 7), 4)
        pygame.draw.circle(leg_surf, self.c_amber, (80, y + 7), 4)
        pygame.draw.circle(leg_surf, self.c_green, (165, y + 7), 4)
        txt_pts = self.font_small.render("Start    WP1-3     Destination", True, (200, 215, 230))
        leg_surf.blit(txt_pts, (28, y + 1))
        
        # 5. Physical Obstacles
        y += 18
        pygame.draw.circle(leg_surf, (255, 75, 75), (20, y + 7), 4)
        txt_obs = self.font_small.render("Physical Obstacles (Pillar, Tower, Sphere, Pylon)", True, (255, 140, 140))
        leg_surf.blit(txt_obs, (28, y + 1))

        self.screen.blit(leg_surf, (leg_x, leg_y))

    def draw_bottom_bar(self, t: float, fps: float, alt: float):
        """Draw bottom status bar and keybindings."""
        bar_w = self.width - 48
        bar_h = 30
        bar_x = 24
        bar_y = self.height - bar_h - 16

        bar_surf = pygame.Surface((bar_w, bar_h), pygame.SRCALPHA)
        bar_surf.fill((14, 18, 26, 215))
        pygame.draw.rect(bar_surf, (45, 58, 78), (0, 0, bar_w, bar_h), 1, border_radius=6)

        info_left = f"Flight Time: {t:5.2f}s  |  FPS: {fps:4.1f}  |  Altitude: {alt:4.2f}m  |  Camera: {self.camera_mode}"
        txt_left = self.font_data_mono.render(info_left, True, self.c_white)
        bar_surf.blit(txt_left, (14, 6))

        info_right = "[Space] Pause  [C] Camera  [H] Toggle HUD  [Drag Cards] Reposition  [R] Reset  [S] Save  [Esc] Exit"
        txt_right = self.font_small.render(info_right, True, self.c_cyan)
        bar_surf.blit(txt_right, (bar_w - txt_right.get_width() - 14, 7))

        self.screen.blit(bar_surf, (bar_x, bar_y))

    def process_events(self) -> dict:
        """Handle keyboard, mouse dragging, and window events."""
        actions = {"quit": False, "pause": False, "reset": False, "camera": False, "screenshot": False, "toggle_hud": False}
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                actions["quit"] = True
            elif event.type == pygame.KEYDOWN:
                if event.key in [pygame.K_ESCAPE, pygame.K_q]:
                    actions["quit"] = True
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                    actions["pause"] = True
                elif event.key == pygame.K_r:
                    actions["reset"] = True
                elif event.key == pygame.K_c:
                    actions["camera"] = True
                elif event.key == pygame.K_h:
                    self.show_hud = not self.show_hud
                    actions["toggle_hud"] = True
                elif event.key == pygame.K_s:
                    actions["screenshot"] = True
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1 and self.show_hud:
                    mx, my = event.pos
                    # 1. Check if clicked inside HUD card (UAV Live Tracking Results)
                    if (self.card_x <= mx <= self.card_x + self.card_w) and (self.card_y <= my <= self.card_y + self.card_h):
                        self.dragging_card = True
                        self.drag_offset_x = mx - self.card_x
                        self.drag_offset_y = my - self.card_y
                    # 2. Check if clicked inside Legend card
                    elif (self.leg_x <= mx <= self.leg_x + self.leg_w) and (self.leg_y <= my <= self.leg_y + self.leg_h):
                        self.dragging_legend = True
                        self.leg_drag_offset_x = mx - self.leg_x
                        self.leg_drag_offset_y = my - self.leg_y
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.dragging_card = False
                    self.dragging_legend = False
            elif event.type == pygame.MOUSEMOTION:
                mx, my = event.pos
                if self.dragging_card:
                    self.card_x = mx - self.drag_offset_x
                    self.card_y = my - self.drag_offset_y
                    # Clamp to screen margins
                    self.card_x = max(4, min(self.width - self.card_w - 4, self.card_x))
                    self.card_y = max(4, min(self.height - self.card_h - 44, self.card_y))
                elif self.dragging_legend:
                    self.leg_x = mx - self.leg_drag_offset_x
                    self.leg_y = my - self.leg_drag_offset_y
                    # Clamp to screen margins
                    self.leg_x = max(4, min(self.width - self.leg_w - 4, self.leg_x))
                    self.leg_y = max(4, min(self.height - self.leg_h - 44, self.leg_y))
                else:
                    # Hover cursor feedback
                    hovering_hud = self.show_hud and (self.card_x <= mx <= self.card_x + self.card_w) and (self.card_y <= my <= self.card_y + self.card_h)
                    hovering_leg = self.show_hud and (self.leg_x <= mx <= self.leg_x + self.leg_w) and (self.leg_y <= my <= self.leg_y + self.leg_h)
                    if hovering_hud or hovering_leg:
                        if not self.cursor_changed:
                            try:
                                pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_SIZEALL)
                                self.cursor_changed = True
                            except Exception:
                                pass
                    elif self.cursor_changed:
                        try:
                            pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)
                            self.cursor_changed = False
                        except Exception:
                            pass
        return actions

    def refresh(self):
        """Flip Pygame display buffers and cap frame rate."""
        pygame.display.flip()
        self.clock.tick(50)

    def save_screenshot(self, filename: str):
        """Save current frame to disk."""
        pygame.image.save(self.screen, filename)

    def close(self):
        """Shut down Pygame cleanly."""
        pygame.quit()
