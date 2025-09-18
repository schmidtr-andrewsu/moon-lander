import math
import random
import sys
import pygame

# -----------------------------
# Config
# -----------------------------
WIDTH, HEIGHT = 900, 650
FPS = 60

GRAVITY = 0.12            # px/s^2
MAIN_THRUST = 0.22        # acceleration when main engine on
SIDE_THRUST = 0.08        # acceleration sideways (RCS)
ROT_SPEED = 120           # deg/s when rotating
FUEL_START = 100.0        # fuel units
FUEL_MAIN_BURN = 20.0     # fuel / second for main engine
FUEL_RCS_BURN = 6.0       # fuel / second for RCS

# Safe landing thresholds
MAX_LAND_ANGLE = 8        # degrees from upright
MAX_VX = 1.8              # px/frame (approx) horizontal speed
MAX_VY = 2.5              # px/frame vertical speed

PAD_WIDTH = 120
PAD_HEIGHT = 8

# Colors
WHITE   = (240, 240, 240)
BLACK   = (10, 10, 10)
GREEN   = (80, 220, 100)
RED     = (240, 80, 80)
ORANGE  = (250, 170, 60)
CYAN    = (90, 200, 220)
GRAY    = (110, 110, 110)
DGRAY   = (40, 40, 40)
YELLOW  = (255, 230, 120)

# -----------------------------
# Helpers
# -----------------------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def rot_point(px, py, cx, cy, angle_deg):
    a = math.radians(angle_deg)
    s, c = math.sin(a), math.cos(a)
    px, py = px - cx, py - cy
    return (px*c - py*s + cx, px*s + py*c + cy)

def line_intersect(a, b, c, d):
    # segments a-b and c-d ; returns (hit, t, u, point)
    (x1, y1), (x2, y2) = a, b
    (x3, y3), (x4, y4) = c, d
    den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if abs(den) < 1e-8:
        return (False, None, None, None)
    t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / den
    u = ((x1-x3)*(y1-y2) - (y1-y3)*(x1-x2)) / den
    if 0 <= t <= 1 and 0 <= u <= 1:
        hitx = x1 + t*(x2-x1)
        hity = y1 + t*(y2-y1)
        return (True, t, u, (hitx, hity))
    return (False, None, None, None)

# -----------------------------
# Terrain
# -----------------------------
def generate_terrain(seed=None):
    rng = random.Random(seed)
    # Build a polyline from left to right with gentle variation
    pts = []
    x = 0
    y = rng.randint(int(HEIGHT*0.45), int(HEIGHT*0.7))
    step = 30
    roughness = 35
    while x <= WIDTH:
        pts.append((x, y))
        x += step
        y += rng.randint(-roughness, roughness)
        y = clamp(y, int(HEIGHT*0.35), int(HEIGHT*0.85))
    # Choose a flat spot for pad
    pad_x = rng.randint(int(WIDTH*0.15), int(WIDTH*0.8))
    # Flatten a segment around pad_x
    pad_y = None
    flat_pts = []
    for (px, py) in pts:
        if pad_x - PAD_WIDTH//2 - 30 <= px <= pad_x + PAD_WIDTH//2 + 30:
            if pad_y is None:
                pad_y = py
            py = pad_y
        flat_pts.append((px, py))
    # Create landing pad as exact flat segment centered at pad_x
    pad_y = pad_y if pad_y is not None else int(HEIGHT*0.6)
    pad_rect = pygame.Rect(pad_x - PAD_WIDTH//2, pad_y - PAD_HEIGHT//2, PAD_WIDTH, PAD_HEIGHT)
    return flat_pts, pad_rect

def terrain_segments(points):
    segs = []
    for i in range(len(points)-1):
        segs.append((points[i], points[i+1]))
    # add vertical edges down to bottom so we can fill polygon
    return segs

# -----------------------------
# Lander
# -----------------------------
class Lander:
    def __init__(self, x, y):
        self.pos = pygame.Vector2(x, y)
        self.vel = pygame.Vector2(0, 0)
        self.angle = 0.0           # 0 = up
        self.fuel = FUEL_START
        self.alive = True
        self.landed = False

        # For visuals / collision, lander body as polygon in local coords
        self.h = 28
        self.w = 18
        self.leg = 12
        # A simple capsule/triangle-ish body with legs
        self.local_points = [
            (0, -self.h//2),                 # top
            ( self.w//2,  self.h//2),        # right bottom
            ( self.w//2 + 8, self.h//2),     # right foot
            ( self.w//2,  self.h//2),        # back to right bottom
            ( -self.w//2, self.h//2),        # left bottom
            ( -self.w//2 - 8, self.h//2),    # left foot
            ( -self.w//2, self.h//2),        # left bottom
        ]
        # Leg tip points for landing check (indices of feet)
        self.left_foot_local = (-self.w//2 - 8, self.h//2)
        self.right_foot_local = (self.w//2 + 8, self.h//2)

    def world_polygon(self):
        pts = []
        for (lx, ly) in self.local_points:
            pts.append(rot_point(self.pos.x+lx, self.pos.y+ly, self.pos.x, self.pos.y, self.angle))
        return pts

    def foot_points(self):
        lf = rot_point(self.pos.x+self.left_foot_local[0],
                       self.pos.y+self.left_foot_local[1],
                       self.pos.x, self.pos.y, self.angle)
        rf = rot_point(self.pos.x+self.right_foot_local[0],
                       self.pos.y+self.right_foot_local[1],
                       self.pos.x, self.pos.y+self.right_foot_local[1]-self.right_foot_local[1],
                       self.angle)  # center is (pos.x, pos.y)
        # the above looks odd—fix: compute properly
        rf = rot_point(self.pos.x+self.right_foot_local[0],
                       self.pos.y+self.right_foot_local[1],
                       self.pos.x, self.pos.y, self.angle)
        return lf, rf

    def update(self, dt, keys):
        if not self.alive or self.landed:
            return

        thrusting = False
        # Rotate
        if keys[pygame.K_LEFT] or keys[pygame.K_a]:
            if self.fuel > 0:
                self.angle -= ROT_SPEED * dt
                self.fuel = max(0.0, self.fuel - FUEL_RCS_BURN * dt)
        if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
            if self.fuel > 0:
                self.angle += ROT_SPEED * dt
                self.fuel = max(0.0, self.fuel - FUEL_RCS_BURN * dt)

        # Main engine (upwards in ship frame)
        acc = pygame.Vector2(0, GRAVITY)
        if (keys[pygame.K_UP] or keys[pygame.K_w] or keys[pygame.K_SPACE]) and self.fuel > 0:
            thrust_vec = pygame.Vector2(0, -MAIN_THRUST).rotate(self.angle)
            acc += thrust_vec
            thrusting = True
            self.fuel = max(0.0, self.fuel - FUEL_MAIN_BURN * dt)

        # Side thrust (translate) with SHIFT + arrows, optional
        if (keys[pygame.K_q] or keys[pygame.K_COMMA]) and self.fuel > 0:
            acc += pygame.Vector2(-SIDE_THRUST, 0)
            self.fuel = max(0.0, self.fuel - FUEL_RCS_BURN * dt)
        if (keys[pygame.K_e] or keys[pygame.K_PERIOD]) and self.fuel > 0:
            acc += pygame.Vector2(SIDE_THRUST, 0)
            self.fuel = max(0.0, self.fuel - FUEL_RCS_BURN * dt)

        # Integrate
        self.vel += acc
        self.pos += self.vel

        # Keep within screen horizontally (wrap)
        if self.pos.x < -30: self.pos.x = WIDTH + 30
        if self.pos.x > WIDTH + 30: self.pos.x = -30

        # Prevent flying off top too far
        if self.pos.y < 20:
            self.pos.y = 20
            self.vel.y = max(self.vel.y, 0)

        # Crash if out bottom
        if self.pos.y > HEIGHT + 50:
            self.alive = False

        return thrusting

    def draw(self, surf, thrusting=False):
        pts = self.world_polygon()
        pygame.draw.polygon(surf, WHITE, pts, 2)

        # Draw flame
        if thrusting and self.fuel > 0 and self.alive and not self.landed:
            # engine nozzle location: midpoint of bottom between legs
            nozzle = rot_point(self.pos.x, self.pos.y + self.h//2, self.pos.x, self.pos.y, self.angle)
            tip = rot_point(self.pos.x, self.pos.y + self.h//2 + 18, self.pos.x, self.pos.y, self.angle)
            pygame.draw.line(surf, ORANGE, nozzle, tip, 3)

        # Small direction line (up vector)
        top = rot_point(self.pos.x, self.pos.y - self.h//2 - 6, self.pos.x, self.pos.y, self.angle)
        pygame.draw.line(surf, CYAN, (self.pos.x, self.pos.y), top, 1)

# -----------------------------
# Collision & Landing
# -----------------------------
def check_collision_and_landing(lander: Lander, terrain_pts, pad_rect):
    # Check segment collisions against lander polygon edges
    poly = lander.world_polygon()
    poly_edges = list(zip(poly, poly[1:] + poly[:1]))

    segs = terrain_segments(terrain_pts)
    hit_any = False
    for a, b in poly_edges:
        for c, d in segs:
            hit, *_ = line_intersect(a, b, c, d)
            if hit:
                hit_any = True
                break
        if hit_any:
            break

    if not hit_any:
        return "none"

    # If any part intersects terrain near/inside pad rectangle, evaluate landing
    # We'll approximate by checking feet positions against pad top edge band
    lf, rf = lander.foot_points()
    feet_in_x = (pad_rect.left <= lf[0] <= pad_rect.right) and (pad_rect.left <= rf[0] <= pad_rect.right)
    feet_on_y = max(lf[1], rf[1]) <= pad_rect.top + 10 and max(lf[1], rf[1]) >= pad_rect.top - 14

    angle_ok = abs((lander.angle + 360) % 360 - 0) <= MAX_LAND_ANGLE or abs((lander.angle + 360) % 360 - 360) <= MAX_LAND_ANGLE
    vx_ok = abs(lander.vel.x) <= MAX_VX
    vy_ok = abs(lander.vel.y) <= MAX_VY

    if feet_in_x and feet_on_y and angle_ok and vx_ok and vy_ok:
        return "land"
    else:
        return "crash"

# -----------------------------
# UI
# -----------------------------
def draw_hud(surf, lander: Lander, pad_rect, score, best, seed):
    font = pygame.font.SysFont("consolas", 20)
    big  = pygame.font.SysFont("consolas", 44, bold=True)

    # Fuel bar
    pygame.draw.rect(surf, DGRAY, (18, 18, 200, 16), 0, 6)
    fuel_w = int(200 * (lander.fuel / FUEL_START))
    pygame.draw.rect(surf, ORANGE if fuel_w > 20 else RED, (18, 18, fuel_w, 16), 0, 6)
    surf.blit(font.render("FUEL", True, WHITE), (22, 36))

    # Velocity
    vel_text = f"Vx:{lander.vel.x:5.2f}  Vy:{lander.vel.y:5.2f}"
    surf.blit(font.render(vel_text, True, WHITE), (18, 62))

    # Angle
    ang = ((lander.angle + 540) % 360) - 180
    ang_text = f"ANG:{ang:6.2f}°"
    surf.blit(font.render(ang_text, True, WHITE), (18, 86))

    # Score
    surf.blit(font.render(f"Score: {score}   Best: {best}", True, YELLOW), (18, 112))
    surf.blit(font.render(f"Seed: {seed}", True, GRAY), (18, 136))

    # Pad indicator
    pygame.draw.rect(surf, GREEN, pad_rect, 0, 2)
    pygame.draw.rect(surf, (20, 60, 20), pad_rect.inflate(8, 8), 2, 3)

def draw_terrain(surf, pts):
    # Fill the ground polygon to the bottom of screen
    poly = pts[:] + [(WIDTH, HEIGHT), (0, HEIGHT)]
    pygame.draw.polygon(surf, (25, 25, 30), poly, 0)
    # Draw the ridge line
    pygame.draw.lines(surf, GRAY, False, pts, 2)

def draw_message_center(surf, title, subtitle="", color=WHITE):
    big  = pygame.font.SysFont("consolas", 44, bold=True)
    small= pygame.font.SysFont("consolas", 22)
    t = big.render(title, True, color)
    s = small.render(subtitle, True, WHITE)
    rect = t.get_rect(center=(WIDTH//2, HEIGHT//2-10))
    rect2 = s.get_rect(center=(WIDTH//2, HEIGHT//2+30))
    surf.blit(t, rect)
    surf.blit(s, rect2)

# -----------------------------
# Game Loop
# -----------------------------
def main():
    pygame.init()
    pygame.display.set_caption("Moon Lander (pygame)")
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()

    # Background stars
    rng_bg = random.Random(1234)
    stars = [(rng_bg.randint(0, WIDTH), rng_bg.randint(0, HEIGHT), rng_bg.randint(1, 2)) for _ in range(180)]

    # Session score
    score = 0
    best = 0

    # World state factory
    def new_world(seed=None):
        if seed is None:
            seed = random.randint(0, 999999)
        terrain, pad = generate_terrain(seed)
        # Spawn lander above the pad with slight offset
        spawn_x = pad.centerx + random.randint(-80, 80)
        spawn_y = 120
        lander = Lander(spawn_x, spawn_y)
        # Give slight randomized initial drift
        lander.vel = pygame.Vector2(random.uniform(-0.6, 0.6), 0.0)
        return seed, terrain, pad, lander

    seed, terrain_pts, pad_rect, lander = new_world()

    state = "PLAY"  # MENU, PLAY, LANDED, CRASHED
    landed_time = 0
    crashed_time = 0

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0  # seconds
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        # Update
        thrusting = False
        if state == "PLAY":
            thrusting = lander.update(dt, keys)
            result = check_collision_and_landing(lander, terrain_pts, pad_rect)
            if result == "land":
                lander.landed = True
                state = "LANDED"
                landed_time = pygame.time.get_ticks()
                # award points for remaining fuel and gentle landing
                bonus = int(lander.fuel) + max(0, int(20 - abs(lander.vel.y)*4))
                score += 100 + bonus
                best = max(best, score)
            elif result == "crash" or not lander.alive:
                lander.alive = False
                state = "CRASHED"
                crashed_time = pygame.time.get_ticks()
                score = max(0, score - 50)

        # Restart / Next
        if state in ("LANDED", "CRASHED"):
            if keys[pygame.K_r]:
                # same seed (try again)
                seed, terrain_pts, pad_rect, lander = new_world(seed)
                state = "PLAY"
            if keys[pygame.K_n]:
                # new seed
                seed, terrain_pts, pad_rect, lander = new_world(None)
                state = "PLAY"

        # Render
        screen.fill((5, 5, 15))
        for (sx, sy, sz) in stars:
            screen.fill((200, 200, 255), ((sx, sy, sz, sz)))

        draw_terrain(screen, terrain_pts)
        lander.draw(screen, thrusting)
        draw_hud(screen, lander, pad_rect, score, best, seed)

        if state == "LANDED":
            draw_message_center(
                screen,
                "TOUCHDOWN!",
                "Press [N] for new terrain  |  [R] retry same seed",
                color=GREEN,
            )
        elif state == "CRASHED":
            draw_message_center(
                screen,
                "CRASH!",
                "Press [R] retry  |  [N] new terrain",
                color=RED,
            )
        else:
            # Controls hint
            font = pygame.font.SysFont("consolas", 18)
            hint = font.render("Thrust: ↑/W/SPACE  Rotate: ←/→ or A/D  Strafe: Q / E  Restart: R  New: N", True, GRAY)
            screen.blit(hint, (WIDTH//2 - hint.get_width()//2, HEIGHT - 28))

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
