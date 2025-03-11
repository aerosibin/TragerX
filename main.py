import pygame
import math
import random
from robot import Robot
from map import World
from slam import GridBasedSLAM
from a_star import AStar

# Initialize pygame
pygame.init()

# Set up the display for a vertical aspect ratio (YouTube short format)
SCREEN_WIDTH, SCREEN_HEIGHT = 900, 750
UI_HEIGHT = 150  # Height reserved for the UI telemetry at the top
MAP_HEIGHT = SCREEN_HEIGHT - UI_HEIGHT  # The remaining height for the simulated map

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.RESIZABLE)
pygame.display.set_caption('TragerX Simple SLAM Simulator with A* Navigation')

# Font for telemetry data and other texts
font = pygame.font.SysFont("Arial", 24)
small_red_font = pygame.font.SysFont("Arial", 18)

# Create robot and world objects
robot = Robot([SCREEN_WIDTH // 2, UI_HEIGHT + MAP_HEIGHT // 2], 0)  # Start robot in the center of the map area
world = World()  # Define the world bounds
slam = GridBasedSLAM(80, 60)  # Start with 80x60 grid for the map

# Initialize pathfinder
pathfinder = None  # We'll initialize this after the first SLAM update

# Timer for path recalculation
path_recalc_timer = 0
PATH_RECALC_INTERVAL = 30  # Recalculate path every 30 frames (about 1 second)

# Set a random destination point (Point B)
# Make sure it's within the world bounds but not too close to the starting point
def set_random_destination():
    # Generate a point that's reasonably far from the robot's starting position
    min_distance = 400  # Minimum distance from start point
    
    while True:
        dest_x = random.randint(100, world.width - 100)
        dest_y = random.randint(100, world.height - 100)
        
        # Calculate distance from start point
        start_x, start_y = robot.position
        distance = math.sqrt((dest_x - start_x)**2 + (dest_y - start_y)**2)
        
        if distance >= min_distance:
            return dest_x, dest_y

# Set the destination
dest_x, dest_y = set_random_destination()
robot.set_destination(dest_x, dest_y)

# Colors
BACKGROUND_COLOR = (0, 0, 0)  # Black for map background
UI_BACKGROUND_COLOR = (50, 50, 50)  # Darker grey for UI section
TEXT_COLOR = (255, 255, 255)  # White text for telemetry
CLEAR_SPACE_COLOR = (128, 128, 128)  # Grey for clear space
OBSTACLE_FIRST_DETECTED_COLOR = (255, 255, 0)  # Yellow for first detection of obstacle
OBSTACLE_CONFIRMED_COLOR = (0, 255, 0)  # Green for confirmed obstacles
FRONTIER_COLOR = (64, 64, 64)  # Dark grey for frontier
ROBOT_COLOR = (0, 0, 255)  # Blue for robot
PATH_COLOR = (255, 255, 255)  # White for path
PLANNED_PATH_COLOR = (255, 0, 255)  # Magenta for planned path
CURRENT_TARGET_COLOR = (255, 0, 0)  # Red for current target
DESTINATION_COLOR = (0, 255, 255)  # Cyan for destination point
RED_TEXT_COLOR = (255, 0, 0)  # Red for the custom simulator text

# Track robot path (real-world coordinates)
path_points = []

# Main game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        # Exit on pressing the Escape key or 'Q'
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                running = False
    # Get sensor data (now scanning in multiple directions)
    sensor_data = robot.simulate_ultrasonic(world.obstacles)

# Update SLAM for each sensor angle
    for sensor_angle, distance in sensor_data.items():
        slam.sensor_update(robot.position, sensor_angle, distance)


    # Initialize or update pathfinder with current map
    if pathfinder is None:
        pathfinder = AStar(slam.occupancy_grid)
    else:
        pathfinder.grid = slam.occupancy_grid

    # Path planning for A-to-B navigation
    path_recalc_timer += 1
    
    # Check if we need to calculate a new path to the destination
    if (robot.current_path is None or path_recalc_timer >= PATH_RECALC_INTERVAL) and not robot.has_reached_destination:
        path_recalc_timer = 0
        
        # Convert robot position to grid coordinates
        robot_grid_x, robot_grid_y = slam.world_to_grid(robot.position)
        
        # Convert destination to grid coordinates
        dest_grid_x, dest_grid_y = slam.world_to_grid(robot.destination)
        
        # Make sure the destination coordinates are within grid bounds
        dest_grid_x = min(dest_grid_x, slam.occupancy_grid.shape[0] - 1)
        dest_grid_y = min(dest_grid_y, slam.occupancy_grid.shape[1] - 1)
        
        # Plan path to destination
        new_path = pathfinder.find_path((robot_grid_x, robot_grid_y), (dest_grid_x, dest_grid_y))
        
        if new_path:
            robot.current_path = new_path
            robot.path_index = 0
            print(f"New path planned with {len(new_path)} points")
        else:
            print("No path found, will try again later")
    
    # Update robot movement based on the current path
    robot.update_navigation(slam.occupancy_grid)

    # Clear the screen with background color
    screen.fill(BACKGROUND_COLOR)

    # Draw the UI section at the top
    pygame.draw.rect(screen, UI_BACKGROUND_COLOR, pygame.Rect(0, 0, SCREEN_WIDTH, UI_HEIGHT))

    # Text for Telemetry display and top UI
    title_text = small_red_font.render("TragerX SLAM simulation", True, RED_TEXT_COLOR)
    robot_text = font.render("Robot sensor: LIDAR", True, TEXT_COLOR)
    
    # Show navigation status
    status = "Reached Airline Counter" if robot.has_reached_destination else "Navigating to Airline Counter"
    telemetry_text = font.render(f"Angle: {robot.angle:.2f}° | Position: ({int(robot.position[0])}, {int(robot.position[1])})", True, TEXT_COLOR)
    status_text = font.render(f"Status: {status}", True, TEXT_COLOR)
    
    screen.blit(title_text, (10, 10))
    screen.blit(robot_text, (10, 40))
    screen.blit(telemetry_text, (10, 70))
    screen.blit(status_text, (10, 100))

    # Track robot's path (append to the path array)
    path_points.append((robot.position[0], robot.position[1]))

    # Centering: Keep the robot centered in the map area
    robot_screen_x = SCREEN_WIDTH // 2
    robot_screen_y = UI_HEIGHT + MAP_HEIGHT // 2

    # Calculate offset based on robot's position relative to the grid
    offset_x = SCREEN_WIDTH // 2 - robot.position[0]
    offset_y = UI_HEIGHT + MAP_HEIGHT // 2 - robot.position[1]

    # Draw the SLAM map (only what the robot has detected)
    slam_map = slam.get_map()
    for x in range(slam_map.shape[0]):
        for y in range(slam_map.shape[1]):
            rect_x = x * 10 + offset_x
            rect_y = y * 10 + offset_y

            if rect_y > UI_HEIGHT:  # Ensure map is drawn below the UI section
                if slam_map[x, y] == 1:  # Clear space (grey)
                    pygame.draw.rect(screen, CLEAR_SPACE_COLOR, pygame.Rect(rect_x, rect_y, 10, 10), 1)
                elif slam_map[x, y] == 2:  # First detected obstacle (yellow)
                    pygame.draw.rect(screen, OBSTACLE_FIRST_DETECTED_COLOR, pygame.Rect(rect_x, rect_y, 10, 10))
                elif slam_map[x, y] == 3:  # Confirmed obstacle (green)
                    pygame.draw.rect(screen, OBSTACLE_CONFIRMED_COLOR, pygame.Rect(rect_x, rect_y, 10, 10))

    # Draw robot's path using the real-world positions
    if len(path_points) > 1:
        # Keep path static relative to the world, rather than shifting with the robot
        transformed_path = [(x - robot.position[0] + robot_screen_x, y - robot.position[1] + robot_screen_y) for (x, y) in path_points]
        # Ensure the path does not draw in the telemetry area
        transformed_path = [(x, y) for (x, y) in transformed_path if y > UI_HEIGHT]
        pygame.draw.lines(screen, PATH_COLOR, False, transformed_path, 2)
    
    # Draw destination point (Point B)
    if robot.destination:
        # Calculate screen coordinates for destination
        dest_screen_x = robot.destination[0] - robot.position[0] + robot_screen_x
        dest_screen_y = robot.destination[1] - robot.position[1] + robot_screen_y
        
        if dest_screen_y > UI_HEIGHT:  # Don't draw in UI area
            # Draw a distinctive marker for the destination
            pygame.draw.circle(screen, DESTINATION_COLOR, (dest_screen_x, dest_screen_y), 8)
            # Add a crosshair
            pygame.draw.line(screen, DESTINATION_COLOR, (dest_screen_x - 12, dest_screen_y), (dest_screen_x + 12, dest_screen_y), 2)
            pygame.draw.line(screen, DESTINATION_COLOR, (dest_screen_x, dest_screen_y - 12), (dest_screen_x, dest_screen_y + 12), 2)
    
    # Draw the planned path
    if robot.current_path:
        # Draw all points in the planned path
        for i, (grid_x, grid_y) in enumerate(robot.current_path):
            # Convert grid coordinates to screen coordinates
            screen_x = grid_x * 10 + 5 + offset_x  # Center of the grid cell
            screen_y = grid_y * 10 + 5 + offset_y  # Center of the grid cell
            
            if screen_y > UI_HEIGHT:  # Don't draw in UI area
                # Highlight the current target
                if i == robot.path_index:
                    pygame.draw.circle(screen, CURRENT_TARGET_COLOR, (screen_x, screen_y), 5)
                else:
                    pygame.draw.circle(screen, PLANNED_PATH_COLOR, (screen_x, screen_y), 3)
        
        # Draw lines connecting the path points
        if len(robot.current_path) > 1:
            path_lines = [(p[0] * 10 + 5 + offset_x, p[1] * 10 + 5 + offset_y) for p in robot.current_path]
            path_lines = [(x, y) for (x, y) in path_lines if y > UI_HEIGHT]  # Filter out UI area
            if path_lines:
                pygame.draw.lines(screen, PLANNED_PATH_COLOR, False, path_lines, 1)

    # Draw the robot at the center of the map area
    pygame.draw.circle(screen, ROBOT_COLOR, (robot_screen_x, robot_screen_y), robot.radius)
    line_length = robot.radius + 10
    direction_x = robot_screen_x + line_length * math.cos(math.radians(robot.angle))
    direction_y = robot_screen_y + line_length * math.sin(math.radians(robot.angle))
    pygame.draw.line(screen, (255, 255, 255), (robot_screen_x, robot_screen_y), (direction_x, direction_y), 2)

    # Update the display
    pygame.display.flip()

    # Small delay to control the frame rate
    pygame.time.delay(30)

pygame.quit()