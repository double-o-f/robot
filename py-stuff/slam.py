

from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.lcd.device import ili9488
from luma.emulator.device import pygame
from PIL import ImageFont, Image, ImageDraw

import time
import serial
import math
from heapq import heappop, heappush
from astar import a_star_search


# # LCD intialization
# LCDserial = spi(port=0, device=0, gpio_DC=23, gpio_RST=24)
# device = ili9488(serial, rotate=2,
#                  gpio_LIGHT=18, active_low=False) # BACKLIGHT PIN = GPIO 18, active High)

device = pygame(width=480, height=320)


# put code from rpi on here



"""
MAP_RES = 4     #cubes per foot

#7.62cm (3in (dick size))

map = [None] * ((MAP_SIZE * MAP_RES) ** 2)
print(len(map))

def scan(ang, dist):
    pass
"""


"""
Real world area: 96x96 in
Screen size: 320x320
Pixel mapping: 1 in = 3.33 pixels -> 3 pixel per in

"""



# map layout
MAP_SIZE = 96    # map size in inches
RESOLUTION = 1   # 1 in per grid cell
REGIONS = 18   # regions in corners in inches (make robot always start in bottom left)
ROBOT_STARTING_POSITION = (6, 94)    # 0.5 ft, 2 in -- may need to flip
OBSTACLE_DIAMETER = 1   # obstacle = 3x3 pixel
ROBOT_DIMENSIONS = (12, 12)     # 1 x 1 ft (12 in)

SCALE = 3   # 1 in = 3 pixels


# adjust map size to pixels
MAP_SIZE_PIXELS = (288, 288)    #288x288 map, 1 in = 3 pixels
LCD_SIZE_PIXELS = (320, 480)



# map bounding box
MAP_BB_BORDER_SIZE = 4  # add on outside of 288x288 
MAP_BB_SIZE = tuple([z + MAP_BB_BORDER_SIZE for z in MAP_SIZE_PIXELS])   # 292x292 based on 288 pixels
MAP_BB_OFFSET_Y = (LCD_SIZE_PIXELS[0] - MAP_SIZE_PIXELS[0] - (2 * MAP_BB_BORDER_SIZE)) // 2 # center vertically
MAP_BB_OFFSET_X = 0     # start on far left of LCD

# map offset
MAP_OFFSET_Y = MAP_BB_OFFSET_Y + MAP_BB_BORDER_SIZE   #shift vertically for border
MAP_OFFSET_X = MAP_BB_BORDER_SIZE  #shift horizontally for border

# latch
LATCH_POSITION = (22, 36) # 1.5 ft, 5 ft
LATCH_BB_SIZE = (4, 0.5)
LATCH_BB_OFFSET_Y = (LCD_SIZE_PIXELS[0] - MAP_SIZE_PIXELS[0] - (2 * MAP_BB_BORDER_SIZE)) // 2 # center vertically
LATCH_BB_OFFSET_X = 4     # start on far left map

# motor encoder
WHEEL_DIAMETER = 4.17323  # inches
WHEEL_BASE = 4.72441 # inches
COUNTS_PER_REVOLUTION = 64

# Encoder data
left_wheel_counts = 0
right_wheel_counts = 0
prev_left_wheel_counts = 0
prev_right_wheel_counts = 0




robot_position_x = ROBOT_STARTING_POSITION[0] + ROBOT_DIMENSIONS[0] / 2
robot_position_y = ROBOT_STARTING_POSITION[1] - ROBOT_DIMENSIONS[1] / 2
robot_direction = 0     # in degrees (0 = vertical/starting orientation)

# adjust for pixel scaling
obstacle_pixel_size = int(OBSTACLE_DIAMETER * SCALE)
robot_pixel_size = tuple([dimension * SCALE for dimension in ROBOT_DIMENSIONS])

# robot mapping data
occupancy_grid = [[-1 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]  # Start with unknown values (-1)
ir_grid = [[0 for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]  # Default to no IR detection (alpha = 0)




# serial connection
# ARD_COM = ''
# ser = serial.Serial(ARD_COM, 115200, timeout=.1)


# fake sensor data
static_ultrasonic_distance = 20  # Distance from static ultrasonic sensor in inches
rotating_sensor_data = [
    {"angle": 0, "distance": 25},   # Straight ahead
    {"angle": 45, "distance": 15},  # Front-right
    {"angle": 90, "distance": 30},  # Right
]
ir_sensor_data = [
    {"angle": 0, "distance": 20, "intensity": 150},   # Straight ahead
    {"angle": 45, "distance": 10, "intensity": 200},  # Front-right
    {"angle": 90, "distance": 25, "intensity": 100},  # Right
]




# ----------- These Are Good and Work ------------- # 


def updateRobotPosition(left_counts, right_counts):
    global robot_position_x, robot_position_y, robot_direction

    # Calculate distances traveled by each wheel
    distance_left = (left_counts / COUNTS_PER_REVOLUTION) * (math.pi * WHEEL_DIAMETER)
    distance_right = (right_counts / COUNTS_PER_REVOLUTION) * (math.pi * WHEEL_DIAMETER)

    # Calculate average distance and change in orientation
    distance_avg = (distance_left + distance_right) / 2
    delta_theta = (distance_right - distance_left) / WHEEL_BASE  # Change in direction (radians)

    # Update orientation (convert to degrees and keep within [0, 360])
    robot_direction = (robot_direction + math.degrees(delta_theta)) % 360

    # Update position using average distance and current orientation
    robot_position_x += distance_avg * math.sin(math.radians(robot_direction))
    robot_position_y -= distance_avg * math.cos(math.radians(robot_direction))  # Minus due to screen coordinates


def updateIrGrid():
    # Process IR sensor data
    for reading in ir_sensor_data:
        # Convert angle to radians
        angle_rad = math.radians(robot_direction + reading["angle"])  # Adjust for robot direction

        # Convert polar to Cartesian
        end_x = robot_position_x + reading["distance"] * math.cos(angle_rad)
        end_y = robot_position_y - reading["distance"] * math.sin(angle_rad)

        # Scale to grid
        grid_x = int(end_x / RESOLUTION)
        grid_y = int(end_y / RESOLUTION)

        # Normalize intensity to alpha value (0-255 range for transparency)
        alpha = int(max(0, min(255, reading["intensity"])))  # Clamp between 0 and 255

        # Update the IR grid
        if 0 <= grid_x < MAP_SIZE and 0 <= grid_y < MAP_SIZE:
            ir_grid[grid_y][grid_x] = alpha  # Store transparency level in the grid


# def render_ir_layer():
#     # Create an empty RGBA image
#     ir_layer = Image.new("RGBA", (MAP_SIZE_PIXELS[0], MAP_SIZE_PIXELS[1]), (0, 0, 0, 0))
#     ir_draw = ImageDraw.Draw(ir_layer)

#     for row in range(MAP_SIZE):
#         for col in range(MAP_SIZE):
#             # Get alpha from IR grid
#             alpha = ir_grid[row][col]

#             if alpha > 0:  # Only draw if there's an IR detection
#                 top_left_x = col * SCALE
#                 top_left_y = row * SCALE
#                 bottom_right_x = top_left_x + SCALE
#                 bottom_right_y = top_left_y + SCALE

#                 # Draw semi-transparent red rectangle
#                 ir_draw.rectangle(
#                     (top_left_x, top_left_y, bottom_right_x, bottom_right_y),
#                     fill=(255, 0, 0, alpha)  # Red with transparency
#                 )

#     return ir_layer


def updateOccupancyGridUltrasonic():
    # Update grid with static ultrasonic sensor
    angle_rad_static = math.radians(robot_direction)  # Robot's current direction
    static_end_x = robot_position_x + static_ultrasonic_distance * math.cos(angle_rad_static)
    static_end_y = robot_position_y - static_ultrasonic_distance * math.sin(angle_rad_static)

    grid_x_static = int(static_end_x / RESOLUTION)
    grid_y_static = int(static_end_y / RESOLUTION)

    if 0 <= grid_x_static < MAP_SIZE and 0 <= grid_y_static < MAP_SIZE:
        occupancy_grid[grid_y_static][grid_x_static] = 1  # Mark obstacle

    # Update grid with rotating sensors
    for reading in rotating_sensor_data:
        angle_rad = math.radians(robot_direction + reading["angle"])  # Adjust for rotation

        # Convert polar to Cartesian
        end_x = robot_position_x + reading["distance"] * math.cos(angle_rad)
        end_y = robot_position_y - reading["distance"] * math.sin(angle_rad)

        grid_x = int(end_x / RESOLUTION)
        grid_y = int(end_y / RESOLUTION)

        # Mark obstacle
        if 0 <= grid_x < MAP_SIZE and 0 <= grid_y < MAP_SIZE:
            occupancy_grid[grid_y][grid_x] = 1

        # Mark free space leading up to obstacle
        steps = int(reading["distance"] / RESOLUTION)
        for i in range(steps):
            clear_x = int((robot_position_x + i * math.cos(angle_rad)) / RESOLUTION)
            clear_y = int((robot_position_y - i * math.sin(angle_rad)) / RESOLUTION)
            if 0 <= clear_x < MAP_SIZE and 0 <= clear_y < MAP_SIZE:
                occupancy_grid[clear_y][clear_x] = 0


def updateLCD():
    # Step 1: Create a blank RGBA image for the IR layer
    ir_layer = Image.new("RGBA", (480, 320), (0, 0, 0, 0))  # Transparent base layer
    ir_draw = ImageDraw.Draw(ir_layer)

    # Step 2: Render IR data on the IR layer
    for row in range(MAP_SIZE):
        for col in range(MAP_SIZE):
            alpha = ir_grid[row][col]  # Intensity affects transparency
            if alpha > 0:  # Only draw cells with IR detection
                # Scale to pixel coordinates
                top_left_x = col * SCALE + MAP_OFFSET_X
                top_left_y = row * SCALE + MAP_OFFSET_Y
                bottom_right_x = top_left_x + SCALE
                bottom_right_y = top_left_y + SCALE

                # Draw semi-transparent red rectangles
                ir_draw.rectangle(
                    (top_left_x, top_left_y, bottom_right_x, bottom_right_y),
                    fill=(255, 0, 0, alpha)  # Red color with transparency
                )

    # Step 3: Render the main map and robot on another RGBA image
    main_canvas = Image.new("RGBA", (480, 320), (255, 255, 255, 255))  # Base white canvas
    draw = ImageDraw.Draw(main_canvas)

    # Draw map walls
    draw.rectangle(
        (
            MAP_BB_OFFSET_X,
            MAP_BB_OFFSET_Y,
            MAP_BB_OFFSET_X + MAP_SIZE_PIXELS[0] + (2 * MAP_BB_BORDER_SIZE),
            MAP_BB_OFFSET_Y + MAP_SIZE_PIXELS[1] + (2 * MAP_BB_BORDER_SIZE),
        ),
        outline="black",
        width=MAP_BB_BORDER_SIZE
    )

    # Draw occupancy grid
    for row in range(MAP_SIZE):
        for col in range(MAP_SIZE):
            top_left_x = col * SCALE + MAP_OFFSET_X
            top_left_y = row * SCALE + MAP_OFFSET_Y
            bottom_right_x = top_left_x + SCALE
            bottom_right_y = top_left_y + SCALE

            if occupancy_grid[row][col] == 1:  # Obstacle
                fill_color = "black"
            else:
                fill_color = "white"

            draw.rectangle(
                (top_left_x, top_left_y, bottom_right_x, bottom_right_y),
                fill=fill_color,
                outline=None
    )
            

    # Draw latch
    latch_pixel_x = int(LATCH_POSITION[0] * SCALE) + LATCH_BB_OFFSET_X
    latch_pixel_y = int(LATCH_POSITION[1] * SCALE) + LATCH_BB_OFFSET_Y

    draw.rectangle(
        (
            latch_pixel_x,
            latch_pixel_y,
            latch_pixel_x + LATCH_BB_SIZE[0] * SCALE,
            latch_pixel_y + LATCH_BB_SIZE[1] * SCALE
        ),
        fill="green"
    )



    # Draw the robot
    robot_pixel_x = int(robot_position_x * SCALE) + MAP_OFFSET_X
    robot_pixel_y = int(robot_position_y * SCALE) + MAP_OFFSET_Y

    draw.rectangle(
        (
            robot_pixel_x - robot_pixel_size[0] // 2,
            robot_pixel_y - robot_pixel_size[1] // 2,
            robot_pixel_x + robot_pixel_size[0] // 2,
            robot_pixel_y + robot_pixel_size[1] // 2,
        ),
        fill="blue"
    )

    # Draw direction arrow
    arrow_length = robot_pixel_size[0]
    arrow_angle_rad = math.radians(robot_direction)

    end_x = robot_pixel_x + arrow_length * math.sin(arrow_angle_rad)
    end_y = robot_pixel_y - arrow_length * math.cos(arrow_angle_rad)

    draw.line(
        [(robot_pixel_x, robot_pixel_y), (end_x, end_y)],
        fill="red",
        width=2
    )


    # Step 4: Combine IR layer with the main canvas
    combined_image = Image.alpha_composite(main_canvas, ir_layer)

    # Step 5: Convert to RGB and display
    device.display(combined_image.convert("RGB"))




# ------------------------ # 










def main():

    global static_ultrasonic_distance, rotating_sensor_data, ir_sensor_data

    # fake sensor data
    static_ultrasonic_distance = 20  # Distance from static ultrasonic sensor in inches
    rotating_sensor_data = [
        {"angle": 0, "distance": 25},   # Straight ahead
        {"angle": 45, "distance": 15},  # Front-right
        {"angle": 90, "distance": 30},  # Right
    ]
    ir_sensor_data = [
        {"angle": 0, "distance": 20, "intensity": 150},   # Straight ahead
        {"angle": 45, "distance": 10, "intensity": 200},  # Front-right
        {"angle": 90, "distance": 25, "intensity": 100},  # Right
    ]

    # update sensor grids
    updateOccupancyGridUltrasonic()
    updateIrGrid()

    # display results
    updateLCD()


    time.sleep(5)



    # encoder initialization
    prev_left_wheel_counts = 0
    prev_right_wheel_counts = 0
        
    # Simulated new encoder counts
    left_wheel_counts = 100  # Example: Encoder counts for left wheel
    right_wheel_counts = 100  # Example: Encoder counts for right wheel


    delta_left_counts = left_wheel_counts - prev_left_wheel_counts
    delta_right_counts = right_wheel_counts - prev_right_wheel_counts

    updateRobotPosition(delta_left_counts, delta_right_counts)

    prev_left_wheel_counts = left_wheel_counts
    prev_right_wheel_counts = right_wheel_counts

    # update for visualization
    robot_pixel_x = int(robot_position_x * SCALE) + MAP_OFFSET_X
    robot_pixel_y = int(robot_position_y * SCALE) + MAP_OFFSET_Y

    print(f"Robot Position: ({robot_position_x:.2f}, {robot_position_y:.2f})")
    print(f"Robot Direction: {robot_direction:.2f} degrees")




    # fake sensor data
    static_ultrasonic_distance = 30  # Distance from static ultrasonic sensor in inches
    rotating_sensor_data = []
    ir_sensor_data = []

    # update sensor grids
    updateOccupancyGridUltrasonic()
    updateIrGrid()

    # display results
    updateLCD()
    input("Press Enter to close the emulator...")



main()











