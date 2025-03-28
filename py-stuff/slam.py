

from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.lcd.device import ili9488
from PIL import ImageFont

import time
import serial


# # LCD intialization
# serial = spi(port=0, device=0, gpio_DC=23, gpio_RST=24)
# device = ili9488(serial, rotate=2,
#                  gpio_LIGHT=18, active_low=False) # BACKLIGHT PIN = GPIO 18, active High)


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
MAP_SIZE = (96, 96)    # map size in inches
REGIONS = 18   # regions in corners in inches (make robot always start in bottom left)
ROBOT_STARTING_POSITION = (6, 2)    # 0.5 ft, 2 in -- may need to flip
OBSTACLE_DIAMETER = 1   # obstacle = 3x3 pixel
ROBOT_DIMENSIONS = (12, 12)     # 1 x 1 ft (12 in)

SCALE = 3   # 1 in = 3 pixels


# adjust map size to pixels
MAP_SIZE_PIXELS = (288, 288)    #288x288 map, 1 in = 3 pixels
LCD_SIZE_PIXELS = (320, 480)

# map bounding box
MAP_BB_BORDER_SIZE = 4  # add on outside of 288x288 
MAP_BB_SIZE = tuple([z + MAP_BB_BORDER_SIZE for z in MAP_SIZE_PIXELS])   # 192x192 based on 288 pixels
MAP_BB_OFFSET_Y = (LCD_SIZE_PIXELS[0] - MAP_SIZE_PIXELS[0]) // 2 # center vertically
MAP_BB_OFFSET_X = 0     # start on far left of LCD

# map offset
MAP_OFFSET_Y = MAP_BB_BORDER_SIZE   #shift vertically for border
MAP_OFFSET_X = MAP_BB_BORDER_SIZE   #shift horizontally for border




robot_position = ROBOT_STARTING_POSITION
robot_direction = 0     # in degrees (0 = vertical/starting orientation)

# adjust for pixel scaling
robot_pixel_size = tuple([z * SCALE for z in ROBOT_DIMENSIONS])
obstacle_pixel_size = int(OBSTACLE_DIAMETER * SCALE)


# store robot mapping data
robot_visited = [0] * 96



def updateLCD():


    # translate real-world to 288x288 display (in to pixels)
    robot_pixel_y = int(robot_position[0] * SCALE)
    robot_pixel_x = int(robot_position[1] * SCALE)

    with canvas(device) as draw:

        # draw map walls (ouside of the 288x288 map)
        draw.rectangle(
            (MAP_BB_OFFSET_X, MAP_BB_OFFSET_Y, MAP_BB_SIZE, MAP_BB_SIZE),
            outline = "white",
            width = MAP_BB_BORDER_SIZE
        )

        # draw the robot
        robot_pixel = (int(robot_position[0] * SCALE), int(robot_position[1] * SCALE))
        draw.el
















