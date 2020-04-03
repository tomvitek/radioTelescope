import board
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306 # Library for controlling OLED

# Init I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Init OLED
oled = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)

# Clear the display
oled.fill(0)
oled.show()
BORDER = 5
# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
image = Image.new('1', (oled.width, oled.height))
 
# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)
 
# Draw a white background
draw.rectangle((0, 0, oled.width, oled.height), outline=255, fill=255)
 
# Draw a smaller inner rectangle
draw.rectangle((BORDER, BORDER, oled.width - BORDER - 1, oled.height - BORDER - 1),
               outline=0, fill=0)
 
# Load default font.
font = ImageFont.load_default()
 
# Draw Some Text
text = "Hello World!"
(font_width, font_height) = font.getsize(text)
draw.text((oled.width//2 - font_width//2, oled.height//2 - font_height//2),
          text, font=font, fill=255)
 
# Display image
oled.image(image)
oled.show()