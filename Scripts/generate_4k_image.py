from PIL import Image, ImageDraw, ImageFilter
import random
import math
import numpy as np

def generate_4k_image(output_path="4k_image.png"):
    # 4K resolution (3840 × 2160)
    width, height = 3840, 2160
    
    # Create a new image with RGB mode
    img = Image.new('RGB', (width, height))
    draw = ImageDraw.Draw(img)
    
    # Generate a gradient background
    for y in range(height):
        # Create a horizontal gradient from left to right
        for x in range(width):
            # Base color with some variation
            r = int((x / width) * 255)
            g = int((y / height) * 255)
            b = int(((x + y) / (width + height)) * 255)
            
            # Add some noise for texture
            r = min(255, max(0, r + random.randint(-20, 20)))
            g = min(255, max(0, g + random.randint(-20, 20)))
            b = min(255, max(0, b + random.randint(-20, 20)))
            
            draw.point((x, y), (r, g, b))
    
    # Add some abstract shapes
    for _ in range(15):
        shape_type = random.choice(['circle', 'rectangle', 'polygon'])
        x, y = random.randint(0, width), random.randint(0, height)
        size = random.randint(100, 800)
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        
        if shape_type == 'circle':
            draw.ellipse([x, y, x+size, y+size], fill=color, outline=None)
        elif shape_type == 'rectangle':
            draw.rectangle([x, y, x+size, y+size*0.6], fill=color, outline=None)
        else:  # polygon
            points = []
            for _ in range(5):
                points.append((x + random.randint(-size, size), 
                              y + random.randint(-size, size)))
            draw.polygon(points, fill=color, outline=None)
    
    # Apply some filters for a more artistic look
    img = img.filter(ImageFilter.GaussianBlur(radius=1))
    img = img.filter(ImageFilter.SMOOTH)
    
    # Save the image
    img.save(output_path, quality=95)
    print(f"4K image generated and saved to {output_path}")

# Generate the image
generate_4k_image()
