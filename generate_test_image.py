#!/usr/bin/env python3

import os
import cv2
import numpy as np


class TestImageGenerator:
    def __init__(self, output_dir="test_images"):
        self.output_dir = output_dir
        self.sizes = [(640, 480), (1280, 720), (1920, 1080)]
        os.makedirs(self.output_dir, exist_ok=True)

    def generate_all(self):
        print("Generating test images...")

        for cat in ["basic", "complexity", "special"]:
            os.makedirs(f"{self.output_dir}/{cat}", exist_ok=True)

        for color in [(0, 0, 255, 'red'), (0, 255, 0, 'green'), (255, 0, 0, 'blue')]:
            for w, h in self.sizes:
                cv2.imwrite(f"{self.output_dir}/basic/{color[3]}_{w}x{h}.png", 
                            np.full((h, w, 3), color[:3], dtype=np.uint8))
        

        for w, h in self.sizes:
            cv2.imwrite(f"{self.output_dir}/complexity/gradient_{w}x{h}.jpg",
                        np.tile(np.linspace(0, 255, w, dtype=np.uint8), (h, 1)))
            
        for w, h in self.sizes:
            cv2.imwrite(f"{self.output_dir}/complexity/noise_{w}x{h}.jpg", 
                        np.random.randint(0, 256, (h, w, 3), dtype=np.uint8))
        
        cv2.imwrite(f"{self.output_dir}/special/wide_4000x200.jpg", 
                    np.random.randint(0, 256, (200, 4000, 3), dtype=np.uint8))
    
        print(f"Test images generated in {self.output_dir}.")

if __name__ == "__main__":
    TestImageGenerator().generate_all()
