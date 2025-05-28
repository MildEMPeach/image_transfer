#!/usr/bin/env python3

import os
import cv2
import numpy as np
from pathlib import Path
import requests
from PIL import Image, ImageDraw, ImageFont
import random

class DiverseImageGenerator:
    def __init__(self, output_dir="test_images"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # 多种分辨率测试
        self.resolutions = [
            (320, 240),    # QVGA
            (640, 480),    # VGA
            (800, 600),    # SVGA
            (1024, 768),   # XGA
            (1280, 720),   # HD
            (1920, 1080),  # Full HD
            (2560, 1440),  # 2K
            (3840, 2160),  # 4K
        ]
        
        # 特殊尺寸
        self.special_resolutions = [
            (4000, 200),   # 超宽
            (200, 4000),   # 超高
            (1000, 1000),  # 正方形
            (1600, 900),   # 16:9
            (1440, 1080),  # 4:3
        ]

    def create_directories(self):
        """创建测试目录结构"""
        categories = [
            'basic',           # 基础颜色
            'complexity',      # 复杂度测试
            'special',         # 特殊尺寸
            'natural',         # 自然图像模拟
            'text',            # 文本图像
            'patterns',        # 图案纹理
            'gradients',       # 渐变
            'mixed'            # 混合内容
        ]
        
        for category in categories:
            (self.output_dir / category).mkdir(exist_ok=True)

    def generate_basic_colors(self):
        """生成基础颜色图像"""
        print("生成基础颜色图像...")
        colors = [
            (0, 0, 255, 'red'),
            (0, 255, 0, 'green'), 
            (255, 0, 0, 'blue'),
            (0, 255, 255, 'yellow'),
            (255, 255, 0, 'cyan'),
            (255, 0, 255, 'magenta'),
            (255, 255, 255, 'white'),
            (0, 0, 0, 'black'),
            (128, 128, 128, 'gray')
        ]
        
        for color in colors:
            for w, h in self.resolutions[:6]:  # 前6种分辨率
                img = np.full((h, w, 3), color[:3], dtype=np.uint8)
                filename = f"{color[3]}_{w}x{h}.png"
                cv2.imwrite(str(self.output_dir / 'basic' / filename), img)

    def generate_complexity_images(self):
        """生成不同复杂度的图像"""
        print("生成复杂度测试图像...")
        
        for w, h in self.resolutions[:6]:
            # 1. 纯噪声 (高复杂度)
            noise = np.random.randint(0, 256, (h, w, 3), dtype=np.uint8)
            cv2.imwrite(str(self.output_dir / 'complexity' / f'noise_{w}x{h}.jpg'), noise)
            
            # 2. 渐变 (低复杂度)
            gradient = np.zeros((h, w, 3), dtype=np.uint8)
            for i in range(w):
                gradient[:, i] = [int(255 * i / w)] * 3
            cv2.imwrite(str(self.output_dir / 'complexity' / f'gradient_{w}x{h}.jpg'), gradient)
            
            # 3. 棋盘格 (中等复杂度)
            checkerboard = np.zeros((h, w, 3), dtype=np.uint8)
            square_size = min(w, h) // 16
            for i in range(0, h, square_size):
                for j in range(0, w, square_size):
                    if (i // square_size + j // square_size) % 2 == 0:
                        checkerboard[i:i+square_size, j:j+square_size] = 255
            cv2.imwrite(str(self.output_dir / 'complexity' / f'checkerboard_{w}x{h}.png'), checkerboard)
            
            # 4. 条纹 (中等复杂度)
            stripes = np.zeros((h, w, 3), dtype=np.uint8)
            stripe_width = w // 20
            for i in range(0, w, stripe_width * 2):
                stripes[:, i:i+stripe_width] = 255
            cv2.imwrite(str(self.output_dir / 'complexity' / f'stripes_{w}x{h}.png'), stripes)

    def generate_natural_like_images(self):
        """生成类似自然图像的测试图像"""
        print("生成自然图像模拟...")
        
        for w, h in self.resolutions[:5]:
            # 1. 模拟天空渐变
            sky = np.zeros((h, w, 3), dtype=np.uint8)
            for i in range(h):
                intensity = int(255 * (1 - i / h))
                sky[i, :] = [intensity, intensity // 2, 0]  # 橙色到黑色
            cv2.imwrite(str(self.output_dir / 'natural' / f'sky_{w}x{h}.jpg'), sky)
            
            # 2. 模拟草地纹理
            grass = np.random.normal(50, 20, (h, w, 3)).astype(np.uint8)
            grass[:, :, 1] = np.clip(grass[:, :, 1] + 100, 0, 255)  # 增强绿色
            cv2.imwrite(str(self.output_dir / 'natural' / f'grass_{w}x{h}.jpg'), grass)
            
            # 3. 模拟水波纹
            water = np.zeros((h, w, 3), dtype=np.uint8)
            for i in range(h):
                for j in range(w):
                    wave = int(128 + 50 * np.sin(i * 0.1) * np.cos(j * 0.1))
                    water[i, j] = [wave, wave, 255]
            cv2.imwrite(str(self.output_dir / 'natural' / f'water_{w}x{h}.jpg'), water)

    def generate_text_images(self):
        """生成包含文本的图像"""
        print("生成文本图像...")
        
        texts = [
            "Hello World",
            "ROS2 Image Transfer Test",
            "JPEG Compression Analysis",
            "Performance Evaluation",
            "图像传输性能测试"
        ]
        
        for w, h in self.resolutions[:4]:
            for i, text in enumerate(texts):
                # 创建白色背景
                img = Image.new('RGB', (w, h), color='white')
                draw = ImageDraw.Draw(img)
                
                # 尝试使用系统字体
                try:
                    font_size = min(w, h) // 20
                    font = ImageFont.truetype("arial.ttf", font_size)
                except:
                    font = ImageFont.load_default()
                
                # 计算文本位置
                bbox = draw.textbbox((0, 0), text, font=font)
                text_w = bbox[2] - bbox[0]
                text_h = bbox[3] - bbox[1]
                x = (w - text_w) // 2
                y = (h - text_h) // 2
                
                # 绘制文本
                draw.text((x, y), text, fill='black', font=font)
                
                filename = f'text_{i}_{w}x{h}.png'
                img.save(self.output_dir / 'text' / filename)

    def generate_patterns(self):
        """生成各种图案"""
        print("生成图案纹理...")
        
        for w, h in self.resolutions[:4]:
            # 1. 同心圆
            circles = np.zeros((h, w, 3), dtype=np.uint8)
            center_x, center_y = w // 2, h // 2
            for i in range(h):
                for j in range(w):
                    dist = np.sqrt((i - center_y)**2 + (j - center_x)**2)
                    if int(dist) % 20 < 10:
                        circles[i, j] = 255
            cv2.imwrite(str(self.output_dir / 'patterns' / f'circles_{w}x{h}.png'), circles)
            
            # 2. 螺旋图案
            spiral = np.zeros((h, w, 3), dtype=np.uint8)
            for i in range(h):
                for j in range(w):
                    x, y = j - w//2, i - h//2
                    angle = np.arctan2(y, x)
                    radius = np.sqrt(x**2 + y**2)
                    if int(radius + angle * 10) % 20 < 10:
                        spiral[i, j] = [255, 128, 0]
            cv2.imwrite(str(self.output_dir / 'patterns' / f'spiral_{w}x{h}.png'), spiral)

    def generate_gradients(self):
        """生成各种渐变"""
        print("生成渐变图像...")
        
        for w, h in self.resolutions[:4]:
            # 1. 线性渐变
            linear = np.zeros((h, w, 3), dtype=np.uint8)
            for i in range(w):
                linear[:, i] = [int(255 * i / w), 0, int(255 * (1 - i / w))]
            cv2.imwrite(str(self.output_dir / 'gradients' / f'linear_{w}x{h}.jpg'), linear)
            
            # 2. 径向渐变
            radial = np.zeros((h, w, 3), dtype=np.uint8)
            center_x, center_y = w // 2, h // 2
            max_dist = np.sqrt(center_x**2 + center_y**2)
            for i in range(h):
                for j in range(w):
                    dist = np.sqrt((i - center_y)**2 + (j - center_x)**2)
                    intensity = int(255 * (1 - dist / max_dist))
                    radial[i, j] = [intensity, intensity, intensity]
            cv2.imwrite(str(self.output_dir / 'gradients' / f'radial_{w}x{h}.jpg'), radial)

    def generate_mixed_content(self):
        """生成混合内容图像"""
        print("生成混合内容图像...")
        
        for w, h in self.resolutions[:3]:
            # 创建混合图像：部分纯色，部分噪声，部分渐变
            mixed = np.zeros((h, w, 3), dtype=np.uint8)
            
            # 左侧：纯色
            mixed[:, :w//3] = [255, 0, 0]
            
            # 中间：渐变
            for i in range(w//3, 2*w//3):
                intensity = int(255 * (i - w//3) / (w//3))
                mixed[:, i] = [0, intensity, 0]
            
            # 右侧：噪声
            noise_section = np.random.randint(0, 256, (h, w//3, 3), dtype=np.uint8)
            mixed[:, 2*w//3:] = noise_section
            
            cv2.imwrite(str(self.output_dir / 'mixed' / f'mixed_{w}x{h}.png'), mixed)

    def generate_special_sizes(self):
        """生成特殊尺寸图像"""
        print("生成特殊尺寸图像...")
        
        for w, h in self.special_resolutions:
            # 超宽图像
            if w > h * 2:
                img = np.random.randint(0, 256, (h, w, 3), dtype=np.uint8)
                cv2.imwrite(str(self.output_dir / 'special' / f'wide_{w}x{h}.jpg'), img)
            
            # 超高图像
            elif h > w * 2:
                img = np.random.randint(0, 256, (h, w, 3), dtype=np.uint8)
                cv2.imwrite(str(self.output_dir / 'special' / f'tall_{w}x{h}.jpg'), img)
            
            # 正方形图像
            elif abs(w - h) < 10:
                img = np.full((h, w, 3), [128, 128, 128], dtype=np.uint8)
                cv2.imwrite(str(self.output_dir / 'special' / f'square_{w}x{h}.png'), img)
            
            # 其他特殊比例
            else:
                img = np.random.randint(0, 256, (h, w, 3), dtype=np.uint8)
                cv2.imwrite(str(self.output_dir / 'special' / f'ratio_{w}x{h}.jpg'), img)

    def generate_all(self):
        """生成所有测试图像"""
        print("开始生成多样化测试图像...")
        
        self.create_directories()
        
        self.generate_basic_colors()
        self.generate_complexity_images()
        self.generate_natural_like_images()
        self.generate_text_images()
        self.generate_patterns()
        self.generate_gradients()
        self.generate_mixed_content()
        self.generate_special_sizes()
        
        # 统计生成的图像
        total_images = 0
        for category in self.output_dir.iterdir():
            if category.is_dir():
                count = len(list(category.glob('*')))
                print(f"{category.name}: {count} 张图像")
                total_images += count
        
        print(f"\n总共生成 {total_images} 张测试图像")
        print(f"图像保存在: {self.output_dir}")

def main():
    generator = DiverseImageGenerator()
    generator.generate_all()

if __name__ == "__main__":
    main() 