#!/usr/bin/env python3

import subprocess
import os
import time
import json
import pandas as pd
import numpy as np
from datetime import datetime
from pathlib import Path
import logging
from typing import List, Dict, Tuple

class QuickTestRunner:
    def __init__(self, output_dir="test_results"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # 设置日志
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(self.output_dir / 'quick_test.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        
        # 快速测试配置 - 减少测试数量但保持统计学意义
        self.test_configs = {
            'compression_types': ['none', 'jpeg'],
            'jpeg_qualities': [70, 95],  # 只测试2种质量
            'chunk_sizes': [0, 2000],    # 只测试1种分片大小
            'repeat_times': 3,           # 减少到3次重复
        }
        
        # 只测试基础图像类别
        self.image_categories = {
            'basic': 'test_images/basic',
        }

    def check_prerequisites(self) -> bool:
        """检查测试前置条件"""
        self.logger.info("检查测试前置条件...")
        
        # 检查ROS2环境
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                self.logger.error("ROS2环境未正确配置")
                return False
        except Exception as e:
            self.logger.error(f"ROS2环境检查失败: {e}")
            return False
            
        # 检查服务器是否运行
        try:
            result = subprocess.run(['ros2', 'service', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            if '/image_transfer' not in result.stdout:
                self.logger.warning("图像传输服务器可能未运行")
        except Exception as e:
            self.logger.warning(f"服务检查失败: {e}")
            
        # 检查测试图像
        missing_images = []
        for category, path in self.image_categories.items():
            if not Path(path).exists():
                missing_images.append(path)
                
        if missing_images:
            self.logger.error(f"缺少测试图像目录: {missing_images}")
            return False
            
        self.logger.info("前置条件检查通过")
        return True

    def run_single_test(self, image_path: str, compression_type: str, 
                       quality: int = 95, chunk_size: int = 0) -> Dict:
        """运行单次测试"""
        cmd = [
            'ros2', 'run', 'image_transfer', 'image_transfer_client',
            '--ros-args',
            '-p', f'image_path:={image_path}',
            '-p', 'test_mode:=true',
            '-p', f'compression_type:={compression_type}',
            '-p', f'quality:={quality}',
            '-p', f'chunk_size:={chunk_size}'
        ]
        
        start_time = time.time()
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, 
                                  timeout=60, check=True)  # 减少超时时间
            end_time = time.time()
            
            return {
                'success': True,
                'execution_time': end_time - start_time,
                'stdout': result.stdout,
                'stderr': result.stderr
            }
        except subprocess.TimeoutExpired:
            self.logger.error(f"测试超时: {image_path}")
            return {'success': False, 'error': 'timeout'}
        except subprocess.CalledProcessError as e:
            self.logger.error(f"测试失败: {image_path}, 错误: {e}")
            return {'success': False, 'error': str(e)}
        except Exception as e:
            self.logger.error(f"未知错误: {e}")
            return {'success': False, 'error': str(e)}

    def get_test_images(self) -> List[Tuple[str, str]]:
        """获取测试图像，只选择代表性的图像"""
        images = []
        for category, base_path in self.image_categories.items():
            path = Path(base_path)
            if path.exists():
                img_files = list(path.glob('*'))
                img_files = [f for f in img_files if f.suffix.lower() in ['.png', '.jpg', '.jpeg']]
                
                # 只选择部分代表性图像
                if len(img_files) > 12:
                    # 选择不同分辨率的代表性图像
                    selected = []
                    resolutions = ['640x480', '1280x720', '1920x1080']
                    colors = ['red', 'green', 'blue', 'white']
                    
                    for res in resolutions:
                        for color in colors:
                            for img_file in img_files:
                                if res in str(img_file) and color in str(img_file):
                                    selected.append(img_file)
                                    break
                    
                    img_files = selected[:12]  # 最多12张图像
                
                for img_file in img_files:
                    images.append((str(img_file), category))
        
        self.logger.info(f"选择了 {len(images)} 张代表性图像进行测试")
        return images

    def run_quick_tests(self):
        """运行快速测试套件"""
        if not self.check_prerequisites():
            self.logger.error("前置条件检查失败，退出测试")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 清理旧的测试结果文件
        if Path("test_results.csv").exists():
            backup_file = self.output_dir / f"backup_test_results_{timestamp}.csv"
            Path("test_results.csv").rename(backup_file)
            self.logger.info(f"备份旧结果到: {backup_file}")
        
        test_images = self.get_test_images()
        
        # 计算总测试数
        basic_tests = len(test_images) * len(self.test_configs['compression_types']) * self.test_configs['repeat_times']
        quality_tests = len(test_images) * len(self.test_configs['jpeg_qualities']) * self.test_configs['repeat_times']
        chunk_tests = len(test_images) * (len(self.test_configs['chunk_sizes']) - 1) * 2 * self.test_configs['repeat_times']
        
        total_tests = basic_tests + quality_tests + chunk_tests
        
        self.logger.info(f"快速测试模式 - 总计 {total_tests} 个测试")
        self.logger.info(f"预计耗时: {total_tests * 5 / 60:.1f} 分钟")
        
        completed = 0
        failed = 0
        
        # 基础压缩测试
        self.logger.info("开始基础压缩测试...")
        for img_path, category in test_images:
            for compression_type in self.test_configs['compression_types']:
                for repeat in range(self.test_configs['repeat_times']):
                    self.logger.info(f"进度: {completed+1}/{total_tests} - {Path(img_path).name} ({compression_type}) - 重复 {repeat+1}")
                    
                    result = self.run_single_test(img_path, compression_type)
                    if result['success']:
                        completed += 1
                    else:
                        failed += 1
                    
                    time.sleep(0.5)  # 减少等待时间
        
        # JPEG质量测试
        self.logger.info("开始JPEG质量测试...")
        for img_path, category in test_images:
            for quality in self.test_configs['jpeg_qualities']:
                for repeat in range(self.test_configs['repeat_times']):
                    self.logger.info(f"进度: {completed+1}/{total_tests} - {Path(img_path).name} (jpeg q={quality}) - 重复 {repeat+1}")
                    
                    result = self.run_single_test(img_path, 'jpeg', quality=quality)
                    if result['success']:
                        completed += 1
                    else:
                        failed += 1
                    
                    time.sleep(0.5)
        
        # 分片传输测试
        self.logger.info("开始分片传输测试...")
        for img_path, category in test_images:
            for chunk_size in self.test_configs['chunk_sizes']:
                if chunk_size > 0:
                    for compression_type in ['jpeg', 'none']:
                        for repeat in range(self.test_configs['repeat_times']):
                            self.logger.info(f"进度: {completed+1}/{total_tests} - {Path(img_path).name} ({compression_type}, chunk={chunk_size}) - 重复 {repeat+1}")
                            
                            result = self.run_single_test(img_path, compression_type, chunk_size=chunk_size)
                            if result['success']:
                                completed += 1
                            else:
                                failed += 1
                            
                            time.sleep(0.5)
        
        self.logger.info(f"快速测试完成! 成功: {completed}, 失败: {failed}")
        
        # 简单分析
        self.quick_analysis()

    def quick_analysis(self):
        """快速分析结果"""
        if not Path("test_results.csv").exists():
            self.logger.error("未找到测试结果文件")
            return
            
        df = pd.read_csv("test_results.csv")
        
        print("\n" + "="*50)
        print("快速测试结果摘要")
        print("="*50)
        print(f"总测试数: {len(df)}")
        print(f"测试图像数: {df['image_path'].nunique()}")
        
        for comp_type in df['compression_type'].unique():
            subset = df[df['compression_type'] == comp_type]
            print(f"\n{comp_type.upper()} 压缩:")
            print(f"  测试次数: {len(subset)}")
            print(f"  平均总时间: {subset['total_time'].mean():.4f}±{subset['total_time'].std():.4f}s")
            print(f"  平均传输时间: {subset['transfer_time'].mean():.4f}±{subset['transfer_time'].std():.4f}s")
            
            if comp_type == 'jpeg':
                print(f"  平均压缩率: {subset['compression_ratio'].mean():.2f}±{subset['compression_ratio'].std():.2f}%")
        
        print(f"\n数据已保存到 test_results.csv")
        print("可以运行 python Scripts/advanced_analysis.py 进行详细分析")

def main():
    runner = QuickTestRunner()
    runner.run_quick_tests()

if __name__ == "__main__":
    main() 