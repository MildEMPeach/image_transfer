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
import statistics

class EnhancedTestRunner:
    def __init__(self, output_dir="test_results"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # 设置日志
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(self.output_dir / 'test.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        
        # 测试配置
        self.test_configs = {
            'compression_types': ['none', 'jpeg'],
            'jpeg_qualities': [50, 70, 85, 95],  # 不同JPEG质量
            'chunk_sizes': [0, 1000, 2000, 5000, 10000],  # 不同分片大小
            'repeat_times': 5,  # 每个配置重复5次
        }
        
        # 测试图像分类
        self.image_categories = {
            'basic': 'test_images/basic',
            'complexity': 'test_images/complexity', 
            'special': 'test_images/special'
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
                                  timeout=120, check=True)
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
        """获取所有测试图像及其类别"""
        images = []
        for category, base_path in self.image_categories.items():
            path = Path(base_path)
            if path.exists():
                for img_file in path.glob('*'):
                    if img_file.suffix.lower() in ['.png', '.jpg', '.jpeg']:
                        images.append((str(img_file), category))
        return images

    def run_comprehensive_tests(self):
        """运行全面的测试套件"""
        if not self.check_prerequisites():
            self.logger.error("前置条件检查失败，退出测试")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_file = self.output_dir / f"comprehensive_results_{timestamp}.csv"
        
        # 清理旧的测试结果文件
        if Path("test_results.csv").exists():
            backup_file = self.output_dir / f"backup_test_results_{timestamp}.csv"
            Path("test_results.csv").rename(backup_file)
            self.logger.info(f"备份旧结果到: {backup_file}")
        
        test_images = self.get_test_images()
        total_tests = len(test_images) * len(self.test_configs['compression_types']) * self.test_configs['repeat_times']
        
        # 为JPEG添加质量测试
        for img_path, category in test_images:
            for quality in self.test_configs['jpeg_qualities']:
                total_tests += self.test_configs['repeat_times']
                
        # 为分片传输添加测试
        for img_path, category in test_images:
            for chunk_size in self.test_configs['chunk_sizes']:
                if chunk_size > 0:  # 只对非零分片大小测试
                    total_tests += self.test_configs['repeat_times'] * 2  # jpeg和none
        
        self.logger.info(f"开始运行 {total_tests} 个测试...")
        
        completed = 0
        failed = 0
        
        # 基础压缩测试
        for img_path, category in test_images:
            for compression_type in self.test_configs['compression_types']:
                for repeat in range(self.test_configs['repeat_times']):
                    self.logger.info(f"进度: {completed+1}/{total_tests} - {img_path} ({compression_type}) - 重复 {repeat+1}")
                    
                    result = self.run_single_test(img_path, compression_type)
                    if result['success']:
                        completed += 1
                    else:
                        failed += 1
                        self.logger.error(f"测试失败: {img_path}")
                    
                    time.sleep(1)  # 避免系统负载过高
        
        # JPEG质量测试
        for img_path, category in test_images:
            for quality in self.test_configs['jpeg_qualities']:
                for repeat in range(self.test_configs['repeat_times']):
                    self.logger.info(f"进度: {completed+1}/{total_tests} - {img_path} (jpeg q={quality}) - 重复 {repeat+1}")
                    
                    result = self.run_single_test(img_path, 'jpeg', quality=quality)
                    if result['success']:
                        completed += 1
                    else:
                        failed += 1
                    
                    time.sleep(1)
        
        # 分片传输测试
        for img_path, category in test_images:
            for chunk_size in self.test_configs['chunk_sizes']:
                if chunk_size > 0:
                    for compression_type in ['jpeg', 'none']:
                        for repeat in range(self.test_configs['repeat_times']):
                            self.logger.info(f"进度: {completed+1}/{total_tests} - {img_path} ({compression_type}, chunk={chunk_size}) - 重复 {repeat+1}")
                            
                            result = self.run_single_test(img_path, compression_type, chunk_size=chunk_size)
                            if result['success']:
                                completed += 1
                            else:
                                failed += 1
                            
                            time.sleep(1)
        
        self.logger.info(f"测试完成! 成功: {completed}, 失败: {failed}")
        
        # 分析结果
        self.analyze_results()

    def analyze_results(self):
        """分析测试结果并生成统计报告"""
        if not Path("test_results.csv").exists():
            self.logger.error("未找到测试结果文件")
            return
            
        df = pd.read_csv("test_results.csv")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 生成统计报告
        report = {
            'summary': {
                'total_tests': len(df),
                'unique_images': df['image_path'].nunique(),
                'compression_types': df['compression_type'].unique().tolist(),
                'test_timestamp': timestamp
            },
            'statistics': {}
        }
        
        # 按压缩类型分组统计
        for comp_type in df['compression_type'].unique():
            subset = df[df['compression_type'] == comp_type]
            
            stats = {
                'count': len(subset),
                'total_time': {
                    'mean': subset['total_time'].mean(),
                    'std': subset['total_time'].std(),
                    'min': subset['total_time'].min(),
                    'max': subset['total_time'].max(),
                    'median': subset['total_time'].median()
                },
                'transfer_time': {
                    'mean': subset['transfer_time'].mean(),
                    'std': subset['transfer_time'].std(),
                    'min': subset['transfer_time'].min(),
                    'max': subset['transfer_time'].max(),
                    'median': subset['transfer_time'].median()
                }
            }
            
            if comp_type == 'jpeg':
                stats['compression_ratio'] = {
                    'mean': subset['compression_ratio'].mean(),
                    'std': subset['compression_ratio'].std(),
                    'min': subset['compression_ratio'].min(),
                    'max': subset['compression_ratio'].max()
                }
                stats['compression_time'] = {
                    'mean': subset['compression_time'].mean(),
                    'std': subset['compression_time'].std()
                }
            
            report['statistics'][comp_type] = stats
        
        # 保存报告
        report_file = self.output_dir / f"analysis_report_{timestamp}.json"
        with open(report_file, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        self.logger.info(f"分析报告已保存到: {report_file}")
        
        # 打印简要统计
        print("\n" + "="*50)
        print("测试结果统计摘要")
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

def main():
    runner = EnhancedTestRunner()
    runner.run_comprehensive_tests()

if __name__ == "__main__":
    main() 