#!/usr/bin/env python3

import subprocess
import os
import time
from pathlib import Path

def run_test(image_path: str, chunk_size: int = 0, repeat_num: int = 1):
    """运行单次分片测试"""
    cmd = [
        'ros2', 'run', 'image_transfer', 'image_transfer_client',
        '--ros-args',
        '-p', f'image_path:={image_path}',
        '-p', 'test_mode:=true',
        '-p', f'chunk_size:={chunk_size}',
    ]

    try:
        chunk_type = "No Chunking" if chunk_size == 0 else f"Chunk Size: {chunk_size}"
        print(f"  第{repeat_num}次测试: {Path(image_path).name} ({chunk_type})")
        subprocess.run(cmd, check=True)
        return True
    except Exception as e:
        print(f"  测试失败: {e}")
        return False

def main():
    """主函数：对每张图片进行5次重复分片测试"""
    base_dir = "test_images/4k"
    repeat_times = 5  # 每张图片重复测试5次
    chunk_sizes = [0, 1000000]  # 0表示不分片，1000000表示1MB分片
    
    # 收集所有图片文件
    image_files = []
    for root, dirs, files in os.walk(base_dir):
        for file in files:
            if file.lower().endswith(('.png', '.jpg', '.jpeg')):
                abs_path = os.path.join(root, file)
                rel_path = os.path.relpath(abs_path, base_dir)
                full_rel_path = os.path.normpath(os.path.join(base_dir, rel_path))
                image_files.append(full_rel_path)
    
    total_tests = len(image_files) * len(chunk_sizes) * repeat_times  # 图片数 × 分片方式 × 重复次数
    current_test = 0
    
    print(f"开始分片测试，共{len(image_files)}张图片，每张图片测试{repeat_times}次")
    print(f"分片方式: {len(chunk_sizes)}种 (无分片, 1MB分片)")
    print(f"总计{total_tests}个测试")
    print("="*60)
    
    for i, image_path in enumerate(image_files, 1):
        print(f"\n[{i}/{len(image_files)}] 测试图片: {Path(image_path).name}")
        
        # 对每种分片方式重复测试5次
        for chunk_size in chunk_sizes:
            chunk_type = "无分片" if chunk_size == 0 else f"1MB分片"
            print(f"  分片方式: {chunk_type}")
            
            for repeat in range(1, repeat_times + 1):
                current_test += 1
                print(f"    进度: {current_test}/{total_tests}")
                
                success = run_test(image_path, chunk_size, repeat)
                if not success:
                    print(f"    警告: 测试失败")
                
                # 添加短暂延迟，避免系统负载过高
                time.sleep(0.5)
    
    print("\n" + "="*60)
    print("所有分片测试完成！")
    print("结果已保存到 chunk_test_results.csv")
    print("可以运行 python3 Scripts/plot_chunk_graph.py 生成图表")

if __name__ == "__main__":
    main()
