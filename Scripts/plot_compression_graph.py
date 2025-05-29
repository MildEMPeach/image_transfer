#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']  # 支持中文显示
plt.rcParams['axes.unicode_minus'] = False  # 正常显示负号

# 如果上述字体不可用，尝试使用系统默认字体
try:
    import matplotlib.font_manager as fm
    # 查找系统中可用的中文字体
    font_list = [f.name for f in fm.fontManager.ttflist if 'SimHei' in f.name or 'Microsoft YaHei' in f.name or 'SimSun' in f.name]
    if font_list:
        plt.rcParams['font.family'] = font_list[0]
except:
    # 如果找不到中文字体，使用英文标签
    print("警告: 未找到中文字体，将使用英文标签")

# 读取测试结果
df = pd.read_csv('test_results.csv')

# 按图片路径和压缩类型分组，计算平均值
grouped = df.groupby(['image_path', 'compression_type']).agg({
    'total_time': ['mean', 'std', 'count'],
    'transfer_time': 'mean',
    'compression_ratio': 'mean'
}).round(4)

# 重置列名
grouped.columns = ['total_time_mean', 'total_time_std', 'test_count', 'transfer_time_mean', 'compression_ratio_mean']
grouped = grouped.reset_index()

# 分离JPEG和None的数据
jpeg_df = grouped[grouped["compression_type"] == "jpeg"].reset_index(drop=True)
none_df = grouped[grouped["compression_type"] == "none"].reset_index(drop=True)

# 确保两个数据框的图片顺序一致
jpeg_df = jpeg_df.sort_values('image_path').reset_index(drop=True)
none_df = none_df.sort_values('image_path').reset_index(drop=True)

# 创建图表
plt.figure(figsize=(15, 8))
bar_width = 0.35
index = np.arange(len(jpeg_df))

# 绘制柱状图
bars1 = plt.bar(
    index,
    jpeg_df["total_time_mean"],
    bar_width,
    label="JPEG Compression",
    color="skyblue",
    alpha=0.8,
    yerr=jpeg_df["total_time_std"],  # 添加误差线
    capsize=3
)

bars2 = plt.bar(
    index + bar_width,
    none_df["total_time_mean"],
    bar_width,
    label="No Compression",
    color="lightcoral",
    alpha=0.8,
    yerr=none_df["total_time_std"],  # 添加误差线
    capsize=3
)

# 在柱状图顶部添加数值标签
def add_value_labels(bars, values, std_values=None):
    """在柱状图上添加数值标签"""
    for bar, value, std in zip(bars, values, std_values if std_values is not None else [0]*len(values)):
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height + std + 0.001,
                f'{value:.3f}s',
                ha='center', va='bottom', fontsize=9, fontweight='bold')

# 添加数值标签
add_value_labels(bars1, jpeg_df["total_time_mean"], jpeg_df["total_time_std"])
add_value_labels(bars2, none_df["total_time_mean"], none_df["total_time_std"])

# 设置图表属性
plt.xlabel("Images", fontsize=12)
plt.ylabel("Average Total Time (seconds)", fontsize=12)
plt.title("JPEG vs No Compression Transfer Time Comparison (5-test average)", fontsize=14, fontweight='bold')

# 设置x轴标签
image_names = [path.split('/')[-1] for path in jpeg_df["image_path"]]
plt.xticks(
    index + bar_width / 2,
    image_names,
    rotation=45,
    ha="right",
    fontsize=10
)

plt.legend(fontsize=11)
plt.grid(axis='y', alpha=0.3)
plt.tight_layout()

# 保存图表
plt.savefig("compression_time_comparison.png", dpi=300, bbox_inches='tight')
print("Chart saved as: compression_time_comparison.png")

# 显示统计信息
print("\n" + "="*60)
print("Test Results Statistics (5-test average)")
print("="*60)

for i in range(len(jpeg_df)):
    image_name = image_names[i]
    jpeg_time = jpeg_df.iloc[i]["total_time_mean"]
    jpeg_std = jpeg_df.iloc[i]["total_time_std"]
    none_time = none_df.iloc[i]["total_time_mean"]
    none_std = none_df.iloc[i]["total_time_std"]
    
    improvement = ((none_time - jpeg_time) / none_time * 100) if none_time > 0 else 0
    
    print(f"{image_name}:")
    print(f"  JPEG: {jpeg_time:.3f}±{jpeg_std:.3f}s")
    print(f"  None: {none_time:.3f}±{none_std:.3f}s")
    print(f"  Improvement: {improvement:.1f}%")
    print()

# 显示整体统计
print("Overall Average:")
print(f"  JPEG: {jpeg_df['total_time_mean'].mean():.3f}s")
print(f"  None: {none_df['total_time_mean'].mean():.3f}s")
overall_improvement = ((none_df['total_time_mean'].mean() - jpeg_df['total_time_mean'].mean()) / none_df['total_time_mean'].mean() * 100)
print(f"  Overall Improvement: {overall_improvement:.1f}%")

plt.show()


