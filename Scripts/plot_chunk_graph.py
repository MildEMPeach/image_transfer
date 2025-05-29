#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 读取测试结果
df = pd.read_csv('chunk_test_results.csv')

# 按图片路径和分片大小分组，计算平均值
grouped = df.groupby(['image_path', 'chunk_size']).agg({
    'total_time': ['mean', 'std', 'count'],
    'transfer_time': 'mean',
    'compression_ratio': 'mean'
}).round(4)

# 重置列名
grouped.columns = ['total_time_mean', 'total_time_std', 'test_count', 'transfer_time_mean', 'compression_ratio_mean']
grouped = grouped.reset_index()

# 分离分片和非分片的数据
chunk_df = grouped[grouped["chunk_size"] > 0].reset_index(drop=True)
normal_df = grouped[grouped["chunk_size"] == 0].reset_index(drop=True)

# 确保两个数据框的图片顺序一致
chunk_df = chunk_df.sort_values('image_path').reset_index(drop=True)
normal_df = normal_df.sort_values('image_path').reset_index(drop=True)

# 创建图表
plt.figure(figsize=(15, 8))
bar_width = 0.35
index = np.arange(len(chunk_df))

# 绘制柱状图
bars1 = plt.bar(
    index,
    normal_df["total_time_mean"],
    bar_width,
    label="No Chunking",
    color="skyblue",
    alpha=0.8,
    yerr=normal_df["total_time_std"],  # 添加误差线
    capsize=3
)

bars2 = plt.bar(
    index + bar_width,
    chunk_df["total_time_mean"],
    bar_width,
    label="1MB Chunking",
    color="lightcoral",
    alpha=0.8,
    yerr=chunk_df["total_time_std"],  # 添加误差线
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
add_value_labels(bars1, normal_df["total_time_mean"], normal_df["total_time_std"])
add_value_labels(bars2, chunk_df["total_time_mean"], chunk_df["total_time_std"])

# 设置图表属性
plt.xlabel("Images", fontsize=12)
plt.ylabel("Average Total Time (seconds)", fontsize=12)
plt.title("Chunking vs No Chunking Transfer Time Comparison (5-test average)", fontsize=14, fontweight='bold')

# 设置x轴标签
image_names = [path.split('/')[-1] for path in chunk_df["image_path"]]
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
plt.savefig("chunk_vs_normal_comparison.png", dpi=300, bbox_inches='tight')
print("Chart saved as: chunk_vs_normal_comparison.png")

# 显示统计信息
print("\n" + "="*60)
print("Chunking Test Results Statistics (5-test average)")
print("="*60)

for i in range(len(chunk_df)):
    image_name = image_names[i]
    normal_time = normal_df.iloc[i]["total_time_mean"]
    normal_std = normal_df.iloc[i]["total_time_std"]
    chunk_time = chunk_df.iloc[i]["total_time_mean"]
    chunk_std = chunk_df.iloc[i]["total_time_std"]
    
    improvement = ((normal_time - chunk_time) / normal_time * 100) if normal_time > 0 else 0
    
    print(f"{image_name}:")
    print(f"  No Chunking: {normal_time:.3f}±{normal_std:.3f}s")
    print(f"  1MB Chunking: {chunk_time:.3f}±{chunk_std:.3f}s")
    print(f"  Improvement: {improvement:.1f}%")
    print()

# 显示整体统计
print("Overall Average:")
print(f"  No Chunking: {normal_df['total_time_mean'].mean():.3f}s")
print(f"  1MB Chunking: {chunk_df['total_time_mean'].mean():.3f}s")
overall_improvement = ((normal_df['total_time_mean'].mean() - chunk_df['total_time_mean'].mean()) / normal_df['total_time_mean'].mean() * 100)
print(f"  Overall Improvement: {overall_improvement:.1f}%")

plt.show()
