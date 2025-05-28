#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
from pathlib import Path
import json
from datetime import datetime
import warnings
warnings.filterwarnings('ignore')

class AdvancedAnalyzer:
    def __init__(self, csv_file="test_results.csv", output_dir="analysis_results"):
        self.csv_file = csv_file
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # 设置绘图样式
        plt.style.use('seaborn-v0_8')
        sns.set_palette("husl")
        
    def load_and_clean_data(self):
        """加载和清理数据"""
        if not Path(self.csv_file).exists():
            raise FileNotFoundError(f"测试结果文件 {self.csv_file} 不存在")
        
        df = pd.read_csv(self.csv_file)
        
        # 数据清理
        df = df.dropna()  # 删除空值
        df = df[df['total_time'] > 0]  # 删除异常时间
        
        # 添加派生列
        df['image_size_mb'] = df['original_size'] / (1024 * 1024)
        df['compressed_size_mb'] = df['compressed_size'] / (1024 * 1024)
        df['compression_efficiency'] = df['compressed_size'] / df['original_size']
        
        # 提取图像尺寸信息
        df['resolution'] = df['image_path'].str.extract(r'(\d+x\d+)')
        df['width'] = df['resolution'].str.extract(r'(\d+)x\d+').astype(float)
        df['height'] = df['resolution'].str.extract(r'\d+x(\d+)').astype(float)
        df['pixel_count'] = df['width'] * df['height']
        
        # 图像类型分类
        df['image_category'] = df['image_path'].apply(self._categorize_image)
        
        return df
    
    def _categorize_image(self, path):
        """根据路径分类图像"""
        if 'basic' in path:
            return 'Basic Colors'
        elif 'complexity' in path:
            return 'Complexity Test'
        elif 'natural' in path:
            return 'Natural-like'
        elif 'text' in path:
            return 'Text Images'
        elif 'patterns' in path:
            return 'Patterns'
        elif 'gradients' in path:
            return 'Gradients'
        elif 'mixed' in path:
            return 'Mixed Content'
        elif 'special' in path:
            return 'Special Sizes'
        else:
            return 'Other'
    
    def statistical_summary(self, df):
        """生成统计摘要"""
        print("="*60)
        print("统计摘要报告")
        print("="*60)
        
        summary = {
            'basic_stats': {
                'total_tests': len(df),
                'unique_images': df['image_path'].nunique(),
                'compression_types': df['compression_type'].unique().tolist(),
                'image_categories': df['image_category'].unique().tolist(),
                'resolution_range': f"{df['pixel_count'].min():.0f} - {df['pixel_count'].max():.0f} pixels"
            }
        }
        
        print(f"总测试数: {summary['basic_stats']['total_tests']}")
        print(f"唯一图像数: {summary['basic_stats']['unique_images']}")
        print(f"压缩类型: {', '.join(summary['basic_stats']['compression_types'])}")
        print(f"图像类别: {', '.join(summary['basic_stats']['image_categories'])}")
        print(f"分辨率范围: {summary['basic_stats']['resolution_range']}")
        
        # 按压缩类型分组统计
        print("\n" + "-"*40)
        print("按压缩类型统计")
        print("-"*40)
        
        for comp_type in df['compression_type'].unique():
            subset = df[df['compression_type'] == comp_type]
            print(f"\n{comp_type.upper()}:")
            print(f"  样本数: {len(subset)}")
            print(f"  总时间: {subset['total_time'].mean():.4f}±{subset['total_time'].std():.4f}s")
            print(f"  传输时间: {subset['transfer_time'].mean():.4f}±{subset['transfer_time'].std():.4f}s")
            
            if comp_type == 'jpeg':
                print(f"  压缩率: {subset['compression_ratio'].mean():.2f}±{subset['compression_ratio'].std():.2f}%")
                print(f"  压缩时间: {subset['compression_time'].mean():.4f}±{subset['compression_time'].std():.4f}s")
            
            summary[f'{comp_type}_stats'] = {
                'count': len(subset),
                'total_time_mean': subset['total_time'].mean(),
                'total_time_std': subset['total_time'].std(),
                'transfer_time_mean': subset['transfer_time'].mean(),
                'transfer_time_std': subset['transfer_time'].std()
            }
            
            if comp_type == 'jpeg':
                summary[f'{comp_type}_stats'].update({
                    'compression_ratio_mean': subset['compression_ratio'].mean(),
                    'compression_ratio_std': subset['compression_ratio'].std(),
                    'compression_time_mean': subset['compression_time'].mean(),
                    'compression_time_std': subset['compression_time'].std()
                })
        
        return summary
    
    def hypothesis_testing(self, df):
        """进行假设检验"""
        print("\n" + "="*60)
        print("假设检验结果")
        print("="*60)
        
        results = {}
        
        # 检验JPEG vs None的传输时间差异
        jpeg_data = df[df['compression_type'] == 'jpeg']['total_time']
        none_data = df[df['compression_type'] == 'none']['total_time']
        
        if len(jpeg_data) > 0 and len(none_data) > 0:
            # t检验
            t_stat, p_value = stats.ttest_ind(jpeg_data, none_data)
            
            print(f"JPEG vs None 传输时间 t检验:")
            print(f"  t统计量: {t_stat:.4f}")
            print(f"  p值: {p_value:.6f}")
            print(f"  结论: {'显著差异' if p_value < 0.05 else '无显著差异'} (α=0.05)")
            
            results['jpeg_vs_none_ttest'] = {
                't_statistic': t_stat,
                'p_value': p_value,
                'significant': p_value < 0.05
            }
            
            # Mann-Whitney U检验 (非参数检验)
            u_stat, u_p_value = stats.mannwhitneyu(jpeg_data, none_data, alternative='two-sided')
            print(f"\nMann-Whitney U检验:")
            print(f"  U统计量: {u_stat:.4f}")
            print(f"  p值: {u_p_value:.6f}")
            print(f"  结论: {'显著差异' if u_p_value < 0.05 else '无显著差异'} (α=0.05)")
            
            results['jpeg_vs_none_mannwhitney'] = {
                'u_statistic': u_stat,
                'p_value': u_p_value,
                'significant': u_p_value < 0.05
            }
        
        # 检验图像大小与传输时间的相关性
        correlation, corr_p_value = stats.pearsonr(df['image_size_mb'], df['total_time'])
        print(f"\n图像大小与传输时间相关性:")
        print(f"  Pearson相关系数: {correlation:.4f}")
        print(f"  p值: {corr_p_value:.6f}")
        print(f"  结论: {'显著相关' if corr_p_value < 0.05 else '无显著相关'} (α=0.05)")
        
        results['size_time_correlation'] = {
            'correlation': correlation,
            'p_value': corr_p_value,
            'significant': corr_p_value < 0.05
        }
        
        return results
    
    def generate_visualizations(self, df):
        """生成可视化图表"""
        print("\n生成可视化图表...")
        
        # 设置图表样式
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.titlesize'] = 12
        plt.rcParams['axes.labelsize'] = 10
        
        # 1. 压缩类型对比箱线图
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 3, 1)
        sns.boxplot(data=df, x='compression_type', y='total_time')
        plt.title('总时间对比')
        plt.ylabel('时间 (秒)')
        
        plt.subplot(2, 3, 2)
        sns.boxplot(data=df, x='compression_type', y='transfer_time')
        plt.title('传输时间对比')
        plt.ylabel('时间 (秒)')
        
        plt.subplot(2, 3, 3)
        jpeg_df = df[df['compression_type'] == 'jpeg']
        if len(jpeg_df) > 0:
            sns.histplot(jpeg_df['compression_ratio'], bins=20)
            plt.title('JPEG压缩率分布')
            plt.xlabel('压缩率 (%)')
        
        # 2. 图像大小与性能关系
        plt.subplot(2, 3, 4)
        for comp_type in df['compression_type'].unique():
            subset = df[df['compression_type'] == comp_type]
            plt.scatter(subset['image_size_mb'], subset['total_time'], 
                       label=comp_type, alpha=0.6)
        plt.xlabel('图像大小 (MB)')
        plt.ylabel('总时间 (秒)')
        plt.title('图像大小 vs 传输时间')
        plt.legend()
        
        # 3. 不同图像类别的性能
        plt.subplot(2, 3, 5)
        category_stats = df.groupby(['image_category', 'compression_type'])['total_time'].mean().unstack()
        category_stats.plot(kind='bar', ax=plt.gca())
        plt.title('不同类别图像性能')
        plt.ylabel('平均总时间 (秒)')
        plt.xticks(rotation=45)
        plt.legend(title='压缩类型')
        
        # 4. 分辨率影响
        plt.subplot(2, 3, 6)
        resolution_stats = df.groupby(['resolution', 'compression_type'])['total_time'].mean().unstack()
        if len(resolution_stats) > 0:
            resolution_stats.plot(kind='bar', ax=plt.gca())
            plt.title('分辨率对性能的影响')
            plt.ylabel('平均总时间 (秒)')
            plt.xticks(rotation=45)
            plt.legend(title='压缩类型')
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'performance_analysis.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        # 2. 详细的压缩效率分析
        if len(df[df['compression_type'] == 'jpeg']) > 0:
            plt.figure(figsize=(15, 5))
            
            jpeg_df = df[df['compression_type'] == 'jpeg']
            
            plt.subplot(1, 3, 1)
            plt.scatter(jpeg_df['image_size_mb'], jpeg_df['compression_ratio'], alpha=0.6)
            plt.xlabel('原始图像大小 (MB)')
            plt.ylabel('压缩率 (%)')
            plt.title('图像大小 vs 压缩率')
            
            plt.subplot(1, 3, 2)
            plt.scatter(jpeg_df['compression_ratio'], jpeg_df['compression_time'], alpha=0.6)
            plt.xlabel('压缩率 (%)')
            plt.ylabel('压缩时间 (秒)')
            plt.title('压缩率 vs 压缩时间')
            
            plt.subplot(1, 3, 3)
            plt.scatter(jpeg_df['compressed_size_mb'], jpeg_df['transfer_time'], alpha=0.6)
            plt.xlabel('压缩后大小 (MB)')
            plt.ylabel('传输时间 (秒)')
            plt.title('压缩后大小 vs 传输时间')
            
            plt.tight_layout()
            plt.savefig(self.output_dir / 'compression_analysis.png', dpi=300, bbox_inches='tight')
            plt.show()
    
    def confidence_intervals(self, df):
        """计算置信区间"""
        print("\n" + "="*60)
        print("95% 置信区间")
        print("="*60)
        
        ci_results = {}
        
        for comp_type in df['compression_type'].unique():
            subset = df[df['compression_type'] == comp_type]
            
            # 总时间的置信区间
            mean_time = subset['total_time'].mean()
            sem_time = stats.sem(subset['total_time'])
            ci_time = stats.t.interval(0.95, len(subset)-1, loc=mean_time, scale=sem_time)
            
            print(f"\n{comp_type.upper()} 总时间:")
            print(f"  均值: {mean_time:.4f}s")
            print(f"  95% CI: [{ci_time[0]:.4f}, {ci_time[1]:.4f}]s")
            
            ci_results[f'{comp_type}_total_time'] = {
                'mean': mean_time,
                'ci_lower': ci_time[0],
                'ci_upper': ci_time[1]
            }
            
            # 传输时间的置信区间
            mean_transfer = subset['transfer_time'].mean()
            sem_transfer = stats.sem(subset['transfer_time'])
            ci_transfer = stats.t.interval(0.95, len(subset)-1, loc=mean_transfer, scale=sem_transfer)
            
            print(f"  传输时间均值: {mean_transfer:.4f}s")
            print(f"  传输时间95% CI: [{ci_transfer[0]:.4f}, {ci_transfer[1]:.4f}]s")
            
            ci_results[f'{comp_type}_transfer_time'] = {
                'mean': mean_transfer,
                'ci_lower': ci_transfer[0],
                'ci_upper': ci_transfer[1]
            }
        
        return ci_results
    
    def effect_size_analysis(self, df):
        """效应量分析"""
        print("\n" + "="*60)
        print("效应量分析")
        print("="*60)
        
        jpeg_data = df[df['compression_type'] == 'jpeg']['total_time']
        none_data = df[df['compression_type'] == 'none']['total_time']
        
        if len(jpeg_data) > 0 and len(none_data) > 0:
            # Cohen's d
            pooled_std = np.sqrt(((len(jpeg_data) - 1) * jpeg_data.var() + 
                                 (len(none_data) - 1) * none_data.var()) / 
                                (len(jpeg_data) + len(none_data) - 2))
            cohens_d = (jpeg_data.mean() - none_data.mean()) / pooled_std
            
            print(f"Cohen's d: {cohens_d:.4f}")
            
            if abs(cohens_d) < 0.2:
                effect_size = "小"
            elif abs(cohens_d) < 0.5:
                effect_size = "中等"
            elif abs(cohens_d) < 0.8:
                effect_size = "大"
            else:
                effect_size = "非常大"
            
            print(f"效应量: {effect_size}")
            
            return {
                'cohens_d': cohens_d,
                'effect_size_interpretation': effect_size
            }
        
        return {}
    
    def generate_report(self, df):
        """生成完整的分析报告"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 执行所有分析
        summary = self.statistical_summary(df)
        hypothesis_results = self.hypothesis_testing(df)
        ci_results = self.confidence_intervals(df)
        effect_size = self.effect_size_analysis(df)
        
        # 生成可视化
        self.generate_visualizations(df)
        
        # 合并所有结果
        full_report = {
            'timestamp': timestamp,
            'summary': summary,
            'hypothesis_testing': hypothesis_results,
            'confidence_intervals': ci_results,
            'effect_size': effect_size,
            'recommendations': self._generate_recommendations(df, hypothesis_results, effect_size)
        }
        
        # 保存报告
        report_file = self.output_dir / f'comprehensive_analysis_{timestamp}.json'
        with open(report_file, 'w', encoding='utf-8') as f:
            json.dump(full_report, f, indent=2, ensure_ascii=False, default=str)
        
        print(f"\n完整分析报告已保存到: {report_file}")
        
        return full_report
    
    def _generate_recommendations(self, df, hypothesis_results, effect_size):
        """生成建议"""
        recommendations = []
        
        # 基于样本量的建议
        total_samples = len(df)
        if total_samples < 30:
            recommendations.append("建议增加样本量到至少30个以提高统计检验的可靠性")
        
        # 基于假设检验结果的建议
        if 'jpeg_vs_none_ttest' in hypothesis_results:
            if hypothesis_results['jpeg_vs_none_ttest']['significant']:
                recommendations.append("JPEG压缩与无压缩在传输时间上存在显著差异，建议进一步分析具体场景")
            else:
                recommendations.append("JPEG压缩与无压缩在传输时间上无显著差异，可能需要更大样本量或检查测试条件")
        
        # 基于效应量的建议
        if 'cohens_d' in effect_size:
            if abs(effect_size['cohens_d']) < 0.2:
                recommendations.append("压缩方法间的效应量较小，实际应用中差异可能不明显")
            elif abs(effect_size['cohens_d']) > 0.8:
                recommendations.append("压缩方法间存在大的效应量，建议在实际应用中优先考虑性能更好的方法")
        
        # 基于数据分布的建议
        jpeg_cv = df[df['compression_type'] == 'jpeg']['total_time'].std() / df[df['compression_type'] == 'jpeg']['total_time'].mean()
        if jpeg_cv > 0.5:
            recommendations.append("JPEG压缩的性能变异较大，建议分析影响因素并优化测试条件")
        
        return recommendations

def main():
    analyzer = AdvancedAnalyzer()
    
    try:
        df = analyzer.load_and_clean_data()
        report = analyzer.generate_report(df)
        
        print("\n" + "="*60)
        print("分析完成！")
        print("="*60)
        print("生成的文件:")
        print(f"- 性能分析图表: {analyzer.output_dir}/performance_analysis.png")
        print(f"- 压缩分析图表: {analyzer.output_dir}/compression_analysis.png")
        print(f"- 完整分析报告: {analyzer.output_dir}/comprehensive_analysis_*.json")
        
    except FileNotFoundError as e:
        print(f"错误: {e}")
        print("请先运行测试生成数据文件")
    except Exception as e:
        print(f"分析过程中出现错误: {e}")

if __name__ == "__main__":
    main() 