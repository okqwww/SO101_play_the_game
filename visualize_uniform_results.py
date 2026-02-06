#!/usr/bin/env python3
"""
均匀网格IK测试结果可视化

显示882个均匀分布的测试点，绿色=误差<10mm，红色=误差≥10mm
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
import sys

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


class UniformResultVisualizer:
    def __init__(self, csv_file):
        """加载测试结果"""
        print(f"📊 加载测试结果: {csv_file}")
        self.df = pd.read_csv(csv_file)
        
        # 只保留成功的测试
        self.df_success = self.df[self.df['success'] == True].copy()
        
        print(f"✅ 成功加载 {len(self.df_success)} 条有效数据")
        
        # 输出文件夹
        self.output_dir = Path(csv_file).parent / "visualizations"
        self.output_dir.mkdir(exist_ok=True)
    
    def plot_3d_precision_map(self):
        """3D精度分布图 - 绿色=<10mm，红色=≥10mm"""
        print("\n📈 生成3D精度分布图...")
        
        fig = plt.figure(figsize=(20, 15))
        
        # 分离好点和差点
        good_mask = self.df_success['actual_error_mm'] < 10.0
        good_data = self.df_success[good_mask]
        bad_data = self.df_success[~good_mask]
        
        print(f"  精度达标点: {len(good_data)} 个（绿色）")
        print(f"  精度不达标: {len(bad_data)} 个（红色）")
        
        # 子图1: 3D全景图
        ax1 = fig.add_subplot(221, projection='3d')
        
        # 绘制好点（绿色）
        if len(good_data) > 0:
            ax1.scatter(good_data['target_x'], good_data['target_y'], good_data['target_z'],
                       c='limegreen', s=50, alpha=0.8, label=f'精度达标 (<10mm): {len(good_data)}',
                       edgecolors='darkgreen', linewidth=0.5)
        
        # 绘制差点（红色）
        if len(bad_data) > 0:
            ax1.scatter(bad_data['target_x'], bad_data['target_y'], bad_data['target_z'],
                       c='red', s=50, alpha=0.8, label=f'精度不达标 (≥10mm): {len(bad_data)}',
                       edgecolors='darkred', linewidth=0.5)
        
        ax1.set_xlabel('X (m)', fontsize=14, fontweight='bold')
        ax1.set_ylabel('Y (m)', fontsize=14, fontweight='bold')
        ax1.set_zlabel('Z (m)', fontsize=14, fontweight='bold')
        ax1.set_title('3D精度分布图（全景）', fontsize=16, fontweight='bold', pad=20)
        ax1.legend(fontsize=12, loc='upper right')
        ax1.grid(True, alpha=0.3)
        
        # 子图2: XY平面俯视图（Z轴被投影）
        ax2 = fig.add_subplot(222)
        
        if len(good_data) > 0:
            ax2.scatter(good_data['target_x'], good_data['target_y'],
                       c='limegreen', s=80, alpha=0.6, label=f'<10mm: {len(good_data)}',
                       edgecolors='darkgreen', linewidth=0.5)
        
        if len(bad_data) > 0:
            ax2.scatter(bad_data['target_x'], bad_data['target_y'],
                       c='red', s=80, alpha=0.6, label=f'≥10mm: {len(bad_data)}',
                       edgecolors='darkred', linewidth=0.5)
        
        ax2.set_xlabel('X (m)', fontsize=14, fontweight='bold')
        ax2.set_ylabel('Y (m)', fontsize=14, fontweight='bold')
        ax2.set_title('XY平面俯视图', fontsize=16, fontweight='bold')
        ax2.legend(fontsize=12)
        ax2.grid(True, alpha=0.3)
        ax2.set_aspect('equal', adjustable='box')
        
        # 子图3: XZ平面侧视图
        ax3 = fig.add_subplot(223)
        
        if len(good_data) > 0:
            ax3.scatter(good_data['target_x'], good_data['target_z'],
                       c='limegreen', s=80, alpha=0.6, label=f'<10mm: {len(good_data)}',
                       edgecolors='darkgreen', linewidth=0.5)
        
        if len(bad_data) > 0:
            ax3.scatter(bad_data['target_x'], bad_data['target_z'],
                       c='red', s=80, alpha=0.6, label=f'≥10mm: {len(bad_data)}',
                       edgecolors='darkred', linewidth=0.5)
        
        ax3.set_xlabel('X (m)', fontsize=14, fontweight='bold')
        ax3.set_ylabel('Z (m)', fontsize=14, fontweight='bold')
        ax3.set_title('XZ平面侧视图', fontsize=16, fontweight='bold')
        ax3.legend(fontsize=12)
        ax3.grid(True, alpha=0.3)
        
        # 子图4: YZ平面正视图
        ax4 = fig.add_subplot(224)
        
        if len(good_data) > 0:
            ax4.scatter(good_data['target_y'], good_data['target_z'],
                       c='limegreen', s=80, alpha=0.6, label=f'<10mm: {len(good_data)}',
                       edgecolors='darkgreen', linewidth=0.5)
        
        if len(bad_data) > 0:
            ax4.scatter(bad_data['target_y'], bad_data['target_z'],
                       c='red', s=80, alpha=0.6, label=f'≥10mm: {len(bad_data)}',
                       edgecolors='darkred', linewidth=0.5)
        
        ax4.set_xlabel('Y (m)', fontsize=14, fontweight='bold')
        ax4.set_ylabel('Z (m)', fontsize=14, fontweight='bold')
        ax4.set_title('YZ平面正视图', fontsize=16, fontweight='bold')
        ax4.legend(fontsize=12)
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        output_file = self.output_dir / "3d_precision_map.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ 已保存: {output_file}")
        plt.close()
    
    def plot_error_heatmap(self):
        """误差热力图 - 显示误差的连续分布"""
        print("\n🗺️  生成误差热力图...")
        
        fig = plt.figure(figsize=(20, 15))
        
        # 为每个Z层创建一个XY热力图
        z_values = sorted(self.df_success['target_z'].unique())
        
        for idx, z_val in enumerate(z_values, 1):
            z_data = self.df_success[self.df_success['target_z'] == z_val]
            
            ax = fig.add_subplot(2, 3, idx)
            
            # 绘制散点图，颜色表示误差
            scatter = ax.scatter(z_data['target_x'], z_data['target_y'],
                               c=z_data['actual_error_mm'],
                               cmap='RdYlGn_r', s=200, alpha=0.8,
                               vmin=0, vmax=50,  # 统一颜色范围
                               edgecolors='black', linewidth=1)
            
            # 在点上标注误差值
            for _, row in z_data.iterrows():
                if row['actual_error_mm'] < 10:
                    color = 'white'
                    weight = 'bold'
                else:
                    color = 'black'
                    weight = 'normal'
                
                ax.text(row['target_x'], row['target_y'], 
                       f"{row['actual_error_mm']:.1f}",
                       fontsize=6, ha='center', va='center',
                       color=color, weight=weight)
            
            ax.set_xlabel('X (m)', fontsize=12)
            ax.set_ylabel('Y (m)', fontsize=12)
            ax.set_title(f'Z = {z_val*1000:.1f}mm 平面', fontsize=14, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal', adjustable='box')
            
            cbar = plt.colorbar(scatter, ax=ax)
            cbar.set_label('实际误差 (mm)', fontsize=10)
        
        # 添加整体统计子图
        ax_stats = fig.add_subplot(2, 3, 4)
        ax_stats.axis('off')
        
        good_count = len(self.df_success[self.df_success['actual_error_mm'] < 10])
        total_count = len(self.df_success)
        
        stats_text = f"""
        测试统计
        {'='*40}
        
        总测试点数: {total_count}
        
        精度达标 (<10mm): {good_count} ({good_count/total_count*100:.1f}%)
        精度不达标 (≥10mm): {total_count-good_count} ({(total_count-good_count)/total_count*100:.1f}%)
        
        误差统计:
          最小: {self.df_success['actual_error_mm'].min():.2f} mm
          最大: {self.df_success['actual_error_mm'].max():.2f} mm
          平均: {self.df_success['actual_error_mm'].mean():.2f} mm
          中位数: {self.df_success['actual_error_mm'].median():.2f} mm
        
        各Z层达标率:
        """
        
        for z_val in z_values:
            z_data = self.df_success[self.df_success['target_z'] == z_val]
            z_good = len(z_data[z_data['actual_error_mm'] < 10])
            z_total = len(z_data)
            stats_text += f"  Z={z_val*1000:.1f}mm: {z_good}/{z_total} ({z_good/z_total*100:.1f}%)\n"
        
        ax_stats.text(0.1, 0.5, stats_text, fontsize=12, family='monospace',
                     verticalalignment='center')
        
        plt.tight_layout()
        output_file = self.output_dir / "error_heatmap.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ 已保存: {output_file}")
        plt.close()
    
    def plot_ik_error_heatmap(self):
        """IK计算误差热力图 - 显示IK求解精度的空间分布"""
        print("\n🗺️  生成IK误差热力图...")
        
        fig = plt.figure(figsize=(20, 15))
        
        # 为每个Z层创建一个XY热力图
        z_values = sorted(self.df_success['target_z'].unique())
        
        for idx, z_val in enumerate(z_values, 1):
            z_data = self.df_success[self.df_success['target_z'] == z_val]
            
            ax = fig.add_subplot(2, 3, idx)
            
            # 绘制散点图，颜色表示IK误差
            scatter = ax.scatter(z_data['target_x'], z_data['target_y'],
                               c=z_data['ik_error_mm'],
                               cmap='RdYlGn_r', s=200, alpha=0.8,
                               vmin=0, vmax=50,  # 统一颜色范围
                               edgecolors='black', linewidth=1)
            
            # 在点上标注误差值
            for _, row in z_data.iterrows():
                if row['ik_error_mm'] < 10:
                    color = 'white'
                    weight = 'bold'
                else:
                    color = 'black'
                    weight = 'normal'
                
                ax.text(row['target_x'], row['target_y'], 
                       f"{row['ik_error_mm']:.1f}",
                       fontsize=6, ha='center', va='center',
                       color=color, weight=weight)
            
            ax.set_xlabel('X (m)', fontsize=12)
            ax.set_ylabel('Y (m)', fontsize=12)
            ax.set_title(f'Z = {z_val*1000:.1f}mm 平面 - IK误差', fontsize=14, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal', adjustable='box')
            
            cbar = plt.colorbar(scatter, ax=ax)
            cbar.set_label('IK误差 (mm)', fontsize=10)
        
        # 添加整体统计子图
        ax_stats = fig.add_subplot(2, 3, 4)
        ax_stats.axis('off')
        
        good_count = len(self.df_success[self.df_success['ik_error_mm'] < 10])
        total_count = len(self.df_success)
        
        stats_text = f"""
        IK计算误差统计
        {'='*40}
        
        总测试点数: {total_count}
        
        IK精度达标 (<10mm): {good_count} ({good_count/total_count*100:.1f}%)
        IK精度不达标 (≥10mm): {total_count-good_count} ({(total_count-good_count)/total_count*100:.1f}%)
        
        IK误差统计:
          最小: {self.df_success['ik_error_mm'].min():.2f} mm
          最大: {self.df_success['ik_error_mm'].max():.2f} mm
          平均: {self.df_success['ik_error_mm'].mean():.2f} mm
          中位数: {self.df_success['ik_error_mm'].median():.2f} mm
        
        各Z层IK精度:
        """
        
        for z_val in z_values:
            z_data = self.df_success[self.df_success['target_z'] == z_val]
            z_good = len(z_data[z_data['ik_error_mm'] < 10])
            z_total = len(z_data)
            z_avg = z_data['ik_error_mm'].mean()
            stats_text += f"  Z={z_val*1000:.1f}mm: {z_good}/{z_total} ({z_good/z_total*100:.1f}%), 平均{z_avg:.2f}mm\n"
        
        ax_stats.text(0.1, 0.5, stats_text, fontsize=12, family='monospace',
                     verticalalignment='center')
        
        plt.tight_layout()
        output_file = self.output_dir / "ik_error_heatmap.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ 已保存: {output_file}")
        plt.close()
    
    def plot_motor_error_heatmap(self):
        """电机执行误差热力图 - 显示电机控制精度的空间分布"""
        print("\n🗺️  生成电机误差热力图...")
        
        fig = plt.figure(figsize=(20, 15))
        
        # 为每个Z层创建一个XY热力图
        z_values = sorted(self.df_success['target_z'].unique())
        
        for idx, z_val in enumerate(z_values, 1):
            z_data = self.df_success[self.df_success['target_z'] == z_val]
            
            ax = fig.add_subplot(2, 3, idx)
            
            # 绘制散点图，颜色表示电机误差
            scatter = ax.scatter(z_data['target_x'], z_data['target_y'],
                               c=z_data['motor_error_mm'],
                               cmap='plasma', s=200, alpha=0.8,
                               vmin=0, vmax=30,  # 电机误差通常较小，调整范围
                               edgecolors='black', linewidth=1)
            
            # 在点上标注误差值
            for _, row in z_data.iterrows():
                if row['motor_error_mm'] < 10:
                    color = 'white'
                    weight = 'bold'
                else:
                    color = 'black'
                    weight = 'normal'
                
                ax.text(row['target_x'], row['target_y'], 
                       f"{row['motor_error_mm']:.1f}",
                       fontsize=6, ha='center', va='center',
                       color=color, weight=weight)
            
            ax.set_xlabel('X (m)', fontsize=12)
            ax.set_ylabel('Y (m)', fontsize=12)
            ax.set_title(f'Z = {z_val*1000:.1f}mm 平面 - 电机误差', fontsize=14, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal', adjustable='box')
            
            cbar = plt.colorbar(scatter, ax=ax)
            cbar.set_label('电机误差 (mm)', fontsize=10)
        
        # 添加整体统计子图
        ax_stats = fig.add_subplot(2, 3, 4)
        ax_stats.axis('off')
        
        good_count = len(self.df_success[self.df_success['motor_error_mm'] < 10])
        total_count = len(self.df_success)
        
        stats_text = f"""
        电机执行误差统计
        {'='*40}
        
        总测试点数: {total_count}
        
        电机精度达标 (<10mm): {good_count} ({good_count/total_count*100:.1f}%)
        电机精度不达标 (≥10mm): {total_count-good_count} ({(total_count-good_count)/total_count*100:.1f}%)
        
        电机误差统计:
          最小: {self.df_success['motor_error_mm'].min():.2f} mm
          最大: {self.df_success['motor_error_mm'].max():.2f} mm
          平均: {self.df_success['motor_error_mm'].mean():.2f} mm
          中位数: {self.df_success['motor_error_mm'].median():.2f} mm
        
        各Z层电机精度:
        """
        
        for z_val in z_values:
            z_data = self.df_success[self.df_success['target_z'] == z_val]
            z_good = len(z_data[z_data['motor_error_mm'] < 10])
            z_total = len(z_data)
            z_avg = z_data['motor_error_mm'].mean()
            stats_text += f"  Z={z_val*1000:.1f}mm: {z_good}/{z_total} ({z_good/z_total*100:.1f}%), 平均{z_avg:.2f}mm\n"
        
        ax_stats.text(0.1, 0.5, stats_text, fontsize=12, family='monospace',
                     verticalalignment='center')
        
        plt.tight_layout()
        output_file = self.output_dir / "motor_error_heatmap.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ 已保存: {output_file}")
        plt.close()
    
    def plot_error_distribution_analysis(self):
        """误差分布分析"""
        print("\n📊 生成误差分布分析图...")
        
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        
        # 1. 误差直方图
        axes[0, 0].hist(self.df_success['actual_error_mm'], bins=30,
                       color='steelblue', alpha=0.7, edgecolor='black')
        axes[0, 0].axvline(10, color='red', linestyle='--', linewidth=2, label='10mm阈值')
        axes[0, 0].axvline(self.df_success['actual_error_mm'].mean(),
                          color='green', linestyle='--', linewidth=2, label='平均值')
        axes[0, 0].set_xlabel('实际误差 (mm)', fontsize=12)
        axes[0, 0].set_ylabel('频数', fontsize=12)
        axes[0, 0].set_title('误差分布直方图', fontsize=14, fontweight='bold')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # 2. X轴误差趋势
        x_groups = self.df_success.groupby('target_x')['actual_error_mm'].agg(['mean', 'std', 'count'])
        axes[0, 1].errorbar(x_groups.index, x_groups['mean'], yerr=x_groups['std'],
                           fmt='o-', capsize=5, capthick=2, linewidth=2, markersize=8)
        axes[0, 1].axhline(10, color='red', linestyle='--', linewidth=2, label='10mm阈值')
        axes[0, 1].set_xlabel('X位置 (m)', fontsize=12)
        axes[0, 1].set_ylabel('平均误差 (mm)', fontsize=12)
        axes[0, 1].set_title('X轴误差趋势', fontsize=14, fontweight='bold')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # 3. Y轴误差趋势
        y_groups = self.df_success.groupby('target_y')['actual_error_mm'].agg(['mean', 'std', 'count'])
        axes[1, 0].errorbar(y_groups.index, y_groups['mean'], yerr=y_groups['std'],
                           fmt='o-', capsize=5, capthick=2, linewidth=2, markersize=8)
        axes[1, 0].axhline(10, color='red', linestyle='--', linewidth=2, label='10mm阈值')
        axes[1, 0].set_xlabel('Y位置 (m)', fontsize=12)
        axes[1, 0].set_ylabel('平均误差 (mm)', fontsize=12)
        axes[1, 0].set_title('Y轴误差趋势', fontsize=14, fontweight='bold')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        # 4. Z轴误差对比
        z_groups = self.df_success.groupby('target_z')['actual_error_mm'].agg(['mean', 'std', 'count'])
        axes[1, 1].bar(range(len(z_groups)), z_groups['mean'], 
                      yerr=z_groups['std'], capsize=10,
                      color=['green' if m < 10 else 'red' for m in z_groups['mean']],
                      alpha=0.7, edgecolor='black', linewidth=2)
        axes[1, 1].axhline(10, color='red', linestyle='--', linewidth=2, label='10mm阈值')
        axes[1, 1].set_xticks(range(len(z_groups)))
        axes[1, 1].set_xticklabels([f'{z*1000:.1f}mm' for z in z_groups.index])
        axes[1, 1].set_ylabel('平均误差 (mm)', fontsize=12)
        axes[1, 1].set_title('Z轴误差对比', fontsize=14, fontweight='bold')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        output_file = self.output_dir / "error_distribution_analysis.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ 已保存: {output_file}")
        plt.close()
    
    def generate_all_plots(self):
        """生成所有图表"""
        print("\n" + "="*70)
        print("开始生成可视化图表")
        print("="*70)
        
        self.plot_3d_precision_map()
        self.plot_error_heatmap()
        self.plot_ik_error_heatmap()
        self.plot_motor_error_heatmap()
        self.plot_error_distribution_analysis()
        
        print("\n" + "="*70)
        print(f"✅ 所有图表已保存到: {self.output_dir}")
        print("="*70)
        
        # 列出所有生成的文件
        print("\n📁 生成的文件：")
        for file in sorted(self.output_dir.glob("*.png")):
            print(f"  - {file.name}")


def main():
    """主程序"""
    if len(sys.argv) < 2:
        print("用法: python visualize_uniform_results.py <csv文件路径>")
        print("示例: python visualize_uniform_results.py outputs/ik_accuracy_test_uniform/test_results_20260122_150051.csv")
        
        # 尝试找到最新的CSV文件
        test_dir = Path("outputs/ik_accuracy_test_uniform")
        if test_dir.exists():
            csv_files = sorted(test_dir.glob("test_results_*.csv"), reverse=True)
            if csv_files:
                print(f"\n💡 找到最新的测试结果: {csv_files[0]}")
                response = input("是否使用此文件？(y/n): ")
                if response.lower() == 'y':
                    csv_file = csv_files[0]
                else:
                    sys.exit(1)
            else:
                print("\n❌ 未找到测试结果文件")
                sys.exit(1)
        else:
            print("\n❌ 测试结果目录不存在")
            sys.exit(1)
    else:
        csv_file = sys.argv[1]
    
    try:
        visualizer = UniformResultVisualizer(csv_file)
        visualizer.generate_all_plots()
        
        print("\n✅ 可视化完成！")
        print(f"\n📂 打开文件夹查看: {visualizer.output_dir}")
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
