#!/usr/bin/env python3
"""
IK测试结果可视化工具

功能：
1. 3D散点图显示误差分布
2. XY/XZ/YZ平面热力图
3. 误差直方图
4. 区域箱线图
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm
from pathlib import Path
import json
import sys

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


class IKResultVisualizer:
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
        
    def plot_3d_scatter(self):
        """3D散点图 - 显示IK误差在空间中的分布"""
        print("\n📈 生成3D散点图...")
        
        fig = plt.figure(figsize=(16, 12))
        
        # 子图1: IK误差
        ax1 = fig.add_subplot(221, projection='3d')
        scatter1 = ax1.scatter(
            self.df_success['target_x'],
            self.df_success['target_y'],
            self.df_success['target_z'],
            c=self.df_success['ik_error_mm'],
            cmap='RdYlGn_r',  # 红-黄-绿倒序（红=大误差）
            s=100,
            alpha=0.6,
            edgecolors='black',
            linewidth=0.5
        )
        ax1.set_xlabel('X (m)', fontsize=12)
        ax1.set_ylabel('Y (m)', fontsize=12)
        ax1.set_zlabel('Z (m)', fontsize=12)
        ax1.set_title('IK误差分布 (mm)', fontsize=14, fontweight='bold')
        cbar1 = plt.colorbar(scatter1, ax=ax1, pad=0.1, shrink=0.8)
        cbar1.set_label('IK误差 (mm)', fontsize=10)
        
        # 子图2: 实际误差
        ax2 = fig.add_subplot(222, projection='3d')
        scatter2 = ax2.scatter(
            self.df_success['target_x'],
            self.df_success['target_y'],
            self.df_success['target_z'],
            c=self.df_success['actual_error_mm'],
            cmap='RdYlGn_r',
            s=100,
            alpha=0.6,
            edgecolors='black',
            linewidth=0.5
        )
        ax2.set_xlabel('X (m)', fontsize=12)
        ax2.set_ylabel('Y (m)', fontsize=12)
        ax2.set_zlabel('Z (m)', fontsize=12)
        ax2.set_title('实际误差分布 (mm)', fontsize=14, fontweight='bold')
        cbar2 = plt.colorbar(scatter2, ax=ax2, pad=0.1, shrink=0.8)
        cbar2.set_label('实际误差 (mm)', fontsize=10)
        
        # 子图3: 电机误差
        ax3 = fig.add_subplot(223, projection='3d')
        scatter3 = ax3.scatter(
            self.df_success['target_x'],
            self.df_success['target_y'],
            self.df_success['target_z'],
            c=self.df_success['motor_error_mm'],
            cmap='plasma',
            s=100,
            alpha=0.6,
            edgecolors='black',
            linewidth=0.5
        )
        ax3.set_xlabel('X (m)', fontsize=12)
        ax3.set_ylabel('Y (m)', fontsize=12)
        ax3.set_zlabel('Z (m)', fontsize=12)
        ax3.set_title('电机误差分布 (mm)', fontsize=14, fontweight='bold')
        cbar3 = plt.colorbar(scatter3, ax=ax3, pad=0.1, shrink=0.8)
        cbar3.set_label('电机误差 (mm)', fontsize=10)
        
        # 子图4: 最佳区域高亮
        ax4 = fig.add_subplot(224, projection='3d')
        # 绿色=低误差，红色=高误差
        colors = ['green' if err < 20 else 'orange' if err < 30 else 'red' 
                  for err in self.df_success['actual_error_mm']]
        ax4.scatter(
            self.df_success['target_x'],
            self.df_success['target_y'],
            self.df_success['target_z'],
            c=colors,
            s=100,
            alpha=0.6,
            edgecolors='black',
            linewidth=0.5
        )
        ax4.set_xlabel('X (m)', fontsize=12)
        ax4.set_ylabel('Y (m)', fontsize=12)
        ax4.set_zlabel('Z (m)', fontsize=12)
        ax4.set_title('最佳区域 (绿色<20mm)', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        output_file = self.output_dir / "3d_scatter.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ 已保存: {output_file}")
        plt.close()
    
    def plot_2d_heatmaps(self):
        """2D热力图 - XY, XZ, YZ平面"""
        print("\n🗺️  生成2D热力图...")
        
        fig = plt.figure(figsize=(18, 12))
        
        # 定义投影平面
        planes = [
            ('target_x', 'target_y', 'XY平面 (俯视图)', 'X (m)', 'Y (m)', 231),
            ('target_x', 'target_z', 'XZ平面 (侧视图)', 'X (m)', 'Z (m)', 232),
            ('target_y', 'target_z', 'YZ平面 (正视图)', 'Y (m)', 'Z (m)', 233),
        ]
        
        for x_col, y_col, title, xlabel, ylabel, subplot_idx in planes:
            ax = fig.add_subplot(subplot_idx)
            
            # 创建网格
            x = self.df_success[x_col].values
            y = self.df_success[y_col].values
            z = self.df_success['actual_error_mm'].values
            
            # 散点图 + 颜色映射
            scatter = ax.scatter(x, y, c=z, cmap='RdYlGn_r', s=150, 
                               alpha=0.7, edgecolors='black', linewidth=1)
            
            # 添加数值标签
            for i in range(len(x)):
                if z[i] < 25:  # 只标注低误差点
                    ax.annotate(f'{z[i]:.0f}', (x[i], y[i]), 
                              fontsize=7, ha='center', va='center',
                              color='white', weight='bold')
            
            ax.set_xlabel(xlabel, fontsize=12)
            ax.set_ylabel(ylabel, fontsize=12)
            ax.set_title(title, fontsize=14, fontweight='bold')
            ax.grid(True, alpha=0.3)
            
            cbar = plt.colorbar(scatter, ax=ax)
            cbar.set_label('实际误差 (mm)', fontsize=10)
        
        # IK误差的XY平面图
        planes_ik = [
            ('target_x', 'target_y', 'XY平面 - IK误差', 'X (m)', 'Y (m)', 234),
            ('target_x', 'target_z', 'XZ平面 - IK误差', 'X (m)', 'Z (m)', 235),
            ('target_y', 'target_z', 'YZ平面 - IK误差', 'Y (m)', 'Z (m)', 236),
        ]
        
        for x_col, y_col, title, xlabel, ylabel, subplot_idx in planes_ik:
            ax = fig.add_subplot(subplot_idx)
            
            x = self.df_success[x_col].values
            y = self.df_success[y_col].values
            z = self.df_success['ik_error_mm'].values
            
            scatter = ax.scatter(x, y, c=z, cmap='RdYlGn_r', s=150,
                               alpha=0.7, edgecolors='black', linewidth=1)
            
            ax.set_xlabel(xlabel, fontsize=12)
            ax.set_ylabel(ylabel, fontsize=12)
            ax.set_title(title, fontsize=14, fontweight='bold')
            ax.grid(True, alpha=0.3)
            
            cbar = plt.colorbar(scatter, ax=ax)
            cbar.set_label('IK误差 (mm)', fontsize=10)
        
        plt.tight_layout()
        output_file = self.output_dir / "2d_heatmaps.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ 已保存: {output_file}")
        plt.close()
    
    def plot_error_distributions(self):
        """误差分布直方图"""
        print("\n📊 生成误差分布图...")
        
        fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        
        # IK误差直方图
        axes[0, 0].hist(self.df_success['ik_error_mm'], bins=20, 
                       color='steelblue', alpha=0.7, edgecolor='black')
        axes[0, 0].axvline(self.df_success['ik_error_mm'].mean(), 
                          color='red', linestyle='--', linewidth=2, label='平均值')
        axes[0, 0].axvline(self.df_success['ik_error_mm'].median(),
                          color='green', linestyle='--', linewidth=2, label='中位数')
        axes[0, 0].set_xlabel('IK误差 (mm)', fontsize=12)
        axes[0, 0].set_ylabel('频数', fontsize=12)
        axes[0, 0].set_title('IK误差分布', fontsize=14, fontweight='bold')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # 实际误差直方图
        axes[0, 1].hist(self.df_success['actual_error_mm'], bins=20,
                       color='coral', alpha=0.7, edgecolor='black')
        axes[0, 1].axvline(self.df_success['actual_error_mm'].mean(),
                          color='red', linestyle='--', linewidth=2, label='平均值')
        axes[0, 1].axvline(self.df_success['actual_error_mm'].median(),
                          color='green', linestyle='--', linewidth=2, label='中位数')
        axes[0, 1].set_xlabel('实际误差 (mm)', fontsize=12)
        axes[0, 1].set_ylabel('频数', fontsize=12)
        axes[0, 1].set_title('实际误差分布', fontsize=14, fontweight='bold')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # 电机误差直方图
        axes[0, 2].hist(self.df_success['motor_error_mm'], bins=20,
                       color='lightgreen', alpha=0.7, edgecolor='black')
        axes[0, 2].axvline(self.df_success['motor_error_mm'].mean(),
                          color='red', linestyle='--', linewidth=2, label='平均值')
        axes[0, 2].axvline(self.df_success['motor_error_mm'].median(),
                          color='green', linestyle='--', linewidth=2, label='中位数')
        axes[0, 2].set_xlabel('电机误差 (mm)', fontsize=12)
        axes[0, 2].set_ylabel('频数', fontsize=12)
        axes[0, 2].set_title('电机误差分布', fontsize=14, fontweight='bold')
        axes[0, 2].legend()
        axes[0, 2].grid(True, alpha=0.3)
        
        # X轴误差分布
        axes[1, 0].scatter(self.df_success['target_x'], 
                          self.df_success['actual_error_mm'],
                          c=self.df_success['actual_error_mm'],
                          cmap='RdYlGn_r', s=50, alpha=0.6)
        axes[1, 0].set_xlabel('X位置 (m)', fontsize=12)
        axes[1, 0].set_ylabel('实际误差 (mm)', fontsize=12)
        axes[1, 0].set_title('X轴 vs 误差', fontsize=14, fontweight='bold')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Y轴误差分布
        axes[1, 1].scatter(self.df_success['target_y'],
                          self.df_success['actual_error_mm'],
                          c=self.df_success['actual_error_mm'],
                          cmap='RdYlGn_r', s=50, alpha=0.6)
        axes[1, 1].set_xlabel('Y位置 (m)', fontsize=12)
        axes[1, 1].set_ylabel('实际误差 (mm)', fontsize=12)
        axes[1, 1].set_title('Y轴 vs 误差', fontsize=14, fontweight='bold')
        axes[1, 1].grid(True, alpha=0.3)
        
        # Z轴误差分布
        axes[1, 2].scatter(self.df_success['target_z'],
                          self.df_success['actual_error_mm'],
                          c=self.df_success['actual_error_mm'],
                          cmap='RdYlGn_r', s=50, alpha=0.6)
        axes[1, 2].set_xlabel('Z位置 (m)', fontsize=12)
        axes[1, 2].set_ylabel('实际误差 (mm)', fontsize=12)
        axes[1, 2].set_title('Z轴 vs 误差', fontsize=14, fontweight='bold')
        axes[1, 2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        output_file = self.output_dir / "error_distributions.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ 已保存: {output_file}")
        plt.close()
    
    def plot_region_boxplots(self):
        """区域箱线图 - 比较不同区域的误差"""
        print("\n📦 生成区域箱线图...")
        
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        
        # X轴区域
        self.df_success['x_region'] = pd.cut(
            self.df_success['target_x'],
            bins=[0.18, 0.21, 0.24, 0.35],
            labels=['近(0.18-0.21)', '中(0.21-0.24)', '远(0.24-0.35)']
        )
        
        self.df_success.boxplot(column='actual_error_mm', by='x_region', ax=axes[0])
        axes[0].set_xlabel('X轴区域', fontsize=12)
        axes[0].set_ylabel('实际误差 (mm)', fontsize=12)
        axes[0].set_title('X轴区域误差对比', fontsize=14, fontweight='bold')
        axes[0].get_figure().suptitle('')  # 移除自动标题
        
        # Y轴区域
        self.df_success['y_region'] = pd.cut(
            self.df_success['target_y'],
            bins=[-0.09, -0.03, 0.03, 0.09],
            labels=['右(-0.09~-0.03)', '中(-0.03~0.03)', '左(0.03~0.09)']
        )
        
        self.df_success.boxplot(column='actual_error_mm', by='y_region', ax=axes[1])
        axes[1].set_xlabel('Y轴区域', fontsize=12)
        axes[1].set_ylabel('实际误差 (mm)', fontsize=12)
        axes[1].set_title('Y轴区域误差对比', fontsize=14, fontweight='bold')
        
        # Z轴区域
        self.df_success['z_region'] = pd.cut(
            self.df_success['target_z'],
            bins=[0.01, 0.03, 0.045, 0.06],
            labels=['低(0.01-0.03)', '中(0.03-0.045)', '高(0.045-0.06)']
        )
        
        self.df_success.boxplot(column='actual_error_mm', by='z_region', ax=axes[2])
        axes[2].set_xlabel('Z轴区域', fontsize=12)
        axes[2].set_ylabel('实际误差 (mm)', fontsize=12)
        axes[2].set_title('Z轴区域误差对比', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        output_file = self.output_dir / "region_boxplots.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ 已保存: {output_file}")
        plt.close()
    
    def plot_best_region_highlight(self):
        """高亮显示最佳精度区域"""
        print("\n🌟 生成最佳区域高亮图...")
        
        # 定义最佳区域（实际误差 < 20mm）
        best_mask = self.df_success['actual_error_mm'] < 20
        best_data = self.df_success[best_mask]
        other_data = self.df_success[~best_mask]
        
        fig = plt.figure(figsize=(18, 6))
        
        # XY平面
        ax1 = fig.add_subplot(131)
        ax1.scatter(other_data['target_x'], other_data['target_y'],
                   c='lightcoral', s=100, alpha=0.5, label='误差≥20mm',
                   edgecolors='black', linewidth=0.5)
        ax1.scatter(best_data['target_x'], best_data['target_y'],
                   c='limegreen', s=100, alpha=0.8, label='误差<20mm',
                   edgecolors='black', linewidth=1)
        ax1.set_xlabel('X (m)', fontsize=12)
        ax1.set_ylabel('Y (m)', fontsize=12)
        ax1.set_title('XY平面 - 最佳区域', fontsize=14, fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # XZ平面
        ax2 = fig.add_subplot(132)
        ax2.scatter(other_data['target_x'], other_data['target_z'],
                   c='lightcoral', s=100, alpha=0.5, label='误差≥20mm',
                   edgecolors='black', linewidth=0.5)
        ax2.scatter(best_data['target_x'], best_data['target_z'],
                   c='limegreen', s=100, alpha=0.8, label='误差<20mm',
                   edgecolors='black', linewidth=1)
        ax2.set_xlabel('X (m)', fontsize=12)
        ax2.set_ylabel('Z (m)', fontsize=12)
        ax2.set_title('XZ平面 - 最佳区域', fontsize=14, fontweight='bold')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # YZ平面
        ax3 = fig.add_subplot(133)
        ax3.scatter(other_data['target_y'], other_data['target_z'],
                   c='lightcoral', s=100, alpha=0.5, label='误差≥20mm',
                   edgecolors='black', linewidth=0.5)
        ax3.scatter(best_data['target_y'], best_data['target_z'],
                   c='limegreen', s=100, alpha=0.8, label='误差<20mm',
                   edgecolors='black', linewidth=1)
        ax3.set_xlabel('Y (m)', fontsize=12)
        ax3.set_ylabel('Z (m)', fontsize=12)
        ax3.set_title('YZ平面 - 最佳区域', fontsize=14, fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        output_file = self.output_dir / "best_region_highlight.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"✅ 已保存: {output_file}")
        plt.close()
        
        # 打印最佳区域统计
        if len(best_data) > 0:
            print(f"\n🌟 最佳区域统计（误差<20mm）：")
            print(f"  数量: {len(best_data)}/{len(self.df_success)} ({len(best_data)/len(self.df_success)*100:.1f}%)")
            print(f"  X范围: {best_data['target_x'].min():.3f} ~ {best_data['target_x'].max():.3f} m")
            print(f"  Y范围: {best_data['target_y'].min():.3f} ~ {best_data['target_y'].max():.3f} m")
            print(f"  Z范围: {best_data['target_z'].min():.3f} ~ {best_data['target_z'].max():.3f} m")
            print(f"  平均误差: {best_data['actual_error_mm'].mean():.2f} mm")
    
    def generate_all_plots(self):
        """生成所有图表"""
        print("\n" + "="*70)
        print("开始生成可视化图表")
        print("="*70)
        
        self.plot_3d_scatter()
        self.plot_2d_heatmaps()
        self.plot_error_distributions()
        self.plot_region_boxplots()
        self.plot_best_region_highlight()
        
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
        print("用法: python visualize_ik_results.py <csv文件路径>")
        print("示例: python visualize_ik_results.py outputs/ik_accuracy_test/test_results_20260122_150051.csv")
        
        # 尝试找到最新的CSV文件
        test_dir = Path("outputs/ik_accuracy_test")
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
        visualizer = IKResultVisualizer(csv_file)
        visualizer.generate_all_plots()
        
        print("\n✅ 可视化完成！")
        print(f"\n📂 打开文件夹查看: {visualizer.output_dir}")
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
