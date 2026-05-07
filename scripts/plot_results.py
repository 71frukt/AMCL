import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_results.py <path_to_csv>")
        sys.exit(1)

    csv_file = sys.argv[1]
    df = pd.read_csv(csv_file)

    try:
        plt.style.use('seaborn-whitegrid')
    except OSError:
        plt.style.use('ggplot')
        
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    # --- ДОБАВЛЕНИЕ КАРТЫ НА ФОН ---
    # Транспонированная геометрия maps::GetMap1()
    map_grid = np.array([
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,
        1,0,0,0,0,0,1,0,0,1,1,1,1,0,0,1,
        1,0,1,1,0,0,1,0,0,1,0,0,1,0,0,1,
        1,0,1,1,0,0,0,0,0,1,0,0,1,0,0,1,
        1,0,0,0,0,0,0,0,0,1,1,0,1,0,0,1,
        1,1,1,1,1,0,0,1,1,1,1,0,1,0,0,1,
        1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,
        1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,
        1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,1,
        1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,
        1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,
        1,0,1,1,1,1,1,0,0,0,0,1,0,0,0,1,
        1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,
        1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
    ]).reshape(16, 16)

    # Отрисовка матрицы. cmap='Greys' задает цвет (0 - белый, 1 - черный).
    # alpha=0.3 делает препятствия полупрозрачными.
    ax1.imshow(map_grid, extent=[0, 16, 0, 16], origin='lower', cmap='Greys', alpha=0.3)

    # --- График 1: Траектории (Вид сверху на карту) ---
    ax1.plot(df['gt_x']  .to_numpy(), df['gt_y']  .to_numpy(), label='Ground Truth',  color='green', linewidth=2)
    ax1.plot(df['odom_x'].to_numpy(), df['odom_y'].to_numpy(), label='Odometry',      color='red',   linestyle='dashed')
    ax1.plot(df['amcl_x'].to_numpy(), df['amcl_y'].to_numpy(), label='AMCL Estimate', color='blue',  alpha=0.7)
    
    ax1.set_title('Trajectory Comparison', fontsize=14)
    ax1.set_xlabel('X (meters)')
    ax1.set_ylabel('Y (meters)')
    ax1.legend()
    ax1.axis('equal') 
    
    # Жесткая фиксация границ графика по размеру карты
    ax1.set_xlim(0, 16)
    ax1.set_ylim(0, 16)

    # --- График 2: Ошибка позиционирования во времени (RMSE) ---
    ax2.plot(df['step'].to_numpy(), df['err_odom'].to_numpy(), label='Odometry Error', color='red')
    ax2.plot(df['step'].to_numpy(), df['err_amcl'].to_numpy(), label='AMCL Error', color='blue')
    
    ax2.set_title('Positioning Error over Time', fontsize=14)
    ax2.set_xlabel('Simulation Step')
    ax2.set_ylabel('Absolute Error (meters)')
    ax2.legend()

    plt.tight_layout()
    plt.savefig('benchmark_plot.png', dpi=300)
    plt.show()

if __name__ == '__main__':
    main()