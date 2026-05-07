import pandas as pd
import matplotlib.pyplot as plt
import sys

def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_results.py <path_to_csv>")
        sys.exit(1)

    csv_file = sys.argv[1]
    df = pd.read_csv(csv_file)

    # Настройка стиля
    plt.style.use('seaborn-whitegrid') 
    
    # Создаем окно с двумя графиками (1 строка, 2 колонки)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    # --- График 1: Траектории (Вид сверху на карту) ---
    ax1.plot(df['gt_x'].to_numpy(), df['gt_y'].to_numpy(), label='Ground Truth (Истина)', color='green', linewidth=2)
    ax1.plot(df['odom_x'].to_numpy(), df['odom_y'].to_numpy(), label='Dead Reckoning (Слепая одометрия)', color='red', linestyle='dashed')
    ax1.plot(df['amcl_x'].to_numpy(), df['amcl_y'].to_numpy(), label='AMCL Estimate', color='blue', alpha=0.7)
    
    ax1.set_title('Trajectory Comparison', fontsize=14)
    ax1.set_xlabel('X (meters)')
    ax1.set_ylabel('Y (meters)')
    ax1.legend()
    ax1.axis('equal') # Сохранение пропорций геометрии

    # --- График 2: Ошибка позиционирования во времени (RMSE) ---
    ax2.plot(df['step'].to_numpy(), df['err_odom'].to_numpy(), label='Odometry Error', color='red')
    ax2.plot(df['step'].to_numpy(), df['err_amcl'].to_numpy(), label='AMCL Error', color='blue')
    
    ax2.set_title('Positioning Error over Time', fontsize=14)
    ax2.set_xlabel('Simulation Step')
    ax2.set_ylabel('Absolute Error (meters)')
    ax2.legend()
    
    # Ограничение по оси Y для наглядности (если одометрия улетает в бесконечность)
    max_amcl_err = df['err_amcl'].max()
    ax2.set_ylim(0, max_amcl_err * 3) 

    plt.tight_layout()
    plt.savefig('benchmark_plot.png', dpi=300)
    print("Plot saved to benchmark_plot.png")
    plt.show()

if __name__ == '__main__':
    main()