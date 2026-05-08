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
        plt.style.use('seaborn-v0_8-whitegrid')
    except OSError:
        plt.style.use('ggplot')
        
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(22, 6))

    # --- КАРТА (ФОН) ---
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

    ax1.imshow(map_grid, extent=[0, 16, 0, 16], origin='lower', cmap='Greys', alpha=0.2)

    # --- ПОДГОТОВКА ДАННЫХ (Явное приведение к numpy) ---
    steps = df['step'].to_numpy()
    
    gt_x, gt_y = df['gt_x'].to_numpy(), df['gt_y'].to_numpy()
    odom_x, odom_y = df['odom_x'].to_numpy(), df['odom_y'].to_numpy()
    amcl_x, amcl_y = df['amcl_x'].to_numpy(), df['amcl_y'].to_numpy()
    mcl_x, mcl_y = df['mcl_x'].to_numpy(), df['mcl_y'].to_numpy()
    naive_x, naive_y = df['naive_x'].to_numpy(), df['naive_y'].to_numpy()
    
    err_odom = df['err_odom'].to_numpy()
    err_amcl = df['err_amcl'].to_numpy()
    err_mcl = df['err_mcl'].to_numpy()
    err_naive = df['err_naive'].to_numpy()
    
    t_amcl = df['time_amcl_us'].to_numpy()
    t_mcl = df['time_mcl_us'].to_numpy()
    t_naive = df['time_naive_us'].to_numpy()

    # --- ГРАФИК 1: ТРАЕКТОРИИ ---
    ax1.plot(gt_x, gt_y, label='Ground Truth', color='green', linewidth=2)
    ax1.plot(odom_x, odom_y, label='Odometry', color='red', linestyle='--', alpha=0.6)
    ax1.plot(amcl_x, amcl_y, label='AMCL', color='blue', alpha=0.8)
    ax1.plot(mcl_x, mcl_y, label='MCL', color='cyan', alpha=0.8)
    ax1.plot(naive_x, naive_y, label='Naive Search', color='darkorange', linestyle=':', alpha=0.9)
    
    ax1.set_title('Trajectory Comparison')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.legend()
    ax1.axis('equal')
    ax1.set_xlim(0, 16)
    ax1.set_ylim(0, 16)

    # --- ГРАФИК 2: ОШИБКИ ---
    ax2.plot(steps, err_odom, label='Odom Error', color='red', alpha=0.5)
    ax2.plot(steps, err_amcl, label='AMCL Error', color='blue')
    ax2.plot(steps, err_mcl, label='MCL Error', color='cyan')
    ax2.plot(steps, err_naive, label='Naive Error', color='darkorange')
    
    ax2.set_title('Absolute Errors')
    ax2.set_xlabel('Step')
    ax2.set_ylabel('Error (meters)')
    ax2.legend()

    # --- ГРАФИК 3: ВРЕМЯ ---
    ax3.plot(steps, t_amcl, label='AMCL', color='blue')
    ax3.plot(steps, t_mcl, label='MCL', color='cyan')
    ax3.plot(steps, t_naive, label='Naive', color='darkorange')
    
    ax3.set_yscale('log')
    ax3.set_title('Computation Time (Log Scale)')
    ax3.set_xlabel('Step')
    ax3.set_ylabel('Time (microseconds)')
    ax3.legend()

    plt.tight_layout()
    plt.savefig('benchmark_comparison.png', dpi=300)
    plt.show()

if __name__ == '__main__':
    main()