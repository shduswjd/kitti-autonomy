"""Plot navigation results: python results/navigation/plot.py [--show]."""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

RESULT_DIR = Path(__file__).resolve().parent


def plot_results(result_file=RESULT_DIR / 'result.csv', *, show=False):
    """Save trajectory/error plots and return differences from GPS/OXTS.

    These metrics compare against the correction input, not independent truth.
    """
    result_file = Path(result_file)
    df = pd.read_csv(result_file)
    estimate = df[['p_e_m', 'p_n_m', 'p_u_m']].to_numpy()
    gps = df[['gps_e_m', 'gps_n_m', 'gps_u_m']].to_numpy()
    time = df['time_s'].to_numpy()
    if len(time) == 0 or not np.all(np.isfinite(np.c_[time, estimate, gps])):
        raise ValueError('Expected nonempty, finite position/time data')
    if np.any(np.diff(time) <= 0):
        raise ValueError('Time must be strictly increasing')
    error = estimate - gps
    axis_rmse = np.sqrt(np.mean(error**2, axis=0))
    max_abs = np.max(np.abs(error), axis=0)
    metrics = pd.DataFrame({
        'rmse_m': np.r_[axis_rmse,
                       np.sqrt(np.mean(np.sum(error[:, :2]**2, axis=1))),
                       np.sqrt(np.mean(np.sum(error**2, axis=1)))],
        'max_abs_or_distance_m': np.r_[max_abs,
                                      np.max(np.linalg.norm(error[:, :2], axis=1)),
                                      np.max(np.linalg.norm(error, axis=1))],
    }, index=['east', 'north', 'up', 'horizontal', '3d'])
    metrics.index.name = 'component'
    metrics.to_csv(result_file.parent / 'metrics.csv')

    fig, ax = plt.subplots()
    ax.plot(gps[:, 0], gps[:, 1], label='GPS/OXTS', linewidth=4)
    ax.plot(estimate[:, 0], estimate[:, 1], label='ESKF', linewidth=2)
    ax.set(xlabel='East [m]', ylabel='North [m]', title='ESKF navigation trajectory')
    ax.grid(True)
    ax.legend()
    ax.set_aspect('equal', adjustable='box')
    fig.tight_layout()
    fig.savefig(result_file.parent / 'trajectory.png', dpi=150)

    error_fig, ax = plt.subplots()
    for i, axis in enumerate(('East', 'North', 'Up')):
        ax.plot(time - time[0], error[:, i], label=f'{axis} difference')
    ax.set(xlabel='Time [s]', ylabel='ESKF - GPS/OXTS [m]',
           title='Position difference from correction input')
    ax.grid(True)
    ax.legend()
    error_fig.tight_layout()
    error_fig.savefig(result_file.parent / 'result_error.png', dpi=150)
    if show:
        plt.show()
    plt.close(fig)
    plt.close(error_fig)
    return metrics


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()
    print(plot_results(show=args.show).to_string(float_format=lambda x: f'{x:.3f}'))
